#!/usr/bin/env python3
"""
Autonomous Lane Following Driver
Integrates:
- Lane Detection (Sliding Window + Polynomial Fit)
- PID Controller (Smooth Steering)
- Adaptive Speed Control
- Robot Hardware Interface

Threading model:
  - _processing_loop() rulează în thread separat (daemon)
  - recv() din WebRTC pune frame-uri în _frame_queue și citește din _result_queue
  - Astfel recv() returnează imediat, fără să blocheze event loop-ul asyncio
"""

import cv2
import numpy as np
import time
import logging
import threading
import queue
from lane_detection import LaneDetector, DetectionState
from pid_controller import PIDController, AdaptiveSpeedController
from config import (
    CFG,
    PROCESS_WIDTH, PROCESS_HEIGHT,
    CONTROL_DT, CONTROL_LOOP_HZ,
    LANE_WIDTH_METERS, LOOKAHEAD_FRAC,
)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class AutonomousDriver:
    """
    Complete autonomous driving system.
    Processes video frames and controls robot steering and speed.

    Arhitectură cu threading:
      _frame_queue  : WebRTC → thread de procesare (frame-uri brute)
      _result_queue : thread de procesare → WebRTC (rezultate + frame adnotat)
    """

    def __init__(self,
                 img_width=1920,
                 img_height=1080,
                 enable_hardware=False,
                 show_visualization=True,
                 draw_text=True,
                 mqtt_handler=None):
        """
        Initialize autonomous driver.

        Args:
            img_width: Rezoluția ORIGINALĂ a camerei (pentru overlay și vizualizare)
            img_height: Rezoluția ORIGINALĂ a camerei
            enable_hardware: If True, control actual robot hardware
            show_visualization: If True, display annotated frames
            draw_text: If True, draw steering and speed overlay
        """
        self.img_width = img_width
        self.img_height = img_height
        self.mqtt_handler = mqtt_handler
        self.enable_hardware = enable_hardware
        self.show_visualization = show_visualization
        self.draw_text = draw_text

        logger.info("Initializing Autonomous Driver...")

        # Lane detection — instanțiat la rezoluția de procesare (640×480)
        self.lane_detector = LaneDetector(
            img_width=PROCESS_WIDTH,
            img_height=PROCESS_HEIGHT,
            lane_width_meters=LANE_WIDTH_METERS,
        )

        # PID controller for steering — parametrii din config.yaml [pid]
        _pid = CFG["pid"]
        self.pid_controller = PIDController(
            kp=_pid["kp"],
            ki=_pid["ki"],
            kd=_pid["kd"],
            max_angle=_pid["max_angle"],
            alpha=_pid["alpha"],
        )

        # Speed controller — parametrii din config.yaml [speed]
        _spd = CFG["speed"]
        self.speed_controller = AdaptiveSpeedController(
            base_speed=_spd["base"],
            min_speed=_spd["min"],
            max_speed=_spd["max"],
        )

        # Hardware interface (optional)
        self.robot = None
        if enable_hardware:
            try:
                from robot_controller import RobotCarController
                self.robot = RobotCarController()
                logger.info("Robot hardware connected")
            except Exception as e:
                logger.error(f"Failed to connect robot hardware: {e}")
                self.enable_hardware = False

        # Performance metrics
        self.frame_count = 0
        self.fps = 0
        self.processing_time_avg = 0
        self.start_time = time.time()

        # Fallback state
        self.last_steering_angle = 0.0
        self.lookahead_y_frac = LOOKAHEAD_FRAC

        # Reset PID state tracking
        self._prev_state = None

        # ── Threading ──────────────────────────────────────────────────────────
        # maxsize=2: dacă procesarea e mai lentă decât camera, aruncăm frame-uri
        # vechi în loc să acumulăm o coadă infinită (latență crescută).
        self._frame_queue  = queue.Queue(maxsize=2)
        self._result_queue = queue.Queue(maxsize=2)
        self._stop_event   = threading.Event()

        # Rezultatul fallback returnat când coada e goală (primul frame)
        self._last_result = {
            'steering_angle': 0.0,
            'speed': self.speed_controller.min_speed,
            'lane_detected': False,
            'offset': None,
            'detection_mode': 'fallback',
            'annotated_frame': None,
            'processing_time': 0.0,
        }

        # Pornim worker-ul de procesare
        self._worker = threading.Thread(
            target=self._processing_loop,
            name='AutonomousDriver-Worker',
            daemon=True   # se oprește automat când procesul principal moare
        )
        self._worker.start()
        logger.info("Autonomous Driver initialized — processing thread started.")

    # ──────────────────────────────────────────────────────────────────────────
    # API PUBLIC
    # ──────────────────────────────────────────────────────────────────────────

    def process_frame_async(self, frame):
        """
        Interfață non-blocantă pentru recv() din WebRTC.

        Pune frame-ul în coadă pentru procesare și returnează IMEDIAT
        ultimul rezultat disponibil (din iterația anterioară).

        Dacă coada de frame-uri e plină (procesarea e mai lentă decât camera),
        frame-ul curent e ignorat — preferăm să sărim frame-uri decât să
        acumulăm latență.

        Returns:
            dict cu steering_angle, speed, annotated_frame etc.
        """
        # Încearcă să pună frame-ul în coadă; dacă e plină, ignoră
        try:
            self._frame_queue.put_nowait(frame)
        except queue.Full:
            logger.debug("Frame queue full — skipping frame (processing slower than camera)")

        # Returnează cel mai recent rezultat disponibil, fără blocare
        try:
            result = self._result_queue.get_nowait()
            self._last_result = result
        except queue.Empty:
            pass  # nu s-a terminat niciun frame — returnăm ultimul cunoscut

        return self._last_result

    def process_frame(self, frame):
        """
        Interfață sincronă (blocantă) — păstrată pentru compatibilitate
        cu test_lane_video.py și alte scripturi de test.
        """
        return self._process_frame_internal(frame)

    def cleanup(self):
        """Oprește thread-ul de procesare și eliberează resursele hardware."""
        logger.info("Cleaning up Autonomous Driver...")
        self._stop_event.set()
        # Deblocăm thread-ul dacă e blocat pe queue.get()
        try:
            self._frame_queue.put_nowait(None)
        except queue.Full:
            pass
        self._worker.join(timeout=2.0)
        if self.robot:
            self.robot.cleanup()
        logger.info("Cleanup complete")

    def __del__(self):
        self.cleanup()

    # ──────────────────────────────────────────────────────────────────────────
    # THREAD DE PROCESARE
    # ──────────────────────────────────────────────────────────────────────────

    def _processing_loop(self):
        """
        Rulează în thread separat.
        Preia frame-uri din _frame_queue, procesează, pune rezultatul în _result_queue.
        """
        logger.info("[Worker] Processing loop started")
        while not self._stop_event.is_set():
            try:
                # Blocare cu timeout ca să putem verifica _stop_event periodic
                frame = self._frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if frame is None:
                # Semnal de oprire trimis din cleanup()
                break

            try:
                result = self._process_frame_internal(frame)
            except Exception as e:
                import traceback
                logger.error(f"[Worker] Crash in _process_frame_internal: {e}\n{traceback.format_exc()}")
                print(f"CRITICAL WORKER CRASH: {e}\n{traceback.format_exc()}")
                # Păstrăm ultimul rezultat ca să nu se blocheze interfața
                result = self._last_result

            # Pune rezultatul în coadă; dacă e plină, înlocuiește rezultatul vechi
            # (preferăm date proaspete față de date vechi)
            if self._result_queue.full():
                try:
                    self._result_queue.get_nowait()
                except queue.Empty:
                    pass
            try:
                self._result_queue.put_nowait(result)
            except queue.Full:
                pass

        logger.info("[Worker] Processing loop stopped")

    # ──────────────────────────────────────────────────────────────────────────
    # LOGICA DE PROCESARE (neschimbată față de versiunea anterioară)
    # ──────────────────────────────────────────────────────────────────────────

    def _process_frame_internal(self, frame):
        """
        Procesează un singur frame și generează comenzile de control.
        Această metodă rulează în thread-ul worker, NU în event loop-ul asyncio.
        """
        start_time = time.time()

        # Redimensionăm la rezoluția de procesare
        small = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT))

        # Detectăm benzile pe imaginea mică
        detection = self.lane_detector.detect_lanes(small)

        left_fit  = detection['left_fit']
        right_fit = detection['right_fit']
        state     = detection['detection_state']
        side      = detection['one_lane_side']
        lane_w    = self.lane_detector.lane_width_pixels

        lookahead_y = int(PROCESS_HEIGHT * self.lookahead_y_frac)

        steering_angle    = 0.0
        speed             = self.speed_controller.min_speed
        detection_mode    = 'fallback'
        raw_detected_angle = 0.0
        offset            = None

        if state in (DetectionState.TRACKING, DetectionState.EXTRAPOLATING):
            _, offset = self.lane_detector.calculate_lane_center(
                left_fit, right_fit, y_eval=lookahead_y
            )
            if offset is None:
                offset = detection['offset']
            detection_mode = 'both'

        elif state == DetectionState.ONE_LANE and lane_w is not None:
            _, offset = self.lane_detector.calculate_lane_center(
                left_fit, right_fit, y_eval=lookahead_y
            )
            if offset is None:
                offset = detection['offset']
            detection_mode = f'{side}_only'
            logger.warning(f"[{detection_mode.upper()}] offset={offset:.1f}px lane_w={lane_w:.0f}px")

        else:
            steering_angle = self.last_steering_angle
            speed = 0.0
            logger.warning(f"[FALLBACK] state={state.name} - stopping car")

        if offset is not None:
            raw_detected_angle = (offset / (PROCESS_WIDTH / 2)) * self.pid_controller.max_angle

            # dt=None permite PID-ului sa calculeze delta time real intre cadre
            steering_angle = self.pid_controller.calculate_steering_angle(
                lane_offset_pixels=offset,
                img_width=PROCESS_WIDTH,
                dt=None
            )
            speed = self.speed_controller.calculate_speed(
                steering_angle=steering_angle,
                max_angle=50.0
            )
            self.last_steering_angle = steering_angle

        # Publică telemetria PID pe robot/pid_telemetry (grafic receiver)
        if self.mqtt_handler is not None and offset is not None:
            import time as _time
            _norm = max(-1.0, min(1.0, offset / (PROCESS_WIDTH / 2.0)))
            self.mqtt_handler.publish_pid_telemetry(
                response=_norm,
                steering_angle=steering_angle,
                timestamp_ms=int(_time.time() * 1000),
            )

        # Reset integral PID la recuperare din SEARCHING
        current_state = detection['detection_state']
        if self._prev_state == DetectionState.SEARCHING and current_state != DetectionState.SEARCHING:
            self.pid_controller.reset()
            logger.info("[PID] Integral reset — ieșit din SEARCHING")
        self._prev_state = current_state

        # Gardă finală viteză (doar dacă nu suntem în fallback)
        if detection_mode != 'fallback':
            speed = max(self.speed_controller.min_speed, speed)

        # Hardware
        if self.enable_hardware and self.robot:
            try:
                robot_angle = max(-50.0, min(50.0, steering_angle))
                self.robot.update(angle=robot_angle, speed=speed)
            except Exception as e:
                logger.error(f"Hardware control error: {e}")

        # Vizualizare: LaneDetector e calibrat la PROCESS_WIDTH×PROCESS_HEIGHT (640×480),
        # deci visualize_lanes() trebuie să primească tot frame-ul mic (small).
        # Rezultatul adnotat e redimensionat înapoi la rezoluția originală a camerei.
        annotated = None
        if self.show_visualization:
            annotated_small = self.lane_detector.visualize_lanes(
                small, detection, steering_angle=steering_angle
            )
            # Redimensionăm la rezoluția originală pentru streaming WebRTC
            annotated = cv2.resize(annotated_small, (self.img_width, self.img_height),
                                   interpolation=cv2.INTER_LINEAR)
            if self.draw_text:
                scale = self.img_width / 1920.0
                rx = self.img_width - int(520 * scale)
                font_scale = max(0.4, 1.0 * scale)
                thick = max(1, int(2 * scale))

                cv2.putText(annotated, f"Angle detected: {raw_detected_angle:.1f} deg",
                            (rx, int(50 * scale)), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 200, 255), thick)
                cv2.putText(annotated, f"Steering (PID): {steering_angle:.1f} deg",
                            (rx, int(100 * scale)), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 0), thick)
                cv2.putText(annotated, f"Speed: {speed:.1f}%",
                            (rx, int(150 * scale)), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 0), thick)
                mode_colors = {
                    'both':       (0, 255, 0),
                    'left_only':  (0, 165, 255),
                    'right_only': (0, 165, 255),
                    'fallback':   (0, 0, 255),
                }
                mode_color = mode_colors.get(detection_mode, (255, 255, 255))
                cv2.putText(annotated, f"MODE: {detection_mode.upper()}",
                            (rx, int(250 * scale)), cv2.FONT_HERSHEY_SIMPLEX, font_scale, mode_color, thick)

        # Metrici de performanță
        processing_time = time.time() - start_time
        self.frame_count += 1
        self.processing_time_avg = self.processing_time_avg * 0.9 + processing_time * 0.1
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            self.fps = self.frame_count / elapsed

        return {
            'steering_angle':  steering_angle,
            'speed':           speed,
            'lane_detected':   detection['detected'],
            'offset':          detection['offset'],
            'detection_mode':  detection_mode,
            'annotated_frame': annotated,
            'processing_time': processing_time,
        }


if __name__ == "__main__":
    print("=" * 60)
    print("Autonomous Driver - Component Test")
    print("=" * 60)

    driver = AutonomousDriver(
        img_width=1920,
        img_height=1080,
        enable_hardware=False,
        show_visualization=False
    )

    print("\nAutonomous Driver initialized successfully!")
    print("\nComponents:")
    print("  - Lane Detector: OK")
    print("  - PID Controller: OK")
    print("  - Speed Controller: OK")

    # Test sincron
    dummy_frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    result = driver.process_frame(dummy_frame)
    print(f"\nSync test:")
    print(f"  - Lanes detected: {result['lane_detected']}")
    print(f"  - Processing time: {result['processing_time']*1000:.1f}ms")

    # Test async (simulează recv())
    time.sleep(0.1)
    result2 = driver.process_frame_async(dummy_frame)
    print(f"\nAsync test:")
    print(f"  - Detection mode: {result2['detection_mode']}")

    driver.cleanup()
    print("\nTest complete!")