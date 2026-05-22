#!/usr/bin/env python3
"""
Video track using Raspberry Pi Camera Module 3 NoIR
Replaces file-based video with live camera feed

Modificări față de versiunea anterioară:
  - recv() folosește process_frame_async() — nu mai blochează event loop-ul asyncio
  - Telemetria MQTT e throttled la TELEMETRY_HZ (10Hz) în loc de 30Hz per frame
  - AutonomousDriver instanțiat cu rezoluția ORIGINALĂ a camerei (pentru overlay corect)
"""
import psutil
import os
import time
import numpy as np
import cv2
from aiortc import VideoStreamTrack
from av import VideoFrame
from picamera2 import Picamera2
from libcamera import controls
import logging
from autonomous_driver import AutonomousDriver
from config import CFG, PROCESS_WIDTH, PROCESS_HEIGHT, TELEMETRY_HZ, TELEMETRY_INTERVAL

logger = logging.getLogger(__name__)


class PiCameraTrackWithMQTT(VideoStreamTrack):
    """
    Video track that captures from Raspberry Pi Camera Module 3 NoIR
    and integrates MQTT sensor data overlay.

    recv() este non-blocant:
      - captează frame de la cameră
      - pune frame-ul în coada de procesare (process_frame_async)
      - returnează imediat ultimul frame adnotat disponibil
      - procesarea AI rulează în thread separat (AutonomousDriver)
    """

    def __init__(self, width: int = 1920, height: int = 1080, fps: int = 30, mqtt_handler=None):
        super().__init__()
        self.width  = width
        self.height = height
        self.target_fps  = fps
        self.mqtt_handler = mqtt_handler
        self.frame_count  = 0
        self.start_time   = time.time()

        # Throttling telemetrie MQTT
        self._last_telemetry_time = 0.0

        # Initialize Pi Camera
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={
                "FrameRate": fps,
                "AeEnable": True,
                "AwbEnable": True,
            }
        )
        self.picam2.configure(config)

        try:
            self.picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        except Exception:
            logger.warning("Camera autofocus not available")

        self.picam2.start()
        logger.info(f"Pi Camera started: {width}x{height} @ {fps} FPS")
        time.sleep(2)

        # AutonomousDriver instanțiat cu rezoluția ORIGINALĂ a camerei.
        # Intern, LaneDetector lucrează la PROCESS_WIDTH×PROCESS_HEIGHT (640×480),
        # dar overlay-ul text e calculat față de img_width/img_height (1920×1080).
        self.autonomous_driver = AutonomousDriver(
            img_width=width,
            img_height=height,
            enable_hardware=False,
            show_visualization=True,
            draw_text=False
        )

    def get_system_stats(self):
        """Extrage utilizarea CPU, RAM și Temperatura."""
        cpu = psutil.cpu_percent()
        ram = psutil.virtual_memory().percent
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = float(f.read()) / 1000.0
        except Exception:
            temp = 0.0
        bat = 0.0
        if hasattr(self, 'ups') and self.ups:
            try:
                bat = self.ups.get_percentage()
            except Exception:
                bat = 0.0
        return cpu, ram, temp, bat

    def _get_sensor_data(self):
        """Get sensor data from MQTT commands."""
        if self.mqtt_handler and self.mqtt_handler.connected:
            commands = self.mqtt_handler.received_commands
            unghi = commands.get('unghi_manual', 0)
            viteza = commands.get('viteza_manual', 0)
            mode   = commands.get('mod_de_functionare', 0)
            return unghi, viteza, mode
        return 0, 0, 0

    async def recv(self):
        """
        Capture frame from camera with sensor data overlay.

        NON-BLOCANT: process_frame_async() pune frame-ul în coada worker-ului
        și returnează imediat ultimul rezultat disponibil.
        Event loop-ul asyncio nu mai e blocat de procesarea OpenCV.
        """
        pts, time_base = await self.next_timestamp()

        # Captare frame de la cameră
        frame = self.picam2.capture_array()

        current_time = time.time()
        timestamp_ms = int(current_time * 1000)
        elapsed      = current_time - self.start_time
        self.frame_count += 1

        unghi, viteza, mode = self._get_sensor_data()

        if mode == 0:  # AUTO
            # Non-blocant — returnează imediat ultimul rezultat din thread-ul worker
            result = self.autonomous_driver.process_frame_async(frame)

            unghi  = result['steering_angle']
            viteza = result['speed']
            annotated_frame = result['annotated_frame']

            if annotated_frame is not None:
                frame = annotated_frame

            # Control hardware — decuplat de WebRTC
            if self.mqtt_handler and self.mqtt_handler.robot_controller:
                self.mqtt_handler.robot_controller.update(angle=unghi, speed=viteza)
        else:
            # MANUAL — frame brut, unghi/viteza vin din MQTT
            pass

        # ── Telemetrie MQTT throttled la TELEMETRY_HZ ─────────────────────────
        # În loc să publicăm de 30 ori/secundă, publicăm la 10Hz.
        # Broker-ul MQTT și dashboard-ul nu au nevoie de date mai dese.
        if self.mqtt_handler and self.mqtt_handler.connected:
            if current_time - self._last_telemetry_time >= TELEMETRY_INTERVAL:
                self.mqtt_handler.publish_sensor_data(unghi, viteza, timestamp_ms)
                self._last_telemetry_time = current_time

        # ── Overlay text pe frame ─────────────────────────────────────────────
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, f"Timestamp: {timestamp_ms}",
                    (15, 40), font, 1.0, (0, 255, 255), 3)
        cv2.putText(frame, f"Frame: {self.frame_count} | Time: {elapsed:.2f}s",
                    (15, 80), font, 0.8, (0, 255, 255), 2)
        cv2.putText(frame, f"Unghi: {unghi:.2f}deg | Viteza: {viteza:.1f}%",
                    (15, 120), font, 0.8, (0, 255, 255), 2)
        mode_text = "AUTO" if mode == 0 else "MANUAL"
        cv2.putText(frame, f"Mode: {mode_text} | Cmd: {unghi:.0f}deg, {viteza:.0f}%",
                    (15, 160), font, 0.8, (255, 0, 255), 2)
        mqtt_status = "Connected" if (self.mqtt_handler and self.mqtt_handler.connected) else "Disconnected"
        cv2.putText(frame, f"MQTT: {mqtt_status}",
                    (15, self.height - 20), font, 0.8, (0, 255, 255), 2)

        new_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        new_frame.pts       = pts
        new_frame.time_base = time_base
        return new_frame

    def stop(self):
        """Stop camera and processing thread."""
        self.autonomous_driver.cleanup()
        if self.picam2:
            self.picam2.stop()
            logger.info("Pi Camera stopped")


def test_camera():
    """Test Pi Camera capture."""
    print("Testing Pi Camera Module 3...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (1920, 1080)})
    picam2.configure(config)
    picam2.start()
    print("Camera started. Capturing test frame...")
    time.sleep(2)
    frame = picam2.capture_array()
    print(f"Captured frame: {frame.shape}")
    cv2.imwrite("/tmp/camera_test.jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    print("Test image saved to /tmp/camera_test.jpg")
    picam2.stop()
    print("Camera test complete!")


if __name__ == "__main__":
    test_camera()