#!/usr/bin/env python3
"""
Integrated WebRTC Sender + MQTT Client pentru Raspberry Pi.
- Trimite video prin WebRTC cu timestamp
- Publică date senzori prin MQTT (unghi, viteza)
- Primește comenzi prin MQTT (mod, unghi_manual, viteza_manual)
"""

import psutil

import asyncio
import argparse
import logging
import sys
import time
import cv2
import numpy as np
import json
from pathlib import Path
from datetime import datetime
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame
import paho.mqtt.client as mqtt
import threading

# Ensure sender's own directory is first on sys.path so that
# `from config import CFG` resolves to sender/config.py (not the root config.py)
sys.path.insert(0, str(Path(__file__).parent))
# Also add parent for common.signaling
sys.path.insert(1, str(Path(__file__).parent.parent))
from common.signaling import SignalingClient
from config import CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ============================================================================
# Waveshare UPS HAT - INA219 Battery Reader (I2C 0x42)
# ============================================================================

class INA219:
    """
    Driver pentru Waveshare UPS HAT (INA219 la adresa 0x42).
    Citește tensiunea, curentul și calculează procentajul bateriei.
    Formula baterie: 2x 18650 în serie → 6V=0%, 8.4V=100%
    """
    _REG_CONFIG = 0x00
    _REG_SHUNTVOLTAGE = 0x01
    _REG_BUSVOLTAGE = 0x02
    _REG_POWER = 0x03
    _REG_CURRENT = 0x04
    _REG_CALIBRATION = 0x05

    def __init__(self, addr=0x42):
        self.addr = addr
        self._bus = None
        self._available = False
        try:
            import smbus
            self._bus = smbus.SMBus(1)
            # Configurare INA219: 32V range, shunt ±320mV, 12-bit, continuous
            config = 0x3C1F
            self._write_register(self._REG_CONFIG, config)
            # Calibrare pentru 5A max, shunt 0.1 ohm
            self._write_register(self._REG_CALIBRATION, 4096)
            self._available = True
            logger.info(f"INA219 UPS HAT initialized at I2C 0x{addr:02X}")
        except Exception as e:
            logger.warning(f"INA219 not available: {e} (UPS HAT not connected?)")

    def _write_register(self, reg, value):
        self._bus.write_word_data(self.addr, reg,
                                  ((value & 0xFF) << 8) | ((value >> 8) & 0xFF))

    def _read_register(self, reg):
        raw = self._bus.read_word_data(self.addr, reg)
        return ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF)

    def getBusVoltage_V(self):
        raw = self._read_register(self._REG_BUSVOLTAGE)
        return (raw >> 3) * 0.004  # LSB = 4mV

    def getShuntVoltage_mV(self):
        raw = self._read_register(self._REG_SHUNTVOLTAGE)
        if raw > 32767:
            raw -= 65535
        return raw * 0.01

    def getCurrent_mA(self):
        raw = self._read_register(self._REG_CURRENT)
        if raw > 32767:
            raw -= 65535
        return raw * 0.1

    def getPower_W(self):
        raw = self._read_register(self._REG_POWER)
        return raw * 0.002

    def getBatteryPercent(self):
        """Calculează % baterie din tensiunea bus (2x 18650: 6V=0%, 8.4V=100%)."""
        v = self.getBusVoltage_V()
        p = (v - 6.0) / 2.4 * 100.0
        return max(0.0, min(100.0, p))

    def isCharging(self):
        """Curent pozitiv = încărcare, negativ = descărcare."""
        return self.getCurrent_mA() > 0

    @property
    def available(self):
        return self._available


# ============================================================================
# MQTT Configuration and Handlers
# ============================================================================

class MQTTHandler:
    """Handles MQTT communication for sensor data and commands."""

    def __init__(self, broker_address: str, broker_port: int = 1883, robot_controller=None):
        self.broker_address = broker_address
        self.broker_port = broker_port
        self.client_id = "raspberry_pi_webrtc"

        # Robot hardware controller (optional)
        self.robot_controller = robot_controller

        # Topics
        self.topic_publish = "robot/senzori"
        self.topic_publish_sistem = "robot/sistem"
        self.topic_subscribe_mod = "robot/control/mod"
        self.topic_subscribe_unghi = "robot/control/unghi_manual"
        self.topic_subscribe_viteza = "robot/control/viteza_manual"

        # Clock sync topics (ping/pong)
        self.topic_ping = "robot/ping"
        self.topic_pong = "robot/pong"

        # Per-frame video timestamp topic
        self.topic_video_ts = "robot/video_ts"

        # UPS HAT INA219
        self.ina219 = INA219(addr=0x42)

        # Data storage
        self.sensor_data = {
            "unghi": 0.0,
            "viteza": 0.0,
            "timestamp": None
        }

        self.received_commands = {
            "mod_de_functionare": 0,  # 0 = automat, 1 = manual
            "unghi_manual": 0,
            "viteza_manual": 0
        }

        self.client = None
        self.connected = False

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info(f"MQTT: Connected to broker {self.broker_address}")
            self.connected = True
            # Subscribe to command topics
            client.subscribe(self.topic_subscribe_mod)
            client.subscribe(self.topic_subscribe_unghi)
            client.subscribe(self.topic_subscribe_viteza)
            # Subscribe to clock sync ping topic
            client.subscribe(self.topic_ping)
            logger.info("MQTT: Subscribed to command topics + clock sync")
        else:
            logger.error(f"MQTT: Connection failed, code: {rc}")
            self.connected = False

    def on_disconnect(self, client, userdata, rc):
        """Called when MQTT disconnects."""
        self.connected = False
        if rc != 0:
            logger.warning(f"MQTT: Unexpected disconnect! Code: {rc}")
            print(f"[MQTT DEBUG] Connection lost! Trying to reconnect...")
        else:
            logger.info("MQTT: Disconnected")

    def on_message(self, client, userdata, message):
        topic = message.topic
        payload = message.payload.decode()

        try:
            data = json.loads(payload)

            # ── Clock sync: respond to ping immediately ──
            if topic == self.topic_ping:
                t_sender = int(time.time() * 1000)
                pong_payload = json.dumps({
                    "t_receiver_send": data.get("t_receiver_send", 0),
                    "t_sender": t_sender
                })
                client.publish(self.topic_pong, pong_payload)
                return


            if topic == self.topic_subscribe_mod:
                self.received_commands["mod_de_functionare"] = data.get("mod_de_functionare", 0)
                mode = "MANUAL" if self.received_commands["mod_de_functionare"] == 1 else "AUTOMAT"
                logger.info(f"[MQTT] Mode received: {mode}")
                print(f"[MQTT DEBUG] Mode changed to: {mode}")

                # If switching to AUTO mode, stop robot
                if self.robot_controller and self.received_commands["mod_de_functionare"] == 0:
                    self.robot_controller.update(angle=0, speed=0)
                    logger.info("[ROBOT] Switched to AUTO - motors stopped")

            elif topic == self.topic_subscribe_unghi:
                self.received_commands["unghi_manual"] = data.get("unghi_manual", 0)
                logger.info(f"[MQTT] Manual angle: {self.received_commands['unghi_manual']}°")
                print(f"[MQTT DEBUG] Angle command: {self.received_commands['unghi_manual']}°")

                # Apply angle to robot if in MANUAL mode
                if self.robot_controller and self.received_commands["mod_de_functionare"] == 1:
                    angle = self.received_commands["unghi_manual"]
                    speed = self.received_commands["viteza_manual"]
                    print("MQTT MESSAGE RECEIVED:", message.topic, message.payload)
                    self.robot_controller.update(angle=angle, speed=speed)
                    logger.debug(f"[ROBOT] Updated: angle={angle}°, speed={speed}")

            elif topic == self.topic_subscribe_viteza:
                self.received_commands["viteza_manual"] = data.get("viteza_manual", 0)
                logger.info(f"[MQTT] Manual speed: {self.received_commands['viteza_manual']} RPM")
                print(f"[MQTT DEBUG] Speed command: {self.received_commands['viteza_manual']} RPM")

                # Apply speed to robot if in MANUAL mode
                if self.robot_controller and self.received_commands["mod_de_functionare"] == 1:
                    angle = self.received_commands["unghi_manual"]
                    speed = self.received_commands["viteza_manual"]
                    print("MQTT MESSAGE RECEIVED:", message.topic, message.payload)
                    self.robot_controller.update(angle=angle, speed=speed)
                    logger.debug(f"[ROBOT] Updated: angle={angle}°, speed={speed}")

        except json.JSONDecodeError:
            logger.error(f"[MQTT] JSON decode error: {payload}")
        except Exception as e:
            logger.error(f"[MQTT] Error processing command: {e}")

    def connect(self):
        """Connect to MQTT broker."""
        try:
            self.client = mqtt.Client(
                client_id=self.client_id,
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1
            )
            self.client.on_connect = self.on_connect
            self.client.on_disconnect = self.on_disconnect
            self.client.on_message = self.on_message

            logger.info(f"MQTT: Connecting to {self.broker_address}:{self.broker_port}")
            self.client.connect(self.broker_address, self.broker_port, 60)
            self.client.loop_start()

            # Wait for connection
            timeout = 10
            for _ in range(timeout * 2):
                if self.connected:
                    return True
                time.sleep(0.5)

            logger.warning("MQTT: Connection timeout")
            return False

        except Exception as e:
            logger.error(f"MQTT: Connection error: {e}")
            return False

    def publish_sensor_data(self, unghi: float, viteza: float, timestamp_ms: int):
        """Publish only realtime sensor data (unghi, viteza) with timestamp."""
        if not self.connected:
            return False

        self.sensor_data = {
            "unghi": round(unghi, 2),
            "viteza": round(viteza, 2),
            "timestamp": timestamp_ms,
            "timestamp_iso": datetime.fromtimestamp(timestamp_ms / 1000).isoformat()
        }

        try:
            result = self.client.publish(self.topic_publish, json.dumps(self.sensor_data))
            return result.rc == mqtt.MQTT_ERR_SUCCESS
        except Exception as e:
            logger.error(f"MQTT: Publish error: {e}")
            return False

    def publish_pid_telemetry(self, response: float, steering_angle: float, timestamp_ms: int):
        """Publică telemetria PID pe robot/pid_telemetry (pentru graficul din receiver)."""
        if not self.connected:
            return False
        payload = json.dumps({
            "response":       round(response, 4),
            "steering_angle": round(steering_angle, 2),
            "timestamp":      timestamp_ms,
        })
        try:
            self.client.publish("robot/pid_telemetry", payload, qos=0)
            return True
        except Exception as e:
            logger.error(f"MQTT: PID telemetry publish error: {e}")
            return False

    def publish_video_timestamp(self, timestamp_ms: int):
        """Publish per-frame video timestamp (lightweight, at full frame rate)."""
        if not self.connected:
            return False
        try:
            payload = json.dumps({"ts": timestamp_ms})
            self.client.publish(self.topic_video_ts, payload, qos=0)
        except Exception:
            pass

    def publish_sistem_data(self):
        """Publish slow system data: CPU, RAM, Temp, Battery (from INA219)."""
        if not self.connected:
            return False

        try:
            cpu = psutil.cpu_percent(interval=None)
            ram = psutil.virtual_memory().percent
            temp = 0.0
            try:
                temps = psutil.sensors_temperatures()
                if "cpu_thermal" in temps:
                    temp = temps["cpu_thermal"][0].current
                elif "coretemp" in temps:
                    temp = temps["coretemp"][0].current
            except Exception:
                pass

            # Baterie din UPS HAT INA219
            if self.ina219.available:
                battery = round(self.ina219.getBatteryPercent(), 1)
                bat_volt = round(self.ina219.getBusVoltage_V(), 2)
                bat_curr = round(self.ina219.getCurrent_mA(), 1)
                charging = self.ina219.isCharging()
            else:
                battery, bat_volt, bat_curr, charging = 0.0, 0.0, 0.0, False

            payload = {
                "cpu_usage": round(cpu, 1),
                "ram_usage": round(ram, 1),
                "temperature": round(temp, 1),
                "battery": battery,
                "bat_voltage": bat_volt,
                "bat_current": bat_curr,
                "charging": charging,
                "timestamp": int(time.time() * 1000)
            }

            result = self.client.publish(self.topic_publish_sistem, json.dumps(payload))
            logger.debug(f"[SISTEM] CPU={cpu}% RAM={ram}% Temp={temp}°C Bat={battery}%"
                         f" ({bat_volt}V, {bat_curr}mA, charging={charging})")
            return result.rc == mqtt.MQTT_ERR_SUCCESS
        except Exception as e:
            logger.error(f"MQTT: Sistem publish error: {e}")
            return False

    def start_sistem_publisher(self, interval_sec: float = 5.0):
        """Start background thread that publishes system data every interval_sec."""

        def _loop():
            logger.info(f"[SISTEM] Publisher started (every {interval_sec}s)")
            while self.connected or True:  # rulează cât timp handler-ul există
                if self.connected:
                    self.publish_sistem_data()
                time.sleep(interval_sec)

        t = threading.Thread(target=_loop, daemon=True, name="sistem-publisher")
        t.start()
        logger.info("[SISTEM] Background publisher thread started")

    def disconnect(self):
        """Disconnect from broker."""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            logger.info("MQTT: Disconnected")


# ============================================================================
# WebRTC Video Track with MQTT Integration
# ============================================================================

class VideoFileTrackWithMQTT(VideoStreamTrack):
    """
    Video track that reads from file and integrates MQTT sensor data.
    Displays timestamp and sensor data on each frame.
    """

    def __init__(self, video_path: str, fps: int = CAMERA_FPS, mqtt_handler: MQTTHandler = None):
        super().__init__()
        self.video_path = video_path
        self.target_fps = fps
        self.mqtt_handler = mqtt_handler
        self.cap = None
        self.frame_count = 0
        self.start_time = time.time()

        # Simulated sensor data (if no real sensors)
        self.simulated_angle = 0.0
        self.simulated_speed = 50.0

        # Timestamp tracking for custom FPS (aiortc hardcodes 30fps)
        self._custom_start = None
        self._custom_timestamp = None
        self._VIDEO_CLOCK_RATE = 90000

        self._open_video()

    async def next_timestamp(self):
        """Override aiortc's next_timestamp() to use self.target_fps."""
        import asyncio
        import fractions

        if self._custom_timestamp is not None:
            ptime = 1 / self.target_fps
            self._custom_timestamp += int(self._VIDEO_CLOCK_RATE * ptime)
            wait = self._custom_start + (self._custom_timestamp / self._VIDEO_CLOCK_RATE) - time.time()
            if wait > 0:
                await asyncio.sleep(wait)
        else:
            self._custom_start = time.time()
            self._custom_timestamp = 0

        return self._custom_timestamp, fractions.Fraction(1, self._VIDEO_CLOCK_RATE)

    def _open_video(self):
        """Open video file."""
        self.cap = cv2.VideoCapture(self.video_path)

        if not self.cap.isOpened():
            raise ValueError(f"Cannot open video: {self.video_path}")

        self.original_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        logger.info(f"Video loaded: {self.width}x{self.height}, "
                    f"{self.original_fps:.2f} FPS, {self.total_frames} frames")

    def _simulate_sensor_data(self):
        """Simulate sensor data for testing."""
        # Simulate angle oscillation
        self.simulated_angle = 45 * np.sin(time.time() * 0.5)
        # Simulate speed variation
        self.simulated_speed = 50 + 30 * np.sin(time.time() * 0.3)
        return self.simulated_angle, self.simulated_speed

    async def recv(self):
        """Receive next frame with sensor data overlay."""
        pts, time_base = await self.next_timestamp()

        # Read frame
        ret, frame = self.cap.read()

        if not ret:
            logger.info("Video ended, restarting...")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()

            if not ret:
                logger.error("Cannot read frame")
                frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Get current timestamp
        current_time = time.time()
        timestamp_ms = int(current_time * 1000)
        elapsed = current_time - self.start_time

        # Get sensor data (simulated or real)
        if self.mqtt_handler and self.mqtt_handler.connected:
            # Use received commands or simulate
            unghi, viteza = self._simulate_sensor_data()

            # Publish to MQTT
            self.mqtt_handler.publish_sensor_data(unghi, viteza, timestamp_ms)

            # Get commands
            commands = self.mqtt_handler.received_commands
        else:
            unghi, viteza = self._simulate_sensor_data()
            commands = {"mod_de_functionare": 0, "unghi_manual": 0, "viteza_manual": 0}

        # Add overlays to frame with larger, more readable text
        # Timestamp
        cv2.putText(
            frame,
            f"Timestamp: {timestamp_ms}",
            (15, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            3
        )

        # Frame info
        cv2.putText(
            frame,
            f"Frame: {self.frame_count} | Time: {elapsed:.2f}s",
            (15, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        # Sensor data
        cv2.putText(
            frame,
            f"Unghi: {unghi:.1f}deg | Viteza: {viteza:.1f} RPM",
            (15, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 0),
            3
        )

        # Commands received
        mode_text = "MANUAL" if commands["mod_de_functionare"] == 1 else "AUTO"
        cv2.putText(
            frame,
            f"Mode: {mode_text} | Cmd: {commands['unghi_manual']:.0f}deg, {commands['viteza_manual']:.0f}RPM",
            (15, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 0, 255),
            2
        )

        # MQTT status
        mqtt_status = "MQTT: Connected" if (self.mqtt_handler and self.mqtt_handler.connected) else "MQTT: Disconnected"
        cv2.putText(
            frame,
            mqtt_status,
            (15, self.height - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        self.frame_count += 1

        # Convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Create VideoFrame
        video_frame = VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        return video_frame

    def __del__(self):
        if self.cap is not None:
            self.cap.release()


# ============================================================================
# Main WebRTC + MQTT Sender
# ============================================================================

async def run_sender(video_source: str, server_url: str, fps: int,
                     mqtt_broker: str = None, mqtt_port: int = 1883, use_camera: bool = False,
                     camera_width: int = CAMERA_WIDTH, camera_height: int = CAMERA_HEIGHT):
    """
    Run integrated WebRTC sender with MQTT support.

    Args:
        video_source: Path to video file OR 'camera' for Pi Camera
        use_camera: If True, use Pi Camera instead of video file
        camera_width: Camera resolution width
        camera_height: Camera resolution height
    """
    logger.info(f"Starting integrated sender")

    if use_camera:
        logger.info(f"Camera: {camera_width}x{camera_height}, FPS: {fps}")
    else:
        logger.info(f"Video: {video_source}, FPS: {fps}")

    logger.info(f"WebRTC Signaling: {server_url}")
    if mqtt_broker:
        logger.info(f"MQTT Broker: {mqtt_broker}:{mqtt_port}")

    # Check video source
    if not use_camera:
        if not Path(video_source).exists():
            logger.error(f"Video file not found: {video_source}")
            return
    else:
        logger.info("Using Pi Camera Module")

    # Initialize Robot Controller (optional)
    robot_controller = None
    try:
        from robot_controller import RobotCarController
        robot_controller = RobotCarController()
        logger.info("✅ Robot hardware controller initialized")
    except ImportError:
        logger.warning("⚠️  robot_controller.py not found - running without hardware control")
    except Exception as e:
        logger.warning(f"⚠️  Failed to initialize robot hardware: {e}")
        logger.warning("Continuing without hardware control (commands will be logged only)")

    # Setup MQTT
    mqtt_handler = None
    if mqtt_broker:
        mqtt_handler = MQTTHandler(mqtt_broker, mqtt_port, robot_controller=robot_controller)
        if not mqtt_handler.connect():
            logger.warning("MQTT connection failed, continuing without MQTT")
            mqtt_handler = None
        else:
            # Pornește publicarea datelor de sistem la 5 secunde
            mqtt_handler.start_sistem_publisher(interval_sec=2.0)

    # Create peer connection
    pc = RTCPeerConnection()

    # Add video track (camera or file)
    try:
        if use_camera:
            # Import camera track
            from camera_track import PiCameraTrackWithMQTT
            video_track = PiCameraTrackWithMQTT(
                width=camera_width,
                height=camera_height,
                fps=fps,
                mqtt_handler=mqtt_handler
            )
            logger.info("Pi Camera track created")
        else:
            video_track = VideoFileTrackWithMQTT(video_source, fps=fps, mqtt_handler=mqtt_handler)
            logger.info("Video file track created")

        pc.addTrack(video_track)
        logger.info("Video track added")
    except Exception as e:
        logger.error(f"Error creating video track: {e}")
        if mqtt_handler:
            mqtt_handler.disconnect()
        return

    # Event handlers
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        logger.info(f"WebRTC connection state: {pc.connectionState}")
        if pc.connectionState == "failed":
            await pc.close()

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        logger.info(f"ICE connection state: {pc.iceConnectionState}")

    # Connect to signaling server
    async with SignalingClient(server_url) as signaling:
        logger.info("Checking signaling server...")
        if not await signaling.check_health():
            logger.error("Signaling server not responding")
            if mqtt_handler:
                mqtt_handler.disconnect()
            return

        logger.info("Creating WebRTC offer...")
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        logger.info("Sending offer...")
        if not await signaling.send_offer(
                pc.localDescription.sdp,
                pc.localDescription.type
        ):
            logger.error("Failed to send offer")
            if mqtt_handler:
                mqtt_handler.disconnect()
            return

        logger.info("Waiting for answer...")
        answer_data = await signaling.get_answer(timeout=60)

        if not answer_data:
            logger.error("Did not receive answer")
            if mqtt_handler:
                mqtt_handler.disconnect()
            return

        answer = RTCSessionDescription(
            sdp=answer_data["sdp"],
            type=answer_data["type"]
        )
        await pc.setRemoteDescription(answer)

        logger.info("=" * 60)
        logger.info("WebRTC connection established!")
        logger.info(f"Streaming video at {fps} FPS")
        if mqtt_handler and mqtt_handler.connected:
            logger.info("MQTT connected - sensor data synchronized")
        logger.info("=" * 60)

        # Keep running
        try:
            while pc.connectionState != "closed":
                await asyncio.sleep(1)

        except KeyboardInterrupt:
            logger.info("Stopping sender...")

        finally:
            logger.info("Closing connections...")
            await pc.close()
            if mqtt_handler:
                mqtt_handler.disconnect()
            if robot_controller:
                robot_controller.cleanup()
                logger.info("Robot hardware cleaned up")


def main():
    parser = argparse.ArgumentParser(
        description="Integrated WebRTC + MQTT Sender (Raspberry Pi)"
    )
    parser.add_argument(
        "--video",
        default="video.mp4",
        help="Path to video file (ignored if --camera is used)"
    )
    parser.add_argument(
        "--camera",
        action="store_true",
        help="Use Pi Camera Module instead of video file"
    )
    parser.add_argument(
        "--camera-width",
        type=int,
        default=CAMERA_WIDTH,
        help=f"Camera resolution width (default: {CAMERA_WIDTH})"
    )
    parser.add_argument(
        "--camera-height",
        type=int,
        default=CAMERA_HEIGHT,
        help=f"Camera resolution height (default: {CAMERA_HEIGHT})"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=CAMERA_FPS,
        help=f"Target FPS for streaming (default: {CAMERA_FPS})"
    )
    parser.add_argument(
        "--server-ip",
        required=True,
        help="Signaling server IP (Windows laptop IP)"
    )
    parser.add_argument(
        "--server-port",
        type=int,
        default=8080,
        help="Signaling server port"
    )
    parser.add_argument(
        "--mqtt-broker",
        help="MQTT broker IP (usually same as server-ip)"
    )
    parser.add_argument(
        "--mqtt-port",
        type=int,
        default=1883,
        help="MQTT broker port"
    )

    args = parser.parse_args()

    # Run sender
    server_url = f"http://{args.server_ip}:{args.server_port}"
    mqtt_broker = args.mqtt_broker if args.mqtt_broker else args.server_ip

    try:
        asyncio.run(run_sender(
            video_source=args.video,
            server_url=server_url,
            fps=args.fps,
            mqtt_broker=mqtt_broker,
            mqtt_port=args.mqtt_port,
            use_camera=args.camera,
            camera_width=args.camera_width,
            camera_height=args.camera_height
        ))
    except KeyboardInterrupt:
        logger.info("Sender stopped by user")


if __name__ == "__main__":
    main()