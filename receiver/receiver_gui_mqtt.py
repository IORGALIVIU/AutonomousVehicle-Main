#!/usr/bin/env python3
"""
Integrated Receiver GUI cu MQTT pentru Windows.
- Primește video WebRTC cu timestamp
- Primește date senzori prin MQTT sincronizate cu frame-urile
- Trimite comenzi prin MQTT (mod, unghi, viteza)
- Rulează MQTT broker local
"""

import asyncio
import argparse
import logging
import sys
import time
import queue
import threading
import json
import socket
from pathlib import Path
from typing import Optional
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from PIL import Image, ImageTk
import cv2
import numpy as np
import collections
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from aiortc import RTCPeerConnection, RTCSessionDescription
from av import VideoFrame
import paho.mqtt.client as mqtt

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Import signaling
sys.path.append(str(Path(__file__).parent.parent))
from common.signaling import SignalingClient

try:
    from lane_debug_viz import DebugLaneDetector
    from config import PROCESS_WIDTH, PROCESS_HEIGHT
    from autonomous_driver import AutonomousDriver
    _LANE_DETECTOR_AVAILABLE = True
except ImportError as _e:
    logger.warning(f"DebugLaneDetector / AutonomousDriver not available: {_e}")
    _LANE_DETECTOR_AVAILABLE = False
    PROCESS_WIDTH, PROCESS_HEIGHT = 640, 480


# ============================================================================
# MQTT Handler for Receiver
# ============================================================================

class MQTTReceiverHandler:
    """Handles MQTT for receiving sensor data and sending commands."""

    def __init__(self, broker_address: str = "localhost", broker_port: int = 1883):
        self.broker_address = broker_address
        self.broker_port = broker_port
        self.client_id = "windows_receiver_gui"

        # Topics
        self.topic_subscribe_senzori = "robot/senzori"
        self.topic_subscribe_sistem = "robot/sistem"
        self.topic_publish_mod = "robot/control/mod"
        self.topic_publish_unghi = "robot/control/unghi_manual"
        self.topic_publish_viteza = "robot/control/viteza_manual"

        # Clock sync topics (ping/pong)
        self.topic_ping = "robot/ping"
        self.topic_pong = "robot/pong"
        self.clock_offset = 0  # offset = T_sender - T_receiver (ms)
        self._ping_thread = None
        self._ping_stop_event = threading.Event()

        # Per-frame video timestamp topic
        self.topic_video_ts = "robot/video_ts"
        self.latest_video_timestamp = 0  # Updated at 30Hz by sender

        # Date realtime (unghi/viteza) — actualizate la fiecare frame
        self.sensor_data_buffer = {}
        self.latest_sensor_data = {
            "unghi": 0.0,
            "viteza": 0.0,
            "timestamp": 0,
        }

        # Date sistem (cpu/ram/temp/baterie) — actualizate la ~5s
        self.latest_sistem_data = {
            "cpu_usage": 0.0,
            "ram_usage": 0.0,
            "temperature": 0.0,
            "battery": 0.0,
            "bat_voltage": 0.0,
            "bat_current": 0.0,
            "charging": False,
        }

        self.sistem_callback = None

        self.client = None
        self.connected = False
        self.message_callback = None

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info(f"MQTT: Connected to broker {self.broker_address}")
            self.connected = True
            client.subscribe(self.topic_subscribe_senzori)
            client.subscribe(self.topic_subscribe_sistem)
            client.subscribe(self.topic_pong)
            client.subscribe(self.topic_video_ts)
            logger.info("MQTT: Subscribed to robot/senzori, robot/sistem, robot/pong, robot/video_ts")
        else:
            logger.error(f"MQTT: Connection failed, code: {rc}")
            self.connected = False

    def on_disconnect(self, client, userdata, rc):
        """Called when MQTT disconnects."""
        self.connected = False
        if rc != 0:
            logger.warning(f"MQTT: Unexpected disconnect! Code: {rc}")
        else:
            logger.info("MQTT: Disconnected")

    def on_message(self, client, userdata, message):
        topic = message.topic
        payload = message.payload.decode()

        try:
            data = json.loads(payload)

            if topic == self.topic_subscribe_senzori:
                # Date realtime: unghi + viteza
                timestamp = data.get("timestamp", 0)
                self.latest_sensor_data = {
                    "unghi": data.get("unghi", 0.0),
                    "viteza": data.get("viteza", 0.0),
                    "timestamp": timestamp,
                }
                # Buffer pentru sincronizare cu frame-ul video
                self.sensor_data_buffer[timestamp] = self.latest_sensor_data.copy()
                if len(self.sensor_data_buffer) > 100:
                    del self.sensor_data_buffer[min(self.sensor_data_buffer.keys())]

                print(f"[SENZORI] unghi={self.latest_sensor_data['unghi']:.1f}"
                      f" viteza={self.latest_sensor_data['viteza']:.1f} ts={timestamp}")

                if self.message_callback:
                    self.message_callback(self.latest_sensor_data)

            elif topic == self.topic_subscribe_sistem:
                # Date sistem lente: CPU, RAM, Temp, Baterie
                self.latest_sistem_data = {
                    "cpu_usage": data.get("cpu_usage", 0.0),
                    "ram_usage": data.get("ram_usage", 0.0),
                    "temperature": data.get("temperature", 0.0),
                    "battery": data.get("battery", 0.0),
                    "bat_voltage": data.get("bat_voltage", 0.0),
                    "bat_current": data.get("bat_current", 0.0),
                    "charging": data.get("charging", False),
                }
                print(f"[SISTEM] CPU={self.latest_sistem_data['cpu_usage']}%"
                      f" RAM={self.latest_sistem_data['ram_usage']}%"
                      f" Temp={self.latest_sistem_data['temperature']}°C"
                      f" Bat={self.latest_sistem_data['battery']}%"
                      f" ({self.latest_sistem_data['bat_voltage']}V"
                      f" {'↑charging' if self.latest_sistem_data['charging'] else '↓discharging'})")

                if self.sistem_callback:
                    self.sistem_callback(self.latest_sistem_data)

            elif topic == self.topic_pong:
                # Clock sync: calculate RTT and offset
                t_receiver_send = data.get("t_receiver_send", 0)
                t_sender = data.get("t_sender", 0)
                t_receiver_receive = int(time.time() * 1000)

                rtt = t_receiver_receive - t_receiver_send
                # offset = cât e ceasul sender-ului în avans față de receiver
                self.clock_offset = t_sender - (t_receiver_send + rtt // 2)
                logger.info(f"[MQTT SYNC] RTT={rtt}ms, Clock Offset={self.clock_offset}ms")

            elif topic == self.topic_video_ts:
                # Per-frame video timestamp de la sender (30Hz)
                self.latest_video_timestamp = data.get("ts", 0)

        except json.JSONDecodeError:
            logger.error(f"MQTT: JSON decode error: {payload}")
        except Exception as e:
            logger.error(f"MQTT: Error processing message: {e}")

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

    def _ping_loop(self):
        """Background thread that sends clock sync pings every 5 seconds."""
        while not self._ping_stop_event.is_set():
            if self.connected and self.client:
                t_receiver_send = int(time.time() * 1000)
                ping_payload = json.dumps({"t_receiver_send": t_receiver_send})
                try:
                    self.client.publish(self.topic_ping, ping_payload)
                except Exception as e:
                    logger.warning(f"MQTT SYNC: Ping send error: {e}")
            self._ping_stop_event.wait(5)

    def start_ping_loop(self):
        """Start the clock sync ping loop in a background thread."""
        if self._ping_thread is not None:
            return
        self._ping_stop_event.clear()
        self._ping_thread = threading.Thread(target=self._ping_loop, daemon=True)
        self._ping_thread.start()
        logger.info("MQTT SYNC: Ping loop started (every 5s)")

    def stop_ping_loop(self):
        """Stop the clock sync ping loop."""
        self._ping_stop_event.set()
        if self._ping_thread:
            self._ping_thread.join(timeout=2)
            self._ping_thread = None

    def get_sensor_data_at_timestamp(self, timestamp: int, tolerance_ms: int = 100):
        """Get sensor data closest to given timestamp."""
        if not self.sensor_data_buffer:
            return self.latest_sensor_data

        # Find closest timestamp
        closest_ts = min(self.sensor_data_buffer.keys(),
                         key=lambda x: abs(x - timestamp))

        if abs(closest_ts - timestamp) <= tolerance_ms:
            return self.sensor_data_buffer[closest_ts]

        return self.latest_sensor_data

    def send_command_mode(self, mode: int):
        """Send mode command (0=auto, 1=manual)."""
        if not self.connected:
            return False

        message = {"mod_de_functionare": mode}
        try:
            result = self.client.publish(self.topic_publish_mod, json.dumps(message))
            return result.rc == mqtt.MQTT_ERR_SUCCESS
        except Exception as e:
            logger.error(f"MQTT: Send mode error: {e}")
            return False

    def send_command_angle(self, angle: float):
        """Send manual angle command."""
        if not self.connected:
            return False

        message = {"unghi_manual": angle}
        try:
            result = self.client.publish(self.topic_publish_unghi, json.dumps(message))
            return result.rc == mqtt.MQTT_ERR_SUCCESS
        except Exception as e:
            logger.error(f"MQTT: Send angle error: {e}")
            return False

    def send_command_speed(self, speed: float):
        """Send manual speed command."""
        if not self.connected:
            return False

        message = {"viteza_manual": speed}
        try:
            result = self.client.publish(self.topic_publish_viteza, json.dumps(message))
            return result.rc == mqtt.MQTT_ERR_SUCCESS
        except Exception as e:
            logger.error(f"MQTT: Send speed error: {e}")
            return False

    def disconnect(self):
        """Disconnect from broker."""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            logger.info("MQTT: Disconnected")


# ============================================================================
# Integrated Video Receiver GUI with MQTT
# ============================================================================

class VideoReceiverGUI_MQTT:
    """
    GUI application for receiving video and synchronized MQTT sensor data.
    Also sends commands to Raspberry Pi.
    """

    def __init__(self, root: tk.Tk, server_url: str, mqtt_broker: str):
        self.root = root
        self.server_url = server_url
        self.mqtt_broker = mqtt_broker
        self.root.title("WebRTC + MQTT Receiver")
        self.root.geometry("1200x800")

        # State
        self.pc: Optional[RTCPeerConnection] = None
        self.is_connected = False
        self.frame_queue = queue.Queue(maxsize=10)
        self.current_frame_timestamp = 0

        # Flag pentru log unic "MQTT not connected"
        self._mqtt_not_connected_logged = False
        # Ultimele valori sistem logate (pentru a evita spam)
        self._last_sistem_log = ""

        # Stats
        self.stats = {
            "frames_received": 0,
            "last_timestamp": 0,
            "fps": 0,
            "resolution": "N/A",
            "connection_state": "Disconnected",
            "mqtt_state": "Disconnected",
            "video_delay_ms": 0,
            "mqtt_delay_ms": 0
        }

        # Lane Detector — init dupa _setup_ui (necesita debug_status_label)
        self._local_lane_detector = None

        # MQTT Handler
        self.mqtt_handler = MQTTReceiverHandler(mqtt_broker)
        self.mqtt_handler.message_callback = self._on_mqtt_message
        self.mqtt_handler.sistem_callback = self._on_sistem_message

        # Gamepad Controller for keyboard input
        from gamepad_controller import GamepadController
        self.gamepad = GamepadController(
            angle_max=50, angle_min=-50,
            speed_max=100, speed_min=-100,  # Allow reverse!
            accel_rate=50.0,  # 50 units/second
            decel_rate=90.0,  # 80 units/second (faster return)
            quick_tap_boost=10.0  # +10 for quick taps
        )

        # Control values
        self.control_mode = tk.IntVar(value=1)  # 0=auto, 1=manual (Default to manual)
        self.control_angle = tk.DoubleVar(value=0.0)
        self.control_speed = tk.DoubleVar(value=0.0)

        # Sensor display values — realtime (robot/senzori)
        self.sensor_angle = tk.StringVar(value="0.0")
        self.sensor_speed = tk.StringVar(value="0.0")
        self.sensor_timestamp = tk.StringVar(value="N/A")

        # Sensor display values — sistem lent (robot/sistem)
        self.sensor_cpu = tk.StringVar(value="-- %")
        self.sensor_ram = tk.StringVar(value="-- %")
        self.sensor_temp = tk.StringVar(value="-- °C")
        self.sensor_battery = tk.StringVar(value="-- %")
        self.sensor_bat_info = tk.StringVar(value="")  # tensiune + stare încărcare

        # Asyncio loop pentru async operations
        self.loop = None
        self.loop_thread = None

        # PID telemetry
        self.pid_data_queue: queue.Queue = queue.Queue(maxsize=2000)
        self._mqtt_pid_client = None
        self._mqtt_pid_connected = False
        self._pid_graph_window = None

        # Debug visualization windows
        self._birds_eye_window = None
        self._sliding_window_window = None
        self._debug_input_queue: queue.Queue = queue.Queue(maxsize=2)
        self._birds_eye_queue: queue.Queue = queue.Queue(maxsize=2)
        self._sliding_queue: queue.Queue = queue.Queue(maxsize=2)
        self._debug_thread_running = False
        self._debug_thread = None

        # Setup GUI
        self._setup_ui()

        # Init local remote autonomous driver (and debug detector)
        self._init_local_lane_detector()
        
        self.remote_driver = None
        if _LANE_DETECTOR_AVAILABLE:
            # Inițializăm cu rezoluția 1280x720 pentru o scalare optimă a textului pe overlay
            self.remote_driver = AutonomousDriver(
                img_width=1280,
                img_height=720,
                enable_hardware=False,
                show_visualization=True,
                draw_text=True,
                mqtt_handler=None
            )

        # Start asyncio loop
        self._start_async_loop()

        # Start MQTT PID subscriber (topic robot/pid_telemetry)
        self._start_mqtt_pid_subscriber()

        # Update loops
        self._update_video_display()
        self._update_stats_display()

        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

    def _init_local_lane_detector(self):
        if _LANE_DETECTOR_AVAILABLE:
            self._local_lane_detector = DebugLaneDetector(
                img_width=PROCESS_WIDTH,
                img_height=PROCESS_HEIGHT,
            )

    def _setup_ui(self):
        """Setup GUI layout."""

        # ===== Top Control Panel =====
        control_frame = ttk.Frame(self.root)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)

        # Connection buttons
        self.connect_btn = ttk.Button(
            control_frame,
            text="Connect WebRTC",
            command=self._on_connect_webrtc_clicked
        )
        self.connect_btn.grid(row=0, column=0, padx=5)

        self.disconnect_btn = ttk.Button(
            control_frame,
            text="Disconnect",
            command=self._on_disconnect_clicked,
            state=tk.DISABLED
        )
        self.disconnect_btn.grid(row=0, column=1, padx=5)

        # MQTT connect
        self.mqtt_connect_btn = ttk.Button(
            control_frame,
            text="Connect MQTT",
            command=self._on_connect_mqtt_clicked
        )
        self.mqtt_connect_btn.grid(row=0, column=2, padx=5)

        # Status labels
        self.webrtc_status = ttk.Label(
            control_frame,
            text="WebRTC: Disconnected",
            foreground="red"
        )
        self.webrtc_status.grid(row=0, column=3, padx=20)

        self.mqtt_status = ttk.Label(
            control_frame,
            text="MQTT: Disconnected",
            foreground="red"
        )
        self.mqtt_status.grid(row=0, column=4, padx=20)

        # ===== Main Content =====
        content_frame = ttk.Frame(self.root)
        content_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=5)

        # Ordinea de pack in tkinter conteaza:
        # Coloanele fixe se pun cu side=RIGHT intai, altfel left_frame
        # cu expand=True le inghite spatiul.

        # Right: Control Panel + Stats + Sensor Data  (pus primul — rightmost)
        right_frame = ttk.Frame(content_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5)

        # Middle: Debug Tools  (pus al doilea cu RIGHT — apare la stanga coloanei drepte)
        middle_frame = ttk.Frame(content_frame)
        middle_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5)

        # Left: Video (pus ultimul cu LEFT + expand=True — umple restul)
        left_frame = ttk.Frame(content_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        # Video display
        video_frame = ttk.LabelFrame(left_frame, text="Video Stream")
        video_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=5)

        self.video_label = ttk.Label(video_frame, text="No video stream")
        self.video_label.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)


        # --- Debug Tools ---
        debug_panel = ttk.LabelFrame(middle_frame, text="Debug Tools")
        debug_panel.pack(side=tk.TOP, fill=tk.X, pady=(5, 3))

        self.birds_eye_btn = ttk.Button(
            debug_panel,
            text="Bird's Eye (binary)",
            command=self._open_birds_eye_window,
        )
        self.birds_eye_btn.pack(fill=tk.X, padx=8, pady=(8, 3))

        self.sliding_win_btn = ttk.Button(
            debug_panel,
            text="Sliding Window",
            command=self._open_sliding_window_window,
        )
        self.sliding_win_btn.pack(fill=tk.X, padx=8, pady=(0, 3))

        self.debug_status_label = ttk.Label(
            debug_panel, text="Detector: --", foreground="gray", font=("Arial", 8)
        )
        self.debug_status_label.pack(anchor=tk.W, padx=8, pady=(0, 6))

        # --- PID Telemetry (mutat din panoul drept) ---
        pid_panel = ttk.LabelFrame(middle_frame, text="PID Telemetry")
        pid_panel.pack(side=tk.TOP, fill=tk.X, pady=(3, 5))

        self.pid_graph_btn = ttk.Button(
            pid_panel,
            text="Open PID Graph",
            command=self._open_pid_graph,
        )
        self.pid_graph_btn.pack(fill=tk.X, padx=8, pady=(8, 3))

        self.pid_mqtt_status_label = ttk.Label(
            pid_panel, text="MQTT PID: --", foreground="gray", font=("Arial", 8)
        )
        self.pid_mqtt_status_label.pack(anchor=tk.W, padx=8, pady=(0, 6))

        # Control panel
        control_panel = ttk.LabelFrame(right_frame, text="Control Commands")
        control_panel.pack(side=tk.TOP, fill=tk.X, pady=5)

        # Mode control
        ttk.Label(control_panel, text="Mode:").grid(row=0, column=0, sticky=tk.W, padx=10, pady=5)
        ttk.Radiobutton(control_panel, text="Auto", variable=self.control_mode, value=0,
                        command=self._on_mode_changed).grid(row=0, column=1, sticky=tk.W, padx=5)
        ttk.Radiobutton(control_panel, text="Manual", variable=self.control_mode, value=1,
                        command=self._on_mode_changed).grid(row=0, column=2, sticky=tk.W, padx=5)
        ttk.Radiobutton(control_panel, text="Auto Remote", variable=self.control_mode, value=2,
                        command=self._on_mode_changed).grid(row=0, column=3, sticky=tk.W, padx=5)

        # Keyboard control instructions (Manual mode only)
        self.keyboard_label = ttk.Label(control_panel, text="Use Arrow Keys to Control",
                                        foreground="blue", font=("Arial", 9, "bold"))
        self.keyboard_label.grid(row=1, column=0, columnspan=4, pady=(10, 5))
        # visible by default since we start in manual mode

        # Arrow keys help
        self.arrows_help = ttk.Label(control_panel,
                                     text="↑↓ Speed  |  ←→ Angle",
                                     foreground="gray", font=("Arial", 8))
        self.arrows_help.grid(row=2, column=0, columnspan=4, pady=(0, 10))
        # visible by default since we start in manual mode

        # Angle display (read-only, no slider)
        ttk.Label(control_panel, text="Manual Angle:").grid(row=3, column=0, sticky=tk.W, padx=10, pady=5)
        self.angle_value_label = ttk.Label(control_panel, textvariable=self.control_angle,
                                           font=("Arial", 14, "bold"), foreground="blue")
        self.angle_value_label.grid(row=3, column=1, sticky=tk.W, padx=5)
        ttk.Label(control_panel, text="°").grid(row=3, column=2, sticky=tk.W)

        # Speed display (read-only, no slider)
        ttk.Label(control_panel, text="Manual Speed:").grid(row=4, column=0, sticky=tk.W, padx=10, pady=5)
        self.speed_value_label = ttk.Label(control_panel, textvariable=self.control_speed,
                                           font=("Arial", 14, "bold"), foreground="blue")
        self.speed_value_label.grid(row=4, column=1, sticky=tk.W, padx=5)
        ttk.Label(control_panel, text="RPM").grid(row=4, column=2, sticky=tk.W)

        # Send button (removed - auto-send with keyboard)
        # Last command sent display
        ttk.Label(control_panel, text="Last Sent:").grid(row=5, column=0, sticky=tk.W, padx=10, pady=5)
        self.last_command_label = ttk.Label(control_panel, text="None", foreground="green", wraplength=200)
        self.last_command_label.grid(row=5, column=1, columnspan=2, sticky=tk.W, padx=5)

        # Stats panel
        stats_frame = ttk.LabelFrame(right_frame, text="Statistics")
        stats_frame.pack(side=tk.TOP, fill=tk.X, pady=5)

        self.stats_labels = {}
        stats_items = [
            ("WebRTC State", "connection_state"),
            ("MQTT State", "mqtt_state"),
            ("Frames Received", "frames_received"),
            ("Current FPS", "fps"),
            ("Resolution", "resolution"),
            ("Video Delay (ms)", "video_delay_ms"),
            ("MQTT Delay (ms)", "mqtt_delay_ms"),
        ]

        for i, (label_text, key) in enumerate(stats_items):
            label = ttk.Label(stats_frame, text=f"{label_text}:")
            label.grid(row=i, column=0, sticky=tk.W, padx=10, pady=5)

            value_label = ttk.Label(stats_frame, text="N/A", foreground="blue",
                                    font=("Arial", 10))
            value_label.grid(row=i, column=1, sticky=tk.W, padx=10, pady=5)

            self.stats_labels[key] = value_label

        # Sensor data display - MOVED HERE under statistics
        sensor_frame = ttk.LabelFrame(right_frame, text="Sensor Data (Synced)")
        sensor_frame.pack(side=tk.TOP, fill=tk.X, pady=5)

        sensor_grid = ttk.Frame(sensor_frame)
        sensor_grid.pack(padx=10, pady=10)

        # --- Realtime ---
        ttk.Label(sensor_grid, text="Angle:").grid(row=0, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor_grid, textvariable=self.sensor_angle, foreground="blue", font=("Arial", 10)).grid(
            row=0, column=1, sticky=tk.W, padx=5)
        ttk.Label(sensor_grid, text="°").grid(row=0, column=2, sticky=tk.W)

        ttk.Label(sensor_grid, text="Speed:").grid(row=1, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor_grid, textvariable=self.sensor_speed, foreground="blue", font=("Arial", 10)).grid(
            row=1, column=1, sticky=tk.W, padx=5)
        ttk.Label(sensor_grid, text="RPM").grid(row=1, column=2, sticky=tk.W)

        ttk.Label(sensor_grid, text="Timestamp:").grid(row=2, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor_grid, textvariable=self.sensor_timestamp, foreground="blue", font=("Arial", 10)).grid(row=2,
                                                                                                               column=1,
                                                                                                               columnspan=2,
                                                                                                               sticky=tk.W,
                                                                                                               padx=5)

        # --- Separator ---
        ttk.Separator(sensor_grid, orient="horizontal").grid(row=3, column=0, columnspan=3, sticky="ew", pady=(6, 4))
        ttk.Label(sensor_grid, text="Sistem Data (actualizare ~2s)", foreground="black",
                  font=("Arial", 8, "italic")).grid(row=4, column=0, columnspan=3, sticky=tk.W, padx=5)

        # --- Sistem lent ---
        ttk.Label(sensor_grid, text="CPU:").grid(row=5, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor_grid, textvariable=self.sensor_cpu, foreground="blue", font=("Arial", 10)).grid(row=5,
                                                                                                                 column=1,
                                                                                                                 sticky=tk.W,
                                                                                                                 padx=5)

        ttk.Label(sensor_grid, text="RAM:").grid(row=6, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor_grid, textvariable=self.sensor_ram, foreground="blue", font=("Arial", 10)).grid(row=6,
                                                                                                                 column=1,
                                                                                                                 sticky=tk.W,
                                                                                                                 padx=5)

        ttk.Label(sensor_grid, text="Temp:").grid(row=7, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor_grid, textvariable=self.sensor_temp, foreground="blue", font=("Arial", 10)).grid(row=7,
                                                                                                                  column=1,
                                                                                                                  sticky=tk.W,
                                                                                                                  padx=5)

        ttk.Label(sensor_grid, text="Battery:").grid(row=8, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor_grid, textvariable=self.sensor_battery, foreground="blue", font=("Arial", 10)).grid(
            row=8, column=1, sticky=tk.W, padx=5)
        ttk.Label(sensor_grid, textvariable=self.sensor_bat_info, foreground="gray",
                  font=("Arial", 9)).grid(row=9, column=0, columnspan=3, sticky=tk.W, padx=5, pady=(0, 4))

        # PID Graph button a fost mutat in coloana middle (debug_panel)

        # Log area - bigger now with more space available
        log_frame = ttk.LabelFrame(self.root, text="Logs")
        log_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=15, state=tk.DISABLED, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Setup keyboard bindings
        self._setup_keyboard()

        # Start gamepad update loop
        self._start_gamepad_update()

    def _setup_keyboard(self):
        """Setup keyboard event handlers."""
        # Bind arrow keys
        self.root.bind('<KeyPress-Left>', lambda e: self._on_key_press('left'))
        self.root.bind('<KeyPress-Right>', lambda e: self._on_key_press('right'))
        self.root.bind('<KeyPress-Up>', lambda e: self._on_key_press('up'))
        self.root.bind('<KeyPress-Down>', lambda e: self._on_key_press('down'))

        self.root.bind('<KeyRelease-Left>', lambda e: self._on_key_release('left'))
        self.root.bind('<KeyRelease-Right>', lambda e: self._on_key_release('right'))
        self.root.bind('<KeyRelease-Up>', lambda e: self._on_key_release('up'))
        self.root.bind('<KeyRelease-Down>', lambda e: self._on_key_release('down'))

    def _on_key_press(self, key):
        """Handle key press - only in manual mode."""
        if self.control_mode.get() == 1:  # Manual mode
            self.gamepad.key_press(key)

    def _on_key_release(self, key):
        """Handle key release."""
        if self.control_mode.get() == 1:  # Manual mode
            self.gamepad.key_release(key)

    def _start_gamepad_update(self):
        """Start gamepad controller and update loop."""
        self.gamepad.start()
        self._update_from_gamepad()

    def _update_from_gamepad(self):
        """Update control values from gamepad at 60 FPS."""
        if self.control_mode.get() == 1:  # Manual mode
            # Get current values from gamepad
            angle, speed = self.gamepad.get_values()

            # Update GUI
            new_angle = round(angle, 1)
            new_speed = round(speed, 1)
            
            self.control_angle.set(new_angle)
            self.control_speed.set(new_speed)

            # Auto-send to MQTT (throttle la max 20Hz si do-not-repeat)
            if not hasattr(self, '_last_manual_send_time'):
                self._last_manual_send_time = 0
                self._last_sent_angle = None
                self._last_sent_speed = None
                
            now = time.time()
            # Trimitem daca a trecut suficient timp (50ms) SI (valorile s-au schimbat)
            if now - self._last_manual_send_time >= 0.05:
                if new_angle != self._last_sent_angle or new_speed != self._last_sent_speed:
                    self._send_all_commands()
                    self._last_manual_send_time = now
                    self._last_sent_angle = new_angle
                    self._last_sent_speed = new_speed

        # Schedule next update (~60 FPS)
        self.root.after(16, self._update_from_gamepad)

    def _log(self, message: str):
        """Add message to log."""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        # Only auto-scroll if user is at the bottom
        if self.log_text.yview()[1] >= 0.95:
            self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        logger.info(message)

    def _on_mqtt_message(self, sensor_data):
        """Callback pentru robot/senzori — date realtime (unghi, viteza)."""
        current_time_ms = int(time.time() * 1000)
        data_timestamp = sensor_data.get('timestamp', current_time_ms)

        # Corectam timestamp-ul sender-ului cu offset-ul de ceas calculat prin ping/pong
        clock_offset = self.mqtt_handler.clock_offset
        corrected_sender_time = data_timestamp - clock_offset
        data_age_ms = max(0, current_time_ms - corrected_sender_time)

        self.stats["mqtt_delay_ms"] = data_age_ms

        def update_gui():
            self.sensor_angle.set(f"{sensor_data['unghi']:.1f}")
            self.sensor_speed.set(f"{sensor_data['viteza']:.1f}")
            self.sensor_timestamp.set(f"{data_timestamp}")

        self.root.after(0, update_gui)

    def _on_sistem_message(self, sistem_data):
        """Callback pentru robot/sistem — date lente (CPU, RAM, Temp, Baterie)."""
        charging = sistem_data.get('charging', False)
        bat_v = sistem_data.get('bat_voltage', 0.0)
        bat_i = sistem_data.get('bat_current', 0.0)
        bat_pct = sistem_data.get('battery', 0.0)
        charge_icon = "+" if charging else "-"

        # Logam doar daca valorile s-au modificat semnificativ
        sistem_summary = (f"CPU={sistem_data.get('cpu_usage', 0):.0f}%"
                          f" RAM={sistem_data.get('ram_usage', 0):.0f}%"
                          f" Temp={sistem_data.get('temperature', 0):.0f}C"
                          f" Bat={bat_pct:.0f}% {charge_icon}")
        if sistem_summary != self._last_sistem_log:
            self._last_sistem_log = sistem_summary
            self._log(f"Sistem: CPU={sistem_data.get('cpu_usage', 0):.1f}%"
                      f" RAM={sistem_data.get('ram_usage', 0):.1f}%"
                      f" Temp={sistem_data.get('temperature', 0):.1f}C"
                      f" Bat={bat_pct:.1f}% ({bat_v:.2f}V, {bat_i:.0f}mA)")

        def update_gui():
            self.sensor_cpu.set(f"{sistem_data.get('cpu_usage', 0):.1f} %")
            self.sensor_ram.set(f"{sistem_data.get('ram_usage', 0):.1f} %")
            self.sensor_temp.set(f"{sistem_data.get('temperature', 0):.1f} °C")
            self.sensor_battery.set(f"{bat_pct:.1f} %")
            charge_str = f"{bat_v:.2f}V  {'⚡încărcare' if charging else '🔋descărcare'}  {bat_i:.0f}mA"
            self.sensor_bat_info.set(charge_str)

        self.root.after(0, update_gui)

    def _on_mode_changed(self):
        """Mode radio button changed."""
        mode = self.control_mode.get()

        if mode == 1:  # Manual mode
            # Show keyboard instructions
            self.keyboard_label.grid()
            self.arrows_help.grid()
            self.gamepad.reset()
            self.control_angle.set(0.0)
            self.control_speed.set(0.0)
            self._log("Manual mode - Use arrow keys to control")
        elif mode == 2:  # Auto Remote mode
            # Hide keyboard instructions
            self.keyboard_label.grid_remove()
            self.arrows_help.grid_remove()
            # Reset gamepad to zero
            self.gamepad.reset()
            self.control_angle.set(0.0)
            self.control_speed.set(0.0)
            self._log("Auto Remote mode activated - Processing frames locally")
        else:  # Auto mode (0)
            # Hide keyboard instructions
            self.keyboard_label.grid_remove()
            self.arrows_help.grid_remove()
            # Reset gamepad to zero
            self.gamepad.reset()
            self.control_angle.set(0.0)
            self.control_speed.set(0.0)
            self._log("Auto mode activated - Raspberry Pi is in control")

        # Send mode change
        self._send_all_commands()

    # Removed _on_angle_changed and _on_speed_changed (no more sliders!)

    def _send_all_commands(self):
        """Send all control commands."""
        if not self.mqtt_handler.connected:
            if not self._mqtt_not_connected_logged:
                self._log("MQTT not connected!")
                self.last_command_label.config(text="ERROR: MQTT not connected", foreground="red")
                self._mqtt_not_connected_logged = True
            return

        # Reset flag-ul cand suntem reconectati
        self._mqtt_not_connected_logged = False

        mode = self.control_mode.get()
        angle = self.control_angle.get()
        speed = self.control_speed.get()

        self.mqtt_handler.send_command_mode(mode)
        self.mqtt_handler.send_command_angle(angle)
        self.mqtt_handler.send_command_speed(speed)

        mode_text = "Manual" if mode == 1 else ("Auto Remote" if mode == 2 else "Auto")
        command_str = f"{mode_text}, {angle:.0f}°, {speed:.0f} RPM"
        
        # Evitam update-ul de UI (care este lent) daca textul nu s-a schimbat
        if not hasattr(self, '_last_cmd_str_gui') or self._last_cmd_str_gui != command_str:
            self.last_command_label.config(text=command_str, foreground="green")
            self._last_cmd_str_gui = command_str

    def _start_async_loop(self):
        """Start asyncio loop in separate thread."""

        def run_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_forever()

        self.loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.loop_thread.start()

        while self.loop is None:
            time.sleep(0.01)

    def _run_async(self, coro):
        """Run coroutine in asyncio loop."""
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def _on_connect_mqtt_clicked(self):
        """Connect to MQTT broker."""
        self.mqtt_connect_btn.config(state=tk.DISABLED)
        self._log("Connecting to MQTT broker...")

        def connect_thread():
            if self.mqtt_handler.connect():
                self.stats["mqtt_state"] = "Connected"
                self._log("MQTT connected successfully")
                self.root.after(0, lambda: self.mqtt_status.config(
                    text="MQTT: Connected", foreground="green"))
                # Start clock sync ping loop
                self.mqtt_handler.start_ping_loop()
                # Send the initial manual mode command to sync with robot
                self.root.after(0, self._send_all_commands)
            else:
                self._log("MQTT connection failed")
                self.root.after(0, lambda: self.mqtt_connect_btn.config(state=tk.NORMAL))

        threading.Thread(target=connect_thread, daemon=True).start()

    def _on_connect_webrtc_clicked(self):
        """Connect to WebRTC."""
        self.connect_btn.config(state=tk.DISABLED)
        self._log("Initiating WebRTC connection...")
        self._run_async(self._connect())

    def _on_disconnect_clicked(self):
        """Disconnect both WebRTC and MQTT."""
        self._log("Disconnecting...")
        self._run_async(self._disconnect())

    async def _connect(self):
        """Connect to sender via WebRTC."""
        try:
            self._log(f"Connecting to signaling server: {self.server_url}")

            self.pc = RTCPeerConnection()

            @self.pc.on("track")
            def on_track(track):
                self._log(f"Track received: {track.kind}")

                if track.kind == "video":
                    self._run_async(self._process_video_track(track))

            @self.pc.on("connectionstatechange")
            async def on_connectionstatechange():
                state = self.pc.connectionState
                self.stats["connection_state"] = state
                self._log(f"WebRTC state: {state}")

                if state == "connected":
                    self.is_connected = True
                    self.root.after(0, self._update_connection_ui, True)
                elif state in ["failed", "closed"]:
                    self.is_connected = False
                    self.root.after(0, self._update_connection_ui, False)

            async with SignalingClient(self.server_url) as signaling:
                if not await signaling.check_health():
                    self._log("ERROR: Signaling server not responding")
                    self.root.after(0, self._update_connection_ui, False)
                    return

                self._log("Waiting for offer...")
                offer_data = await signaling.get_offer(timeout=60)

                if not offer_data:
                    self._log("ERROR: Did not receive offer")
                    self.root.after(0, self._update_connection_ui, False)
                    return

                self._log("Creating answer...")
                offer = RTCSessionDescription(
                    sdp=offer_data["sdp"],
                    type=offer_data["type"]
                )
                await self.pc.setRemoteDescription(offer)

                answer = await self.pc.createAnswer()
                await self.pc.setLocalDescription(answer)

                self._log("Sending answer...")
                if not await signaling.send_answer(
                        self.pc.localDescription.sdp,
                        self.pc.localDescription.type
                ):
                    self._log("ERROR: Failed to send answer")
                    self.root.after(0, self._update_connection_ui, False)
                    return

                self._log("WebRTC connected successfully!")

        except Exception as e:
            self._log(f"ERROR: {e}")
            logger.exception("Connection error")
            self.root.after(0, self._update_connection_ui, False)

    async def _disconnect(self):
        """Disconnect all."""
        if self.pc:
            await self.pc.close()
            self.pc = None

        if self.mqtt_handler:
            self.mqtt_handler.stop_ping_loop()
            self.mqtt_handler.disconnect()

        self.is_connected = False
        self._log("Disconnected")
        self.root.after(0, self._update_connection_ui, False)
        self.root.after(0, lambda: self.mqtt_status.config(
            text="MQTT: Disconnected", foreground="red"))

    async def _process_video_track(self, track):
        """Process video track."""
        self._log("Video track processing started")
        frame_times = []

        try:
            while True:
                frame = await track.recv()

                img = frame.to_ndarray(format="bgr24")

                # Update stats
                self.stats["frames_received"] += 1
                self.stats["resolution"] = f"{img.shape[1]}x{img.shape[0]}"

                # Calculate FPS
                current_time = time.time()
                frame_times.append(current_time)
                frame_times = [t for t in frame_times if current_time - t < 1.0]
                self.stats["fps"] = len(frame_times)

                # Store with timestamp (try to extract from frame or use current)
                self.current_frame_timestamp = int(current_time * 1000)
                
                # Folosim timestamp-ul video per-frame (30Hz) în loc de cel de senzori (10Hz)
                sender_timestamp = self.mqtt_handler.latest_video_timestamp
                if sender_timestamp == 0:
                    # Fallback la timestamp-ul de senzori dacă video_ts nu e disponibil
                    sender_timestamp = self.mqtt_handler.latest_sensor_data.get("timestamp", 0)

                # Put frame in queue
                try:
                    self.frame_queue.put_nowait((img, sender_timestamp))
                except queue.Full:
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put_nowait((img, sender_timestamp))
                    except:
                        pass

        except Exception as e:
            self._log(f"ERROR in video processing: {e}")
            logger.exception("Video processing error")

    def _update_video_display(self):
        """Update video display with latest sensor data."""
        try:
            frame_data = None
            # Drenăm coada pentru a prelua doar cel mai recent cadru disponibil
            while True:
                try:
                    frame_data = self.frame_queue.get_nowait()
                except queue.Empty:
                    break

            if frame_data is None:
                # Nu există cadru nou în coadă, replanificăm verificarea
                self.root.after(33, self._update_video_display)
                return

            if isinstance(frame_data, tuple) and len(frame_data) == 2:
                frame, sender_timestamp = frame_data
            else:
                frame = frame_data
                sender_timestamp = 0

            # Calculam Video Delay (corectat cu clock offset)
            current_time_ms = int(time.time() * 1000)
            if sender_timestamp > 0:
                clock_offset = self.mqtt_handler.clock_offset
                corrected_sender_time = sender_timestamp - clock_offset
                video_delay_ms = max(0, current_time_ms - corrected_sender_time)
                self.stats["video_delay_ms"] = video_delay_ms
            else:
                self.stats["video_delay_ms"] = "N/A"

            # Use latest sensor data from MQTT (already updated by callback)
            # No need for complex timestamp synchronization - MQTT callback
            # updates sensor_angle/speed/timestamp in real-time

            # Feed frame to local lane detection debug processor
            self._feed_debug_frame(frame)
            
            # --- AUTO REMOTE PROCESSING ---
            if self.control_mode.get() == 2 and self.remote_driver:
                result = self.remote_driver.process_frame_async(frame)
                
                # Preluăm valorile calculate
                steer = result.get('steering_angle', 0.0)
                spd = result.get('speed', 0.0)
                
                steer_rounded = round(steer, 1)
                spd_rounded = round(spd, 1)
                
                # Actualizăm UI-ul direct
                self.control_angle.set(steer_rounded)
                self.control_speed.set(spd_rounded)
                
                # Trimitem comenzile către MQTT cu Throttle (la aprox 10Hz) pentru a evita spam-ul si lag-ul
                if not hasattr(self, '_last_remote_send_time'):
                    self._last_remote_send_time = 0
                now = time.time()
                if now - self._last_remote_send_time >= 0.1:  # max 10 Hz
                    self._send_all_commands()
                    self._last_remote_send_time = now

                # Inserăm datele PID în coada locală pentru graficul PID
                # Verificam ca rezultatul sa fie NOU (evitam duplicate la 30fps)
                result_time = result.get('processing_time', 0)
                if not hasattr(self, '_last_pid_result_time') or self._last_pid_result_time != result_time:
                    self._last_pid_result_time = result_time
                    offset = result.get('offset')
                    if offset is not None:
                        _norm = max(-1.0, min(1.0, offset / (PROCESS_WIDTH / 2.0)))
                        pid_data = {
                            "response": _norm,
                            "steering_angle": steer,
                            "timestamp": int(time.time() * 1000)
                        }
                        try:
                            self.pid_data_queue.put_nowait(pid_data)
                        except queue.Full:
                            try:
                                self.pid_data_queue.get_nowait()
                                self.pid_data_queue.put_nowait(pid_data)
                            except Exception:
                                pass

                # Folosim frame-ul adnotat (cu linii desenate) pentru vizualizare
                annotated = result.get('annotated_frame')
                if annotated is not None:
                    frame = annotated

            # Resize and display
            display_height = 500
            aspect_ratio = frame.shape[1] / frame.shape[0]
            display_width = int(display_height * aspect_ratio)

            frame_resized = cv2.resize(frame, (display_width, display_height))
            frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
            img_pil = Image.fromarray(frame_rgb)
            img_tk = ImageTk.PhotoImage(image=img_pil)

            self.video_label.config(image=img_tk)
            self.video_label.image = img_tk

        except Exception as e:
            import traceback
            err_msg = traceback.format_exc()
            logger.error(f"Error updating video:\n{err_msg}")
            print(f"CRITICAL ERROR in _update_video_display: {e}\n{err_msg}")

        self.root.after(33, self._update_video_display)

    def _update_stats_display(self):
        """Update statistics display with warning indicators for high delays."""
        for key, label in self.stats_labels.items():
            value = self.stats.get(key, "N/A")
            # Color-code delay values
            if key in ("video_delay_ms", "mqtt_delay_ms") and isinstance(value, (int, float)):
                if value > 400:
                    label.config(text=f"!! {value}", foreground="red")
                elif value > 200:
                    label.config(text=f"! {value}", foreground="orange")
                else:
                    label.config(text=str(value), foreground="green")
            else:
                label.config(text=str(value))

        self.root.after(500, self._update_stats_display)

    def _update_connection_ui(self, connected: bool):
        """Update UI based on connection state."""
        if connected:
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.webrtc_status.config(
                text="WebRTC: Connected",
                foreground="green"
            )
        else:
            self.connect_btn.config(state=tk.NORMAL)
            self.disconnect_btn.config(state=tk.DISABLED)
            self.webrtc_status.config(
                text="WebRTC: Disconnected",
                foreground="red"
            )

    def _start_mqtt_pid_subscriber(self):
        """Conecteaza un client MQTT dedicat pentru topicul robot/pid_telemetry."""
        # Throttle log: un mesaj GUI la fiecare 1 secunda
        self._pid_last_log_time = 0.0
        self._pid_packets_total = 0

        try:
            client = mqtt.Client(
                client_id="receiver_gui_pid_mqtt",
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
            )

            def on_connect(c, userdata, flags, rc):
                if rc == 0:
                    self._mqtt_pid_connected = True
                    c.subscribe("robot/pid_telemetry", qos=0)
                    self.root.after(0, self._refresh_pid_mqtt_label)
                    self.root.after(0, lambda: self._log(
                        "[PID] ✅ MQTT PID conectat — subscris pe robot/pid_telemetry"
                    ))
                else:
                    self.root.after(0, lambda: self._log(
                        f"[PID] ❌ MQTT PID connect esuat rc={rc}"
                    ))

            def on_disconnect(c, userdata, rc):
                self._mqtt_pid_connected = False
                self.root.after(0, self._refresh_pid_mqtt_label)
                self.root.after(0, lambda: self._log("[PID] ⚠️  MQTT PID deconectat"))
                if self._pid_graph_window and self._pid_graph_window.alive:
                    self.root.after(0, self._pid_graph_window.freeze)

            def on_message(c, userdata, msg):
                try:
                    data = json.loads(msg.payload.decode())
                    self._pid_packets_total += 1

                    # Pune in coada pentru grafic
                    try:
                        self.pid_data_queue.put_nowait(data)
                    except queue.Full:
                        try:
                            self.pid_data_queue.get_nowait()
                            self.pid_data_queue.put_nowait(data)
                        except Exception:
                            pass

                    # Log in GUI throttled la 1s
                    now = time.time()
                    if now - self._pid_last_log_time >= 1.0:
                        self._pid_last_log_time = now
                        resp  = data.get("response", 0.0)
                        steer = data.get("steering_angle", 0.0)
                        total = self._pid_packets_total
                        self.root.after(0, lambda r=resp, s=steer, t=total: self._log(
                            f"[PID] 📊 response={r:+.3f}  steering={s:+.1f}°  "
                            f"(total pachete: {t})"
                        ))
                except Exception as e:
                    logger.debug(f"[MQTT PID] parse error: {e}")

            client.on_connect = on_connect
            client.on_disconnect = on_disconnect
            client.on_message = on_message

            client.connect_async(self.mqtt_broker, 1883, keepalive=60)
            client.loop_start()
            self._mqtt_pid_client = client
            self._log(f"[PID] 🔌 Se conectează MQTT PID la {self.mqtt_broker}:1883...")
        except Exception as e:
            self._log(f"[PID] ❌ Nu s-a putut porni subscriber MQTT PID: {e}")

    def _refresh_pid_mqtt_label(self):
        """Actualizeaza indicatorul de stare MQTT PID."""
        if self._mqtt_pid_connected:
            self.pid_mqtt_status_label.config(
                text="● MQTT PID: OK", foreground="green"
            )
        else:
            self.pid_mqtt_status_label.config(
                text="● MQTT PID: OFF", foreground="red"
            )

    def _open_pid_graph(self):
        """Deschide fereastra grafic PID (sau o aduce in fata daca exista)."""
        if self._pid_graph_window and self._pid_graph_window.alive:
            self._pid_graph_window.window.lift()
            return
        # Golim coada de date vechi la fiecare deschidere
        while not self.pid_data_queue.empty():
            try:
                self.pid_data_queue.get_nowait()
            except queue.Empty:
                break
        self._pid_graph_window = PIDGraphWindow(
            parent=self.root,
            data_queue=self.pid_data_queue,
            mqtt_connected_fn=lambda: self._mqtt_pid_connected,
        )

    # ─────────────────────────────────────────────────────────────────────────
    # METODE DEBUG VISUALIZATION
    # ─────────────────────────────────────────────────────────────────────────

    def _start_debug_thread(self):
        """Pornește thread-ul de procesare debug dacă nu rulează deja."""
        if self._debug_thread_running:
            return
        if self._local_lane_detector is None:
            self._log("[Debug] ❌ Detectorul local nu este initializat.")
            return
        self._debug_thread_running = True
        self._debug_thread = threading.Thread(
            target=self._debug_processing_loop,
            name="DebugViz-Worker",
            daemon=True
        )
        self._debug_thread.start()
        self._log("[Debug] Thread de procesare vizualizare pornit")

    def _stop_debug_thread(self):
        """Oprește thread-ul de procesare debug."""
        self._debug_thread_running = False

    def _feed_debug_frame(self, frame):
        """Trimite un frame către thread-ul de procesare debug (non-blocant).
        Apelat din _update_video_display() doar dacă o fereastră debug e deschisă."""
        be_open  = self._birds_eye_window and self._birds_eye_window.alive
        sw_open  = self._sliding_window_window and self._sliding_window_window.alive

        if not (be_open or sw_open):
            return  # nicio fereastră deschisă — nu procesam

        try:
            self._debug_input_queue.put_nowait(frame.copy())
        except queue.Full:
            try:
                self._debug_input_queue.get_nowait()
                self._debug_input_queue.put_nowait(frame.copy())
            except Exception:
                pass

    def _debug_processing_loop(self):
        """
        Thread background: preia frame-uri din _debug_input_queue,
        le procesează prin LaneDetector local la max 10fps și pune
        rezultatele în cozile respective.
        """
        import time as _time
        last_proc = 0.0
        INTERVAL = 0.10  # 10 fps max

        while self._debug_thread_running:
            try:
                frame = self._debug_input_queue.get(timeout=0.15)
            except queue.Empty:
                continue

            now = _time.time()
            if now - last_proc < INTERVAL:
                continue
            last_proc = now

            det = self._local_lane_detector
            if det is None:
                continue

            # Resize la rezoluția de procesare (identic cu ce face Pi-ul)
            small = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT))

            be_open = self._birds_eye_window and self._birds_eye_window.alive
            sw_open = self._sliding_window_window and self._sliding_window_window.alive

            if be_open:
                img = det.get_birds_eye_visualization(small)
                if img is not None:
                    try:
                        self._birds_eye_queue.put_nowait(img)
                    except queue.Full:
                        try:
                            self._birds_eye_queue.get_nowait()
                            self._birds_eye_queue.put_nowait(img)
                        except Exception:
                            pass

            if sw_open:
                img = det.get_sliding_window_visualization(small)
                if img is not None:
                    try:
                        self._sliding_queue.put_nowait(img)
                    except queue.Full:
                        try:
                            self._sliding_queue.get_nowait()
                            self._sliding_queue.put_nowait(img)
                        except Exception:
                            pass

    def _open_birds_eye_window(self):
        """Deschide fereastra Bird's Eye (sau o aduce în față)."""
        if self._birds_eye_window and self._birds_eye_window.alive:
            self._birds_eye_window.window.lift()
            return
        if self._local_lane_detector is None:
            self._init_local_lane_detector()
        self._start_debug_thread()
        self._birds_eye_window = DebugVideoWindow(
            parent=self.root,
            title="🗯️ Bird's Eye — Binary Warped",
            data_queue=self._birds_eye_queue,
            color_hint="alb=pixeli bandă  |  negru=fundal",
        )

    def _open_sliding_window_window(self):
        """Deschide fereastra Sliding Window (sau o aduce în față)."""
        if self._sliding_window_window and self._sliding_window_window.alive:
            self._sliding_window_window.window.lift()
            return
        if self._local_lane_detector is None:
            self._init_local_lane_detector()
        self._start_debug_thread()
        self._sliding_window_window = DebugVideoWindow(
            parent=self.root,
            title="🔍 Sliding Window — Lane Search",
            data_queue=self._sliding_queue,
            color_hint="roșu=stânga  |  albastru=dreapta  |  galben/cyan=ferestre  |  verde=polinoame",
        )

    def _on_closing(self):
        """Handle window close."""
        self._stop_debug_thread()
        if self.is_connected or self.mqtt_handler.connected:
            if messagebox.askokcancel("Quit", "Close connections and quit?"):
                self._run_async(self._disconnect())
                if self.loop:
                    self.loop.call_soon_threadsafe(self.loop.stop)
                if self._mqtt_pid_client:
                    self._mqtt_pid_client.loop_stop()
                    self._mqtt_pid_client.disconnect()
                self.root.destroy()
        else:
            if self.loop:
                self.loop.call_soon_threadsafe(self.loop.stop)
            if self._mqtt_pid_client:
                self._mqtt_pid_client.loop_stop()
                self._mqtt_pid_client.disconnect()
            self.root.destroy()

# =============================================================================
# Debug Video Window — Bird's Eye / Sliding Window
# =============================================================================

class DebugVideoWindow:
    """
    Fereastra pop-up care afișează un flux video debug în timp real.
    Primește imagini RGB pre-procesate prin data_queue și le afișează
    continuu (ca un videoclip), la max ~10fps.
    """
    UPDATE_MS = 100  # 10 Hz refresh

    def __init__(self, parent: tk.Tk, title: str,
                 data_queue: queue.Queue, color_hint: str = ""):
        self.data_queue = data_queue
        self.alive = True
        self._photo = None  # referință reținută pentru garbage collector

        self.window = tk.Toplevel(parent)
        self.window.title(title)
        self.window.geometry("700x560")
        self.window.configure(bg="#1a1a2e")
        self.window.protocol("WM_DELETE_WINDOW", self._on_close)

        # Canvas pentru imagine
        self.canvas = tk.Canvas(
            self.window, bg="#0f0f1a", highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=6, pady=(6, 0))

        # Bara de status
        bar = tk.Frame(self.window, bg="#16213e", relief=tk.GROOVE, bd=1)
        bar.pack(fill=tk.X, padx=6, pady=6)

        self.lbl_status = tk.Label(
            bar, text="⏳ Aștept frame-uri...",
            fg="#e0e0e0", bg="#16213e", font=("Consolas", 9)
        )
        self.lbl_status.pack(side=tk.LEFT, padx=(8, 20))

        self.lbl_fps = tk.Label(
            bar, text="FPS: --",
            fg="#00e676", bg="#16213e", font=("Consolas", 9)
        )
        self.lbl_fps.pack(side=tk.LEFT, padx=(0, 20))

        if color_hint:
            tk.Label(
                bar, text=color_hint,
                fg="#90a4ae", bg="#16213e", font=("Consolas", 8)
            ).pack(side=tk.RIGHT, padx=10)

        self._frame_count = 0
        self._fps_time = time.time()
        self._last_fps = 0.0

        self._schedule_update()

    def _schedule_update(self):
        if self.alive:
            try:
                self._update()
            except Exception as e:
                logger.debug(f"Error in DebugVideoWindow update: {e}")
            if self.alive:
                self.window.after(self.UPDATE_MS, self._schedule_update)

    def _update(self):
        """Drenează coada și afișează cel mai recent frame disponibil."""
        if not self.alive:
            return

        latest = None
        while True:
            try:
                latest = self.data_queue.get_nowait()
            except queue.Empty:
                break

        if latest is None:
            return  # niciun frame nou — păstrăm imaginea anterioară

        try:
            # Calcul FPS
            self._frame_count += 1
            now = time.time()
            elapsed = now - self._fps_time
            if elapsed >= 1.0:
                self._last_fps = self._frame_count / elapsed
                self._frame_count = 0
                self._fps_time = now

            # Scalăm imaginea să umple canvas-ul, menținând aspect ratio
            cw = self.canvas.winfo_width()
            ch = self.canvas.winfo_height()
            if cw < 10 or ch < 10:
                cw, ch = 640, 480

            h, w = latest.shape[:2]
            scale = min(cw / w, ch / h)
            nw, nh = int(w * scale), int(h * scale)
            if nw > 0 and nh > 0:
                # Folosim INTER_NEAREST pentru performanță maximă (mult mai rapid decât INTER_LINEAR)
                resized = cv2.resize(latest, (nw, nh), interpolation=cv2.INTER_NEAREST)
                img_pil = Image.fromarray(resized)
                self._photo = ImageTk.PhotoImage(image=img_pil)
                self.canvas.delete("all")
                x_off = (cw - nw) // 2
                y_off = (ch - nh) // 2
                self.canvas.create_image(x_off, y_off, anchor=tk.NW, image=self._photo)

            # Actualizăm bara de status
            self.lbl_status.config(text=f"✅ Live  {w}x{h}")
            self.lbl_fps.config(text=f"FPS: {self._last_fps:.1f}")
        except (tk.TclError, Exception) as e:
            logger.debug(f"DebugVideoWindow redraw error: {e}")

    def _on_close(self):
        self.alive = False
        self.window.destroy()


# =============================================================================
# PID Graph Window
# =============================================================================

class PIDGraphWindow:
    """
    Fereastra pop-up cu grafic in timp real al controlului PID.
    Arata raspunsul sistemului (offset normalizat [-1, 1]) pe 30 secunde derulante.
    Util pentru Ziegler-Nichols: Ku = Kp la oscilatie sustinuta, Tu = perioada.
    """
    WINDOW_SECONDS = 30
    UPDATE_MS      = 333   # 3 Hz redraw — reduce masiv încărcarea CPU/GUI
    MAX_POINTS     = WINDOW_SECONDS * 100  # Suportă până la 100 Hz (e.g. 30Hz sau 60Hz FPS) fără pierderi pe stânga graficului

    def __init__(self, parent: tk.Tk, data_queue: queue.Queue, mqtt_connected_fn):
        self.data_queue        = data_queue
        self.mqtt_connected_fn = mqtt_connected_fn
        self.alive             = True
        self.frozen            = False

        self.times     = collections.deque(maxlen=self.MAX_POINTS)
        self.responses = collections.deque(maxlen=self.MAX_POINTS)
        self.start_ts  = None

        # Fereastra Toplevel
        self.window = tk.Toplevel(parent)
        self.window.title("📈 PID Real-Time Graph — Ziegler-Nichols Analysis")
        self.window.geometry("1050x620")
        self.window.configure(bg="#f0f0f0")
        self.window.protocol("WM_DELETE_WINDOW", self._on_close)

        # Matplotlib Figure — dpi redus pentru a usura CPU-ul
        self.fig = Figure(figsize=(10.0, 4.8), dpi=80, facecolor="#f0f0f0")
        self.ax  = self.fig.add_subplot(111)
        self._setup_axes()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.window)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8, pady=(8, 0))

        # Bara de stare
        bar = tk.Frame(self.window, bg="#e0e0e0", relief=tk.GROOVE, bd=1)
        bar.pack(fill=tk.X, padx=8, pady=6)

        self.lbl_mqtt = tk.Label(
            bar, text="● MQTT: connecting…",
            fg="#e67e00", bg="#e0e0e0", font=("Consolas", 10)
        )
        self.lbl_mqtt.pack(side=tk.LEFT, padx=(6, 20))

        self.lbl_value = tk.Label(
            bar, text="Response: —",
            fg="#1565c0", bg="#e0e0e0", font=("Consolas", 10)
        )
        self.lbl_value.pack(side=tk.LEFT, padx=(0, 20))

        self.lbl_steering = tk.Label(
            bar, text="Steering: —",
            fg="#2e7d32", bg="#e0e0e0", font=("Consolas", 10)
        )
        self.lbl_steering.pack(side=tk.LEFT)

        self.lbl_frozen = tk.Label(
            bar, text="", fg="#c62828", bg="#e0e0e0",
            font=("Consolas", 10, "bold")
        )
        self.lbl_frozen.pack(side=tk.RIGHT, padx=10)

        tk.Label(
            bar,
            text="Ku = Kp la oscilatie sustinuta  |  Tu = perioada oscilatie (axa X)",
            fg="#666666", bg="#e0e0e0", font=("Consolas", 9)
        ).pack(side=tk.RIGHT, padx=20)

        self._schedule_update()

    def _setup_axes(self):
        ax = self.ax
        ax.set_facecolor("#ffffff")
        ax.grid(True, color="#cccccc", linewidth=0.7, alpha=0.9)
        ax.set_axisbelow(True)
        for spine in ax.spines.values():
            spine.set_color("#aaaaaa")
        ax.set_xlim(0, self.WINDOW_SECONDS)
        ax.set_ylim(-1.25, 1.25)
        ax.set_xlabel("Time (s)", color="#333333", fontsize=11)
        ax.set_ylabel("Normalized Lane Offset", color="#333333", fontsize=11)
        ax.set_title(
            "PID Response — Lane Center Tracking  (robot/pid_telemetry @ 20 Hz)",
            color="#111111", fontsize=12, fontweight="bold", pad=10
        )
        ax.tick_params(colors="#444444", labelsize=9)
        # Linie referință centru
        ax.axhline(y=0,    color="#555555", linestyle="--", linewidth=1.6, alpha=0.7, label="Reference = 0 (centru bandă)")
        # Limite ±1
        ax.axhline(y= 1.0, color="#cc0000", linestyle=":",  linewidth=1.0, alpha=0.5)
        ax.axhline(y=-1.0, color="#cc0000", linestyle=":",  linewidth=1.0, alpha=0.5)
        # Zone colorate
        ax.axhspan(-0.3,  0.3,  alpha=0.08, color="#4caf50")   # zona bună — verde
        ax.axhspan( 0.3,  1.25, alpha=0.05, color="#ff6600")   # oscilație — portocaliu
        ax.axhspan(-1.25,-0.3,  alpha=0.05, color="#ff6600")
        ax.set_yticks([-1.0, -0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75, 1.0])
        # Linia de date — albastru ca valorile din Statistics
        self.line_response, = ax.plot([], [], color="#1565c0", linewidth=2.0, label="Response (offset)")
        ax.legend(loc="upper right", facecolor="#f5f5f5", edgecolor="#cccccc",
                  labelcolor="#333333", fontsize=9)
        self.fig.tight_layout(pad=1.8)

    def _schedule_update(self):
        if self.alive:
            self._update_plot()
            self.window.after(self.UPDATE_MS, self._schedule_update)

    def _update_plot(self):
        """Dreneaza coada de date si redeseneaza graficul la max 5 Hz."""
        if self.frozen:
            return

        # ── 1. Dreneaza TOATA coada (collect la 20 Hz, draw la 5 Hz) ──────────
        changed = False
        last_item = None
        while True:
            try:
                item = self.data_queue.get_nowait()
                last_item = item
                changed = True
                ts = item.get("timestamp", 0)
                if self.start_ts is None:
                    self.start_ts = ts
                t = (ts - self.start_ts) / 1000.0
                self.times.append(t)
                self.responses.append(item.get("response", 0.0))
            except queue.Empty:
                break

        # ── 2. Status bar (intotdeauna, chiar daca nu s-a desenat) ───────────
        self._update_status_bar(last_item)

        # ── 3. Redraw canvas doar daca avem date noi ─────────────────────────
        # draw_idle() gestioneaza singur coalescenta — nu e nevoie de flag manual
        if not changed:
            return

        try:
            if self.times:
                t_now = self.times[-1]
                t_min = max(0.0, t_now - self.WINDOW_SECONDS)
                t_max = t_min + self.WINDOW_SECONDS
                self.ax.set_xlim(t_min, t_max)
                self.line_response.set_data(list(self.times), list(self.responses))
            self.canvas.draw_idle()
        except Exception as e:
            logger.error(f"[PIDGraph] Eroare la redesenare: {e}")

    def _update_status_bar(self, last_item):
        connected = self.mqtt_connected_fn()
        self.lbl_mqtt.config(
            text="● MQTT: OK" if connected else "● MQTT: OFF",
            fg="#2e7d32" if connected else "#c62828"
        )
        if last_item is not None:
            resp  = last_item.get("response", 0.0)
            steer = last_item.get("steering_angle", 0.0)
            self.lbl_value.config(text=f"Response: {resp:+.3f}")
            self.lbl_steering.config(text=f"Steering: {steer:+.1f}°")
        self.lbl_frozen.config(
            text="⏸ ÎNGHEȚAT — MQTT deconectat" if self.frozen else ""
        )

    def freeze(self):
        if not self.frozen:
            self.frozen = True
            self.lbl_frozen.config(text="⏸ INGHEȚAT — MQTT deconectat", fg="#f0ad4e")
            logger.info("[PIDGraph] Frozen — MQTT disconnected")

    def _on_close(self):
        self.alive = False
        self.window.destroy()


def get_local_ip():
    """Obtine adresa IP locala a laptopului (interfata principala)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Nu trimite date, doar afla IP-ul rutat catre internet
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def main():
    local_ip = get_local_ip()
    parser = argparse.ArgumentParser(
        description="WebRTC + MQTT Receiver GUI (Windows)"
    )
    parser.add_argument(
        "--server-ip",
        default=local_ip,
        help=f"Signaling server IP (default: %(default)s)"
    )
    parser.add_argument(
        "--server-port",
        type=int,
        default=8080,
        help="Signaling server port"
    )
    parser.add_argument(
        "--mqtt-broker",
        default="localhost",
        help="MQTT broker address"
    )

    args = parser.parse_args()
    server_url = f"http://{args.server_ip}:{args.server_port}"

    root = tk.Tk()
    app = VideoReceiverGUI_MQTT(root, server_url, args.mqtt_broker)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        logger.info("Application stopped by user")


if __name__ == "__main__":
    main()