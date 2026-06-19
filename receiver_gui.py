#!/usr/bin/env python3
"""
Receiver GUI application pentru Windows.
Afișează video stream-ul în aplicație desktop.
"""

import asyncio
import argparse
import logging
import sys
import time
import queue
import threading
from pathlib import Path
from typing import Optional

import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import cv2
import numpy as np
import collections
import paho.mqtt.client as mqtt
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from aiortc import RTCPeerConnection, RTCSessionDescription
from av import VideoFrame

# Import signaling
sys.path.insert(0, str(Path(__file__).parent.parent))
from common.signaling import SignalingClient

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class VideoReceiverGUI:
    """
    Aplicație GUI pentru primirea și afișarea video stream-ului.
    """
    
    def __init__(self, root: tk.Tk, server_url: str,
                 mqtt_broker: str = "localhost", mqtt_port: int = 1883):
        self.root = root
        self.server_url = server_url
        self.root.title("WebRTC Video Receiver")
        self.root.geometry("1000x700")
        
        # MQTT PID subscriber
        self.mqtt_broker  = mqtt_broker
        self.mqtt_port    = mqtt_port
        self.pid_data_queue: queue.Queue = queue.Queue(maxsize=2000)
        self._mqtt_pid_client  = None
        self._mqtt_pid_connected = False
        self._pid_graph_window = None
        
        # State
        self.pc: Optional[RTCPeerConnection] = None
        self.is_connected = False
        self.frame_queue = queue.Queue(maxsize=10)
        self.stats = {
            "frames_received": 0,
            "last_timestamp": 0,
            "fps": 0,
            "resolution": "N/A",
            "connection_state": "Disconnected"
        }
        
        # Asyncio loop pentru async operations
        self.loop = None
        self.loop_thread = None
        
        # Setup GUI
        self._setup_ui()
        
        # Start asyncio loop in separate thread
        self._start_async_loop()
        
        # Start MQTT PID subscriber
        self._start_mqtt_pid_subscriber()
        
        # Update GUI
        self._update_video_display()
        self._update_stats_display()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
    
    def _setup_ui(self):
        """Configurează interfața grafică."""
        
        # Top control panel
        control_frame = ttk.Frame(self.root)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        
        # Connect button
        self.connect_btn = ttk.Button(
            control_frame,
            text="Connect",
            command=self._on_connect_clicked
        )
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        # Disconnect button
        self.disconnect_btn = ttk.Button(
            control_frame,
            text="Disconnect",
            command=self._on_disconnect_clicked,
            state=tk.DISABLED
        )
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.status_label = ttk.Label(
            control_frame,
            text="Status: Disconnected",
            foreground="red"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Main content frame
        content_frame = ttk.Frame(self.root)
        content_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Video display (left side)
        video_frame = ttk.LabelFrame(content_frame, text="Video Stream")
        video_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        self.video_label = ttk.Label(video_frame, text="No video stream")
        self.video_label.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Stats panel (right side)
        stats_frame = ttk.LabelFrame(content_frame, text="Statistics", width=260, height=360)
        stats_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5)
        stats_frame.pack_propagate(False)
        stats_frame.columnconfigure(0, weight=1)
        stats_frame.columnconfigure(1, weight=1)
        
        # Stats labels
        self.stats_labels = {}
        stats_items = [
            ("Connection State", "connection_state"),
            ("Frames Received", "frames_received"),
            ("Current FPS", "fps"),
            ("Resolution", "resolution"),
            ("Last Timestamp", "last_timestamp"),
        ]
        
        for i, (label_text, key) in enumerate(stats_items):
            label = ttk.Label(stats_frame, text=f"{label_text}:")
            label.grid(row=i, column=0, sticky=tk.W, padx=10, pady=5)
            
            value_label = ttk.Label(stats_frame, text="N/A", foreground="blue")
            value_label.grid(row=i, column=1, sticky=tk.W, padx=10, pady=5)
            
            self.stats_labels[key] = value_label
        
        # Separator
        sep = ttk.Separator(stats_frame, orient=tk.HORIZONTAL)
        sep.grid(row=len(stats_items), column=0, columnspan=2,
                 sticky=tk.EW, padx=10, pady=8)

        # PID Graph button
        self.pid_graph_btn = ttk.Button(
            stats_frame,
            text="📈  PID Graph",
            command=self._open_pid_graph,
        )
        self.pid_graph_btn.grid(
            row=len(stats_items) + 1, column=0, columnspan=2,
            sticky=tk.EW, padx=10, pady=(0, 4)
        )

        # MQTT PID status indicator
        self.pid_mqtt_status_label = ttk.Label(
            stats_frame, text="● MQTT PID: --", foreground="gray"
        )
        self.pid_mqtt_status_label.grid(
            row=len(stats_items) + 2, column=0, columnspan=2,
            sticky=tk.W, padx=10, pady=(0, 6)
        )
        
        # Log text area
        log_frame = ttk.LabelFrame(self.root, text="Logs")
        log_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)
        
        self.log_text = tk.Text(log_frame, height=8, state=tk.DISABLED)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)
    
    def _log(self, message: str):
        """Adaugă mesaj în log."""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        logger.info(message)
    
    def _start_async_loop(self):
        """Pornește asyncio loop în thread separat."""
        def run_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_forever()
        
        self.loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.loop_thread.start()
        
        # Așteaptă până când loop-ul este gata
        while self.loop is None:
            time.sleep(0.01)
    
    def _run_async(self, coro):
        """Rulează corutină în loop-ul asyncio."""
        return asyncio.run_coroutine_threadsafe(coro, self.loop)
    
    def _on_connect_clicked(self):
        """Handler pentru butonul Connect."""
        self.connect_btn.config(state=tk.DISABLED)
        self._log("Initiating connection...")
        self._run_async(self._connect())
    
    def _on_disconnect_clicked(self):
        """Handler pentru butonul Disconnect."""
        self._log("Disconnecting...")
        self._run_async(self._disconnect())
    
    async def _connect(self):
        """Conectează la sender prin WebRTC."""
        try:
            self._log(f"Connecting to signaling server: {self.server_url}")
            
            # Creează peer connection
            self.pc = RTCPeerConnection()
            
            # Event handlers
            @self.pc.on("track")
            def on_track(track):
                self._log(f"Track received: {track.kind}")
                
                if track.kind == "video":
                    self._run_async(self._process_video_track(track))
            
            @self.pc.on("connectionstatechange")
            async def on_connectionstatechange():
                state = self.pc.connectionState
                self.stats["connection_state"] = state
                self._log(f"Connection state: {state}")
                
                if state == "connected":
                    self.is_connected = True
                    self.root.after(0, self._update_connection_ui, True)
                elif state in ["failed", "closed"]:
                    self.is_connected = False
                    self.root.after(0, self._update_connection_ui, False)
            
            # Conectare la signaling server
            async with SignalingClient(self.server_url) as signaling:
                # Verifică server
                if not await signaling.check_health():
                    self._log("ERROR: Signaling server is not responding")
                    self.root.after(0, self._update_connection_ui, False)
                    return
                
                self._log("Waiting for offer from sender...")
                
                # Așteaptă offer
                offer_data = await signaling.get_offer(timeout=60)
                
                if not offer_data:
                    self._log("ERROR: Did not receive offer")
                    self.root.after(0, self._update_connection_ui, False)
                    return
                
                self._log("Offer received, creating answer...")
                
                # Setează remote description
                offer = RTCSessionDescription(
                    sdp=offer_data["sdp"],
                    type=offer_data["type"]
                )
                await self.pc.setRemoteDescription(offer)
                
                # Creează answer
                answer = await self.pc.createAnswer()
                await self.pc.setLocalDescription(answer)
                
                # Trimite answer
                self._log("Sending answer to sender...")
                if not await signaling.send_answer(
                    self.pc.localDescription.sdp,
                    self.pc.localDescription.type
                ):
                    self._log("ERROR: Failed to send answer")
                    self.root.after(0, self._update_connection_ui, False)
                    return
                
                self._log("Answer sent successfully!")
                self._log("Waiting for video stream...")
        
        except Exception as e:
            self._log(f"ERROR: {e}")
            logger.exception("Connection error")
            self.root.after(0, self._update_connection_ui, False)
    
    async def _disconnect(self):
        """Deconectează."""
        if self.pc:
            await self.pc.close()
            self.pc = None
        
        self.is_connected = False
        self._log("Disconnected")
        self.root.after(0, self._update_connection_ui, False)
    
    async def _process_video_track(self, track):
        """Procesează video track-ul primit."""
        self._log("Video track processing started")
        frame_times = []
        
        try:
            while True:
                frame = await track.recv()
                
                # Convert to numpy array
                img = frame.to_ndarray(format="bgr24")
                
                # Update stats
                self.stats["frames_received"] += 1
                self.stats["resolution"] = f"{img.shape[1]}x{img.shape[0]}"
                
                # Calculate FPS
                current_time = time.time()
                frame_times.append(current_time)
                frame_times = [t for t in frame_times if current_time - t < 1.0]
                self.stats["fps"] = len(frame_times)
                
                # Put frame in queue (non-blocking)
                try:
                    self.frame_queue.put_nowait(img)
                except queue.Full:
                    # Remove oldest frame if queue is full
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put_nowait(img)
                    except:
                        pass
        
        except Exception as e:
            self._log(f"ERROR in video processing: {e}")
            logger.exception("Video processing error")
    
    def _update_video_display(self):
        """Actualizează afișajul video."""
        try:
            # Get frame from queue
            frame = self.frame_queue.get_nowait()
            
            # Resize for display (maintain aspect ratio)
            display_height = 500
            aspect_ratio = frame.shape[1] / frame.shape[0]
            display_width = int(display_height * aspect_ratio)
            
            frame_resized = cv2.resize(frame, (display_width, display_height))
            
            # Convert to PIL Image
            frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
            img_pil = Image.fromarray(frame_rgb)
            img_tk = ImageTk.PhotoImage(image=img_pil)
            
            # Update label
            self.video_label.config(image=img_tk)
            self.video_label.image = img_tk  # Keep reference
        
        except queue.Empty:
            pass
        except Exception as e:
            logger.error(f"Error updating video: {e}")
        
        # Schedule next update
        self.root.after(33, self._update_video_display)  # ~30 FPS
    
    def _update_stats_display(self):
        """Actualizează afișajul statisticilor."""
        for key, label in self.stats_labels.items():
            value = self.stats.get(key, "N/A")
            label.config(text=str(value))
        
        # Schedule next update
        self.root.after(500, self._update_stats_display)  # Every 0.5s
    
    def _update_connection_ui(self, connected: bool):
        """Actualizează UI-ul în funcție de starea conexiunii."""
        if connected:
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.status_label.config(
                text="Status: Connected",
                foreground="green"
            )
        else:
            self.connect_btn.config(state=tk.NORMAL)
            self.disconnect_btn.config(state=tk.DISABLED)
            self.status_label.config(
                text="Status: Disconnected",
                foreground="red"
            )
    
    def _on_closing(self):
        """Handler pentru închiderea ferestrei."""
        if self.is_connected:
            if messagebox.askokcancel("Quit", "Are you sure you want to quit?"):
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

    # -------------------------------------------------------------------------
    # MQTT PID subscriber
    # -------------------------------------------------------------------------

    def _start_mqtt_pid_subscriber(self):
        """Conectează un client MQTT dedicat pentru topicul robot/pid_telemetry."""
        try:
            client = mqtt.Client(
                client_id="receiver_gui_pid",
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
            )

            def on_connect(c, userdata, flags, rc):
                if rc == 0:
                    self._mqtt_pid_connected = True
                    c.subscribe("robot/pid_telemetry", qos=0)
                    logger.info("[MQTT PID] Connected & subscribed to robot/pid_telemetry")
                    self.root.after(0, self._refresh_pid_mqtt_label)
                else:
                    logger.warning(f"[MQTT PID] Connect failed rc={rc}")

            def on_disconnect(c, userdata, rc):
                self._mqtt_pid_connected = False
                logger.info("[MQTT PID] Disconnected")
                self.root.after(0, self._refresh_pid_mqtt_label)
                # Freeze the graph if it’s open
                if self._pid_graph_window and self._pid_graph_window.alive:
                    self._pid_graph_window.freeze()

            def on_message(c, userdata, msg):
                try:
                    import json
                    data = json.loads(msg.payload.decode())
                    # Push into queue (drop oldest if full)
                    try:
                        self.pid_data_queue.put_nowait(data)
                    except queue.Full:
                        try:
                            self.pid_data_queue.get_nowait()
                            self.pid_data_queue.put_nowait(data)
                        except Exception:
                            pass
                except Exception as e:
                    logger.debug(f"[MQTT PID] message parse error: {e}")

            client.on_connect    = on_connect
            client.on_disconnect = on_disconnect
            client.on_message    = on_message

            client.connect_async(self.mqtt_broker, self.mqtt_port, keepalive=60)
            client.loop_start()
            self._mqtt_pid_client = client
            logger.info(f"[MQTT PID] Connecting to {self.mqtt_broker}:{self.mqtt_port}...")
        except Exception as e:
            logger.warning(f"[MQTT PID] Could not start subscriber: {e}")

    def _refresh_pid_mqtt_label(self):
        """Actualizează indicatorul de stare MQTT PID în UI."""
        if self._mqtt_pid_connected:
            self.pid_mqtt_status_label.config(
                text="● MQTT PID: OK", foreground="green"
            )
        else:
            self.pid_mqtt_status_label.config(
                text="● MQTT PID: OFF", foreground="red"
            )

    def _open_pid_graph(self):
        """Deschide fereastra grafic PID (sau o aduce în față dacă există)."""
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


# =============================================================================
# PID Graph Window
# =============================================================================

class PIDGraphWindow:
    """
    Fereastră pop-up cu grafic în timp real al controlului PID.

    Arată:
      - Linia de referință (0.0 — centrul benzii)
      - Răspunsul sistemului (offset normalizat [-1, 1])

    Fereastra de timp: 30 de secunde derulate automat.
    La pierderea conexiunii MQTT graficul îngheță.

    Util pentru Ziegler-Nichols:
      Ku = Kp la care apare oscilație susținută (din grafic)
      Tu = perioada oscilației (măsurată pe axa X)
    """
    WINDOW_SECONDS = 30
    UPDATE_MS      = 50    # 20 Hz refresh
    MAX_POINTS     = WINDOW_SECONDS * 20 + 60  # buffer cu mic surplus

    def __init__(self, parent: tk.Tk, data_queue: queue.Queue,
                 mqtt_connected_fn):
        self.data_queue         = data_queue
        self.mqtt_connected_fn  = mqtt_connected_fn
        self.alive              = True
        self.frozen             = False

        # Buffers
        self.times     = collections.deque(maxlen=self.MAX_POINTS)
        self.responses = collections.deque(maxlen=self.MAX_POINTS)
        self.start_ts  = None   # primul timestamp primit (ms)

        # ── Fereastră Toplevel ────────────────────────────────────────────────
        self.window = tk.Toplevel(parent)
        self.window.title("📈 PID Real-Time Graph — Ziegler-Nichols Analysis")
        self.window.geometry("1050x660")
        self.window.configure(bg="#0d1117")
        self.window.protocol("WM_DELETE_WINDOW", self._on_close)

        # ── Matplotlib Figure ─────────────────────────────────────────────────
        self.fig = Figure(figsize=(10.5, 5.2), dpi=100, facecolor="#0d1117")
        self.ax  = self.fig.add_subplot(111)
        self._setup_axes()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.window)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True,
                                         padx=8, pady=(8, 0))

        # ── Bara de stare (jos) ───────────────────────────────────────────────
        bar = tk.Frame(self.window, bg="#0d1117")
        bar.pack(fill=tk.X, padx=8, pady=6)

        self.lbl_mqtt = tk.Label(
            bar, text="● MQTT: connecting…",
            fg="#f0ad4e", bg="#0d1117",
            font=("Consolas", 10)
        )
        self.lbl_mqtt.pack(side=tk.LEFT, padx=(0, 20))

        self.lbl_value = tk.Label(
            bar, text="Response: —",
            fg="#00e5ff", bg="#0d1117",
            font=("Consolas", 10)
        )
        self.lbl_value.pack(side=tk.LEFT, padx=(0, 20))

        self.lbl_steering = tk.Label(
            bar, text="Steering: —",
            fg="#b8e986", bg="#0d1117",
            font=("Consolas", 10)
        )
        self.lbl_steering.pack(side=tk.LEFT)

        self.lbl_frozen = tk.Label(
            bar, text="",
            fg="#f0ad4e", bg="#0d1117",
            font=("Consolas", 10, "bold")
        )
        self.lbl_frozen.pack(side=tk.RIGHT, padx=10)

        # Hint Ziegler-Nichols
        tk.Label(
            bar,
            text="Ku = Kp la oscilație susținută  |  Tu = perioadă oscilație (axă X)",
            fg="#555d6b", bg="#0d1117",
            font=("Consolas", 9)
        ).pack(side=tk.RIGHT, padx=20)

        # Pornim loop-ul de actualizare
        self._schedule_update()

    # -------------------------------------------------------------------------

    def _setup_axes(self):
        ax = self.ax
        ax.set_facecolor("#0a0f1a")

        # Grid subtil
        ax.grid(True, color="#1d2535", linewidth=0.7, alpha=0.8)
        ax.set_axisbelow(True)

        # Borduri
        for spine in ax.spines.values():
            spine.set_color("#2d3748")

        # Axe
        ax.set_xlim(0, self.WINDOW_SECONDS)
        ax.set_ylim(-1.25, 1.25)
        ax.set_xlabel("Time (s)", color="#8899aa", fontsize=11)
        ax.set_ylabel("Normalized Lane Offset", color="#8899aa", fontsize=11)
        ax.set_title(
            "PID Response — Lane Center Tracking  (robot/pid_telemetry @ 20 Hz)",
            color="#e0e8f0", fontsize=12, fontweight="bold", pad=10
        )
        ax.tick_params(colors="#8899aa", labelsize=9)

        # Linii orizontale de referință
        ax.axhline(y=0,    color="#ffffff", linestyle="--",
                   linewidth=1.8, alpha=0.55, label="Reference = 0 (lane center)")
        ax.axhline(y= 1.0, color="#ff4d4d", linestyle=":",
                   linewidth=0.9, alpha=0.4)
        ax.axhline(y=-1.0, color="#ff4d4d", linestyle=":",
                   linewidth=0.9, alpha=0.4)

        # Zonele colorate: central OK / margini atenție
        ax.axhspan(-0.3,  0.3,  alpha=0.04, color="#00ff88")   # zona bună
        ax.axhspan( 0.3,  1.25, alpha=0.03, color="#ff6600")   # oscilație
        ax.axhspan(-1.25,-0.3,  alpha=0.03, color="#ff6600")

        # Tickuri Y clare
        ax.set_yticks([-1.0, -0.75, -0.5, -0.25, 0,
                       0.25, 0.5, 0.75, 1.0])

        # Liniile de date (inițial goale)
        self.line_response, = ax.plot(
            [], [], color="#00e5ff", linewidth=2.0, label="Response (offset)"
        )

        legend = ax.legend(
            loc="upper right",
            facecolor="#0d1117", edgecolor="#2d3748",
            labelcolor="#99aabb", fontsize=9
        )

        self.fig.tight_layout(pad=1.8)

    # -------------------------------------------------------------------------

    def _schedule_update(self):
        if self.alive:
            self._update_plot()
            self.window.after(self.UPDATE_MS, self._schedule_update)

    def _update_plot(self):
        """Consumă coada de date şi redesenează graficul."""
        if self.frozen:
            return

        # Consumăm tot ce e în coadă
        changed = False
        last_item = None
        while True:
            try:
                item = self.data_queue.get_nowait()
                last_item = item
                changed = True

                ts = item.get("timestamp", 0)  # ms epoch
                if self.start_ts is None:
                    self.start_ts = ts

                t = (ts - self.start_ts) / 1000.0  # secunde relative
                self.times.append(t)
                self.responses.append(item.get("response", 0.0))

            except queue.Empty:
                break

        if not changed:
            # Actualizăm doar statusul MQTT
            self._update_status_bar(last_item)
            return

        # Fereastră derulantă 30 s
        if self.times:
            t_now = self.times[-1]
            t_min = max(0.0, t_now - self.WINDOW_SECONDS)
            t_max = t_min + self.WINDOW_SECONDS

            self.ax.set_xlim(t_min, t_max)
            self.line_response.set_data(list(self.times), list(self.responses))

        self.canvas.draw_idle()
        self._update_status_bar(last_item)

    def _update_status_bar(self, last_item):
        """Actualizează etichetele din bara de stare."""
        connected = self.mqtt_connected_fn()
        if connected:
            self.lbl_mqtt.config(text="● MQTT: OK",  fg="#5cb85c")
        else:
            self.lbl_mqtt.config(text="● MQTT: OFF", fg="#d9534f")

        if last_item is not None:
            resp  = last_item.get("response",       0.0)
            steer = last_item.get("steering_angle", 0.0)
            self.lbl_value.config(
                text=f"Response: {resp:+.3f}"
            )
            self.lbl_steering.config(
                text=f"Steering: {steer:+.1f}°"
            )

        if self.frozen:
            self.lbl_frozen.config(text="⏸ INGHEȚAT — MQTT deconectat")
        else:
            self.lbl_frozen.config(text="")

    def freeze(self):
        """Opreste actualizarea graficului (MQTT pierdut)."""
        if not self.frozen:
            self.frozen = True
            self.lbl_frozen.config(
                text="⏸ INGHEȚAT — MQTT deconectat", fg="#f0ad4e"
            )
            logger.info("[PIDGraph] Frozen — MQTT disconnected")

    def _on_close(self):
        self.alive = False
        self.window.destroy()


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="WebRTC Video Receiver GUI (Windows)"
    )
    parser.add_argument(
        "--server-ip",
        required=True,
        help="Signaling server IP (Raspberry Pi IP)"
    )
    parser.add_argument(
        "--server-port",
        type=int,
        default=9000,
        help="Signaling server port"
    )
    parser.add_argument(
        "--mqtt-broker",
        default="localhost",
        help="MQTT broker IP/hostname pentru PID telemetry (default: localhost)"
    )
    parser.add_argument(
        "--mqtt-port",
        type=int,
        default=1883,
        help="MQTT broker port (default: 1883)"
    )

    args = parser.parse_args()
    server_url = f"http://{args.server_ip}:{args.server_port}"

    # Create GUI
    root = tk.Tk()
    app = VideoReceiverGUI(
        root,
        server_url,
        mqtt_broker=args.mqtt_broker,
        mqtt_port=args.mqtt_port,
    )

    # Run
    try:
        root.mainloop()
    except KeyboardInterrupt:
        logger.info("Application stopped by user")


if __name__ == "__main__":
    main()

