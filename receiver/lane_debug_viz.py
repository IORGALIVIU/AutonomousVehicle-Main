#!/usr/bin/env python3
"""
lane_debug_viz.py  —  Receiver-side only, Windows debug visualization.

Subclasează LaneDetector (importat din sender/ via sys.path) și adaugă
două metode de vizualizare care rulează EXCLUSIV pe Windows:
  - get_birds_eye_visualization(frame)  → imagine RGB alb/negru warped
  - get_sliding_window_visualization(frame) → imagine RGB cu ferestre colorate

NU este importat / rulat pe Raspberry Pi.
"""

import cv2
import numpy as np
import logging

logger = logging.getLogger(__name__)

# LaneDetector este importat din sender/ (sys.path configurat in receiver_gui_mqtt.py)
from lane_detection import LaneDetector
from config import CFG

_ld = CFG["lane_detection"]


class DebugLaneDetector(LaneDetector):
    """
    Extinde LaneDetector cu metode de vizualizare pentru debug pe receiver.
    Parametrii și logica de detecție rămân identice cu cei de pe Pi.
    """

    def get_birds_eye_visualization(self, frame):
        """
        Returnează imaginea binary Bird's Eye (warped) ca RGB pentru afișare.

        Args:
            frame: BGR image la rezoluția de procesare (ex. 640×480),
                   identic cu ce trimite Pi-ul prin WebRTC după resize.

        Returns:
            ndarray RGB (H, W, 3) — alb = pixeli de bandă, negru = fundal.
            None dacă procesarea eșuează.
        """
        try:
            binary = self.preprocess_frame(frame)       # valori 0/1
            warped = self.perspective_transform(binary)  # valori 0/1

            gray = (warped * 255).astype(np.uint8)
            rgb = np.dstack((gray, gray, gray))

            h, w = rgb.shape[:2]
            cv2.putText(rgb, f"Bird's Eye Binary  {w}x{h}",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 120), 1, cv2.LINE_AA)
            return rgb
        except Exception as e:
            logger.debug(f"[BirdsEye] {e}")
            return None

    def get_sliding_window_visualization(self, frame):
        """
        Returnează imaginea warped cu ferestrele sliding window desenate color.

        Culori:
          - Roșu   : pixelii benzii stângi detectați
          - Albastru: pixelii benzii drepte detectați
          - Galben : dreptunghiuri de căutare stânga
          - Cyan   : dreptunghiuri de căutare dreapta
          - Verde  : polinoamele fit curente

        Args:
            frame: BGR image la rezoluția de procesare (ex. 640×480)

        Returns:
            ndarray RGB (H, W, 3) sau None dacă procesarea eșuează.
        """
        try:
            binary = self.preprocess_frame(frame)
            warped = self.perspective_transform(binary)

            gray = (warped * 255).astype(np.uint8)
            out_img = np.dstack((gray, gray, gray))

            h, w = warped.shape[:2]

            # ── Histogram pentru pozițiile de start ───────────────────────────
            from scipy.ndimage import uniform_filter1d
            histogram = np.sum(warped[h * 2 // 3:, :], axis=0)
            hist_smooth = uniform_filter1d(histogram.astype(float), size=30)

            midpoint = w // 2
            global_max = np.max(hist_smooth)
            peak_thr = max(30, global_max * _ld["peak_threshold_frac"])

            left_detected  = np.max(hist_smooth[:midpoint]) > peak_thr
            right_detected = np.max(hist_smooth[midpoint:])  > peak_thr

            leftx_base  = int(np.argmax(hist_smooth[:midpoint])) \
                          if left_detected else w // 4
            rightx_base = int(np.argmax(hist_smooth[midpoint:]) + midpoint) \
                          if right_detected else 3 * w // 4

            # Fallback din ultimul polinoam cunoscut
            if not left_detected and self.left_fit is not None:
                leftx_base = int(
                    self.left_fit[0] * h**2 + self.left_fit[1] * h + self.left_fit[2]
                )
            if not right_detected and self.right_fit is not None:
                rightx_base = int(
                    self.right_fit[0] * h**2 + self.right_fit[1] * h + self.right_fit[2]
                )

            params        = self.STATE_PARAMS[self.state]
            base_margin   = params["margin"]
            window_minpix = params["minpix"]
            win_height    = h // self.nwindows

            nonzero  = warped.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])

            leftx_cur  = leftx_base
            rightx_cur = rightx_base
            left_inds  = []
            right_inds = []

            MIN_SEP  = _ld["min_separation"]
            MAX_JUMP = _ld["max_window_jump"]

            for win in range(self.nwindows):
                y_low  = h - (win + 1) * win_height
                y_high = h - win * win_height
                margin = max(40, base_margin - win * 2)
                minpix = max(10, window_minpix - win * 5)

                # ── Stânga ────────────────────────────────────────────────────
                hard_right = (rightx_cur - MIN_SEP) if rightx_cur is not None else w // 2
                xl_low  = leftx_cur - margin
                xl_high = min(leftx_cur + margin, hard_right)
                if xl_low < xl_high:
                    cv2.rectangle(out_img, (xl_low, y_low), (xl_high, y_high),
                                  (200, 200, 0), 2)          # galben
                    gl = ((nonzeroy >= y_low) & (nonzeroy < y_high) &
                          (nonzerox >= xl_low) & (nonzerox < xl_high)).nonzero()[0]
                    left_inds.append(gl)
                    if len(gl) > minpix:
                        nx = int(np.mean(nonzerox[gl]))
                        if abs(nx - leftx_cur) < MAX_JUMP:
                            leftx_cur = nx

                # ── Dreapta ───────────────────────────────────────────────────
                hard_left = (leftx_cur + MIN_SEP) if leftx_cur is not None else w // 2
                xr_low  = max(rightx_cur - margin, hard_left)
                xr_high = rightx_cur + margin
                if xr_low < xr_high:
                    cv2.rectangle(out_img, (xr_low, y_low), (xr_high, y_high),
                                  (0, 200, 200), 2)           # cyan
                    gr = ((nonzeroy >= y_low) & (nonzeroy < y_high) &
                          (nonzerox >= xr_low) & (nonzerox < xr_high)).nonzero()[0]
                    right_inds.append(gr)
                    if len(gr) > minpix:
                        nx = int(np.mean(nonzerox[gr]))
                        if abs(nx - rightx_cur) < MAX_JUMP:
                            rightx_cur = nx

            # ── Colorăm pixelii detectați ─────────────────────────────────────
            if left_inds:
                all_l = np.concatenate(left_inds)
                out_img[nonzeroy[all_l], nonzerox[all_l]] = (220, 50, 50)   # roșu

            if right_inds:
                all_r = np.concatenate(right_inds)
                out_img[nonzeroy[all_r], nonzerox[all_r]] = (50, 100, 220)  # albastru

            # ── Polinoame fit ─────────────────────────────────────────────────
            ploty = np.linspace(0, h - 1, h)
            for fit, color in [(self.left_fit, (0, 255, 80)),
                               (self.right_fit, (0, 255, 80))]:
                if fit is not None:
                    fitx = (fit[0] * ploty**2 + fit[1] * ploty + fit[2]).astype(int)
                    for yi, xi in zip(ploty.astype(int), fitx):
                        if 0 <= xi < w:
                            cv2.circle(out_img, (xi, yi), 2, color, -1)

            # ── Legendă ───────────────────────────────────────────────────────
            cv2.putText(out_img, f"Sliding Window  [{self.state.name}]",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 100), 1, cv2.LINE_AA)
            for i, (rect_color, label) in enumerate([
                ((220, 50, 50),  "Left px"),
                ((50, 100, 220), "Right px"),
                ((200, 200, 0),  "L-win"),
                ((0, 200, 200),  "R-win"),
            ]):
                x = 8 + i * 80
                cv2.rectangle(out_img, (x, 30), (x + 10, 40), rect_color, -1)
                cv2.putText(out_img, label, (x + 13, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, rect_color, 1)

            # Returnăm RGB (Tkinter nu înțelege BGR)
            return cv2.cvtColor(out_img, cv2.COLOR_BGR2RGB)

        except Exception as e:
            logger.debug(f"[SlidingWindow] {e}")
            return None
