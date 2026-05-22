#!/usr/bin/env python3
"""
Advanced Lane Detection for Autonomous Driving
- Sliding Window Search
- Polynomial Curve Fitting
- Bird's Eye View (Perspective Transform)
- Optimized for Raspberry Pi 5
"""

import cv2
import numpy as np
import logging
from enum import Enum
from scipy.ndimage import uniform_filter1d

logger = logging.getLogger(__name__)

class DetectionState(Enum):
    SEARCHING = "searching"
    TRACKING = "tracking"
    ONE_LANE = "one_lane"
    EXTRAPOLATING = "extrapolating"

class LaneDetector:
    """
    Detects lane lines using sliding window search and polynomial fitting.
    Optimized for real-time processing on Raspberry Pi 5.
    """
    
    def __init__(self, 
                 img_width=1920, 
                 img_height=1080,
                 lane_width_meters=0.6,  # Physical lane width (adjust for your track)
                 camera_height_meters=0.2):  # Camera height from ground
        """
        Initialize lane detector.
        
        Args:
            img_width: Input image width
            img_height: Input image height
            lane_width_meters: Physical width between lane lines
            camera_height_meters: Camera mounting height
        """
        self.img_width = img_width
        self.img_height = img_height
        self.lane_width_meters = lane_width_meters
        
        # Perspective transform matrices (will be calculated)
        self.M = None  # Transform matrix
        self.Minv = None  # Inverse transform matrix
        
        # Sliding window parameters
        self.nwindows = 9  # Number of sliding windows
        
        # State Machine parameters
        self.state = DetectionState.SEARCHING
        self.lost_frames = 0
        self.max_lost_frames = 5
        self.lane_width_pixels = None  # Calibrated dynamically
        
        self.STATE_PARAMS = {
            DetectionState.SEARCHING:     {'margin': 120, 'minpix': 20},
            DetectionState.TRACKING:      {'margin': 60,  'minpix': 40},
            DetectionState.ONE_LANE:      {'margin': 80,  'minpix': 25},
            DetectionState.EXTRAPOLATING: {'margin': 100, 'minpix': 15},
        }
        
        # Lane detection confidence
        self.left_fit = None  # Left lane polynomial coefficients
        self.right_fit = None  # Right lane polynomial coefficients
        self.lane_detected = False
        
        # ROI (Region of Interest) - focus on road area
        self.roi_vertices = self._calculate_roi()
        
        # Calculate perspective transform
        self._calculate_perspective_transform()
        
        logger.info(f"LaneDetector initialized: {img_width}x{img_height}")
    
    def _calculate_roi(self):
        """Calculate ROI trapezoid vertices."""
        # Define trapezoid for road area
        # Bottom: full width, Top: narrower (perspective)
        bottom_left = [int(self.img_width * 0.00), self.img_height]
        bottom_right = [int(self.img_width * 1.0), self.img_height]
        top_left = [int(self.img_width * 0.10), int(self.img_height * 0.10)]
        top_right = [int(self.img_width * 0.90), int(self.img_height * 0.10)]
        
        return np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    
    def _calculate_perspective_transform(self):
        """Calculate perspective transform for bird's eyeq view."""
        # Source points (trapezoid in original image)
        src = np.float32([
            [int(self.img_width * 0.10), int(self.img_height * 0.10)],  # Top-left
            [int(self.img_width * 0.90), int(self.img_height * 0.10)],  # Top-right
            [int(self.img_width * 1.00), self.img_height],             # Bottom-right
            [int(self.img_width * 0.00), self.img_height]              # Bottom-left
        ])
        
        # Destination points (rectangle in bird's eye view)
        dst = np.float32([
            [0, 0],                          # Top-left (colțul 0,0)
            [self.img_width, 0],             # Top-right (lățime maximă, 0)
            [self.img_width, self.img_height],# Bottom-right (lățime maximă, înălțime maximă)
            [0, self.img_height]             # Bottom-left (0, înălțime maximă)
        ])
        
        # Calculate transform matrices
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)
        
        logger.info("Perspective transform calculated")
    
    def preprocess_frame(self, frame):
        """
        Preprocess frame for lane detection.
        """
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        l_channel = hls[:, :, 1]
        s_channel = hls[:, :, 2]
        
        # EROAREA CRITICĂ A FOST AICI: 
        # cv2.GaussianBlur(s_channel, (9, 9), 0) pe o linie foarte subțire
        # "împrăștie" intensitatea pixelilor. Un pixel de S=80 devine S=25 după blur,
        # căzând sub orice prag de detecție. Liniile subțiri dispar complet!
        
        # 1. Threshold pe S channel FĂRĂ blur anterior.
        # Banda galbenă are saturație. Scădem puțin pragul pentru siguranță.
        s_thresh_min = 35
        s_thresh_max = 255
        s_binary = np.zeros_like(s_channel, dtype=np.uint8)
        s_binary[(s_channel >= s_thresh_min) & (s_channel <= s_thresh_max)] = 1
        
        # 2. Threshold pe L channel pentru a elimina umbrele (unde S poate fi "zgomotos")
        l_thresh_min = 100
        l_thresh_max = 255
        l_binary = np.zeros_like(l_channel, dtype=np.uint8)
        l_binary[(l_channel >= l_thresh_min) & (l_channel <= l_thresh_max)] = 1
        
        # 3. Combinăm S și L
        combined_binary = cv2.bitwise_and(s_binary, l_binary)
        
        # 4. AICI eliminăm textura parchetului! 
        # Morphological opening (Erosion urmat de Dilation) va șterge punctele
        # mici de zgomot izolate, dar va păstra linia continuă intactă.
        kernel = np.ones((5, 5), np.uint8)
        combined_binary = cv2.morphologyEx(combined_binary, cv2.MORPH_OPEN, kernel)
        
        # ROI mask
        mask = np.zeros_like(combined_binary)
        cv2.fillPoly(mask, self.roi_vertices, 1)
        masked_binary = cv2.bitwise_and(combined_binary, mask)
        
        return masked_binary, l_binary, s_binary, combined_binary
    
    def perspective_transform(self, binary):
        """
        Apply perspective transform to get bird's eye view.
        
        Args:
            binary: Binary thresholded image
            
        Returns:
            warped: Bird's eye view image
        """
        warped = cv2.warpPerspective(binary, self.M, (self.img_width, self.img_height))
        return warped
    
    def _estimate_missing_lane(self, known_fit, known_side):
        """
        Deduce linia lipsă din linia cunoscută + lățimea benzii.
        known_side: 'left' sau 'right'
        """
        if self.lane_width_pixels is None or known_fit is None:
            return None
            
        # Translatăm direct coeficienții — stabil numeric
        estimated_fit = known_fit.copy()
        
        if known_side == 'right':
            estimated_fit[2] -= self.lane_width_pixels  # Stânga = dreapta - lățime
        else:
            estimated_fit[2] += self.lane_width_pixels  # Dreapta = stânga + lățime
            
        return estimated_fit

    def find_lane_pixels(self, warped):
        """
        Find lane pixels using sliding window search cu mașină de stări.
        """
        out_img = np.dstack((warped, warped, warped)) * 255
        
        # Folosim doar 1/3 de jos pentru histogramă pentru stabilitate
        histogram = np.sum(warped[warped.shape[0]*2//3:, :], axis=0)
        
        # Smooth histogram to eliminate noise peaks
        histogram_smooth = uniform_filter1d(histogram.astype(float), size=30)
        
        midpoint = histogram_smooth.shape[0] // 2
        left_peak_val = np.max(histogram_smooth[:midpoint])
        right_peak_val = np.max(histogram_smooth[midpoint:])
        
        # Prag relativ pentru a ignora reflexiile rămase când linia lipsește
        global_max = np.max(histogram_smooth)
        peak_threshold = max(30, global_max * 0.3)
        
        left_detected = left_peak_val > peak_threshold
        right_detected = right_peak_val > peak_threshold
        
        # Preluăm parametrii specifici stării (stabilită în detect_lanes la frame-ul anterior)
        params = self.STATE_PARAMS[self.state]
        base_margin = params['margin']
        base_minpix = params['minpix']
        
        leftx_base = np.argmax(histogram_smooth[:midpoint]) if left_detected else None
        rightx_base = np.argmax(histogram_smooth[midpoint:]) + midpoint if right_detected else None
        
        # Fallback pseudo-bază pentru sliding windows dacă avem o predicție bună
        if not left_detected and self.state in [DetectionState.TRACKING, DetectionState.EXTRAPOLATING, DetectionState.ONE_LANE] and self.left_fit is not None:
            leftx_base = int(self.left_fit[0]*warped.shape[0]**2 + self.left_fit[1]*warped.shape[0] + self.left_fit[2])
        if not right_detected and self.state in [DetectionState.TRACKING, DetectionState.EXTRAPOLATING, DetectionState.ONE_LANE] and self.right_fit is not None:
            rightx_base = int(self.right_fit[0]*warped.shape[0]**2 + self.right_fit[1]*warped.shape[0] + self.right_fit[2])
            
        window_height = warped.shape[0] // self.nwindows
        nonzero = warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        leftx_current = leftx_base
        rightx_current = rightx_base
        
        # Flag-uri de continuare — False înseamnă că am pierdut linia
        # și nu mai colectăm pixeli pentru restul ferestrelor acestui frame
        left_active  = leftx_current is not None
        right_active = rightx_current is not None

        # Câte ferestre consecutive fără pixeli tolerăm înainte să oprim
        # 2 = tolerant față de mici discontinuități; 1 = strict
        MAX_EMPTY_WINDOWS = 2
        left_empty_count  = 0
        right_empty_count = 0

        left_lane_inds  = []
        right_lane_inds = []

        MAX_JUMP = 80
        MIN_SEP  = 150

        for window in range(self.nwindows):
            win_y_low  = warped.shape[0] - (window + 1) * window_height
            win_y_high = warped.shape[0] - window * window_height
            
            # Parametri adaptați pe perspectivă (sus e mai incert)
            window_margin = base_margin + (window * 3)
            window_minpix = max(10, base_minpix - (window * 5))

            # ── Fereastra STÂNGĂ ──────────────────────────────────────────
            if left_active and leftx_current is not None:
                win_xleft_low  = leftx_current - window_margin
                win_xleft_high = leftx_current + window_margin

                # Bara spațială se bazează pe POZIȚIE, nu pe starea de căutare —
                # rightx_current rămâne barieră chiar dacă right_active = False
                hard_right_limit = (rightx_current - MIN_SEP) if rightx_current is not None \
                                   else (warped.shape[1] // 2)
                win_xleft_high = min(win_xleft_high, hard_right_limit)

                if win_xleft_low < win_xleft_high:
                    good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                     (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
                    cv2.rectangle(out_img, (win_xleft_low, win_y_low),
                                           (win_xleft_high, win_y_high), (0, 255, 0), 2)
                    left_lane_inds.append(good_left_inds)

                    if len(good_left_inds) > window_minpix:
                        new_leftx = int(np.mean(nonzerox[good_left_inds]))
                        if abs(new_leftx - leftx_current) < MAX_JUMP:
                            leftx_current = new_leftx
                        left_empty_count = 0  # Pixeli găsiți → resetăm contorul
                    else:
                        left_empty_count += 1
                        if left_empty_count >= MAX_EMPTY_WINDOWS:
                            left_active = False  # Oprim căutarea stânga pe acest frame
                            logger.debug(f"Left lane lost at window {window}")
                else:
                    # Fereastră inversată — marcăm vizual cu verde-închis subțire pentru debug
                    cv2.rectangle(out_img, (leftx_current - 5, win_y_low),
                                           (leftx_current + 5, win_y_high), (0, 100, 0), 1)

            # ── Fereastra DREAPTĂ ─────────────────────────────────────────
            if right_active and rightx_current is not None:
                win_xright_low  = rightx_current - window_margin
                win_xright_high = rightx_current + window_margin

                # Bara spațială se bazează pe POZIȚIE, nu pe starea de căutare —
                # leftx_current rămâne barieră chiar dacă left_active = False
                hard_left_limit = (leftx_current + MIN_SEP) if leftx_current is not None \
                                  else (warped.shape[1] // 2)
                win_xright_low = max(win_xright_low, hard_left_limit)

                if win_xright_low < win_xright_high:
                    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                      (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
                    cv2.rectangle(out_img, (win_xright_low, win_y_low),
                                           (win_xright_high, win_y_high), (0, 255, 0), 2)
                    right_lane_inds.append(good_right_inds)

                    if len(good_right_inds) > window_minpix:
                        new_rightx = int(np.mean(nonzerox[good_right_inds]))
                        if abs(new_rightx - rightx_current) < MAX_JUMP:
                            rightx_current = new_rightx
                        right_empty_count = 0  # Pixeli găsiți → resetăm contorul
                    else:
                        right_empty_count += 1
                        if right_empty_count >= MAX_EMPTY_WINDOWS:
                            right_active = False  # Oprim căutarea dreapta pe acest frame
                            logger.debug(f"Right lane lost at window {window}")
                else:
                    # Fereastră inversată — marcăm vizual cu roșu-închis subțire pentru debug
                    cv2.rectangle(out_img, (rightx_current - 5, win_y_low),
                                           (rightx_current + 5, win_y_high), (100, 0, 0), 1)
        
        leftx, lefty, rightx, righty = [], [], [], []
        if left_lane_inds:
            left_lane_inds = np.concatenate(left_lane_inds)
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            out_img[lefty, leftx] = [255, 0, 0]
            
        if right_lane_inds:
            right_lane_inds = np.concatenate(right_lane_inds)
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]
            out_img[righty, rightx] = [0, 0, 255]
            
        # Returnăm array-urile de pixeli și imaginile de debug
        # În plus, returnăm booleanul dacă am găsit suficient peak (real sau dedus) 
        # care a ghidat sliding window-ul, util pentru faza de fit polinomial
        return leftx, lefty, rightx, righty, out_img, left_detected, right_detected
    
    def fit_polynomial(self, leftx, lefty, rightx, righty):
        """
        Fit 2nd order polynomial to lane pixels.
        """
        # Prag adaptat la stare
        min_pixels = {
            DetectionState.TRACKING:      300,
            DetectionState.ONE_LANE:      150,
            DetectionState.EXTRAPOLATING: 100,
            DetectionState.SEARCHING:     200,
        }.get(self.state, 300)
        
        if len(leftx) > min_pixels:
            left_fit = np.polyfit(lefty, leftx, 2)
        else:
            left_fit = None
            
        if len(rightx) > min_pixels:
            right_fit = np.polyfit(righty, rightx, 2)
        else:
            right_fit = None
        
        return left_fit, right_fit
    
    def calculate_lane_center(self, left_fit, right_fit, y_eval=None):
        """
        Calculate lane center position and offset from car center.
        
        Args:
            left_fit: Left lane polynomial coefficients
            right_fit: Right lane polynomial coefficients
            y_eval: Y position to evaluate (default: bottom of image)
            
        Returns:
            lane_center: X position of lane center
            offset: Offset from image center (negative = left, positive = right)
        """
        if y_eval is None:
            y_eval = self.img_height  # Bottom of image
        
        # Calculate x positions at y_eval
        if left_fit is not None:
            left_x = left_fit[0] * y_eval**2 + left_fit[1] * y_eval + left_fit[2]
        else:
            left_x = None
            
        if right_fit is not None:
            right_x = right_fit[0] * y_eval**2 + right_fit[1] * y_eval + right_fit[2]
        else:
            right_x = None
        
        # Calculate lane center
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2
            car_center = self.img_width / 2
            offset = lane_center - car_center
            return lane_center, offset
        else:
            return None, None
    
    def detect_lanes(self, frame):
        """
        Complete lane detection pipeline.
        
        Args:
            frame: Input BGR image
            
        Returns:
            result: Dictionary with detection results
        """
        # Preprocess
        binary, l_bin, s_bin, combined_bin = self.preprocess_frame(frame)
        
        # Perspective transform
        warped = self.perspective_transform(binary)
        warped_color = cv2.warpPerspective(frame, self.M, (self.img_width, self.img_height))
        
        # Find lane pixels
        leftx, lefty, rightx, righty, sliding_window_img, left_detected, right_detected = self.find_lane_pixels(warped)
        
        # Fit polynomial
        left_fit, right_fit = self.fit_polynomial(leftx, lefty, rightx, righty)
        
        # State Machine Logic for final fits
        if left_fit is not None and right_fit is not None:
            self.state = DetectionState.TRACKING
            self.lost_frames = 0
            
            # Update lane width estimation
            y_mid = self.img_height * 0.75
            current_width = (right_fit[0]*y_mid**2 + right_fit[1]*y_mid + right_fit[2]) - \
                            (left_fit[0]*y_mid**2 + left_fit[1]*y_mid + left_fit[2])
            
            alpha = 0.1
            if self.lane_width_pixels is None:
                self.lane_width_pixels = current_width
            else:
                self.lane_width_pixels = (alpha * current_width + (1 - alpha) * self.lane_width_pixels)
                
        elif left_fit is not None or right_fit is not None:
            self.state = DetectionState.ONE_LANE
            self.lost_frames = 0
            # Deduce missing lane from the known one
            if left_fit is not None and right_fit is None:
                right_fit = self._estimate_missing_lane(left_fit, 'left')
            elif right_fit is not None and left_fit is None:
                left_fit = self._estimate_missing_lane(right_fit, 'right')
                
        elif self.left_fit is not None and self.lost_frames < self.max_lost_frames:
            self.state = DetectionState.EXTRAPOLATING
            self.lost_frames += 1
            left_fit = self.left_fit
            right_fit = self.right_fit
            
        else:
            self.state = DetectionState.SEARCHING
            self.lost_frames = 0
        
        # Calculate lane center and offset
        lane_center, offset_pixels = self.calculate_lane_center(left_fit, right_fit)
        
        # Convert pixel offset to meters (approximate)
        meters_per_pixel = self.lane_width_meters / (self.img_width * 0.5)  # Rough estimate
        offset_meters = offset_pixels * meters_per_pixel if offset_pixels is not None else None
        
        # Determine if lanes detected
        detected = (left_fit is not None and right_fit is not None and lane_center is not None)
        
        # Store for next iteration
        if detected:
            self.left_fit = left_fit
            self.right_fit = right_fit
            self.lane_detected = True
        else:
            self.lane_detected = False
        
        result = {
            'left_fit': left_fit,
            'right_fit': right_fit,
            'lane_center': lane_center,
            'offset': offset_pixels,
            'offset_meters': offset_meters,
            'detected': detected,
            'binary': binary,
            'warped': warped,
            'sliding_windows': sliding_window_img,
            'l_binary': l_bin,
            's_binary': s_bin,
            'combined_binary': combined_bin,
            'warped_color': warped_color
        }
        
        return result
    
    def visualize_lanes(self, frame, detection_result, steering_angle=None):
        """
        Draw detected lanes on original frame.
        
        Args:
            frame: Original BGR image
            detection_result: Result from detect_lanes()
            steering_angle: Optional, from PID controller to draw intended direction
            
        Returns:
            annotated: Annotated image with lane overlay
        """
        annotated = frame.copy()
        
        if not detection_result['detected']:
            cv2.putText(annotated, "NO LANES DETECTED", (50, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
            return annotated
        
        # Generate y values
        ploty = np.linspace(0, self.img_height - 1, self.img_height)
        
        # Calculate x values from polynomials
        left_fit = detection_result['left_fit']
        right_fit = detection_result['right_fit']
        
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
        
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(detection_result['warped']).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        
        # O imagine separată pentru linii pentru a le desena complet opace (fără alpha 0.3)
        line_warp = np.zeros_like(color_warp)
        
        # Recast x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        
        # Draw the lane onto warped blank image (verde)
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
        
        # 1. Traiectoria ideală (Mijlocul pe mijlocul lane-ului) -> Culoare: Albastru
        center_fitx = (left_fitx + right_fitx) / 2
        pts_center = np.array([np.transpose(np.vstack([center_fitx, ploty]))], np.int32)
        cv2.polylines(line_warp, pts_center, isClosed=False, color=(255, 0, 0), thickness=15)
        
        # 2. Direcția dorită de robot (Output-ul de la logica de conducere) -> Culoare: Roșu
        if steering_angle is not None:
            import math
            # Punctul de pornire: centrul de jos al imaginii (poziția mașinii)
            start_point = (int(self.img_width / 2), self.img_height)
            
            # Lungimea vizuală a liniei (până la jumătatea imaginii în bird's eye view)
            line_length = self.img_height // 2
            
            # Unghiul zero înseamnă drept înainte (pe axa Y negativă)
            # Volan la dreapta (pozitiv) înseamnă X crește, volan stânga (negativ) X scade
            theta = math.radians(steering_angle)
            end_x = int(start_point[0] + line_length * math.sin(theta))
            end_y = int(start_point[1] - line_length * math.cos(theta))
            
            cv2.line(line_warp, start_point, (end_x, end_y), (0, 0, 255), thickness=15)

        # 3. Linia perfect dreaptă la jumătatea imaginii (Centrul fizic al camerei) -> Culoare: Galben
        cv2.line(line_warp, (int(self.img_width / 2), 0), (int(self.img_width / 2), self.img_height), (0, 255, 255), thickness=8)
        
        # Warp back to original image space for both polygon and lines
        newwarp_poly = cv2.warpPerspective(color_warp, self.Minv, (self.img_width, self.img_height))
        newwarp_lines = cv2.warpPerspective(line_warp, self.Minv, (self.img_width, self.img_height))
        
        # Combine lane polygon with original image using transparency
        result = cv2.addWeighted(annotated, 1, newwarp_poly, 0.3, 0)
        
        # Overlay lines perfectly opaque
        lines_mask = cv2.cvtColor(newwarp_lines, cv2.COLOR_BGR2GRAY)
        result[lines_mask > 0] = newwarp_lines[lines_mask > 0]
        
        # Add text overlay
        offset = detection_result['offset']
        offset_m = detection_result['offset_meters']
        
        if offset is not None:
            direction = "LEFT" if offset < 0 else "RIGHT"
            cv2.putText(result, f"Offset: {abs(offset):.0f}px ({abs(offset_m):.3f}m) {direction}",
                       (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 3)
        
        cv2.putText(result, "LANES DETECTED", (50, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        cv2.putText(result, f"STATE: {self.state.name}", (50, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 255), 3)
        
        return result


if __name__ == "__main__":
    import os
    import glob

    logging.basicConfig(level=logging.INFO)
    
    print("Lane Detection Debug Test")
    
    # Path to test images
    test_images_dir = "d:/INTERNET/webrtc-streaming/sender/test_images"
    output_dir = "d:/INTERNET/webrtc-streaming/sender/debug_output"
    
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    image_paths = glob.glob(os.path.join(test_images_dir, "*.jpg")) + glob.glob(os.path.join(test_images_dir, "*.png"))
    
    if not image_paths:
        print(f"No images found in {test_images_dir}")
        exit()
        
    detector = None
    
    for img_path in image_paths:
        img_name = os.path.basename(img_path)
        print(f"\nProcessing {img_name}...")
        
        # Read image
        frame = cv2.imread(img_path)
        if frame is None:
            print(f"Failed to load {img_path}")
            continue
            
        img_height, img_width = frame.shape[:2]
        
        if detector is None or (detector.img_width != img_width or detector.img_height != img_height):
            detector = LaneDetector(img_width=img_width, img_height=img_height)
        
        # Încărcăm PIDController pentru a simula logica de steering pe imaginile de test
        steering_angle = 0.0
        try:
            from pid_controller import PIDController
            pid = PIDController()
        except ImportError:
            pid = None
        
        # Draw ROI on a copy of original frame
        roi_frame = frame.copy()
        cv2.polylines(roi_frame, [detector.roi_vertices], isClosed=True, color=(0, 255, 255), thickness=3)
        
        # Run detection
        result = detector.detect_lanes(frame)
        
        # Calculăm steering-ul dorit (dacă a fost detectată o bandă)
        if result['detected'] and pid is not None:
            steering_angle = pid.calculate_steering_angle(lane_offset_pixels=result['offset'], img_width=img_width)
            
        # Get annotated final frame with steering line
        annotated = detector.visualize_lanes(frame, result, steering_angle=steering_angle)
        
        # Get intermediate images
        binary = result['binary'] * 255  # Convert 0/1 to 0/255 for saving/displaying
        warped = result['warped'] * 255
        sliding_windows = result['sliding_windows']
        
        l_binary = result['l_binary'] * 255
        s_binary = result['s_binary'] * 255
        combined_binary = result['combined_binary'] * 255
        warped_color = result['warped_color']
        
        # Add polynomial lines to sliding_windows image
        if result['detected']:
            ploty = np.linspace(0, img_height - 1, img_height)
            left_fit = result['left_fit']
            right_fit = result['right_fit']
            left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
            
            pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))], np.int32)
            pts_right = np.array([np.transpose(np.vstack([right_fitx, ploty]))], np.int32)
            cv2.polylines(sliding_windows, pts_left, isClosed=False, color=(0, 255, 255), thickness=4)
            cv2.polylines(sliding_windows, pts_right, isClosed=False, color=(0, 255, 255), thickness=4)
        
        # Save to disk
        base_name = os.path.splitext(img_name)[0]
        cv2.imwrite(os.path.join(output_dir, f"{base_name}_01_roi.jpg"), roi_frame)
        cv2.imwrite(os.path.join(output_dir, f"{base_name}_02_l_channel.jpg"), l_binary)
        cv2.imwrite(os.path.join(output_dir, f"{base_name}_03_s_channel.jpg"), s_binary)
        cv2.imwrite(os.path.join(output_dir, f"{base_name}_04_combined.jpg"), combined_binary)
        cv2.imwrite(os.path.join(output_dir, f"{base_name}_05_binary_masked.jpg"), binary)
        cv2.imwrite(os.path.join(output_dir, f"{base_name}_06_warped_color.jpg"), warped_color)
        cv2.imwrite(os.path.join(output_dir, f"{base_name}_07_warped_binary.jpg"), warped)
        cv2.imwrite(os.path.join(output_dir, f"{base_name}_08_sliding_windows.jpg"), sliding_windows)
        cv2.imwrite(os.path.join(output_dir, f"{base_name}_09_final.jpg"), annotated)
        
        # Display interactively
        cv2.imshow("01 - ROI", cv2.resize(roi_frame, (800, 450)))
        cv2.imshow("02 - L Channel (White)", cv2.resize(l_binary, (800, 450)))
        cv2.imshow("03 - S Channel (Yellow)", cv2.resize(s_binary, (800, 450)))
        cv2.imshow("04 - Combined Binary", cv2.resize(combined_binary, (800, 450)))
        cv2.imshow("05 - Masked Binary", cv2.resize(binary, (800, 450)))
        cv2.imshow("06 - Bird's Eye (Color)", cv2.resize(warped_color, (800, 450)))
        cv2.imshow("07 - Bird's Eye (Binary)", cv2.resize(warped, (800, 450)))
        cv2.imshow("08 - Sliding Windows", cv2.resize(sliding_windows, (800, 450)))
        cv2.imshow("09 - Final Result", cv2.resize(annotated, (800, 450)))
        
        print(f"Press any key to continue to the next image (or 'q' to quit)...")
        key = cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()
        if key == ord('q'):
            break

    print("\nDone!")
