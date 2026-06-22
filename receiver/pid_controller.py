#!/usr/bin/env python3
"""
PID Controller for Smooth Steering
Prevents zigzag motion and provides smooth lane following
"""

import time
import logging

logger = logging.getLogger(__name__)


class PIDController:
    """
    PID controller for smooth steering angle calculation.
    Implements Proportional-Integral-Derivative control with anti-zigzag filtering.
    """
    
    def __init__(self, 
                 kp=10.0,      # Proportional gain
                 ki=0.5,      # Integral gain
                 kd=0.3,      # Derivative gain
                 max_angle=50.0,  # Maximum steering angle (degrees)
                 alpha=0.3):  # Exponential smoothing factor (0-1)
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain (response to current error)
            ki: Integral gain (response to accumulated error)
            kd: Derivative gain (response to rate of change)
            max_angle: Maximum steering angle limit
            alpha: Smoothing factor (lower = smoother, higher = more responsive)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_angle = max_angle
        self.alpha = alpha
        
        # State variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_angle = 0.0
        self.prev_time = None
        
        # Rate limiting
        self.max_angle_change_per_second = 200.0  # Max degrees/second
        
        logger.info(f"PID Controller initialized: Kp={kp}, Ki={ki}, Kd={kd}, alpha={alpha}")
    
    def reset(self):
        """Reset controller state."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_angle = 0.0
        self.prev_time = None
        logger.info("PID Controller reset")
    
    def calculate_steering_angle(self, lane_offset_pixels, 
                                 img_width=1920, 
                                 dt=None):
        """
        Calculate steering angle from lane offset using PID control.
        
        Args:
            lane_offset_pixels: Offset from lane center (negative=left, positive=right)
            img_width: Image width for normalization
            dt: Time delta (seconds), auto-calculated if None
            
        Returns:
            steering_angle: Smooth steering angle in degrees (-max_angle to +max_angle)
        """
        # Calculate dt if not provided
        current_time = time.time()
        if dt is None:
            if self.prev_time is not None:
                dt = current_time - self.prev_time
            else:
                dt = 0.016  # Fallback ~60 FPS
        self.prev_time = current_time
        
        # Normalize error to [-1, 1] range
        # Offset is in pixels, normalize by half image width
        normalized_error = lane_offset_pixels / (img_width / 2.0)
        
        # Clamp error to reasonable range
        normalized_error = max(-1.0, min(1.0, normalized_error))
        
        # PID calculations
        # Proportional term
        p_term = self.kp * normalized_error
        
        # Integral term (with anti-windup)
        self.integral += normalized_error * dt
        # Limit integral to prevent windup
        max_integral = 1.0
        self.integral = max(-max_integral, min(max_integral, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        if dt > 0:
            derivative = (normalized_error - self.prev_error) / dt
        else:
            derivative = 0
        d_term = self.kd * derivative
        
        # Calculate raw PID output
        pid_output = p_term + i_term + d_term
        
        # Convert to steering angle (map [-1, 1] to [-max_angle, max_angle])
        raw_angle = pid_output * self.max_angle
        
        # Apply exponential smoothing
        smooth_angle = self.alpha * raw_angle + (1 - self.alpha) * self.prev_angle
        
        # Apply rate limiting (prevent sudden jumps)
        if dt > 0:
            max_change = self.max_angle_change_per_second * dt
            angle_change = smooth_angle - self.prev_angle
            angle_change = max(-max_change, min(max_change, angle_change))
            final_angle = self.prev_angle + angle_change
        else:
            final_angle = smooth_angle
        
        # Clamp to maximum angle
        final_angle = max(-self.max_angle, min(self.max_angle, final_angle))
        
        # Update state
        self.prev_error = normalized_error
        self.prev_angle = final_angle
        
        return final_angle
    
    def tune_parameters(self, kp=None, ki=None, kd=None, alpha=None):
        """
        Dynamically adjust PID parameters during runtime.
        
        Args:
            kp, ki, kd, alpha: New parameter values (None to keep current)
        """
        if kp is not None:
            self.kp = kp
            logger.info(f"PID: Kp updated to {kp}")
        if ki is not None:
            self.ki = ki
            logger.info(f"PID: Ki updated to {ki}")
        if kd is not None:
            self.kd = kd
            logger.info(f"PID: Kd updated to {kd}")
        if alpha is not None:
            self.alpha = alpha
            logger.info(f"PID: alpha updated to {alpha}")


class AdaptiveSpeedController:
    """
    Adaptive speed controller that adjusts speed based on steering angle.
    Slows down in curves, speeds up on straight roads.
    """
    
    def __init__(self,
                 base_speed=10.0,      # Base cruising speed (%)
                 min_speed=5.0,       # Minimum speed in tight curves (%)
                 max_speed=15.0):      # Maximum speed on straight road (%)
        """
        Initialize adaptive speed controller.
        
        Args:
            base_speed: Normal cruising speed
            min_speed: Minimum speed in curves
            max_speed: Maximum speed on straights
        """
        self.base_speed = base_speed
        self.min_speed = min_speed
        self.max_speed = max_speed
        
        logger.info(f"Adaptive Speed Controller: base={base_speed}%, "
                   f"min={min_speed}%, max={max_speed}%")
    
    def calculate_speed(self, steering_angle, max_angle=50.0):
        """
        Calculate speed based on steering angle.
        
        Args:
            steering_angle: Current steering angle (degrees)
            max_angle: Maximum possible steering angle
            
        Returns:
            speed: Adjusted speed (%)
        """
        # Normalize steering angle to [0, 1]
        normalized_angle = abs(steering_angle) / max_angle
        
        # Speed reduction based on angle
        # straight (0 deg) -> max_speed
        # max turn (50 deg) -> min_speed
        speed = self.max_speed - (self.max_speed - self.min_speed) * normalized_angle
        
        # Ensure within bounds
        speed = max(self.min_speed, min(self.max_speed, speed))
        
        return speed


if __name__ == "__main__":
    # Test PID controller
    logging.basicConfig(level=logging.INFO)
    
    print("=" * 60)
    print("PID Controller Test")
    print("=" * 60)
    
    pid = PIDController(kp=10.0, ki=0.5, kd=0.3, alpha=0.3)
    speed_ctrl = AdaptiveSpeedController()
    
    # Simulate lane following with offset errors
    test_offsets = [100, 150, 120, 80, 50, 20, -10, -30, -20, 0]
    
    print("\nSimulating lane following:")
    print(f"{'Offset (px)':<15} {'Steering (deg)':<20} {'Speed (%)'}")
    print("-" * 55)
    
    for offset in test_offsets:
        angle = pid.calculate_steering_angle(offset, img_width=1920)
        speed = speed_ctrl.calculate_speed(angle)
        print(f"{offset:<15} {angle:<20.2f} {speed:.1f}")
        time.sleep(0.033)  # Simulate 30 FPS
    
    print("\nPID controller working correctly!")