#!/usr/bin/env python3
"""
Hardware Controller for Raspberry Pi Robot Car
- PCA9685 16-Channel PWM Driver (I2C)
- Servo Motor for steering (direct PWM control)
- L298N H-Bridge for 2x DC motors (speed control)
"""

import time
import board
import busio
from adafruit_pca9685 import PCA9685
import logging
from config import CFG

logger = logging.getLogger(__name__)

_hw = CFG["hardware"]
_TRIM_OFFSET    = _hw["trim_offset"]
_PWM_FREQUENCY  = _hw["pwm_frequency"]
_PWM_MIN_PCT    = _hw["pwm_min_percent"]
_PWM_MAX_PCT    = _hw["pwm_max_percent"]



class RobotCarController:
    """
    Controls robot car hardware via PCA9685 PWM driver.

    Hardware Setup:
    - PCA9685 connected to Raspberry Pi via I2C (Seed Studio at 0x7F)
    - Servo motor on channel 0 (steering) - direct PWM control
      Physical range: -90 deg to +90 deg
      PWM: 0.5ms (-90 deg), 1.5ms (0 deg), 2.5ms (+90 deg) at 50Hz
    - L298N Dual H-Bridge connected to channels 1-6 (motor control)
      - Channel 1: Motor A IN1 (forward)
      - Channel 2: Motor A IN2 (backward)
      - Channel 3: Motor A ENA (speed PWM)
      - Channel 4: Motor B IN3 (forward)
      - Channel 5: Motor B IN4 (backward)
      - Channel 6: Motor B ENB (speed PWM)
    """

    def __init__(self,
                 pca9685_address=None,
                 servo_channel=None,
                 motor_a_in1=None, motor_a_in2=None, motor_a_ena=None,
                 motor_b_in3=None, motor_b_in4=None, motor_b_enb=None):
        """
        Initialize hardware controller.
        Valorile implicite sunt citite din config.yaml [hardware].
        Pot fi suprascrise prin argumente explicite (util pentru teste).
        """
        _hw = CFG["hardware"]
        if pca9685_address is None: pca9685_address = _hw["i2c_address"]
        if servo_channel   is None: servo_channel   = _hw["servo_channel"]
        if motor_a_in1     is None: motor_a_in1     = _hw["motor_a_in1"]
        if motor_a_in2     is None: motor_a_in2     = _hw["motor_a_in2"]
        if motor_a_ena     is None: motor_a_ena     = _hw["motor_a_ena"]
        if motor_b_in3     is None: motor_b_in3     = _hw["motor_b_in3"]
        if motor_b_in4     is None: motor_b_in4     = _hw["motor_b_in4"]
        if motor_b_enb     is None: motor_b_enb     = _hw["motor_b_enb"]
        self.enabled = False

        try:
            # Initialize I2C
            i2c = busio.I2C(board.SCL, board.SDA)

            # Initialize PCA9685
            self.pca = PCA9685(i2c, address=pca9685_address)
            self.pca.frequency = _PWM_FREQUENCY

            logger.info(f"PCA9685 initialized at 0x{pca9685_address:02X}, 50Hz")

            # Setup Servo channel
            self.servo_channel = servo_channel

            # Center servo initially (1.5ms pulse)
            self.set_angle(0)
            logger.info(f"Servo initialized on channel {servo_channel}")

            # Setup Motor Channels
            self.motor_a_in1 = motor_a_in1
            self.motor_a_in2 = motor_a_in2
            self.motor_a_ena = motor_a_ena
            self.motor_b_in3 = motor_b_in3
            self.motor_b_in4 = motor_b_in4
            self.motor_b_enb = motor_b_enb

            # Stop motors initially
            self.set_speed(0)
            logger.info(f"Motors initialized (A: IN1={motor_a_in1}, IN2={motor_a_in2}, ENA={motor_a_ena}, "
                        f"B: IN3={motor_b_in3}, IN4={motor_b_in4}, ENB={motor_b_enb})")

            self.enabled = True

        except Exception as e:
            logger.error(f"Failed to initialize hardware: {e}")
            self.enabled = False
            raise

    def set_angle(self, angle: float):
        """
        Set steering angle using direct PWM control.

        Servo specifications:
        - Physical range: -90 deg to +90 deg
        - PWM at 50Hz (20ms period):
          * -90 deg = 0.5ms pulse width
          *   0 deg = 1.5ms pulse width
          * +90 deg = 2.5ms pulse width

        Args:
            angle: Steering angle in degrees (-90 to +90)
                   Internally mapped to -90 deg to +90 deg physical range
        """
        if not self.enabled:
            return

        try:
            # Clamp la inputul primit de la client (-50 la +50)
            clamped_angle = max(-50, min(50, angle))

            # Adăugăm offset-ul constructiv din config.yaml [hardware.trim_offset]
            physical_angle = clamped_angle + _TRIM_OFFSET

            # Convert angle to pulse width (ms)
            pulse_ms = 1.5 + (physical_angle / 90.0) * 1.0

            # Convert pulse width to PWM duty cycle (0-65535)
            # At 50Hz: period = 20ms, PCA9685 uses 16-bit resolution
            duty_cycle = int((pulse_ms / 20.0) * 65535)

            # Set PWM directly on servo channel
            self.pca.channels[self.servo_channel].duty_cycle = duty_cycle

            logger.debug(
                f"Servo: input={angle:.1f}deg -> physical={physical_angle:.1f}deg -> {pulse_ms:.2f}ms -> duty={duty_cycle}")

        except Exception as e:
            logger.error(f"Error setting servo angle: {e}")

    def set_speed(self, speed: float):
        """
        Set motor speed for both motors using L298N.

        Args:
            speed: Speed in percentage (-100 to +100)
                   Positive = forward, Negative = backward, 0 = stop
        """
        if not self.enabled:
            return

        try:
            # Clamp speed to -100 to +100
            speed = max(-100, min(100, speed))

            if speed == 0:
                # Stop both motors
                self._stop_motors()
                logger.debug("Motors stopped")
            elif speed > 0:
                # Forward motion
                self._set_motor_forward(speed)
                logger.debug(f"Motors forward: {speed:.1f}%")
            else:
                # Backward motion
                self._set_motor_backward(abs(speed))
                logger.debug(f"Motors backward: {abs(speed):.1f}%")

        except Exception as e:
            logger.error(f"Error setting motor speed: {e}")

    def _set_motor_forward(self, speed_percent: float):
        """Set both motors to forward direction with PWM speed control."""
        # Convert speed percentage to PWM duty cycle (0-65535)
        # speed_percent vine din joystick (0 la 100)
        # Vrem ca 0% joystick -> 25% putere (minim de mișcare)
        # Vrem ca 100% joystick -> 100% putere (maxim hardware)

        adjusted_speed = _PWM_MIN_PCT + (speed_percent * (_PWM_MAX_PCT - _PWM_MIN_PCT) / 100)

        # Acum calculăm duty_cycle în siguranță
        duty_cycle = int((adjusted_speed / 100.0) * 0xFFFF)

        # Motor A forward: IN1=HIGH, IN2=LOW, ENA=PWM
        self.pca.channels[self.motor_a_in1].duty_cycle = 0xFFFF  # HIGH
        self.pca.channels[self.motor_a_in2].duty_cycle = 0  # LOW
        self.pca.channels[self.motor_a_ena].duty_cycle = duty_cycle  # Speed PWM

        # Motor B forward: IN3=HIGH, IN4=LOW, ENB=PWM
        self.pca.channels[self.motor_b_in3].duty_cycle = 0xFFFF  # HIGH
        self.pca.channels[self.motor_b_in4].duty_cycle = 0  # LOW
        self.pca.channels[self.motor_b_enb].duty_cycle = duty_cycle  # Speed PWM

    def _set_motor_backward(self, speed_percent: float):
        """Set both motors to backward direction with PWM speed control."""
        # Convert speed percentage to PWM duty cycle (0-65535)
        # speed_percent vine din joystick (0 la 100)
        # Vrem ca 0% joystick -> 25% putere (minim de mișcare)
        # Vrem ca 100% joystick -> 100% putere (maxim hardware)

        adjusted_speed = _PWM_MIN_PCT + (speed_percent * (_PWM_MAX_PCT - _PWM_MIN_PCT) / 100)

        # Acum calculăm duty_cycle în siguranță
        duty_cycle = int((adjusted_speed / 100.0) * 0xFFFF)

        # Motor A backward: IN1=LOW, IN2=HIGH, ENA=PWM
        self.pca.channels[self.motor_a_in1].duty_cycle = 0  # LOW
        self.pca.channels[self.motor_a_in2].duty_cycle = 0xFFFF  # HIGH
        self.pca.channels[self.motor_a_ena].duty_cycle = duty_cycle  # Speed PWM

        # Motor B backward: IN3=LOW, IN4=HIGH, ENB=PWM
        self.pca.channels[self.motor_b_in3].duty_cycle = 0  # LOW
        self.pca.channels[self.motor_b_in4].duty_cycle = 0xFFFF  # HIGH
        self.pca.channels[self.motor_b_enb].duty_cycle = duty_cycle  # Speed PWM

    def _stop_motors(self):
        """Stop all motors - set direction LOW and speed to 0."""
        # Motor A stop
        self.pca.channels[self.motor_a_in1].duty_cycle = 0
        self.pca.channels[self.motor_a_in2].duty_cycle = 0
        self.pca.channels[self.motor_a_ena].duty_cycle = 0

        # Motor B stop
        self.pca.channels[self.motor_b_in3].duty_cycle = 0
        self.pca.channels[self.motor_b_in4].duty_cycle = 0
        self.pca.channels[self.motor_b_enb].duty_cycle = 0

    def update(self, angle: float, speed: float):
        """
        Update both servo and motors simultaneously.

        Args:
            angle: Steering angle (-180 to +180)
            speed: Motor speed (-100 to +100)
        """
        self.set_angle(angle)
        self.set_speed(speed)

    def cleanup(self):
        """Cleanup - stop motors and center servo."""
        logger.info("Cleaning up hardware...")
        try:
            self._stop_motors()
            self.pca.channels[self.servo_channel].duty_cycle = 0
            time.sleep(0.5)

            for i in range(16):
                self.pca.channels[i].duty_cycle = 0

            import smbus2
            bus = smbus2.SMBus(1)
            mode1 = bus.read_byte_data(0x7F, 0x00)
            bus.write_byte_data(0x7F, 0x00, mode1 | 0x10)
            bus.close()

            logger.info("All PWM outputs disabled")

            if hasattr(self, 'pca'):
                self.pca.deinit()

        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

        logger.info("Hardware cleanup complete")

    def __del__(self):
        """Destructor - ensure cleanup."""
        if self.enabled:
            self.cleanup()


# Test standalone
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("=" * 60)
    print("Robot Car Hardware Test - L298N with ENA/ENB")
    print("=" * 60)

    try:
        print("\nInitializing hardware...")
        controller = RobotCarController()

        print("\n[Test 1] Center servo (0 deg)")
        controller.set_angle(0)
        time.sleep(2)

        print("\n[Test 2] Turn left (-180 deg)")
        controller.set_angle(-90)
        time.sleep(2)

        print("\n[Test 3] Turn right (180 deg)")
        controller.set_angle(90)
        time.sleep(2)

        print("\n[Test 2] Turn left (-60 deg)")
        controller.set_angle(-60)
        time.sleep(2)

        print("\n[Test 3] Turn right (+60 deg)")
        controller.set_angle(60)
        time.sleep(2)

        print("\n[Test 2] Turn left (-45 deg)")
        controller.set_angle(-40)
        time.sleep(2)

        print("\n[Test 3] Turn right (+45 deg)")
        controller.set_angle(40)
        time.sleep(2)

        print("\n[Test 4] Center servo (0 deg)")
        controller.set_angle(0)
        time.sleep(2)

        """ print("\n[Test 5] Motors forward slow (30%)")
        controller.set_speed(30)
        time.sleep(3)

        print("\n[Test 6] Motors forward fast (70%)")
        controller.set_speed(70)
        time.sleep(3)

        print("\n[Test 7] Stop motors")
        controller.set_speed(0)
        time.sleep(2)

        print("\n[Test 8] Motors backward slow (-30%)")
        controller.set_speed(-30)
        time.sleep(3)

        print("\n[Test 9] Motors backward fast (-70%)")
        controller.set_speed(-70)
        time.sleep(3)

        print("\n[Test 10] Stop motors")
        controller.set_speed(0)
        time.sleep(1)

        print("\n[Test 11] Combined - Turn and drive")
        controller.update(angle=45, speed=50)
        time.sleep(3)                              """

        print("\n[Test 12] Stop and center")
        controller.update(angle=0, speed=0)

        print("\nAll tests passed!")

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest failed: {e}")
        import traceback

        traceback.print_exc()
    finally:
        if 'controller' in locals():
            controller.cleanup()
        print("\nTest complete!")