#!/usr/bin/env python3
"""
ROS2 driver node for ME380 robot: Raspberry Pi GPIO directly drives a CNC Shield V3
(no Arduino). Generates step/dir for 4 steppers and PWM for 3 servos.

Wiring: Pi GPIO (BCM) -> CNC Shield (Arduino Uno digital pin)
  Enable     Pi GPIO  8 -> Shield D8  (LOW = drivers enabled)
  X (J1)     Step 17 -> D2,  Dir 27 -> D5
  Y (J2)     Step 22 -> D3,  Dir 23 -> D6
  Z (J3)     Step 24 -> D4,  Dir 25 -> D7
  A (J4)     Step  5 -> D12, Dir  6 -> D13
  Servo A    GPIO 12 -> D9   (X- endstop)
  Servo B    GPIO 13 -> D10  (Y- endstop)
  End Effector GPIO 18 -> D11 (Z- endstop)

Use 3.3V-to-5V level shifters between Pi and shield for reliable operation.

Joint mapping (JointState.position array only; names are ignored):
  position[0..3] -> stepper J1..J4 (rad; driver uses delta from last)
  position[4], position[5] -> differential servo angles (rad)
  position[6] -> gripper (0 = open, >0.5 = closed)
  For 6-joint: position[0..2] steppers, position[3], position[4] diff, position[5] gripper.
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


RAD_TO_DEG = 180.0 / math.pi

# From esp32_driver.cpp
STEPS_PER_REV = 200.0 * 16.0  # 200 steps/rev, 16 microstepping
GEAR_RATIO_J1 = 2.8
GEAR_RATIO_J2 = 48.0
GEAR_RATIO_J3 = 40.0
GEAR_RATIO_J4 = 40.0  # 4th axis, match J3 if same motor type

SERVO_GEAR_REDUCTION = 3.0 / 4.0
PWM_CENTER = 1500
PWM_MIN = 500
PWM_MAX = 2500
SERVO_RANGE_DEG = 135.0
US_PER_DEG = 2000.0 / 270.0
SERVO_SMOOTH_US_PER_SEC = 400.0
SERVO_UPDATE_PERIOD_S = 0.02
EE_OPEN_DEG = 155.0
EE_CLOSE_DEG = 100.0

SERVO_HZ = 50
STEP_PULSE_US = 5
STEP_PULSE_S = STEP_PULSE_US / 1e6


def _pulse_us_to_duty(pulse_us: float) -> float:
    """Convert pulse width in us to duty cycle percent for 50 Hz (20 ms period)."""
    return (pulse_us / 20000.0) * 100.0


def _angle_deg_to_pulse_us(angle_deg: float) -> float:
    """Map servo angle in degrees (0–180) to pulse width us (500–2500), like Arduino Servo library."""
    return 500.0 + angle_deg * (2000.0 / 180.0)


class DriverNode(Node):
    def __init__(self):
        super().__init__("driver_node_py")

        # --- Parameters: GPIO pins (BCM) ---
        self.declare_parameter("gpio_enable", 8)
        self.declare_parameter("gpio_j1_step", 17)
        self.declare_parameter("gpio_j1_dir", 27)
        self.declare_parameter("gpio_j2_step", 22)
        self.declare_parameter("gpio_j2_dir", 23)
        self.declare_parameter("gpio_j3_step", 24)
        self.declare_parameter("gpio_j3_dir", 25)
        self.declare_parameter("gpio_j4_step", 5)
        self.declare_parameter("gpio_j4_dir", 6)
        self.declare_parameter("gpio_servo_a", 12)
        self.declare_parameter("gpio_servo_b", 13)
        self.declare_parameter("gpio_servo_ee", 18)

        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("max_stepper_delta_per_msg", 360.0)
        self.declare_parameter("max_steps_per_sec", 2000.0)
        self.declare_parameter("gear_ratio_j1", GEAR_RATIO_J1)
        self.declare_parameter("gear_ratio_j2", GEAR_RATIO_J2)
        self.declare_parameter("gear_ratio_j3", GEAR_RATIO_J3)
        self.declare_parameter("gear_ratio_j4", GEAR_RATIO_J4)

        self.gpio_enable = self.get_parameter("gpio_enable").value
        self.gpio_j1_step = self.get_parameter("gpio_j1_step").value
        self.gpio_j1_dir = self.get_parameter("gpio_j1_dir").value
        self.gpio_j2_step = self.get_parameter("gpio_j2_step").value
        self.gpio_j2_dir = self.get_parameter("gpio_j2_dir").value
        self.gpio_j3_step = self.get_parameter("gpio_j3_step").value
        self.gpio_j3_dir = self.get_parameter("gpio_j3_dir").value
        self.gpio_j4_step = self.get_parameter("gpio_j4_step").value
        self.gpio_j4_dir = self.get_parameter("gpio_j4_dir").value
        self.gpio_servo_a = self.get_parameter("gpio_servo_a").value
        self.gpio_servo_b = self.get_parameter("gpio_servo_b").value
        self.gpio_servo_ee = self.get_parameter("gpio_servo_ee").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.max_stepper_delta = self.get_parameter("max_stepper_delta_per_msg").value
        self.max_steps_per_sec = self.get_parameter("max_steps_per_sec").value
        self.gr1 = self.get_parameter("gear_ratio_j1").value
        self.gr2 = self.get_parameter("gear_ratio_j2").value
        self.gr3 = self.get_parameter("gear_ratio_j3").value
        self.gr4 = self.get_parameter("gear_ratio_j4").value

        self.step_pins = [self.gpio_j1_step, self.gpio_j2_step, self.gpio_j3_step, self.gpio_j4_step]
        self.dir_pins = [self.gpio_j1_dir, self.gpio_j2_dir, self.gpio_j3_dir, self.gpio_j4_dir]
        self.gear_ratios = [self.gr1, self.gr2, self.gr3, self.gr4]

        self._pending_steps = [0, 0, 0, 0]
        self._last_step_time = [0.0, 0.0, 0.0, 0.0]
        self._step_lock = threading.Lock()
        self._target_pwm_a = PWM_CENTER
        self._target_pwm_b = PWM_CENTER
        self._current_pwm_a = PWM_CENTER
        self._current_pwm_b = PWM_CENTER
        self._last_servo_update = 0.0
        self._ee_closed = False
        self._last_j_deg = [0.0, 0.0, 0.0, 0.0]

        self._pwma = None
        self._pwmb = None
        self._pwmee = None
        self._stepper_thread = None
        self._servo_timer = None
        self._shutdown = False

        if GPIO is None:
            self.get_logger().error("RPi.GPIO not available. Install with: pip install RPi.GPIO")
            return

        self._setup_gpio()
        self._start_stepper_thread()
        self._start_servo_smoothing()

        self.sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            10,
        )
        self.get_logger().info(
            "GPIO driver: subscribed to %s; step/dir and PWM on Pi BCM pins" % self.joint_state_topic
        )

    def _setup_gpio(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        all_pins = (
            [self.gpio_enable]
            + self.step_pins
            + self.dir_pins
            + [self.gpio_servo_a, self.gpio_servo_b, self.gpio_servo_ee]
        )
        for p in all_pins:
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
        # Enable drivers (LOW = enabled on most CNC shields)
        GPIO.output(self.gpio_enable, GPIO.LOW)

        # Servo PWM at 50 Hz
        self._pwma = GPIO.PWM(self.gpio_servo_a, SERVO_HZ)
        self._pwmb = GPIO.PWM(self.gpio_servo_b, SERVO_HZ)
        self._pwmee = GPIO.PWM(self.gpio_servo_ee, SERVO_HZ)
        self._pwma.start(_pulse_us_to_duty(PWM_CENTER))
        self._pwmb.start(_pulse_us_to_duty(PWM_CENTER))
        self._pwmee.start(_pulse_us_to_duty(_angle_deg_to_pulse_us(EE_OPEN_DEG)))  # open

    def _steps_for_delta_deg(self, axis: int, delta_deg: float) -> int:
        return int(round(delta_deg * STEPS_PER_REV * self.gear_ratios[axis] / 360.0))

    def _do_one_step(self, axis: int, direction: int):
        GPIO.output(self.dir_pins[axis], GPIO.HIGH if direction > 0 else GPIO.LOW)
        GPIO.output(self.step_pins[axis], GPIO.HIGH)
        time.sleep(STEP_PULSE_S)
        GPIO.output(self.step_pins[axis], GPIO.LOW)

    def _stepper_loop(self):
        min_interval = 1.0 / self.max_steps_per_sec
        while not self._shutdown:
            now = time.monotonic()
            with self._step_lock:
                for axis in range(4):
                    if self._pending_steps[axis] == 0:
                        continue
                    if now - self._last_step_time[axis] < min_interval:
                        continue
                    direction = 1 if self._pending_steps[axis] > 0 else -1
                    self._do_one_step(axis, direction)
                    self._pending_steps[axis] -= direction
                    self._last_step_time[axis] = now
            time.sleep(0.0001)

    def _start_stepper_thread(self):
        self._stepper_thread = threading.Thread(target=self._stepper_loop, daemon=True)
        self._stepper_thread.start()

    def _move_differential(self, angle4_deg: float, angle5_deg: float):
        angle_a = SERVO_GEAR_REDUCTION * (angle4_deg + angle5_deg)
        angle_b = SERVO_GEAR_REDUCTION * (-angle4_deg + angle5_deg)
        angle_a = max(-SERVO_RANGE_DEG, min(SERVO_RANGE_DEG, angle_a))
        angle_b = max(-SERVO_RANGE_DEG, min(SERVO_RANGE_DEG, angle_b))
        self._target_pwm_a = int(max(PWM_MIN, min(PWM_MAX, PWM_CENTER + angle_a * US_PER_DEG)))
        self._target_pwm_b = int(max(PWM_MIN, min(PWM_MAX, PWM_CENTER + angle_b * US_PER_DEG)))

    def _step_toward(self, current: float, target: float, step: float) -> float:
        if abs(target - current) <= step:
            return target
        return current + (step if target > current else -step)

    def _servo_tick(self):
        if self._pwma is None:
            return
        now = time.monotonic()
        if now - self._last_servo_update < SERVO_UPDATE_PERIOD_S:
            return
        self._last_servo_update = now
        step_us = SERVO_SMOOTH_US_PER_SEC * SERVO_UPDATE_PERIOD_S
        self._current_pwm_a = self._step_toward(self._current_pwm_a, self._target_pwm_a, step_us)
        self._current_pwm_b = self._step_toward(self._current_pwm_b, self._target_pwm_b, step_us)
        self._pwma.ChangeDutyCycle(_pulse_us_to_duty(self._current_pwm_a))
        self._pwmb.ChangeDutyCycle(_pulse_us_to_duty(self._current_pwm_b))

    def _start_servo_smoothing(self):
        self._servo_timer = self.create_timer(SERVO_UPDATE_PERIOD_S, self._servo_tick)

    def _on_joint_state(self, msg):
        position = list(msg.position) if msg.position else []
        # Expect position only: [j1, j2, j3, j4, j5, j6, gripper] (7) or [j1, j2, j3, j4, j5, gripper] (6). Names ignored.
        if len(position) < 6:
            self.get_logger().warn_throttle(
                1.0, "JointState position must have at least 6 elements (j1..j5 + gripper)"
            )
            return

        if len(position) >= 7:
            # 7-joint: [j1, j2, j3, j4, j5, j6, gripper]
            j1_rad = position[0]
            j2_rad = position[1]
            j3_rad = position[2]
            j4_stepper_rad = position[3]
            diff_a_rad = position[4]
            diff_b_rad = position[5]
            gripper_rad = position[6]
        else:
            # 6-joint: [j1, j2, j3, j4, j5, gripper] — j4,j5 are diff servos, no 4th stepper
            j1_rad = position[0]
            j2_rad = position[1]
            j3_rad = position[2]
            j4_stepper_rad = 0.0
            diff_a_rad = position[3]
            diff_b_rad = position[4]
            gripper_rad = position[5]

        j1_deg = j1_rad * RAD_TO_DEG
        j2_deg = j2_rad * RAD_TO_DEG
        j3_deg = j3_rad * RAD_TO_DEG
        j4_stepper_deg = j4_stepper_rad * RAD_TO_DEG
        diff_a_deg = diff_a_rad * RAD_TO_DEG
        diff_b_deg = diff_b_rad * RAD_TO_DEG

        # Stepper deltas (joint1..4; in 6-joint mode 4th is 0)
        d1 = j1_deg - self._last_j_deg[0]
        d2 = j2_deg - self._last_j_deg[1]
        d3 = j3_deg - self._last_j_deg[2]
        d4 = j4_stepper_deg - self._last_j_deg[3]
        self._last_j_deg[0] = j1_deg
        self._last_j_deg[1] = j2_deg
        self._last_j_deg[2] = j3_deg
        self._last_j_deg[3] = j4_stepper_deg

        cap = self.max_stepper_delta
        d1 = max(-cap, min(cap, d1))
        d2 = max(-cap, min(cap, d2))
        d3 = max(-cap, min(cap, d3))
        d4 = max(-cap, min(cap, d4))

        with self._step_lock:
            self._pending_steps[0] += self._steps_for_delta_deg(0, d1)
            self._pending_steps[1] += self._steps_for_delta_deg(1, d2)
            self._pending_steps[2] += self._steps_for_delta_deg(2, d3)
            self._pending_steps[3] += self._steps_for_delta_deg(3, d4)

        # Differential servos
        self._move_differential(diff_a_deg, diff_b_deg)

        # End effector
        closed = gripper_rad > 0.5
        if closed != self._ee_closed:
            self._ee_closed = closed
            if self._pwmee is not None:
                angle = EE_CLOSE_DEG if closed else EE_OPEN_DEG
                pulse_us = max(PWM_MIN, min(PWM_MAX, _angle_deg_to_pulse_us(angle)))
                self._pwmee.ChangeDutyCycle(_pulse_us_to_duty(pulse_us))

    def destroy_node(self, *args, **kwargs):
        self._shutdown = True
        if self._stepper_thread is not None:
            self._stepper_thread.join(timeout=1.0)
        if self._pwma is not None:
            self._pwma.stop()
        if self._pwmb is not None:
            self._pwmb.stop()
        if self._pwmee is not None:
            self._pwmee.stop()
        if GPIO is not None:
            GPIO.cleanup()
        super().destroy_node(*args, **kwargs)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
