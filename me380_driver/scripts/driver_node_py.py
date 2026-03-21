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

Servos: GPIO 12/13/18 use hardware PWM via pigpio for stable timing and less jitter.
Ensure the daemon is running: sudo pigpiod

Joint mapping (JointState.position array only; names are ignored):
  position[0..3] -> steppers J1..J4 (rad; driver uses delta from last)
  position[4], position[5] -> differential servo angles (rad)
  position[6] -> gripper (0 = open, >0.5 = closed)
  This robot always publishes 7 values in JointState.position.
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import pigpio


RAD_TO_DEG = 180.0 / math.pi

# From esp32_driver.cpp
STEPS_PER_REV = 200.0 * 16.0  # 200 steps/rev, 16 microstepping
GEAR_RATIO_J1 = 5.0  #base
GEAR_RATIO_J2 = 56.25  #shoulder
GEAR_RATIO_J3 = 34.0  #elbow
GEAR_RATIO_J4 = 20.0 #wrist?

SERVO_GEAR_REDUCTION = 3.0 / 4.0
PWM_CENTER = 1500
PWM_MIN = 500
PWM_MAX = 2500
SERVO_RANGE_DEG = 135.0
US_PER_DEG = 2000.0 / 270.0
EE_OPEN_DEG = 155.0
EE_CLOSE_DEG = 100.0

STEP_PULSE_US = 5
STEP_PULSE_S = STEP_PULSE_US / 1e6
# Match ik.cpp (ESP32/Arduino): 20 ms update, step toward target at 400 us/s. Hardware timers there give stable PWM; we match the logic.
SERVO_SPEED_US_PER_SEC = 400.0
SERVO_UPDATE_PERIOD_S = 0.02

GPIO_PARAMS = {
    "gpio_enable": 8,
    "gpio_j1_step": 17,
    "gpio_j1_dir": 27,
    "gpio_j2_step": 22,
    "gpio_j2_dir": 23,
    "gpio_j3_step": 24,
    "gpio_j3_dir": 25,
    "gpio_servo_a": 12,
    "gpio_servo_b": 13,
    "gpio_servo_ee": 18,
}
def _angle_deg_to_pulse_us(angle_deg: float) -> float:
    """Map servo angle in degrees (0–180) to pulse width us (500–2500), like Arduino Servo library."""
    return 500.0 + angle_deg * (2000.0 / 180.0)


class DriverNode(Node):
    def __init__(self):
        super().__init__("driver_node_py")

        for name, default in GPIO_PARAMS.items():
            self.declare_parameter(name, default)
        for name, default in {
            "joint_state_topic": "joint_states",
            "max_stepper_delta_per_msg": 360.0,
            "max_steps_per_sec": 2000.0,
            "gear_ratio_j1": GEAR_RATIO_J1,
            "gear_ratio_j2": GEAR_RATIO_J2,
            "gear_ratio_j3": GEAR_RATIO_J3,
            "gear_ratio_j4": GEAR_RATIO_J4,
        }.items():
            self.declare_parameter(name, default)

        p = lambda name: self.get_parameter(name).value
        self.gpio_enable = p("gpio_enable")
        self.step_pins = [p("gpio_j1_step"), p("gpio_j2_step"), p("gpio_j3_step"), p("gpio_j4_step")]
        self.dir_pins = [p("gpio_j1_dir"), p("gpio_j2_dir"), p("gpio_j3_dir"), p("gpio_j4_dir")]
        self.gpio_servo_a, self.gpio_servo_b, self.gpio_servo_ee = p("gpio_servo_a"), p("gpio_servo_b"), p("gpio_servo_ee")
        self.joint_state_topic = p("joint_state_topic")
        self.max_stepper_delta = p("max_stepper_delta_per_msg")
        self.max_steps_per_sec = p("max_steps_per_sec")
        self.gear_ratios = [p("gear_ratio_j1"), p("gear_ratio_j2"), p("gear_ratio_j3"), p("gear_ratio_j4")]

        self._pending_steps = [0, 0, 0, 0]
        self._last_step_time = [0.0, 0.0, 0.0, 0.0]
        self._step_lock = threading.Lock()
        self._target_pwm_a = PWM_CENTER
        self._target_pwm_b = PWM_CENTER
        self._current_pwm_a = float(PWM_CENTER)
        self._current_pwm_b = float(PWM_CENTER)
        self._last_written_pwm_a = PWM_CENTER
        self._last_written_pwm_b = PWM_CENTER
        self._last_servo_update = 0.0
        self._ee_closed = False
        self._last_j_deg = [0.0, 0.0, 0.0, 0.0]

        self._pi = pigpio.pi()
        self._stepper_thread = None
        self._servo_timer = None
        self._shutdown = False

        if not self._pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon. Start it with: sudo pigpiod")
        self.get_logger().info("Using pigpio hardware PWM for servos (GPIO 12, 13, 18).")


        self._setup_gpio()
        self._start_stepper_thread()
        self._start_servo_update_timer()

        self.sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            10,
        )


    def _setup_gpio(self):
        for p in [self.gpio_enable] + self.step_pins + self.dir_pins:
            self._pi.set_mode(p, pigpio.OUTPUT)
            self._pi.write(p, 0)
        self._pi.write(self.gpio_enable, 0)
        self._pi.set_servo_pulsewidth(self.gpio_servo_a, PWM_CENTER)
        self._pi.set_servo_pulsewidth(self.gpio_servo_b, PWM_CENTER)
        self._pi.set_servo_pulsewidth(self.gpio_servo_ee, int(_angle_deg_to_pulse_us(EE_OPEN_DEG)))

    def _steps_for_delta_deg(self, axis: int, delta_deg: float) -> int:
        return int(round(delta_deg * STEPS_PER_REV * self.gear_ratios[axis] / 360.0))

    def _do_one_step(self, axis: int, direction: int):
        self._pi.write(self.dir_pins[axis], 1 if direction > 0 else 0)
        self._pi.write(self.step_pins[axis], 1)
        time.sleep(STEP_PULSE_S)
        self._pi.write(self.step_pins[axis], 0)

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

    def _step_toward(self, current: float, target: float, step: float) -> float:
        """Match ik.cpp stepToward: move current toward target by at most step."""
        if abs(target - current) <= step:
            return target
        return current + (step if target > current else -step)

    def _servo_tick(self):
        """Match ik.cpp updateServos(): every UPDATE_PERIOD_S, step current toward target and write integer us (stable values)."""
        if not self._pi.connected:
            return
        now = time.monotonic()
        if now - self._last_servo_update < SERVO_UPDATE_PERIOD_S:
            return
        self._last_servo_update = now
        step_us = SERVO_SPEED_US_PER_SEC * SERVO_UPDATE_PERIOD_S
        self._current_pwm_a = self._step_toward(self._current_pwm_a, self._target_pwm_a, step_us)
        self._current_pwm_b = self._step_toward(self._current_pwm_b, self._target_pwm_b, step_us)
        cur_a = int(round(self._current_pwm_a))
        cur_b = int(round(self._current_pwm_b))
        if cur_a != self._last_written_pwm_a:
            self._last_written_pwm_a = cur_a
            self._pi.set_servo_pulsewidth(self.gpio_servo_a, cur_a)
        if cur_b != self._last_written_pwm_b:
            self._last_written_pwm_b = cur_b
            self._pi.set_servo_pulsewidth(self.gpio_servo_b, cur_b)

    def _start_servo_update_timer(self):
        self._servo_timer = self.create_timer(SERVO_UPDATE_PERIOD_S, self._servo_tick)

    def _move_differential(self, angle4_deg: float, angle5_deg: float):
        """Set target PWM for differential servos (ik.cpp: targetPwmA/B). Smoothing happens in _servo_tick."""
        angle_a = SERVO_GEAR_REDUCTION * (angle4_deg + angle5_deg)
        angle_b = SERVO_GEAR_REDUCTION * (-angle4_deg + angle5_deg)
        angle_a = max(-SERVO_RANGE_DEG, min(SERVO_RANGE_DEG, angle_a))
        angle_b = max(-SERVO_RANGE_DEG, min(SERVO_RANGE_DEG, angle_b))
        self._target_pwm_a = int(max(PWM_MIN, min(PWM_MAX, PWM_CENTER + angle_a * US_PER_DEG)))
        self._target_pwm_b = int(max(PWM_MIN, min(PWM_MAX, PWM_CENTER + angle_b * US_PER_DEG)))

    def _on_joint_state(self, msg):
        position = list(msg.position)
        if len(position) < 7:
            self.get_logger().warn_throttle(
                1.0, "JointState position must have 7 elements (j1..j4 + diff servos + gripper)"
            )
            return

        j1_rad, j2_rad, j3_rad, j4_rad, diff_a_rad, diff_b_rad, gripper_rad = position[:7]
        j_deg = [x * RAD_TO_DEG for x in (j1_rad, j2_rad, j3_rad, j4_rad)]
        diff_a_deg = diff_a_rad * RAD_TO_DEG
        diff_b_deg = diff_b_rad * RAD_TO_DEG

        prev_j_deg = self._last_j_deg
        d1, d2, d3, d4 = [cur - last for cur, last in zip(j_deg, prev_j_deg)]
        self._last_j_deg = j_deg

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

        # Differential servos: set targets; _servo_tick steps toward them (same logic as ik.cpp)
        self._move_differential(diff_a_deg, diff_b_deg)

        # End effector
        closed = gripper_rad > 0.5
        if closed != self._ee_closed:
            self._ee_closed = closed
            angle = EE_CLOSE_DEG if closed else EE_OPEN_DEG
            pulse_us = int(max(PWM_MIN, min(PWM_MAX, _angle_deg_to_pulse_us(angle))))
            self._pi.set_servo_pulsewidth(self.gpio_servo_ee, pulse_us)

    def destroy_node(self, *args, **kwargs):
        self._shutdown = True
        if self._servo_timer is not None:
            self._servo_timer.cancel()
            self._servo_timer = None
        if self._stepper_thread is not None:
            self._stepper_thread.join(timeout=1.0)
        self._pi.set_servo_pulsewidth(self.gpio_servo_a, 0)
        self._pi.set_servo_pulsewidth(self.gpio_servo_b, 0)
        self._pi.set_servo_pulsewidth(self.gpio_servo_ee, 0)
        self._pi.stop()
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
