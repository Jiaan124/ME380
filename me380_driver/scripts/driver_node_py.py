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
  position[0..3] -> steppers J1..J4 (rad; driver uses delta from last).
  J2 (Y) and J4 (A) step direction is inverted in software vs J1/J3 to match wiring.
  position[4], position[5] -> differential pair (joint 5 & 6, rad); joint 6 only is sign-flipped in software.
  position[6] -> gripper (0 = open, >0.5 = closed)
  This robot always publishes 7 values in JointState.position.

Tune GPIO, gear ratios, topics, stepper limits, and joint velocities via module constants below
(no ROS parameters).
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import pigpio


RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0

JOINT_FEEDBACK_PERIOD_S = 0.02  # 50 Hz

# --- BCM GPIO (CNC shield) ---
GPIO_ENABLE = 8
GPIO_J1_STEP, GPIO_J1_DIR = 17, 27
GPIO_J2_STEP, GPIO_J2_DIR = 22, 23
GPIO_J3_STEP, GPIO_J3_DIR = 24, 25
GPIO_J4_STEP, GPIO_J4_DIR = 5, 6
GPIO_SERVO_A, GPIO_SERVO_B, GPIO_SERVO_EE = 12, 13, 18

# --- Gear ratios (motor revolutions per joint revolution), J1..J4 ---
GEAR_RATIO_J1 = 5.0
GEAR_RATIO_J2 = 58.5
GEAR_RATIO_J3 = 52.0
GEAR_RATIO_J4 = 4.0



# --- ROS topics ---
JOINT_STATE_TOPIC = "joint_states"
JOINT_FEEDBACK_TOPIC = "actual_joint_position"

# --- Stepper command limits ---
MAX_STEPPER_DELTA_PER_MSG = 360.0  # max |Δθ| (deg) per JointState message per stepper axis
MOTOR_FULL_STEP_DEG = 1.8
STEPPER_MICROSTEPPING = 16
STEPPER_MICROSTEPPING_LIST = [16, 16, 16, 16]
GEAR_RATIO = [GEAR_RATIO_J1, GEAR_RATIO_J2, GEAR_RATIO_J3, GEAR_RATIO_J4]

# Max |joint velocity| at output (rad/s): [J1..J4 steppers, J5 diff, J6 diff, unused index 6]
JOINT_MAX_VELOCITY_RAD_S = [0.5, 1, 1, 0.5, 0.5, 0.5, 1.0]

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
# Per-axis sign for step queue + feedback: +1 default, -1 = joint angle vs motor reversed (J2=Y, J4=A).
STEPPER_JOINT_DIR_SIGN = (1, -1, 1, -1)
DIFF_JOINT6_SIGN = -1
# ESP32/Arduino-style servo timing: 20 ms update, ramp toward target at 400 us/s (stable integer PWM writes).
SERVO_SPEED_US_PER_SEC = 400.0
SERVO_UPDATE_PERIOD_S = 0.02


_JOINT_MAX_VEL_RAD_S_CEIL = 50.0
_SERVO_PWM_US_PER_SEC_FLOOR = 50.0  # minimum PWM slew when wrist ω limits are tiny


class DriverNode(Node):
    def __init__(self):
        super().__init__("driver_node_py")

        self.gpio_enable = GPIO_ENABLE
        self.step_pins = [GPIO_J1_STEP, GPIO_J2_STEP, GPIO_J3_STEP, GPIO_J4_STEP]
        self.dir_pins = [GPIO_J1_DIR, GPIO_J2_DIR, GPIO_J3_DIR, GPIO_J4_DIR]
        self.gpio_servo_a = GPIO_SERVO_A
        self.gpio_servo_b = GPIO_SERVO_B
        self.gpio_servo_ee = GPIO_SERVO_EE
        self.joint_state_topic = JOINT_STATE_TOPIC
        self.max_stepper_delta = max(0.01, min(3600.0, float(MAX_STEPPER_DELTA_PER_MSG)))
        self.gear_ratios = [GEAR_RATIO_J1, GEAR_RATIO_J2, GEAR_RATIO_J3, GEAR_RATIO_J4]
        self._joint_feedback_topic = JOINT_FEEDBACK_TOPIC

        self._motor_full_step_deg = float(MOTOR_FULL_STEP_DEG)
        self._stepper_microstepping = STEPPER_MICROSTEPPING
        self._stepper_microstepping_list = STEPPER_MICROSTEPPING_LIST
        self._steps_per_motor_rev = 200* self._stepper_microstepping #default step/rev
        self._steps_per_motor_rev_list = 200* self._stepper_microstepping_list #default step/rev


        self._joint_max_vel_rad_s = JOINT_MAX_VELOCITY_RAD_S

        self._feedback_lock = threading.Lock()
        self._joint_feedback_rad = [0.0] * 7
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
        self._current_pwm_ee = float(500.0 + EE_OPEN_DEG * (2000.0 / 180.0))

        self._pi = pigpio.pi()
        self._stepper_thread = None
        self._servo_timer = None
        self._shutdown = False

        if not self._pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon. Start it with: sudo pigpiod")
        self.get_logger().info("Using pigpio hardware PWM for servos (GPIO 12, 13, 18).")

        self._setup_gpio()
        self._sync_servo_joints_feedback_from_pwm()
        self._start_stepper_thread()
        self._start_servo_update_timer()

        self._feedback_pub = self.create_publisher(JointState, self._joint_feedback_topic, 10)
        self._feedback_timer = self.create_timer(JOINT_FEEDBACK_PERIOD_S, self._publish_joint_feedback)
        self.get_logger().info(
            "Publishing open-loop joint feedback at %.0f Hz on '%s'"
            % (1.0 / JOINT_FEEDBACK_PERIOD_S, self._joint_feedback_topic)
        )

        self._zero_steppers_srv = self.create_service(
            Trigger,
            "zero_steppers",
            self._zero_steppers_callback,
        )
        self.get_logger().info("Service ready: zero_steppers (std_srvs/srv/Trigger)")

        self.sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            10,
        )

    def _servo_pwm_step_us_per_tick(self) -> float:
        """Max change in PWM (µs) per servo tick from joint 5/6 velocity limits (rad/s)."""
        w4 = self._joint_max_vel_rad_s[4]
        w5 = self._joint_max_vel_rad_s[5]
        # Coupled diff: pwm change rate ~ US_PER_DEG * k * (|dq4/dt|+|dq5/dt|) in deg/s
        pwm_us_per_sec = (
            US_PER_DEG * SERVO_GEAR_REDUCTION * RAD_TO_DEG * (abs(w4) + abs(w5))
        )
        pwm_us_per_sec = max(pwm_us_per_sec, _SERVO_PWM_US_PER_SEC_FLOOR)
        return pwm_us_per_sec * SERVO_UPDATE_PERIOD_S

    def _zero_steppers_callback(self, request, response):
        del request
        with self._feedback_lock:
            self._joint_feedback_rad = [0.0] * 7
        response.success = True
        response.message = "joint feedback estimate zeroed (7 floats)"
        self.get_logger().info("zero_steppers: %s" % response.message)
        return response

    def _setup_gpio(self):
        for pin in [self.gpio_enable] + self.step_pins + self.dir_pins:
            self._pi.set_mode(pin, pigpio.OUTPUT)
            self._pi.write(pin, 0)
        self._pi.write(self.gpio_enable, 0)
        self._pi.set_servo_pulsewidth(self.gpio_servo_a, PWM_CENTER)
        self._pi.set_servo_pulsewidth(self.gpio_servo_b, PWM_CENTER)
        self._pi.set_servo_pulsewidth(
            self.gpio_servo_ee, int(500.0 + EE_OPEN_DEG * (2000.0 / 180.0))
        )

    def _steps_for_delta_deg(self, axis: int, delta_deg: float) -> int:
        return int(
            round(delta_deg * self._steps_per_motor_rev * self.gear_ratios[axis] / 360.0)
        )

    def _sync_servo_joints_feedback_from_pwm(self) -> None:
        pa, pb = self._current_pwm_a, self._current_pwm_b
        angle_a = (pa - PWM_CENTER) / US_PER_DEG
        angle_b = (pb - PWM_CENTER) / US_PER_DEG
        k = SERVO_GEAR_REDUCTION
        if abs(k) < 1e-9:
            d4_deg, d5_deg = 0.0, 0.0
        else:
            d4_deg = (angle_a - angle_b) / (2.0 * k)
            d5_deg = (angle_a + angle_b) / (2.0 * k)
        with self._feedback_lock:
            self._joint_feedback_rad[4] = d4_deg * DEG_TO_RAD
            self._joint_feedback_rad[5] = DIFF_JOINT6_SIGN * d5_deg * DEG_TO_RAD

    def _stepper_loop(self):
        """Background thread: stepping, GPIO pulse, joint-angle feedback update."""
        while not self._shutdown:
            now = time.monotonic()
            with self._step_lock:
                for axis in range(4):
                    if self._pending_steps[axis] == 0:
                        continue
                    omega = self._joint_max_vel_rad_s[axis]
                    rate = omega * (
                        self._steps_per_motor_rev_list[axis]
                        * self.gear_ratios[axis]
                        / (2.0 * math.pi)
                    )
                    # min_interval = 1.0 / rate
                    min_interval = 0.0002
                    if now - self._last_step_time[axis] < min_interval:
                        continue
                    direction = 1 if self._pending_steps[axis] > 0 else -1
                    self._pi.write(self.dir_pins[axis], 1 if direction > 0 else 0)
                    self._pi.write(self.step_pins[axis], 1)
                    time.sleep(STEP_PULSE_S)
                    self._pi.write(self.step_pins[axis], 0)
                    self._pending_steps[axis] -= direction
                    self._last_step_time[axis] = now
                    gr = self.gear_ratios[axis]
                    joint_deg_per_step = 360.0 / (self._steps_per_motor_rev_list[axis] * gr)
                    dtheta = (
                        direction
                        * joint_deg_per_step
                        * DEG_TO_RAD
                        * STEPPER_JOINT_DIR_SIGN[axis]
                    )
                    with self._feedback_lock:
                        self._joint_feedback_rad[axis] += dtheta
            # time.sleep(0.000001)

    def _start_stepper_thread(self):
        self._stepper_thread = threading.Thread(target=self._stepper_loop, daemon=True)
        self._stepper_thread.start()

    def _step_toward(self, current: float, target: float, step: float) -> float:
        if abs(target - current) <= step:
            return target
        return current + (step if target > current else -step)

    def _servo_tick(self):
        if not self._pi.connected:
            return
        now = time.monotonic()
        if now - self._last_servo_update < SERVO_UPDATE_PERIOD_S:
            return
        self._last_servo_update = now
        step_us = self._servo_pwm_step_us_per_tick()
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
        self._sync_servo_joints_feedback_from_pwm()

    def _start_servo_update_timer(self):
        self._servo_timer = self.create_timer(SERVO_UPDATE_PERIOD_S, self._servo_tick)

    def _move_differential(self, angle4_deg: float, angle5_deg: float):
        angle_a = SERVO_GEAR_REDUCTION * (angle4_deg + angle5_deg)
        angle_b = SERVO_GEAR_REDUCTION * (-angle4_deg + angle5_deg)
        angle_a = max(-SERVO_RANGE_DEG, min(SERVO_RANGE_DEG, angle_a))
        angle_b = max(-SERVO_RANGE_DEG, min(SERVO_RANGE_DEG, angle_b))
        self._target_pwm_a = int(max(PWM_MIN, min(PWM_MAX, PWM_CENTER + angle_a * US_PER_DEG)))
        self._target_pwm_b = int(max(PWM_MIN, min(PWM_MAX, PWM_CENTER + angle_b * US_PER_DEG)))

    def _publish_joint_feedback(self) -> None:
        with self._feedback_lock:
            positions = [float(x) for x in self._joint_feedback_rad]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = positions
        self._feedback_pub.publish(msg)

    def _on_joint_state(self, msg):
        position = list(msg.position)
        if len(position) < 7:
            print("JointState position must have 7 elements (j1..j4 + diff servos + gripper)", position)
            return

        j1_rad, j2_rad, j3_rad, j4_rad, diff_a_rad, diff_b_rad, gripper_rad = position[:7]
        j_deg = [x * RAD_TO_DEG for x in (j1_rad, j2_rad, j3_rad, j4_rad)]
        diff_a_deg = diff_a_rad * RAD_TO_DEG
        diff_b_deg = diff_b_rad * RAD_TO_DEG

        prev_j_deg = self._last_j_deg
        d1, d2, d3, d4 = [cur - last for cur, last in zip(j_deg, prev_j_deg)]
        self._last_j_deg = j_deg

        # cap = self.max_stepper_delta
        # d1 = max(-cap, min(cap, d1))
        # d2 = max(-cap, min(cap, d2))
        # d3 = max(-cap, min(cap, d3))
        # d4 = max(-cap, min(cap, d4))

        deltas = (d1, d2, d3, d4)
        with self._step_lock:
            for axis in range(4):
                self._pending_steps[axis] += STEPPER_JOINT_DIR_SIGN[axis] * self._steps_for_delta_deg(
                    axis, deltas[axis]
                )

        self._move_differential(diff_a_deg, DIFF_JOINT6_SIGN * diff_b_deg)

        closed = gripper_rad > 0.5
        if closed != self._ee_closed:
            self._ee_closed = closed
            angle = EE_CLOSE_DEG if closed else EE_OPEN_DEG
            pulse_us = int(
                max(PWM_MIN, min(PWM_MAX, 500.0 + angle * (2000.0 / 180.0)))
            )
            self._current_pwm_ee = float(pulse_us)
            self._pi.set_servo_pulsewidth(self.gpio_servo_ee, pulse_us)
            with self._feedback_lock:
                self._joint_feedback_rad[6] = 1.0 if closed else 0.0

    def destroy_node(self, *args, **kwargs):
        self._shutdown = True
        if self._feedback_timer is not None:
            self._feedback_timer.cancel()
            self._feedback_timer = None
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
