#!/usr/bin/env python3
import math
import threading
import time
from typing import List

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
import serial
from std_srvs.srv import Trigger

RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0

JOINT_STATE_TOPIC = "joint_states"
JOINT_FEEDBACK_TOPIC = "actual_joint_position"

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 0.01
SERIAL_STARTUP_DELAY_S = 2.0
SERIAL_READ_POLL_S = 0.02

CONTROL_DT_S = 0.02  # 20 ms
FEEDBACK_RATE_HZ = 50.0

# Direction multipliers applied at the bridge layer for J1..J6.
# Set J3 to -1.0 to reverse its hardware direction.
JOINT_DIRECTION = [1.0, -1.0, -1.0, 1.0, 1.0, 1.0]

# Max joint velocity limits [J1..J6] in deg/s for bridge-side clipping.
MAX_JOINT_VELOCITY_DEG_S = [
    0.5 * RAD_TO_DEG,
    2.0 * RAD_TO_DEG,
    1.0 * RAD_TO_DEG,
    0.5 * RAD_TO_DEG,
    0.5 * RAD_TO_DEG,
    0.5 * RAD_TO_DEG,
]

# Treat gripper command as degrees when sending to firmware.
GRIPPER_MIN_DEG = 10.0
GRIPPER_MAX_DEG = 145.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class Esp32BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("esp_32_bridge")

        # If True: ESP32 MODE VEL — same joint math as traj mode, but serial row is only
        # 6× joint velocities (deg/s) + gripper. If False: MODE TRAJ — 6× pos/delta + 6× vel + gripper.
        self.declare_parameter("vel_mode", False)
        # Integer ms (not float): float default made ROS treat this as DOUBLE and broke `ros2 param set … 20`.
        self.declare_parameter("vel_time_ms", int(round(CONTROL_DT_S * 1000)))

        # Open-loop state estimate (degrees): [J1..J6, gripper]
        self._state_deg: List[float] = [0.0] * 7
        self._target_deg: List[float] = [0.0] * 7
        self._cmd_vel_deg_s: List[float] = [0.0] * 6
        self._last_cmd_time = time.monotonic()
        self._state_lock = threading.Lock()

        self._vel_mode: bool = bool(self.get_parameter("vel_mode").value)
        self._vel_time_ms: int = int(self.get_parameter("vel_time_ms").value)
        self._serial_write_lock = threading.Lock()
        # Last MODE line applied on the serial link (for idempotent updates).
        self._serial_applied_vel_mode: bool | None = None
        self._serial_applied_vel_time_ms: int | None = None

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._serial = None
        self._connect_serial()

        self._joint_sub = self.create_subscription(
            JointState, JOINT_STATE_TOPIC, self._on_joint_states, 10
        )
        self._feedback_pub = self.create_publisher(JointState, JOINT_FEEDBACK_TOPIC, 10)
        self._feedback_timer = self.create_timer(1.0 / FEEDBACK_RATE_HZ, self._on_feedback_timer)
        self._serial_rx_timer = self.create_timer(SERIAL_READ_POLL_S, self._poll_serial_rx)

        self.srv = self.create_service(
            Trigger, 
            'zero_steppers', 
            self.zero_steppers_callback
        )

        self.get_logger().info(
            "esp_32_bridge online. Subscribing to '%s', publishing '%s' at %.0f Hz."
            % (JOINT_STATE_TOPIC, JOINT_FEEDBACK_TOPIC, FEEDBACK_RATE_HZ)
        )

    def zero_steppers_callback(self, request, response):
        self.get_logger().info('Received request to zero all steppers...')
        if self._vel_mode:
            response.success = False
            response.message = "Cannot zero steppers in velocity mode."
            return response

        if self._serial is None:
            response.success = False
            response.message = "Serial link not connected."
            return response

        # Absolute home pose in hardware frame (deg) for [J1..J6].
        # Firmware expects J1..J4 as deltas and J5..J6 as absolute angles.
        home_pose_abs_deg = [0, 142.48, 56.84, 0, 70.67, 0.0]
        stepper_hardstop_deg = [170, -5, 100,  170]
        # Command to firmware: J1..J4 are DELTAS from the hardstop coordinate.
        stepper_delta_deg = [
            home_pose_abs_deg[i] - stepper_hardstop_deg[i] for i in range(4)
        ]
        traj_frame = stepper_delta_deg + home_pose_abs_deg[4:6]
        traj_frame = [traj_frame[i] * JOINT_DIRECTION[i] for i in range(6)]

        # Jump feedback to the home pose: update bridge open-loop state now.
        # This makes the next feedback timer tick publish home immediately.
        with self._state_lock:
            self._state_deg[0:6] = home_pose_abs_deg
            self._state_deg[6] = GRIPPER_MAX_DEG
            self._target_deg[0:6] = home_pose_abs_deg
            self._target_deg[6] = GRIPPER_MAX_DEG
            for i in range(6):
                self._cmd_vel_deg_s[i] = 0.0
            self._last_cmd_time = time.monotonic()

        self.get_logger().info('Steppers zeroed out; jumping feedback to home pose.')
        self._send_traj_frame(traj_frame, MAX_JOINT_VELOCITY_DEG_S, GRIPPER_MAX_DEG)
        response.success = True
        response.message = "Commanded home pose (feedback jumped to home)."
        return response

    def _on_set_parameters(self, params: List[Parameter]) -> SetParametersResult:
        for p in params:
            if p.name == "vel_mode":
                if p.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False, reason="vel_mode must be a boolean"
                    )
                self._vel_mode = bool(p.value)
                self._push_serial_mode_if_needed(force=True)
            elif p.name == "vel_time_ms":
                if p.type_ == Parameter.Type.INTEGER:
                    t = int(p.value)
                elif p.type_ == Parameter.Type.DOUBLE:
                    t = int(round(float(p.value)))
                else:
                    return SetParametersResult(
                        successful=False,
                        reason="vel_time_ms must be an integer or double (ms)",
                    )
                if t <= 0:
                    return SetParametersResult(
                        successful=False, reason="vel_time_ms must be > 0"
                    )
                self._vel_time_ms = t
                if self._vel_mode:
                    self._push_serial_mode_if_needed(force=True)
        return SetParametersResult(successful=True)

    def _push_serial_mode_if_needed(self, force: bool = False) -> None:
        """Send MODE TRAJ / MODE VEL when mode or vel segment time changes."""
        want_vel = self._vel_mode
        want_ms = self._vel_time_ms
        if (
            not force
            and self._serial_applied_vel_mode == want_vel
            and (not want_vel or self._serial_applied_vel_time_ms == want_ms)
        ):
            return
        if want_vel:
            line = "MODE VEL %d\n" % want_ms
        else:
            line = "MODE TRAJ\n"
        self._serial_write_line(line)
        self._serial_applied_vel_mode = want_vel
        self._serial_applied_vel_time_ms = want_ms if want_vel else None
        self.get_logger().info(
            "Serial mode: %s" % ("VEL (%d ms)" % want_ms if want_vel else "TRAJ")
        )

    def _serial_write_line(self, line: str) -> None:
        if self._serial is None:
            return
        with self._serial_write_lock:
            try:
                self._serial.write(line.encode("ascii"))
                self._serial.flush()
            except Exception as exc:
                self.get_logger().error("Serial write failed: %s" % str(exc))

    def _connect_serial(self) -> None:
        try:
            self._serial = serial.Serial(
                SERIAL_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT_S, write_timeout=SERIAL_TIMEOUT_S
            )
            time.sleep(SERIAL_STARTUP_DELAY_S)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            self.get_logger().info(
                "Serial connected on %s @ %d." % (SERIAL_PORT, SERIAL_BAUD)
            )
            self._serial_applied_vel_mode = None
            self._serial_applied_vel_time_ms = None
            self._push_serial_mode_if_needed(force=True)
        except Exception as exc:
            self._serial = None
            self.get_logger().error("Failed to open serial %s: %s" % (SERIAL_PORT, str(exc)))

    def _poll_serial_rx(self) -> None:
        """Read and print any line-oriented messages from the ESP32."""
        if self._serial is None:
            return
        try:
            while self._serial.in_waiting > 0:
                raw = self._serial.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("ERR"):
                    self.get_logger().error("ESP32: %s" % line)
                elif line.startswith("WARN"):
                    self.get_logger().warn("ESP32: %s" % line)
                else:
                    self.get_logger().info("ESP32: %s" % line)
        except Exception as exc:
            self.get_logger().error("Serial read failed: %s" % str(exc))

    def _on_joint_states(self, msg: JointState) -> None:
        npos = len(msg.position)
        if npos < 6:
            self.get_logger().warn(
                "Expected at least 6 joint positions (rad), got %d." % npos
            )
            return

        incoming_deg = [float(x) * RAD_TO_DEG for x in msg.position[:6]]
        incoming_deg = [incoming_deg[i] * JOINT_DIRECTION[i] for i in range(6)]

        if npos >= 7:
            incoming_gripper = float(msg.position[6])
            if abs(incoming_gripper) <= 1.0:
                incoming_gripper *= GRIPPER_MAX_DEG
            gripper_deg = _clamp(incoming_gripper, GRIPPER_MIN_DEG, GRIPPER_MAX_DEG)
        else:
            with self._state_lock:
                gripper_deg = self._target_deg[6]

        incoming_deg.append(gripper_deg)

        with self._state_lock:
            now = time.monotonic()
            self._last_cmd_time = now

            frame_pos = [0.0] * 6
            frame_vel = [0.0] * 6

            for idx in range(4):
                current = self._state_deg[idx]
                target = incoming_deg[idx]
                delta = target - current
                vel_needed = delta / CONTROL_DT_S

                vel_limit = MAX_JOINT_VELOCITY_DEG_S[idx]
                vel_cmd = vel_needed
                if abs(vel_needed) > vel_limit:
                    vel_cmd = math.copysign(vel_limit, vel_needed)
                    self.get_logger().warn(
                        "J%d velocity %.2f deg/s exceeds max %.2f deg/s; clipping."
                        % (idx + 1, vel_needed, vel_limit)
                    )

                frame_pos[idx] = delta
                frame_vel[idx] = vel_cmd
                self._target_deg[idx] = target
                self._cmd_vel_deg_s[idx] = vel_cmd

            for idx in range(4, 6):
                current = self._state_deg[idx]
                target = incoming_deg[idx]
                vel_needed = (target - current) / CONTROL_DT_S

                vel_limit = MAX_JOINT_VELOCITY_DEG_S[idx]
                vel_cmd = vel_needed
                if abs(vel_needed) > vel_limit:
                    vel_cmd = math.copysign(vel_limit, vel_needed)
                    self.get_logger().warn(
                        "J%d velocity %.2f deg/s exceeds max %.2f deg/s; clipping."
                        % (idx + 1, vel_needed, vel_limit)
                    )

                frame_pos[idx] = target
                frame_vel[idx] = vel_cmd
                self._target_deg[idx] = target
                self._cmd_vel_deg_s[idx] = vel_cmd

            self._target_deg[6] = incoming_deg[6]

        if self._vel_mode:
            self._send_vel_frame(frame_vel, gripper_deg)
        else:
            self._send_traj_frame(frame_pos, frame_vel, gripper_deg)

    def _send_traj_frame(
        self, position_cmd_deg: List[float], velocity_cmd_deg_s: List[float], gripper_deg: float
    ) -> None:
        # Frame: [6 target/stepper increment + 6 velocity + 1 gripper] = 13 values
        frame = position_cmd_deg + velocity_cmd_deg_s + [gripper_deg]
        line = " ".join(f"{x:.6f}" for x in frame) + "\n"
        self._serial_write_line(line)

    def _send_vel_frame(self, velocity_cmd_deg_s: List[float], gripper_deg: float) -> None:
        # Firmware: [0..5] J1..J6 deg/s, [6] gripper (matches esp32.cpp loadVelocityRow).
        frame = velocity_cmd_deg_s + [gripper_deg]
        line = " ".join(f"{x:.6f}" for x in frame) + "\n"
        self._serial_write_line(line)

    def _on_feedback_timer(self) -> None:
        with self._state_lock:
            now = time.monotonic()
            dt = max(0.0, min(0.1, now - self._last_cmd_time))
            self._last_cmd_time = now

            # Open-loop model: d = d0 + v * t (same for traj and vel serial framing)
            for idx in range(6):
                v = self._cmd_vel_deg_s[idx]
                next_pos = self._state_deg[idx] + v * dt
                t = self._target_deg[idx]
                if (v >= 0.0 and next_pos > t) or (v < 0.0 and next_pos < t):
                    next_pos = t
                self._state_deg[idx] = next_pos

            self._state_deg[6] = self._target_deg[6]
            # Convert hardware-frame estimate back into ROS frame for publication.
            state_rad = [
                self._state_deg[i] * JOINT_DIRECTION[i] * DEG_TO_RAD for i in range(6)
            ] + [self._state_deg[6]]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = state_rad
        self._feedback_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self._serial_rx_timer is not None:
            self._serial_rx_timer.cancel()
            self._serial_rx_timer = None
        if self._feedback_timer is not None:
            self._feedback_timer.cancel()
            self._feedback_timer = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        return super().destroy_node()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = Esp32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
