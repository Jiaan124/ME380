#!/usr/bin/env python3
import math
import threading
import time
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial


RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0

JOINT_STATE_TOPIC = "joint_states"
JOINT_FEEDBACK_TOPIC = "actual_joint_position"

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 0.01
SERIAL_STARTUP_DELAY_S = 2.0

CONTROL_DT_S = 0.02  # 20 ms
FEEDBACK_RATE_HZ = 50.0

# Max joint velocity limits [J1..J6] in deg/s for bridge-side clipping.
MAX_JOINT_VELOCITY_DEG_S = [
    0.5 * RAD_TO_DEG,
    1.0 * RAD_TO_DEG,
    1.0 * RAD_TO_DEG,
    0.5 * RAD_TO_DEG,
    0.5 * RAD_TO_DEG,
    0.5 * RAD_TO_DEG,
]

# Treat gripper command as degrees when sending to firmware.
GRIPPER_MIN_DEG = 0.0
GRIPPER_MAX_DEG = 180.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class Esp32BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("esp_32_bridge")

        # Open-loop state estimate (degrees): [J1..J6, gripper]
        self._state_deg: List[float] = [0.0] * 7
        self._target_deg: List[float] = [0.0] * 7
        self._cmd_vel_deg_s: List[float] = [0.0] * 6
        self._last_cmd_time = time.monotonic()
        self._state_lock = threading.Lock()

        self._serial = None
        self._connect_serial()

        self._joint_sub = self.create_subscription(
            JointState, JOINT_STATE_TOPIC, self._on_joint_states, 10
        )
        self._feedback_pub = self.create_publisher(JointState, JOINT_FEEDBACK_TOPIC, 10)
        self._feedback_timer = self.create_timer(1.0 / FEEDBACK_RATE_HZ, self._on_feedback_timer)

        self.get_logger().info(
            "esp_32_bridge online. Subscribing to '%s', publishing '%s' at %.0f Hz."
            % (JOINT_STATE_TOPIC, JOINT_FEEDBACK_TOPIC, FEEDBACK_RATE_HZ)
        )

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
        except Exception as exc:
            self._serial = None
            self.get_logger().error("Failed to open serial %s: %s" % (SERIAL_PORT, str(exc)))

    def _on_joint_states(self, msg: JointState) -> None:
        if len(msg.position) < 7:
            self.get_logger().warn("Expected 7 joint positions, got %d." % len(msg.position))
            return

        # Incoming JointState is radians for joints, gripper passed through as numeric command.
        incoming_deg = [float(x) * RAD_TO_DEG for x in msg.position[:6]]
        incoming_gripper = float(msg.position[6])
        if abs(incoming_gripper) <= 1.0:
            incoming_gripper *= GRIPPER_MAX_DEG
        incoming_deg.append(_clamp(incoming_gripper, GRIPPER_MIN_DEG, GRIPPER_MAX_DEG))

        with self._state_lock:
            now = time.monotonic()
            self._last_cmd_time = now

            frame_pos = [0.0] * 6
            frame_vel = [0.0] * 6

            # J1..J4 steppers: send incremental position and velocity that reaches target in 20 ms.
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

            # J5..J6 servos: absolute position with clipped velocity hint.
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

        self._send_frame(frame_pos, frame_vel, incoming_deg[6])

    def _send_frame(self, position_cmd_deg: List[float], velocity_cmd_deg_s: List[float], gripper_deg: float) -> None:
        # Frame: [6 target/stepper increment + 6 velocity + 1 gripper] = 13 values
        frame = position_cmd_deg + velocity_cmd_deg_s + [gripper_deg]
        line = " ".join(f"{x:.6f}" for x in frame) + "\n"

        if self._serial is None:
            return

        try:
            self._serial.write(line.encode("ascii"))
            self._serial.flush()
        except Exception as exc:
            self.get_logger().error("Serial write failed: %s" % str(exc))

    def _on_feedback_timer(self) -> None:
        with self._state_lock:
            now = time.monotonic()
            dt = max(0.0, min(0.1, now - self._last_cmd_time))
            self._last_cmd_time = now

            # Open-loop model: d = d0 + v * t
            for idx in range(6):
                v = self._cmd_vel_deg_s[idx]
                if idx < 4:
                    # Steppers track absolute target through incremental command stream.
                    next_pos = self._state_deg[idx] + v * dt
                    t = self._target_deg[idx]
                    if (v >= 0.0 and next_pos > t) or (v < 0.0 and next_pos < t):
                        next_pos = t
                    self._state_deg[idx] = next_pos
                else:
                    # Servos are absolute-position driven; integrate toward target with velocity hint.
                    next_pos = self._state_deg[idx] + v * dt
                    t = self._target_deg[idx]
                    if (v >= 0.0 and next_pos > t) or (v < 0.0 and next_pos < t):
                        next_pos = t
                    self._state_deg[idx] = next_pos

            self._state_deg[6] = self._target_deg[6]
            state_rad = [x * DEG_TO_RAD for x in self._state_deg[:6]] + [self._state_deg[6]]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = state_rad
        self._feedback_pub.publish(msg)

    def destroy_node(self) -> bool:
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
