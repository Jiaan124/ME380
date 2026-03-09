#!/usr/bin/env python3
"""
ROS2 Humble driver node (Python) for ME380 robot.
Subscribes to sensor_msgs/msg/JointState and sends commands to the
motor controller (ESP32/Arduino with driver_firmware) over serial.
"""

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def index_of(names: List[str], joint_name: str) -> int:
    try:
        return names.index(joint_name)
    except ValueError:
        return -1


class DriverNodePy(Node):
    RAD_TO_DEG = 180.0 / math.pi

    def __init__(self) -> None:
        super().__init__("driver_node_py")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("max_stepper_delta_per_msg", 360.0)

        self.serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.joint_state_topic = self.get_parameter("joint_state_topic").get_parameter_value().string_value
        self.max_stepper_delta = self.get_parameter("max_stepper_delta_per_msg").get_parameter_value().double_value

        self._serial: Optional["serial.Serial"] = None
        if not self._open_serial():
            self.get_logger().warn("Serial port not open. Commands will be logged only.")

        self._sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            10,
        )

        self.get_logger().info(
            "Subscribed to %s; sending to %s @ %d"
            % (self.joint_state_topic, self.serial_port, self.baud_rate)
        )

        self._last_j1_deg = 0.0
        self._last_j2_deg = 0.0
        self._last_j3_deg = 0.0

    def _open_serial(self) -> bool:
        try:
            import serial
        except ImportError:
            self.get_logger().error("pyserial not installed. Run: pip install pyserial")
            return False
        try:
            self._serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.01,
                write_timeout=0.1,
            )
            self.get_logger().info("Serial %s opened at %d baud" % (self.serial_port, self.baud_rate))
            return True
        except Exception as e:
            self.get_logger().error("Failed to open %s: %s" % (self.serial_port, e))
            self._serial = None
            return False

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def destroy_node(self) -> None:
        self._close_serial()
        super().destroy_node()

    def _on_joint_state(self, msg: JointState) -> None:
        names = msg.name
        position = msg.position

        if len(names) != len(position) or len(names) == 0:
            self.get_logger().warn_throttle(1.0, "JointState name/position size mismatch or empty")
            return

        i1 = index_of(names, "joint1") if index_of(names, "joint1") >= 0 else index_of(names, "j1")
        i2 = index_of(names, "joint2") if index_of(names, "joint2") >= 0 else index_of(names, "j2")
        i3 = index_of(names, "joint3") if index_of(names, "joint3") >= 0 else index_of(names, "j3")
        i4 = index_of(names, "joint4") if index_of(names, "joint4") >= 0 else index_of(names, "j4")
        i5 = index_of(names, "joint5") if index_of(names, "joint5") >= 0 else index_of(names, "j5")
        i6 = index_of(names, "joint6") if index_of(names, "joint6") >= 0 else index_of(names, "gripper")

        j1_rad = position[i1] if i1 >= 0 else 0.0
        j2_rad = position[i2] if i2 >= 0 else 0.0
        j3_rad = position[i3] if i3 >= 0 else 0.0
        j4_rad = position[i4] if i4 >= 0 else 0.0
        j5_rad = position[i5] if i5 >= 0 else 0.0
        j6_rad = position[i6] if i6 >= 0 else 0.0

        j1_deg = j1_rad * self.RAD_TO_DEG
        j2_deg = j2_rad * self.RAD_TO_DEG
        j3_deg = j3_rad * self.RAD_TO_DEG
        j4_deg = j4_rad * self.RAD_TO_DEG
        j5_deg = j5_rad * self.RAD_TO_DEG

        d1 = j1_deg - self._last_j1_deg
        d2 = j2_deg - self._last_j2_deg
        d3 = j3_deg - self._last_j3_deg
        self._last_j1_deg = j1_deg
        self._last_j2_deg = j2_deg
        self._last_j3_deg = j3_deg

        d1 = max(-self.max_stepper_delta, min(self.max_stepper_delta, d1))
        d2 = max(-self.max_stepper_delta, min(self.max_stepper_delta, d2))
        d3 = max(-self.max_stepper_delta, min(self.max_stepper_delta, d3))

        gripper = 1 if j6_rad > 0.5 else 0

        line = "%.4f %.4f %.4f %.4f %.4f %d\n" % (d1, d2, d3, j4_deg, j5_deg, gripper)

        if self._serial is not None:
            try:
                written = self._serial.write(line.encode("ascii"))
                if written != len(line):
                    self.get_logger().warn_throttle(1.0, "Serial write incomplete: %d / %d" % (written, len(line)))
            except Exception as e:
                self.get_logger().warn_throttle(1.0, "Serial write failed: %s" % e)
        else:
            self.get_logger().debug("Would send: %s" % line.strip())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriverNodePy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
