#!/usr/bin/env python3
import os
import time
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
# import matplotlib.pyplot as plt
import math
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R
import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Must match `super().__init__(...)` in me380_driver/scripts/esp_32_bridge.py
ESP32_BRIDGE_NODE_NAME = "esp_32_bridge"

"""
IK node: joint-space targets from pose commands, and Cartesian straight-line moves.

`target_position_xyz_space` carries relative displacements dx, dy, dz (m); the goal is
current FK position + delta (same frame as the URDF chain).
"""

# Cartesian straight-line motion uses a trapezoidal (or triangular) velocity profile along the path.
# SciPy's interpolation modules (e.g. splines) do not provide this; it is standard piecewise-constant
# acceleration in one dimension (distance along the segment), then x = p0 + u * s.


def _default_urdf_path() -> str:
    """Installed URDF: share/me380_driver/urdf/me380_robot.urdf (after colcon build + source)."""
    pkg_share = get_package_share_directory("me380_driver")
    return os.path.join(pkg_share, "urdf", "me380_robot.urdf")


class ikNode(Node):
    def __init__(self):
        super().__init__("ik")
        # Override with absolute path if needed; default is package share URDF
        self.declare_parameter("filepath", _default_urdf_path())
        self.filepath = self.get_parameter("filepath").value
        self.publisher_ = self.create_publisher(JointState, "joint_states", 10) #message type, topic name, buffer size
        self.declare_parameter("cartesian_linear_velocity_m_s", 0.02)
        self.declare_parameter("cartesian_sample_period_s", 0.5)
        self.declare_parameter("cartesian_linear_accel_m_s2", 0.2)
        self.sub = self.create_subscription(  #take in x, y, z and euler angles? or should it take in x y z and rotation matrix?
            Float64MultiArray,  #data type
            'target_position_joint_space', #topic name
            self.joint_targ_callback, #callback
            10,
        )
        self.sub = self.create_subscription(
            Float64MultiArray,
            # Relative Cartesian move (m): Δx, Δy, Δz in the same frame as FK (goal = current + delta).
            "target_position_xyz_space",
            self.xyz_targ_callback,
            10,
        )
        self.sub = self.create_subscription(  #take in x, y, z and euler angles? or should it take in x y z and rotation matrix?
            JointState,
            'actual_joint_position',
            self.actual_state_callback,
            10,
        )

        #toggle between joint space command and cartesian space command
        self.declare_parameter("joint_space", False)
        self.joint_space = self.get_parameter("joint_space").value
        # self.timer = self.create_timer(0.02, self.timer_callback) #50hz control loop

        self.my_chain = ikpy.chain.Chain.from_urdf_file(self.filepath, active_links_mask=[False, True, True, True, True, True, True, False])
        #needed to have correct arm orientation when solving 2.64 rad = 151 deg, 1.57 rad = 90
        self.current_position = [0, 0, 0, 0, 0, 0, 0, 0]
        self.ik_init = [0, 0, 0.8212, -1.6, 0, -0.793, 0, 0]
        print("ik node running")

    def _stamp_joint_state(self, js: JointState) -> None:
        """Set header.stamp to current ROS time (for joint_angles subscribers)."""
        js.header.stamp = self.get_clock().now().to_msg()

    def destroy_node(self) -> None:
        super().destroy_node()

    def actual_state_callback(self, msg):
        # pad with extra 0 at beginning and end so ik can solve
        self.current_position = [0.0] + list(msg.position[:6]) + [0.0]

    def joint_targ_callback(self, msg):
        target_position = msg.data[:3]
        r = R.from_euler('xyz', msg.data[3:], degrees=True)
        target_orientation = r.as_matrix()
        self.joint_space = self.get_parameter("joint_space").value

        if self.joint_space: #move in joint space to target orientation
            #should only need to give end position because velocity is linear
            seed = self.current_position if self.current_position is not None else self.ik_init
            self.ik_init = self.my_chain.inverse_kinematics(
                target_position, target_orientation, orientation_mode="all", initial_position=seed
            )

            out = JointState()
            self._stamp_joint_state(out)
            out.position = self.ik_init[1:7].tolist()  # get rid of dummy link
            # print("Joint Angles: ", list(map(lambda r: math.degrees(r), self.ik[1:7].tolist())))
            out.position.append(0) #TODO: CHANGE THIS> THIS ONLY EXISTS BECAUSE IM LAZY TO DO GRIPPER
            self.publisher_.publish(out)

        else:  #move in xyz space, assume constant rotation
            print("not in joint space mode, doing nothing. pls toggle to joitn space mode with command: ros2 param set /ik joint_space true")


    def xyz_targ_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warn("invalid target position, expected 3 floats (x y z in meters)")
            return

        self.joint_space = self.get_parameter("joint_space").value
        if self.joint_space:
            self.get_logger().warn(
                "not in xyz mode; set: ros2 param set /ik joint_space false"
            )
            return

        v_max = float(self.get_parameter("cartesian_linear_velocity_m_s").value)
        dt = float(self.get_parameter("cartesian_sample_period_s").value)
        # accel = float(self.get_parameter("cartesian_linear_accel_m_s2").value)

        target_xyz = np.array(msg.data[:3], dtype=float)
        # target_xyz = np.array([0.35, 0.0, 0.15]) #0, 0.436, -0.804
        seed = self.current_position if self.current_position is not None else self.ik_init
        # seed = [0, 0, 0.8212, -1.6, 0, -0.793, 0, 0]  #0.25, 0, 0.15
        fk0 = self.my_chain.forward_kinematics(seed)
        current_xyz = np.array(fk0[:3, 3], dtype=float).reshape(3)
        # current_xyz = np.array([0.4, 0.0, 0.2])
        delta_xyz = target_xyz - current_xyz


        #fin total distance
        distance = np.linalg.norm(delta_xyz) 

        if distance > 0:
            unit_direction = delta_xyz / distance
        else:
            unit_direction = np.zeros(3)


        num_samples = int(distance / v_max/ dt)
        waypoints = np.array([current_xyz])

        #create 2d array of all inbetween positions
        for i in range(1, num_samples+1):
            next_point = current_xyz + i * v_max * dt *unit_direction    
            waypoints = np.vstack((waypoints, next_point))

        waypoints = np.vstack((waypoints, target_xyz)) #make sure to take care of any rounding errors
        # print(waypoints)


        target_orientation = fk0[:3, :3]
        joint_cols = np.empty((0, 6))
        for i in range(waypoints.shape[0]):
            xyz_coord = waypoints[i]
            joint_angles = self.my_chain.inverse_kinematics(
                xyz_coord,
                target_orientation,
                orientation_mode="all",
                initial_position=seed,
            )
            seed = joint_angles
            joint_cols = np.vstack((joint_cols, np.array(joint_angles[1:7], dtype=float)))
        # print(joint_cols)
        print(waypoints.shape[0], joint_cols.shape[0])

        joint_vels = np.empty((0, 6))
        for i in range(joint_cols.shape[0]-1):
            joint_vels = np.vstack((joint_vels, (joint_cols[i+1] - joint_cols[i]) / dt))

        self.get_logger().info(
            f"xyz start/end:\n"
            f"  current={current_xyz.tolist()}\n"
            f"  target={target_xyz.tolist()}\n"
            f"joint start/end:\n"
            f"  start={joint_cols[0].tolist()}\n"
            f"  end={joint_cols[-1].tolist()}\n"
            f"delta_norm={float(np.linalg.norm(delta_xyz)):.4f} m"
            # f"seed={seed[1:7]}"
        )

        print(joint_vels)

        gripper_pos = 0.0
        
        #publish from array every dt seconds
        rate = self.create_rate(1.0 / dt)
        for i in range(joint_cols.shape[0]):
            next_time = time.time()
            out = JointState()
            self._stamp_joint_state(out)
            out.position = joint_cols[i].tolist()
            out.position.append(0.9) #TODO: CHANGE THIS> THIS ONLY EXISTS BECAUSE IM LAZY TO DO GRIPPER
            if i < joint_cols.shape[0]-1:
                out.velocity = joint_vels[i].tolist()
            
                # out.velocity = [0.0] * 6
            self.publisher_.publish(out)
            # print(joint_cols[i])
            next_time += dt
            sleep_time = next_time - time.time()        
            if sleep_time > 0:
                time.sleep(sleep_time)



def main(args=None):
    rclpy.init(args=args)
    node = ikNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# #TODO: Add home position
# #TODO: Create way to define straight x y z trajectory
# #TODO: Integrate with ROS node. Should I have high level motion planning in same node or different node - Should this node only 
# #handle IK or should it also remember waypoints. Maybe have this node take in target position and then it generates a trajectory
# #to get to that target position. Maybe have way to toggle between joint space vs cartension control

