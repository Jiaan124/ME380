#!/usr/bin/env python3
import os

import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
# import matplotlib.pyplot as plt
import math
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

'''
Takes in final target position x y z and orientation and creates a joint state array to send to driver node
'''


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
        self.sub = self.create_subscription(  #take in x, y, z and euler angles? or should it take in x y z and rotation matrix?
            Float64MultiArray,  #data type
            'target_position_joint_space', #topic name
            self.joint_targ_callback, #callback
            10,
        )
        self.sub = self.create_subscription(  #take in x, y, z and euler angles? or should it take in x y z and rotation matrix?
            Float64MultiArray,
            'target_position_xyz_space',
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
        self.declare_parameter("joint_space", True)
        self.joint_space = self.get_parameter("joint_space").value
        # self.timer = self.create_timer(0.02, self.timer_callback) #50hz control loop

        self.my_chain = ikpy.chain.Chain.from_urdf_file(self.filepath, active_links_mask=[False, True, True, True, True, True, True, False])
        #needed to have correct arm orientation when solving 2.64 rad = 151 deg, 1.57 rad = 90
        self.current_position = None
        self.ik = [0, 0, 0.5, 0, 0.0, 0.0, 0, 0]
        print("ik node running")

    def _stamp_joint_state(self, js: JointState) -> None:
        """Set header.stamp to current ROS time (for joint_angles subscribers)."""
        js.header.stamp = self.get_clock().now().to_msg()

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
            seed = self.current_position if self.current_position is not None else self.ik
            self.ik = self.my_chain.inverse_kinematics(
                target_position, target_orientation, orientation_mode="all", initial_position=self.ik
            )

            out = JointState()
            self._stamp_joint_state(out)
            out.position = self.ik[1:7].tolist()  # get rid of dummy link
            # print("Joint Angles: ", list(map(lambda r: math.degrees(r), self.ik[1:7].tolist())))
            out.position.append(0) #TODO: CHANGE THIS> THIS ONLY EXISTS BECAUSE IM LAZY TO DO GRIPPER
            self.publisher_.publish(out)

        else:  #move in xyz space, assume constant rotation
            print("not in joint space mode, doing nothing. pls toggle to joitn space mode with command: ros2 param set /ik joint_space true")


    def xyz_targ_callback(self, msg):
        if len(msg.data) != 3:
            print("invalid target position, should be 3 element XYZ")
            return

        target_position = msg.data[:3]
        self.joint_space = self.get_parameter("joint_space").value
        if self.joint_space:
            print("not in xyz mode, doing nothing. pls toggle to cartesian mode with command: ros2 param set /ik joint_space false")
            return

        seed = self.current_position if self.current_position is not None else self.ik
        # Current orientation from FK at seed pose (4x4); IK wants rotation matrix for tip
        fk = self.my_chain.forward_kinematics(seed)
        target_orientation = fk[:3, :3]

        self.ik = self.my_chain.inverse_kinematics(
            target_position, target_orientation, orientation_mode="all", initial_position=seed
        )

        out = JointState()
        self._stamp_joint_state(out)
        out.position = [float(x) for x in self.ik[1:7]]
        print("Joint Angles: ", list(map(lambda r: math.degrees(r), self.ik[1:7].tolist())))
        self.publisher_.publish(out)


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

