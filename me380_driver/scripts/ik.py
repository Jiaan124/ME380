#!/usr/bin/env python3
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
# import matplotlib.pyplot as plt
import math
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

'''
Takes in final target position x y z and orientation and creates a joint state array to send to driver node
'''


class ikNode(Node):
    def __init__(self):
        super().__init__("ik")
        self.declare_parameter("filepath","me380_robot.urdf" )
        self.filepath = self.get_parameter("filepath").value
        self.publisher_ = self.create_publisher(JointState, "joint_angles", 10) #message type, topic name, buffer size
        self.sub = self.create_subscription(  #take in x, y, z and euler angles? or should it take in x y z and rotation matrix?
            Float64MultiArray,
            self.target_position,
            self._on_joint_state,
            10,
        )
        self.sub = self.create_subscription(  #take in x, y, z and euler angles? or should it take in x y z and rotation matrix?
            JointState,
            self.actual_joint_position,
            self._on_motor_state,
            10,
        )

        #toggle between joint space command and cartesian space command
        self.declare_parameter("joint_space", True)
        self.joint_space = self.get_parameter("joint_space").value

    # Create a timer to call the callback at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.my_chain = ikpy.chain.Chain.from_urdf_file("filepath", active_links_mask=[False, True, True, True, True, True, True, False])
        #add initial position fro solver
        self.current_position = [0, 0, 0, 0, 0, 0]

    def actual_joint_position(self, msg):
        self.current_position = msg.position

    def target_position(self, msg):
        target_position = msg[:3]
        r = R.from_euler('xyz', msg[3:], degrees=True)
        target_orientation = r.as_matrix()

        if self.joint_space: #move in joint space to target orientation
            ik = self.my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="all", initial_position=self.old_position)
            self.current_position = ik

            msg = JointState()
            msg.position = ik.tolist()
            print("Joint Angles: ", list(map(lambda r:math.degrees(r),ik.tolist())))
            self.publisher_.publish(msg)    

        else:  #move in xyz space, assume constant rotation
            


    # def timer_callback(self):
    #     target_position = [ 0.2, .2, 0.2]
    #     r = R.from_euler('yz', [-90,45], degrees=True)
    #     target_orientation = r.as_matrix()
    #     ik = self.my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="all", initial_position=self.old_position)
    #     self.old_position = ik

    #     msg = JointState()
    #     msg.position = ik.tolist()
    #     print("Joint Angles: ", list(map(lambda r:math.degrees(r),ik.tolist())))
    #     self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ikNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# # ax = plt.figure().add_subplot(111, projection='3d')
# fig, ax = plot_utils.init_3d_figure()
# fig.set_figheight(9)  
# fig.set_figwidth(13)  
# my_chain.plot(ik, ax, target=target_position)

# plt.xlim(-0.4, 0.4)
# plt.ylim(-0.4, 0.4)
# ax.set_zlim(0, 0.4)
# # Equal axis scale so the robot is not stretched (matplotlib >= 3.3)

# plt.show()

# def doIK():
#     global ik
#     old_position= ik.copy()
#     ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Z", initial_position=old_position)

# def updatePlot():
#     ax.clear()
#     my_chain.plot(ik, ax, target=target_position)
#     plt.xlim(-0.5, 0.5)
#     plt.ylim(-0.5, 0.5)
#     ax.set_zlim(0, 0.6)
#     fig.canvas.draw()
#     fig.canvas.flush_events()
    
# def move(x,y,z):
#     global target_position
#     target_position = [x,y,z]
#     doIK()
#     updatePlot()


# #TODO: Add home position
# #TODO: Create way to define straight x y z trajectory
# #TODO: Integrate with ROS node. Should I have high level motion planning in same node or different node - Should this node only 
# #handle IK or should it also remember waypoints. Maybe have this node take in target position and then it generates a trajectory
# #to get to that target position. Maybe have way to toggle between joint space vs cartension control

