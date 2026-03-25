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

# Cartesian straight-line motion uses a trapezoidal (or triangular) velocity profile along the path.
# SciPy's interpolation modules (e.g. splines) do not provide this; it is standard piecewise-constant
# acceleration in one dimension (distance along the segment), then x = p0 + u * s.


def _s_trapezoid(t: float, L: float, v_max: float, a: float) -> float:
    """Distance along segment [0, L] at time t under accel-limited motion (SI: m, m/s, m/s^2, s)."""
    if L <= 1e-12:
        return 0.0
    a = max(float(a), 1e-6)
    v_max = max(float(v_max), 1e-9)
    s_acc_full = v_max * v_max / (2.0 * a)
    if 2.0 * s_acc_full >= L:
        # Triangular: peak speed sqrt(a * L)
        v_peak = math.sqrt(a * L)
        t_acc = v_peak / a
        T = 2.0 * t_acc
        if t <= 0.0:
            return 0.0
        if t >= T:
            return L
        if t <= t_acc:
            return 0.5 * a * t * t
        tau = t - t_acc
        s_mid = 0.5 * a * t_acc * t_acc
        return s_mid + v_peak * tau - 0.5 * a * tau * tau
    # Trapezoidal
    t_acc = v_max / a
    s_acc = v_max * v_max / (2.0 * a)
    s_const = L - 2.0 * s_acc
    t_const = s_const / v_max
    T = 2.0 * t_acc + t_const
    if t <= 0.0:
        return 0.0
    if t >= T:
        return L
    if t <= t_acc:
        return 0.5 * a * t * t
    if t <= t_acc + t_const:
        return s_acc + v_max * (t - t_acc)
    tau = t - t_acc - t_const
    return s_acc + s_const + v_max * tau - 0.5 * a * tau * tau


def cartesian_line_samples(
    p0: np.ndarray,
    p1: np.ndarray,
    v_max: float,
    a: float,
    dt: float,
) -> np.ndarray:
    """
    Sample positions along a straight line from p0 to p1 (each shape (3,)), meters.
    Returns array of shape (N, 3). Includes endpoints; spacing in time is dt.
    """
    p0 = np.asarray(p0, dtype=float).reshape(3)
    p1 = np.asarray(p1, dtype=float).reshape(3)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    if L < 1e-12:
        return p0.reshape(1, 3)
    u = d / L
    a = max(float(a), 1e-6)
    v_max = max(float(v_max), 1e-9)
    s_acc_full = v_max * v_max / (2.0 * a)
    if 2.0 * s_acc_full >= L:
        v_peak = math.sqrt(a * L)
        T = 2.0 * v_peak / a
    else:
        t_acc = v_max / a
        s_acc = v_max * v_max / (2.0 * a)
        s_const = L - 2.0 * s_acc
        t_const = s_const / v_max
        T = 2.0 * t_acc + t_const
    dt = max(float(dt), 1e-6)
    times = np.arange(0.0, T + 0.5 * dt, dt)
    if times[-1] < T - 1e-9:
        times = np.append(times, T)
    s_vals = np.array([_s_trapezoid(float(t), L, v_max, a) for t in times])
    return p0 + s_vals[:, np.newaxis] * u


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
        self.declare_parameter("cartesian_linear_velocity_m_s", 0.05)
        self.declare_parameter("cartesian_sample_period_s", 0.02)
        self.declare_parameter("cartesian_linear_accel_m_s2", 0.2)
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
        self.current_position = [0, 0, 0, 0, 0, 0, 0, 0]
        self.ik = [0, 0, 0.5, 0, 0.0, 0.0, 0, 0]
        self._traj_timer = None
        self._traj_rows = []
        self._traj_idx = 0
        print("ik node running")

    def _stamp_joint_state(self, js: JointState) -> None:
        """Set header.stamp to current ROS time (for joint_angles subscribers)."""
        js.header.stamp = self.get_clock().now().to_msg()

    def _cancel_trajectory_timer(self) -> None:
        if self._traj_timer is not None:
            self._traj_timer.cancel()
            self._traj_timer = None

    def _traj_timer_callback(self) -> None:
        if self._traj_idx >= len(self._traj_rows):
            self._cancel_trajectory_timer()
            return
        out = JointState()
        self._stamp_joint_state(out)
        out.position = self._traj_rows[self._traj_idx]
        self.publisher_.publish(out)
        self._traj_idx += 1
        if self._traj_idx >= len(self._traj_rows):
            self._cancel_trajectory_timer()

    def destroy_node(self) -> None:
        self._cancel_trajectory_timer()
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
        accel = float(self.get_parameter("cartesian_linear_accel_m_s2").value)

        target_xyz = np.array(msg.data[:3], dtype=float)
        seed = self.current_position if self.current_position is not None else self.ik
        fk0 = self.my_chain.forward_kinematics(seed)
        current_xyz = np.array(fk0[:3, 3], dtype=float).reshape(3)
        delta_xyz = target_xyz - current_xyz
        self.get_logger().info(
            f"xyz move: current={current_xyz.tolist()} target={target_xyz.tolist()} "
            f"delta_norm={float(np.linalg.norm(delta_xyz)):.4f} m"
        )

        # (N, 3) waypoints in meters; trapezoidal speed along the segment (not scipy — see module doc).
        samples = cartesian_line_samples(current_xyz, target_xyz, v_max, accel, dt)
        n = samples.shape[0]

        fk = fk0
        target_orientation = fk[:3, :3]
        joint_cols = np.zeros((6, n), dtype=float)
        q_full = list(seed)

        for i in range(n):
            pt = samples[i]
            q_full = self.my_chain.inverse_kinematics(
                pt,
                target_orientation,
                orientation_mode="all",
                initial_position=q_full,
            )
            self.ik = q_full
            joint_cols[:, i] = np.array(q_full[1:7], dtype=float)

        grip = 0.0
        self._traj_rows = [
            [float(joint_cols[j, k]) for j in range(6)] + [grip] for k in range(n)
        ]

        self._cancel_trajectory_timer()
        self._traj_idx = 0
        period = max(dt, 1e-6)
        if self._traj_rows:
            out0 = JointState()
            self._stamp_joint_state(out0)
            out0.position = self._traj_rows[0]
            self.publisher_.publish(out0)
            self._traj_idx = 1
        if self._traj_idx < len(self._traj_rows):
            self._traj_timer = self.create_timer(period, self._traj_timer_callback)
        self.get_logger().info(
            f"Publishing {n} joint waypoint(s) on 'joint_states' at {1.0/period:.1f} Hz "
            f"(v={v_max} m/s, a={accel} m/s², dt={dt} s)"
        )


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

