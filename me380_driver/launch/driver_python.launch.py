#!/usr/bin/env python3
"""Launch the Python GPIO driver and IK nodes."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    driver_node = Node(
        package='me380_driver',
        executable='driver_node_py.py',
        name='driver_node_py',
        output='screen',
    )

    ik_node = Node(
        package='me380_driver',
        executable='ik.py',
        name='ik',
        output='screen',
    )

    return LaunchDescription([driver_node, ik_node])
