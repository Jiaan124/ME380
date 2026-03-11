#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('me380_driver')
    config = os.path.join(pkg_share, 'config', 'driver_params.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port (unused by GPIO driver; kept for config compatibility)',
    )

    node_py = Node(
        package='me380_driver',
        executable='driver_node_py.py',
        name='driver_node_py',
        output='screen',
        parameters=[config, {'serial_port': LaunchConfiguration('serial_port')}],
    )

    return LaunchDescription([serial_port_arg, node_py])
