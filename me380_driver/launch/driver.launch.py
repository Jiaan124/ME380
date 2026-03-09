#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('me380_driver')
    config = os.path.join(pkg_share, 'config', 'driver_params.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for motor controller (e.g. /dev/ttyUSB0)',
    )

    use_python_arg = DeclareLaunchArgument(
        'use_python',
        default_value='false',
        description='If true, run the Python driver node instead of the C++ one.',
    )

    node_cpp = Node(
        package='me380_driver',
        executable='driver_node',
        name='driver_node',
        output='screen',
        parameters=[config, {'serial_port': LaunchConfiguration('serial_port')}],
        condition=UnlessCondition(LaunchConfiguration('use_python')),
    )
    node_py = Node(
        package='me380_driver',
        executable='driver_node_py.py',
        name='driver_node_py',
        output='screen',
        parameters=[config, {'serial_port': LaunchConfiguration('serial_port')}],
        condition=IfCondition(LaunchConfiguration('use_python')),
    )

    return LaunchDescription([
        serial_port_arg,
        use_python_arg,
        node_cpp,
        node_py,
    ])
