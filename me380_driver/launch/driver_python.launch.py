#!/usr/bin/env python3
"""Launch the Python driver node."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('me380_driver')
    config = os.path.join(pkg_share, 'config', 'driver_params.yaml')

    motion_velocity_arg = DeclareLaunchArgument(
        'motion_velocity',
        default_value='0.5',
        description='Scale for stepper + differential servo speed (1.0 = nominal)',
    )

    node = Node(
        package='me380_driver',
        executable='driver_node_py.py',
        name='driver_node_py',
        output='screen',
        parameters=[
            config,
            {
                'motion_velocity': ParameterValue(
                    LaunchConfiguration('motion_velocity'),
                    value_type=float,
                ),
            },
        ],
    )

    return LaunchDescription([motion_velocity_arg, node])
