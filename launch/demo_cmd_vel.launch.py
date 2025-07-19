#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='deliver_robot',
            executable='demo_cmd_vel',
            name='demo_cmd_vel',
            output='screen',
            parameters=[],
            remappings=[
                # You can add remappings here if needed
                # ('cmd_vel', '/deliver_robot/cmd_vel'),
            ]
        )
    ])

