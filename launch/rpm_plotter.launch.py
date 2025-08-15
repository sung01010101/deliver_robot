#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('deliver_robot')
    
    return LaunchDescription([
        Node(
            package='deliver_robot',
            executable='rpm_plotter.py',
            name='rpm_plotter',
            output='screen',
            parameters=[],
            remappings=[]
        ),
    ])
