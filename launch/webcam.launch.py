#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    
    width = DeclareLaunchArgument('width', default_value='640')
    height = DeclareLaunchArgument('height', default_value='480')
    freq = DeclareLaunchArgument('freq', default_value='30.0')
    device_id = DeclareLaunchArgument('device_id', default_value='0')  # Camera device ID (0 for default camera)

    return LaunchDescription([
        Node(
            package='image_tools',
            executable='cam2image',
            name='webcam_node',
            output='screen',
            parameters=[
                {'width': width},
                {'height': height},
                {'freq': freq},
                {'device_id': device_id},
            ]
        )
    ])