#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    width = DeclareLaunchArgument('width', default_value='640')
    height = DeclareLaunchArgument('height', default_value='480')
    freq = DeclareLaunchArgument('freq', default_value='30.0')
    device_id = DeclareLaunchArgument('device_id', default_value='0')  # Camera device ID (/dev/video0)
    view_vid = DeclareLaunchArgument('view_vid', default_value='False')

    webcam_node = Node(
        package='image_tools',
        executable='cam2image',
        name='webcam_node',
        output='screen',
        parameters=[
            {'width': LaunchConfiguration('width')},
            {'height': LaunchConfiguration('height')},
            {'freq': LaunchConfiguration('freq')},
            {'device_id': LaunchConfiguration('device_id')},
        ]
    )

    view_vid_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_viewer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('view_vid'))
    )

    return LaunchDescription([
        width,
        height,
        freq,
        device_id,
        view_vid,
        webcam_node,
        view_vid_node
    ])