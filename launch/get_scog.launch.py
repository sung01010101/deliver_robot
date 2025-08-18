#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Declare launch arguments
    scog_topic_arg = DeclareLaunchArgument(
        'scog_topic',
        default_value='/scog_data',
        description='Topic name for SCOG data'
    )
    
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='scog_data.csv',
        description='Output CSV filename'
    )
    
    output_directory_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='~/scog',
        description='Output directory for CSV file'
    )
    
    add_timestamp_arg = DeclareLaunchArgument(
        'add_timestamp',
        default_value='true',
        description='Add timestamp to filename'
    )

    # Create the get_scog node
    get_scog_node = Node(
        package='deliver_robot',
        executable='get_scog.py',
        name='get_scog_node',
        output='screen',
        parameters=[{
            'scog_topic': LaunchConfiguration('scog_topic'),
            'output_file': LaunchConfiguration('output_file'),
            'output_directory': LaunchConfiguration('output_directory'),
            'add_timestamp': LaunchConfiguration('add_timestamp'),
        }]
    )

    return LaunchDescription([
        scog_topic_arg,
        output_file_arg,
        output_directory_arg,
        add_timestamp_arg,
        get_scog_node
    ])
