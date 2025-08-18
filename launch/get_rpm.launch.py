#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Declare launch arguments
    rpm_topic_arg = DeclareLaunchArgument(
        'rpm_topic',
        default_value='/rpm_data',
        description='Topic name for RPM data'
    )
    
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='rpm_data.csv',
        description='Output CSV filename'
    )
    
    output_directory_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='~/csv/rpm',
        description='Output directory for CSV file'
    )
    
    add_timestamp_arg = DeclareLaunchArgument(
        'add_timestamp',
        default_value='true',
        description='Add timestamp to filename'
    )

    # Create the get_rpm node
    get_rpm_node = Node(
        package='deliver_robot',
        executable='get_rpm.py',
        name='get_rpm_node',
        output='screen',
        parameters=[{
            'rpm_topic': LaunchConfiguration('rpm_topic'),
            'output_file': LaunchConfiguration('output_file'),
            'output_directory': LaunchConfiguration('output_directory'),
            'add_timestamp': LaunchConfiguration('add_timestamp'),
        }]
    )

    return LaunchDescription([
        rpm_topic_arg,
        output_file_arg,
        output_directory_arg,
        add_timestamp_arg,
        get_rpm_node
    ])
