#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('deliver_robot')
    
    # Declare launch arguments (Default save to home directory)
    declare_output_dir_cmd = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.expanduser('~/csv/amcl_pose'),
        description='Directory to save the pose log CSV file'
    )
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the node'
    )
    
    # Launch configuration variables
    output_dir = LaunchConfiguration('output_dir')
    log_level = LaunchConfiguration('log_level')
    
    # AMCL Pose Logger Node
    pose_logger_node = Node(
        package='deliver_robot',
        executable='get_amcl_pose.py',
        name='amcl_pose_logger',
        output='screen',
        parameters=[
            {'output_dir': output_dir}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_output_dir_cmd)
    ld.add_action(declare_log_level_cmd)
    
    # Add the pose logger node
    ld.add_action(pose_logger_node)
    
    return ld