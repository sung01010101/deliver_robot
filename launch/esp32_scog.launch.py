#!/usr/bin/env python3

import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # directories
    package_dir = get_package_share_directory('deliver_robot')
    esp32_config_file = os.path.join(package_dir, 'config', 'esp32_serial_config.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time if True'
    )

    esp32_scog_node = Node(
        package='deliver_robot',
        executable='esp32_scog_node.py',
        name='esp32_scog_node',
        output='screen',
        parameters=[esp32_config_file],
            remappings=[
                ('/cmd_vel', '/cmd_vel')
            ]
        )

    return LaunchDescription([
        declare_use_sim_time,
        esp32_scog_node
    ])