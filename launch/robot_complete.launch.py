#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('deliver_robot')
    
    # Path to the config file
    config_file = os.path.join(package_dir, 'config', 'esp32_serial_config.yaml')
    
    return LaunchDescription([
        # ESP32 Serial Communication Node
        Node(
            package='deliver_robot',
            executable='esp32_serial_node',
            name='esp32_serial_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                # You can add remappings here if needed
                # ('/cmd_vel', '/robot/cmd_vel'),
                # ('/odom', '/robot/odom'),
            ]
        ),
        
        # Demo CMD_VEL Publisher Node (for testing)
        Node(
            package='deliver_robot',
            executable='cmd_vel_publisher',
            name='cmd_vel_publisher',
            output='screen',
            parameters=[],
            # Uncomment the next line if you want to start this node automatically
            # condition=LaunchConfigurationEquals('start_demo', 'true')
        )
    ])
