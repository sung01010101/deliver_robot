#!/usr/bin/env python3

import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # directories
    package_dir = get_package_share_directory('deliver_robot')
    # ekf_config_file = os.path.join(package_dir, 'config', 'ekf.yaml')
    esp32_config_file = os.path.join(package_dir, 'config', 'esp32_serial_config.yaml')

    # launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time if True'
    )

    declare_use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='True',
        description='Use EKF if True',
    )

    # launch nodes
    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_node',
    #     output='screen',
    #     parameters=[
    #         ekf_config_file,
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ],
    #     condition=IfCondition(LaunchConfiguration('use_imu'))
    # )

    esp32_serial_node = Node(
        package='deliver_robot',
        executable='esp32_serial_node.py',
        name='esp32_serial_node',
        output='screen',
        parameters=[
            esp32_config_file,
            {'use_imu': LaunchConfiguration('use_imu')}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ]
    )

    return LaunchDescription([
        declare_use_imu_arg,
        declare_use_sim_time_arg,
        # robot_localization_node,
        esp32_serial_node
    ])