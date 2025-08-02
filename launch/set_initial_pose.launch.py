#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    x = DeclareLaunchArgument('x', default_value='0.0', description='Position of robot')
    y = DeclareLaunchArgument('y', default_value='0.0', description='Position of robot')
    z = DeclareLaunchArgument('z', default_value='0.0', description='Orientation of robot')
    w = DeclareLaunchArgument('w', default_value='0.0', description='Orientation of robot')

    set_initial_pose_node = Node(
        package='deliver_robot',
        executable='set_initial_pose',
        name='set_initial_pose',
        output='screen',
        parameters=[
            {'x': LaunchConfiguration('x')},
            {'y': LaunchConfiguration('y')},
            {'z': LaunchConfiguration('z')},
            {'w': LaunchConfiguration('w')}
        ],
    )

    return LaunchDescription([
        x,
        y,
        z,
        w,
        set_initial_pose_node
    ])