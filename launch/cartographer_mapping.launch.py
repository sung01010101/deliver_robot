"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

"""
調整參數：
use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False') 原本為 True

額外加入 cartographer_node / cartographer_occupancy_grid_node / rviz_node
cartographer_node 中的 FindPackageShare 的最後一個 argument, 改成 'lidar_with_odometry.lua' (對應的.lua檔名)

加入 rviz 的 config 設定
rviz_config_path = os.path.expanduser('~/cartographer_ws/rviz_config/cartographer_config.rviz')
rviz_node 多一個 arguments=['-d', rviz_config_path],

加入 gui_arg 和 joint_state_publisher_gui_node, 顯示模型
"""


def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

    ## ***** Urdf Node *****
    pkg_share = FindPackageShare('deliver_robot').find('deliver_robot')
    urdf_dir = os.path.join(pkg_share, 'description')
    urdf_file = os.path.join(urdf_dir, 'robot_model.urdf')  # 使用 robot_model.urdf 設定
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output = 'screen'
        )

    ## ***** Map Node *****
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory', FindPackageShare('deliver_robot').find('deliver_robot') + '/config',
            '-configuration_basename', 'cartographer_mapping.lua'],
        remappings = [
            ('scan', 'scan')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
        )

    # Rviz 設定檔儲存路徑  
    rviz_dir = os.path.join(pkg_share, 'rviz')
    rviz_file = os.path.join(rviz_dir, 'cartographer.rviz')

    ## ***** Rviz Node *****
    rviz_node = Node(
        package='rviz2',
        namespace='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='screen')

    # 顯示轉動 gui    
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    
    # 顯示可以轉的元件（車輪）
    joint_state_publisher_gui_node = Node(
    	package='joint_state_publisher_gui',
    	executable='joint_state_publisher_gui',
    	name='joint_state_publisher_gui',
    	condition=IfCondition(LaunchConfiguration('gui')),
    	parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
	)
	                                    
    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
