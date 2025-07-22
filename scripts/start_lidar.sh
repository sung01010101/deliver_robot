#!/bin/bash

# This script is used to start the RPLIDAR A2M12 sensor with ROS 2.
# put this file under rplidar_ws/start_lidar.sh and make it executable: chmod +x start_lidar.sh

source ~/rplidar_ws/install/setup.bash
#sudo chmod 777 /dev/ttyUSB*
ros2 launch sllidar_ros2 sllidar_a2m12_launch.py serial_port:=/dev/rplidar
