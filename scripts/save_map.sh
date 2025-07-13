#!/bin/bash

# Ask user for filename
read -p "Enter map name to save (without extension): " map_name

# Ensure the ~/maps directory exists
mkdir -p ~/dev1_ws/maps
cd ~/maps || { echo "Failed to access ~/maps"; exit 1; }

# Run the map saver
echo "Saving map to ~/maps/${map_name}.pgm and ${map_name}.yaml ..."
ros2 run nav2_map_server map_saver_cli -f "$map_name"

