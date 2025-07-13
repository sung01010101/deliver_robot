#!/bin/bash

# Set directory where maps are stored
MAP_DIR=~/dev1_ws/maps

# Check if map directory exists
if [ ! -d "$MAP_DIR" ]; then
  echo "Map directory $MAP_DIR does not exist. Exiting."
  exit 1
fi

# List .yaml files in the directory
echo "Available maps:"
select map_file in "$MAP_DIR"/*.yaml; do
  if [[ -n "$map_file" ]]; then
    echo "You selected: $map_file"
    break
  else
    echo "Invalid selection. Try again."
  fi
done

# Start map_server in Terminal 1
gnome-terminal --tab --title="Map Server" -- bash -c "ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=$map_file; exec bash"

# Start RViz2 in Terminal 2
RVIZ_CONFIG=~/rviz_config/read_map_config.rviz
if [ -f "$RVIZ_CONFIG" ]; then
  gnome-terminal --tab --title="RViz2" -- bash -c "rviz2 -d $RVIZ_CONFIG; exec bash"
else
  echo "RViz config not found. Starting default RViz2."
  gnome-terminal --tab --title="RViz2" -- bash -c "rviz2; exec bash"
fi

# Start lifecycle configuration in Terminal 3
gnome-terminal --tab --title="Lifecycle Cmds" -- bash -c "\
ros2 lifecycle set /map_server configure; \
sleep 1; \
ros2 lifecycle set /map_server activate; \
sleep 1; \
exec bash"
echo "Note: If Transition failed, try close all existing terminals and try again !"
