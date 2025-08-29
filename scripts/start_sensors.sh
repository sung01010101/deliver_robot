#!/usr/bin/env bash
# Start three processes in separate gnome-terminal windows or tmux session.
# Falls back to background jobs if gnome-terminal not available.

set -euo pipefail

LIDAR_CMD="~/sllidar_ws/start_lidar.sh"
ESP32_CMD="ros2 launch deliver_robot esp32_serial.launch.py"
IMU_CMD="ros2 launch wit_ros2_imu rviz_and_imu.launch.py"

SESSION_NAME="robot_stack"
MODE="jobs"  # default fallback

if command -v tmux >/dev/null 2>&1; then
  MODE="tmux"
elif command -v gnome-terminal >/dev/null 2>&1; then
  MODE="gnome"
fi

echo "Starting sensors using mode: $MODE"

start_tmux() {
  if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "Tmux session $SESSION_NAME already exists. Attach with: tmux attach -t $SESSION_NAME" >&2
    exit 1
  fi
  tmux new-session -d -s "$SESSION_NAME" -n lidar "bash -lc '$LIDAR_CMD'"
  tmux new-window -t "$SESSION_NAME" -n esp32 "bash -lc '$ESP32_CMD'"
  tmux new-window -t "$SESSION_NAME" -n imu "bash -lc '$IMU_CMD'"
  echo "Tmux session $SESSION_NAME started. Attach: tmux attach -t $SESSION_NAME"
}

start_gnome() {
  gnome-terminal -- bash -lc "$LIDAR_CMD; exec bash" &
  gnome-terminal -- bash -lc "$ESP32_CMD; exec bash" &
  gnome-terminal -- bash -lc "$IMU_CMD; exec bash" &
  echo "Started in gnome-terminal windows. Close windows to stop."
}

start_jobs() {
  bash -lc "$LIDAR_CMD" & PID1=$!
  bash -lc "$ESP32_CMD" & PID2=$!
  bash -lc "$IMU_CMD" & PID3=$!
  echo $PID1 > lidar.pid
  echo $PID2 > esp32.pid
  echo $PID3 > imu.pid
  echo "Started background jobs. Use ./stop_sensors.sh to stop them."
}

case "$MODE" in
  tmux) start_tmux ;;
  gnome) start_gnome ;;
  jobs) start_jobs ;;
  *) start_jobs ;;
 esac
