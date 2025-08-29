#!/usr/bin/env bash
# Stop processes started by start_sensors.sh when using background jobs or tmux.

set -euo pipefail
SESSION_NAME="robot_stack"

stop_tmux() {
  if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    tmux kill-session -t "$SESSION_NAME"
    echo "Tmux session $SESSION_NAME terminated."
  else
    echo "No tmux session named $SESSION_NAME found." >&2
  fi
}

stop_jobs() {
  for f in lidar.pid esp32.pid imu.pid; do
    if [[ -f $f ]]; then
      PID=$(cat "$f")
      if kill -0 "$PID" 2>/dev/null; then
        kill "$PID" && echo "Stopped PID $PID ($f)" || echo "Failed to stop PID $PID ($f)"
      else
        echo "Process $PID from $f not running."
      fi
      rm -f "$f"
    fi
  done
}

# Detect if tmux session exists; if so stop it, else stop jobs.
if command -v tmux >/dev/null 2>&1 && tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  stop_tmux
else
  stop_jobs
fi
