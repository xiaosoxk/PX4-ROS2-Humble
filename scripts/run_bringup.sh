#!/usr/bin/env bash
# run_bringup.sh — 机载一键启动（供 systemd uav-ai.service 调用）
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
sudo nvpmodel -m 0 >/dev/null 2>&1 && sudo jetson_clocks >/dev/null 2>&1 || true
export RCUTILS_COLORIZED_OUTPUT=1
exec ros2 launch bringup drone_bringup.launch.py
