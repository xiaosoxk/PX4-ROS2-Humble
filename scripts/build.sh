#!/usr/bin/env bash
# build.sh — 机载 ROS2 工作空间编译（Orin NX, Ubuntu 22.04 + ROS2 Humble）
set -e
cd "$(dirname "$0")/../ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "== 编译完成，source install/setup.bash 后运行 scripts/run_bringup.sh =="
