#!/usr/bin/env bash
# sim.sh — PX4 SITL + Gazebo 仿真：所有 AI 节点先在仿真里过一遍再上实机
# 依赖：PX4-Autopilot(v1.14+) + ros_gz + MAVROS
set -e
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
cd "$PX4_DIR"
DONT_RUN=1 make px4_sitl gz_x500 2>/dev/null &        # 预编译
sleep 5
HEADLESS=1 make px4_sitl gz_x500 &                    # 启动 SITL
sleep 15
ros2 run mavros mavros_node --ros-args -p fcu_url:="udp://:14540@"
ros2 launch bringup drone_bringup.launch.py
