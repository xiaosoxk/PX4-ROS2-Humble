#!/usr/bin/env bash
# deploy_orin.sh — 从 PC 推送工作空间到 Orin NX 并配置开机自启
set -e
ORIN=${1:-orin@192.168.10.1}
rsync -av --exclude '.git' "$(dirname "$0")/../ros2_ws" "$ORIN:~/"
ssh "$ORIN" 'bash -s' <<'REMOTE'
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && colcon build --symlink-install
sudo nvpmodel -m 0 && sudo jetson_clocks        # MAXN 功耗档+锁频(散热必须到位!)
sudo systemctl restart uav-ai.service            # systemd 单元(见 docs/制作指南 §5)
REMOTE
echo "部署完成"
