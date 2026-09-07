#!/bin/bash
# Empty /horizon_localmap and zero wheel odom. Keep lidar, TF, RViz, teleop.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MAP_DIR="$ROOT/maps"
pkill -f "$MAP_DIR/publish_horizon_localmap.py" 2>/dev/null || true
pkill -f "$MAP_DIR/publish_wheel_odom.py" 2>/dev/null || true
sleep 1
unset CONDA_PREFIX BUNKER_CAN_CHANNEL
export PATH=/usr/bin:$PATH
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$MAP_DIR/fastdds_udp.xml}"
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source /home/fafu_robot/rslidar_ws/install/setup.bash
nohup python3 "$MAP_DIR/publish_wheel_odom.py" >/tmp/wheel_odom.log 2>&1 &
sleep 1
nohup python3 "$MAP_DIR/publish_horizon_localmap.py" >/tmp/horizon_localmap.log 2>&1 &
echo "已清空：里程计归零，/horizon_localmap 已空。RViz Fixed Frame=odom，只开 LocalMap。"
echo "停下不会画图；按 W 直线开出去才会加点。"
