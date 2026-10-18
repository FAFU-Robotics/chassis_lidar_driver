#!/usr/bin/env bash
# Agent (conda Python) 不能 import rclpy。本脚本用系统 Python 订 /rslidar_points。
# 不要 bind UDP 6699。FastDDS 与建图相同，走 UDP，避免 Jetson SHM 7411 锁失败。
set -eo pipefail
unset PYTHONHOME
unset PYTHONPATH
MAP_DIR="$(cd "$(dirname "$0")" && pwd)"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # Humble setup.bash 会读未定义的 AMENT_TRACE_SETUP_FILES。
  # 若这里开着 nounset，桥进程瞬间 exit 1，网页雷达就会一直离线。
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "未找到 /opt/ros/humble/setup.bash，无法订 /rslidar_points" >&2
  exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f "$MAP_DIR/fastdds_udp.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$MAP_DIR/fastdds_udp.xml"
fi
PY="${BUNKER_ROS_PYTHON:-/usr/bin/python3}"
exec "$PY" "$MAP_DIR/rslidar_cloud_bridge.py" "$@"
