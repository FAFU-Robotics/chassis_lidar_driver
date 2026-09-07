#!/usr/bin/env bash
# Agent (conda Python) 不能 import rclpy。本脚本用系统 Python 订 TF map→base_link。
# FastDDS 与建图/定位相同，走 UDP，避免 Jetson SHM 7411 锁失败。
set -eo pipefail
unset PYTHONHOME
unset PYTHONPATH
MAP_DIR="$(cd "$(dirname "$0")" && pwd)"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "未找到 /opt/ros/humble/setup.bash，无法订 TF" >&2
  exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f "$MAP_DIR/fastdds_udp.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$MAP_DIR/fastdds_udp.xml"
fi
PY="${BUNKER_ROS_PYTHON:-/usr/bin/python3}"
exec "$PY" "$MAP_DIR/tf_pose_bridge.py" "$@"
