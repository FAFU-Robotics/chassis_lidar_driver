#!/usr/bin/env bash
# Perception-only bringup for nav_demo. No stick, no extra CAN TX, no obstacle_avoidance.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
MAPS="$ROOT/maps"
LOG=/tmp/nav_demo_bringup
mkdir -p "$LOG"
export DISPLAY="${DISPLAY:-:0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$MAPS/fastdds_udp.xml}"
export PATH="/usr/bin:/usr/sbin:$PATH"
export HOME="${HOME:-/home/fafu_robot}"
set +u
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE CONDA_SHLVL
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/fafu_robot/rslidar_ws/install/setup.bash
set -u

alive() { pgrep -f "$1" >/dev/null 2>&1; }

if ! alive 'rslidar_sdk'; then
  echo "start rslidar_sdk"
  nohup ros2 run rslidar_sdk rslidar_sdk_node >"$LOG/rslidar_sdk.log" 2>&1 &
  sleep 2
fi

if ! alive 'tf_airy_mount'; then
  echo "start tf_airy_mount"
  nohup ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0.365 --roll 1.570796 --pitch 0 --yaw 1.570796 \
    --frame-id base_link --child-frame-id rslidar \
    --ros-args -r __node:=tf_airy_mount >"$LOG/tf_airy.log" 2>&1 &
fi

if ! alive 'tf_base_to_footprint'; then
  echo "start tf_base_to_footprint"
  nohup ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id base_footprint \
    --ros-args -r __node:=tf_base_to_footprint >"$LOG/tf_foot.log" 2>&1 &
fi

if ! alive 'publish_wheel_odom.py'; then
  echo "start wheel_odom (listen-only)"
  nohup /usr/bin/python3 "$MAPS/publish_wheel_odom.py" >"$LOG/wheel_odom.log" 2>&1 &
fi

export MOLA_LO_PUBLISH_DESKEWED_SCANS=true
if ! alive 'mola-cli'; then
  echo "start MOLA nav-odometry (generate_simplemap:=False; min_xyz=1.5; no yaml edits)"
  nohup bash "$ROOT/nav_demo/scripts/start_mola_nav.sh" >"$LOG/mola.log" 2>&1 &
fi

echo "waiting for /lidar_odometry/pose ..."
ok=0
for _ in $(seq 1 40); do
  if timeout 2 ros2 topic echo /lidar_odometry/pose --once >/dev/null 2>&1; then
    ok=1
    break
  fi
  sleep 1
done
if [[ "$ok" -ne 1 ]]; then
  echo "FAIL: pose not ready. tail mola.log:"
  tail -40 "$LOG/mola.log" || true
  exit 2
fi
echo "pose ready"

if ! alive 'nav_demo/cloud_to_grid.py'; then
  echo "start cloud_to_grid (perception only)"
  nohup /usr/bin/python3 "$ROOT/nav_demo/cloud_to_grid.py" >"$LOG/cloud_to_grid.log" 2>&1 &
  sleep 2
fi
echo "bringup done. logs in $LOG"
