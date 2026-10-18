#!/usr/bin/env bash
# Start PPT RViz only. Does not overwrite maps/mola_lo.rviz and does not drive.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
CFG="$ROOT/ppt_materials/rviz/ppt_live.rviz"
export DISPLAY="${DISPLAY:-:0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROOT/maps/fastdds_udp.xml}"
set +u
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE CONDA_SHLVL
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/fafu_robot/rslidar_ws/install/setup.bash 2>/dev/null || true
set -u

if [[ ! -f "$CFG" ]]; then
  echo "missing $CFG" >&2
  exit 1
fi

echo "DISPLAY=$DISPLAY"
echo "RViz config (PPT only): $CFG"
echo "Will NOT touch: $ROOT/maps/mola_lo.rviz"
if pgrep -af 'rviz2' | grep -v grep >/dev/null; then
  echo
  echo "NOTE: another rviz2 is already running:"
  pgrep -af 'rviz2' | grep -v grep || true
  echo "You can close the old window (maps/mola_lo.rviz) so PPT view is unique."
  echo "This script does not kill it."
fi
echo
echo "Displays on: RawLidar, DeskewedScan, LocalMap, ObstacleGrid, AStarPlan, LidarPose, TF"
echo "Fixed Frame: map   Background: black"
exec rviz2 -d "$CFG"
