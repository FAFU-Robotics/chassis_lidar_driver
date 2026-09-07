#!/usr/bin/env bash
# Restart MOLA in nav-demo odometry mode (not mapping). No YAML edits. No stick.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
LOG=/tmp/nav_demo_bringup
mkdir -p "$LOG"
export DISPLAY="${DISPLAY:-:0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROOT/maps/fastdds_udp.xml}"
export MOLA_LO_PUBLISH_DESKEWED_SCANS=true
set +u
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE CONDA_SHLVL
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/fafu_robot/rslidar_ws/install/setup.bash
set -u

echo "MOLA_LO_PUBLISH_DESKEWED_SCANS=$MOLA_LO_PUBLISH_DESKEWED_SCANS"

mapfile -t MOLA_PIDS < <(ps -C mola-cli -o pid= | awk '{print $1}')
mapfile -t LAUNCH_PIDS < <(ps -eo pid,cmd | awk '/ros2-lidar-odometry.launch.py/ && !/awk/ {print $1}')
echo "mola-cli PIDs: ${MOLA_PIDS[*]:-none}"
echo "launch PIDs: ${LAUNCH_PIDS[*]:-none}"
for p in "${LAUNCH_PIDS[@]:-}" "${MOLA_PIDS[@]:-}"; do
  [[ -n "${p:-}" ]] || continue
  echo "SIGTERM $p"
  kill "$p" 2>/dev/null || true
done
sleep 2
for p in "${LAUNCH_PIDS[@]:-}" "${MOLA_PIDS[@]:-}"; do
  [[ -n "${p:-}" ]] || continue
  if kill -0 "$p" 2>/dev/null; then
    echo "still alive $p, SIGTERM again"
    kill "$p" 2>/dev/null || true
  fi
done
sleep 1

nohup bash "$ROOT/nav_demo/scripts/start_mola_nav.sh" >"$LOG/mola.log" 2>&1 &
echo "started start_mola_nav.sh pid $!"
ok=0
for i in $(seq 1 45); do
  if timeout 2 ros2 topic echo /lidar_odometry/pose --once >/dev/null 2>&1; then
    ok=1
    echo "pose ready at ${i}s"
    break
  fi
  sleep 1
done
if [[ "$ok" -ne 1 ]]; then
  echo "FAIL: pose not ready"
  tail -40 "$LOG/mola.log" || true
  exit 2
fi
echo "deskewed info:"
timeout 3 ros2 topic info /lidar_odometry/deskewed_scan_points || true
echo "===== confirmed MOLA nav env (from mola-cli) ====="
MOLA_PID="$(ps -C mola-cli -o pid= | awk '{print $1; exit}')"
if [[ -n "${MOLA_PID:-}" && -r "/proc/$MOLA_PID/environ" ]]; then
  tr '\0' '\n' < "/proc/$MOLA_PID/environ" | grep -E '^(MOLA_MAPPING_ENABLED|MOLA_GENERATE_SIMPLEMAP|MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES|MOLA_MIN_ROT_BETWEEN_MAP_UPDATES|MOLA_SAVE_MM|MOLA_SIMPLEMAP_OUTPUT)=' || true
fi
ps -ef | grep 'ros2-lidar-odometry.launch.py' | grep -v grep || true
