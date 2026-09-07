#!/usr/bin/env bash
# Mapping + local autonomy bringup. Does not send CAN unless you pass --i-allow-motion.
#
# Modes:
#   mapping   lidar + TF + wheel odom + MOLA mapping (dense KF). You drive with teleop.
#   save      call /map_save into maps/runs/<name>/
#   sense     lidar + TF + wheel odom + MOLA nav-odometry (dense KF) + obstacle_grid
#   status    check live topics
#   plan      A* dry-run (no stick)
#   stop-mola stop only MOLA (keep lidar)
#   live-avoid  REQUIRES --i-allow-motion; existing run_live_avoid.py
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
MAPS="$ROOT/maps"
LOG=/tmp/nav_demo_bringup
RUNS="$MAPS/runs"
mkdir -p "$LOG" "$RUNS"
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

start_lidar_tf_wheel() {
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
}

stop_mola() {
  mapfile -t MOLA_PIDS < <(ps -C mola-cli -o pid= | awk '{print $1}')
  mapfile -t LAUNCH_PIDS < <(ps -eo pid,cmd | awk '/ros2-lidar-odometry.launch.py/ && !/awk/ {print $1}')
  for p in "${LAUNCH_PIDS[@]:-}" "${MOLA_PIDS[@]:-}"; do
    [[ -n "${p:-}" ]] || continue
    echo "stop MOLA pid $p"
    kill "$p" 2>/dev/null || true
  done
  sleep 2
}

wait_pose() {
  echo "waiting for /lidar_odometry/pose ..."
  local ok=0
  for _ in $(seq 1 40); do
    if timeout 2 ros2 topic echo /lidar_odometry/pose --once >/dev/null 2>&1; then
      ok=1
      break
    fi
    sleep 1
  done
  if [[ "$ok" -ne 1 ]]; then
    echo "FAIL: pose not ready. tail $LOG/mola.log:"
    tail -40 "$LOG/mola.log" || true
    exit 2
  fi
  echo "pose ready"
}

cmd_status() {
  echo "=== processes ==="
  alive 'rslidar_sdk' && echo "rslidar_sdk: yes" || echo "rslidar_sdk: NO"
  alive 'mola-cli' && echo "mola-cli: yes" || echo "mola-cli: NO"
  alive 'cloud_to_grid.py' && echo "cloud_to_grid: yes" || echo "cloud_to_grid: NO"
  alive 'publish_wheel_odom.py' && echo "wheel_odom: yes" || echo "wheel_odom: NO"
  echo "=== topics (2s) ==="
  timeout 3 ros2 topic hz /rslidar_points --window 8 2>/dev/null | tail -3 || echo "no /rslidar_points"
  timeout 3 ros2 topic hz /lidar_odometry/pose --window 8 2>/dev/null | tail -3 || echo "no pose"
  timeout 3 ros2 topic hz /nav_demo/obstacle_grid --window 5 2>/dev/null | tail -3 || echo "no obstacle_grid"
}

cmd_mapping() {
  start_lidar_tf_wheel
  if alive 'mola-cli'; then
    echo "MOLA already running. stop-mola first if you need mapping mode."
    exit 2
  fi
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  local out="$RUNS/hall_${stamp}"
  mkdir -p "$out"
  echo "$out" >"$LOG/last_map_run.txt"
  export MOLA_SIMPLEMAP_OUTPUT="$out/map.simplemap"
  export MOLA_SAVE_MM="$out/map.mm"
  echo "start MOLA mapping -> $out"
  echo "drive SLOW with teleop. pause on corners. no snap spins."
  nohup bash "$MAPS/start_mola_mapping.sh" >"$LOG/mola.log" 2>&1 &
  wait_pose
  echo "mapping live. save with:  bash $ROOT/nav_demo/scripts/autonomy.sh save"
}

cmd_save() {
  local dest
  if [[ -n "${1:-}" ]]; then
    dest="$RUNS/$1"
  elif [[ -f "$LOG/last_map_run.txt" ]]; then
    dest="$(cat "$LOG/last_map_run.txt")"
  else
    dest="$RUNS/hall_$(date +%Y%m%d_%H%M%S)"
  fi
  mkdir -p "$dest"
  echo "map_save prefix=$dest/map"
  bash "$MAPS/map_save.sh" "$dest/map"
  echo "saved. loc-from-map still requires adding this map to maps/qualified.json after you inspect it."
}

cmd_sense() {
  start_lidar_tf_wheel
  if alive 'mola-cli'; then
    echo "MOLA already running (keep it). starting grid if needed."
  else
    echo "start MOLA nav-odometry with dense KF (0.25 m / 20 deg) — env only, no YAML edit"
    export MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES=0.25
    export MOLA_MIN_ROT_BETWEEN_MAP_UPDATES=20
    export MOLA_LO_PUBLISH_DESKEWED_SCANS=true
    nohup bash "$ROOT/nav_demo/scripts/start_mola_nav.sh" >"$LOG/mola.log" 2>&1 &
    wait_pose
  fi
  if ! alive 'nav_demo/cloud_to_grid.py'; then
    echo "start cloud_to_grid"
    nohup /usr/bin/python3 "$ROOT/nav_demo/cloud_to_grid.py" >"$LOG/cloud_to_grid.log" 2>&1 &
    sleep 2
  fi
  echo "sense ready. plan with:  bash $ROOT/nav_demo/scripts/autonomy.sh plan"
}

cmd_plan() {
  exec /usr/bin/python3 "$ROOT/nav_demo/plan_once.py" "$@"
}

cmd_live_avoid() {
  if [[ "${1:-}" != "--i-allow-motion" ]]; then
    echo "REFUSED: live-avoid sends CAN via :9100."
    echo "Re-run only when the chassis is clear and you explicitly allow motion:"
    echo "  bash $ROOT/nav_demo/scripts/autonomy.sh live-avoid --i-allow-motion"
    exit 2
  fi
  shift
  exec /usr/bin/python3 "$ROOT/nav_demo/run_live_avoid.py" "$@"
}

usage() {
  cat <<EOF
usage: autonomy.sh <command>

  mapping              start lidar/TF/wheel + MOLA mapping (you teleop; no CAN from this script)
  save [name]          /map_save into maps/runs/<name or last run>
  sense                start lidar/TF/wheel + dense MOLA odom + obstacle_grid (no CAN)
  status               topic/process check
  plan [plan_once args]   A* dry-run, e.g. plan --forward 1.8
  stop-mola            stop MOLA only
  live-avoid --i-allow-motion   existing live avoid (SENDS CAN)

logs: $LOG
EOF
}

cmd="${1:-}"
shift || true
case "$cmd" in
  mapping) cmd_mapping ;;
  save) cmd_save "${1:-}" ;;
  sense) cmd_sense ;;
  status) cmd_status ;;
  plan) cmd_plan "$@" ;;
  stop-mola) stop_mola ;;
  live-avoid) cmd_live_avoid "$@" ;;
  -h|--help|"") usage ;;
  *) echo "unknown command: $cmd"; usage; exit 2 ;;
esac
