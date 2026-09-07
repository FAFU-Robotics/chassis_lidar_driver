#!/bin/bash
# Official MOLA-LO localization:
#   https://docs.mola-slam.org/latest/tutorial-mola-lo-map-and-localize.html
# 必须显式选择合格图（maps/qualified.json）。不会再偷偷加载 lab / lab2。
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/fafu_robot/rslidar_ws/install/setup.bash
MAP_DIR="$(cd "$(dirname "$0")" && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$MAP_DIR/fastdds_udp.xml}"
PREFIX="$(python3 "$MAP_DIR/resolve_loc_map.py" "${1:-}")"
export MOLA_ROS2_PUBLISH_IN_SIM_TIME=false
export MOLA_OPTIMIZE_TWIST=false
export MOLA_MINIMUM_RANGE_FILTER="${MOLA_MINIMUM_RANGE_FILTER:-0.50}"
export MOLA_MAXIMUM_RANGE_FILTER="${MOLA_MAXIMUM_RANGE_FILTER:-15.0}"
unset IMU_POSE_PITCH IMU_POSE_ROLL IMU_POSE_YAW
echo "MOLA localization map prefix: ${PREFIX}" >&2
exec ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
  lidar_topic_name:=/rslidar_points \
  lidar_topic_type:=PointCloud2 \
  ignore_lidar_pose_from_tf:=False \
  publish_localization_following_rep105:=True \
  enforce_planar_motion:=True \
  lidar_qos_reliability:=best_effort \
  use_rviz:=False \
  use_mola_gui:=False \
  generate_simplemap:=False \
  start_mapping_enabled:=False \
  start_active:=False \
  odom_topic_name:=/wheel_odom \
  mola_initial_map_mm_file:="${PREFIX}.mm" \
  mola_initial_map_sm_file:="${PREFIX}.simplemap" \
  mola_lo_pipeline:="$MAP_DIR/pipelines/lidar3d-icp-airy.yaml"
