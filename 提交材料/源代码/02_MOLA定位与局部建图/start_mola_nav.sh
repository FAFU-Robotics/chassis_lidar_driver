#!/bin/bash
# nav_demo MOLA launcher: lidar odometry for navigation, not mapping, not loc-from-map.
#
# Confirmed (do not guess) from:
#   /opt/ros/humble/share/mola_lidar_odometry/ros2-launchs/ros2-lidar-odometry.launch.py
#     generate_simplemap  -> env MOLA_GENERATE_SIMPLEMAP
#     start_mapping_enabled -> env MOLA_MAPPING_ENABLED
#   maps/pipelines/lidar3d-icp-airy.yaml
#     local_map_updates.enabled: ${MOLA_MAPPING_ENABLED|true}
#     local_map_updates.min_translation_between_keyframes: ${MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES|...}
#     simplemap.generate: ${MOLA_GENERATE_SIMPLEMAP|false}
#
# Modes (keep separate):
#   建图:     maps/start_mola_mapping.sh
#             generate_simplemap:=True  start_mapping_enabled:=True
#             MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES=0.25  and saves lab2.mm/simplemap
#   读图定位: maps/start_mola_localization.sh
#             generate_simplemap:=False start_mapping_enabled:=False
#             loads a saved .mm + .simplemap (start_active:=False)
#   导航里程计 (this file):
#             generate_simplemap:=False  (no simplemap keyframe I/O)
#             start_mapping_enabled:=True (must seed localmap for ICP; no saved map)
#             MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES=1.5  (1 m demo never hits next KF)
#             does NOT set MOLA_SAVE_MM / MOLA_SIMPLEMAP_OUTPUT
#
# Why not start_mapping_enabled:=False here:
#   LidarOdometry.h: mapping enabled=false is localization-only and expects
#   load_existing_local_map. Without a loaded .mm, localmap stays empty.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/fafu_robot/rslidar_ws/install/setup.bash
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
MAP_DIR="$ROOT/maps"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$MAP_DIR/fastdds_udp.xml}"
export MOLA_LO_PUBLISH_DESKEWED_SCANS="${MOLA_LO_PUBLISH_DESKEWED_SCANS:-true}"
export MOLA_ROS2_PUBLISH_IN_SIM_TIME=false
export MOLA_OPTIMIZE_TWIST=false
export MOLA_MAX_TIME_TO_USE_VELOCITY_MODEL="${MOLA_MAX_TIME_TO_USE_VELOCITY_MODEL:-2.5}"
export MOLA_NAVSTATE_SIGMA_POSITION="${MOLA_NAVSTATE_SIGMA_POSITION:-0.06}"
export MOLA_NAVSTATE_SIGMA_ANG="${MOLA_NAVSTATE_SIGMA_ANG:-0.08}"
export MOLA_LO_ROBUST_KERNEL_PRIOR_REF_BLEND="${MOLA_LO_ROBUST_KERNEL_PRIOR_REF_BLEND:-0.8}"
export MOLA_MINIMUM_ICP_QUALITY="${MOLA_MINIMUM_ICP_QUALITY:-0.50}"
export MOLA_SIGMA_INITIAL="${MOLA_SIGMA_INITIAL:-0.15}"
export MOLA_SIGMA_MAX_MOTION="${MOLA_SIGMA_MAX_MOTION:-0.30}"
# Skip localmap KF during the ~1 m live envelope (second live gap was at 0.25 m).
export MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES="${MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES:-1.5}"
export MOLA_MIN_ROT_BETWEEN_MAP_UPDATES="${MOLA_MIN_ROT_BETWEEN_MAP_UPDATES:-90}"
export MOLA_MINIMUM_RANGE_FILTER="${MOLA_MINIMUM_RANGE_FILTER:-0.50}"
export MOLA_MAXIMUM_RANGE_FILTER="${MOLA_MAXIMUM_RANGE_FILTER:-15.0}"
unset IMU_POSE_PITCH IMU_POSE_ROLL IMU_POSE_YAW
unset MOLA_SAVE_MM MOLA_SIMPLEMAP_OUTPUT
echo "MOLA nav-odometry (not mapping, not loc-from-map)" >&2
echo "  generate_simplemap:=False  start_mapping_enabled:=True" >&2
echo "  MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES=$MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES" >&2
echo "  MOLA_MIN_ROT_BETWEEN_MAP_UPDATES=$MOLA_MIN_ROT_BETWEEN_MAP_UPDATES" >&2
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
  start_mapping_enabled:=True \
  odom_topic_name:=/wheel_odom \
  mola_lo_pipeline:="$MAP_DIR/pipelines/lidar3d-icp-airy.yaml"
