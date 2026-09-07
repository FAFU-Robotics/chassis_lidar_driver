#!/bin/bash
# Official MOLA-LO mapping (tutorial):
#   https://docs.mola-slam.org/latest/tutorial-mola-lo-map-and-localize.html
#
# Airy is mounted at vehicle center, looking forward. TF base_link→rslidar
# is roll +90° and yaw +90° (sensor Z into vehicle XY, sensor Y→vehicle Z) plus
# z=0.365 m. Wheels publish odom→base_link; MOLA publishes map→odom (REP-105).
# generate_simplemap:=True so /map_save writes *.mm+*.simplemap.
#
# Keep rslidar_sdk and maps/publish_wheel_odom.py running.
# Jetson: mola_viz GUI segfaults; watch RViz maps/mola_lo.rviz instead.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/fafu_robot/rslidar_ws/install/setup.bash
MAP_DIR="$(cd "$(dirname "$0")" && pwd)"
# FastDDS SHM port 7411 often fails to lock on this Jetson; use UDP so
# MOLA can actually receive /rslidar_points.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$MAP_DIR/fastdds_udp.xml}"
export MOLA_SIMPLEMAP_OUTPUT="${MOLA_SIMPLEMAP_OUTPUT:-$MAP_DIR/lab2.simplemap}"
export MOLA_SAVE_MM="${MOLA_SAVE_MM:-$MAP_DIR/lab2.mm}"
export MOLA_ROS2_PUBLISH_IN_SIM_TIME=false
export MOLA_OPTIMIZE_TWIST=false
export MOLA_MAX_TIME_TO_USE_VELOCITY_MODEL="${MOLA_MAX_TIME_TO_USE_VELOCITY_MODEL:-2.5}"
export MOLA_NAVSTATE_SIGMA_POSITION="${MOLA_NAVSTATE_SIGMA_POSITION:-0.06}"
export MOLA_NAVSTATE_SIGMA_ANG="${MOLA_NAVSTATE_SIGMA_ANG:-0.08}"
export MOLA_LO_ROBUST_KERNEL_PRIOR_REF_BLEND="${MOLA_LO_ROBUST_KERNEL_PRIOR_REF_BLEND:-0.8}"
export MOLA_MINIMUM_ICP_QUALITY="${MOLA_MINIMUM_ICP_QUALITY:-0.50}"
export MOLA_SIGMA_INITIAL="${MOLA_SIGMA_INITIAL:-0.15}"
export MOLA_SIGMA_MAX_MOTION="${MOLA_SIGMA_MAX_MOTION:-0.30}"
export MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES="${MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES:-0.25}"
export MOLA_MIN_ROT_BETWEEN_MAP_UPDATES="${MOLA_MIN_ROT_BETWEEN_MAP_UPDATES:-20}"
export MOLA_MINIMUM_RANGE_FILTER="${MOLA_MINIMUM_RANGE_FILTER:-0.50}"
export MOLA_MAXIMUM_RANGE_FILTER="${MOLA_MAXIMUM_RANGE_FILTER:-15.0}"
unset IMU_POSE_PITCH IMU_POSE_ROLL IMU_POSE_YAW
exec ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
  lidar_topic_name:=/rslidar_points \
  lidar_topic_type:=PointCloud2 \
  ignore_lidar_pose_from_tf:=False \
  publish_localization_following_rep105:=True \
  enforce_planar_motion:=True \
  lidar_qos_reliability:=best_effort \
  use_rviz:=False \
  use_mola_gui:=False \
  generate_simplemap:=True \
  start_mapping_enabled:=True \
  odom_topic_name:=/wheel_odom \
  mola_lo_pipeline:="$MAP_DIR/pipelines/lidar3d-icp-airy.yaml"
