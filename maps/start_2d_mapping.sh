#!/bin/bash
# Indoor 2D mapping: Airy horizontal slice → /scan + wheel odom TF → slam_toolbox.
# Do not run together with MOLA (both want map→base_link). Keep rslidar_sdk running.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/fafu_robot/rslidar_ws/install/setup.bash
MAP_DIR="$(cd "$(dirname "$0")" && pwd)"
# Leave unset so publish_wheel_odom.py sniffs can0/can1 for 0x221.

exec ros2 launch "$MAP_DIR/legacy_2d/airy_2d_mapping.launch.py"
