#!/bin/bash
# Tutorial §1.3: save the current MOLA-LO map (writes prefix.mm + prefix.simplemap).
set -eo pipefail
source /opt/ros/humble/setup.bash
PREFIX="${1:-$(cd "$(dirname "$0")" && pwd)/lab2}"
ros2 service call /map_save mola_msgs/srv/MapSave "{map_path: '${PREFIX}'}"
