#!/bin/bash

set -e
cd "$(dirname "$0")"

ros_distro="$(ls /opt/ros | sort -V | tail -n 1)"
source "/opt/ros/${ros_distro}/setup.bash"

colcon build
source install/setup.bash
ros2 run horizon_detection horizon_detection --ros-args --params-file config.yaml
