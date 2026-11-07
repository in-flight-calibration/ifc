#!/bin/bash

set -e
cd "$(dirname "$0")"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp

ros_distro="$(ls /opt/ros | sort -V | tail -n 1)"
source "/opt/ros/${ros_distro}/setup.bash"

colcon build
source install/setup.bash
ros2 launch ifc launch.py params_file:=config.yaml
