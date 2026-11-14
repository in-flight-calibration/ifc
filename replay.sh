#!/bin/bash
set -e

if [[ $# -ne 1 || ! -d "$1" ]]; then
	echo "Usage: $0 <record>" >&2
	exit 1
fi

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /opt/ros/$(ls /opt/ros | sort -V | tail -n 1)/setup.bash 
source install/setup.bash

zenoh_pid=
play_pid=

cleanup() {
	trap - EXIT INT TERM
	for pid in "$play_pid" "$zenoh_pid"; do
		[[ -n "$pid" ]] || continue
		kill "$pid" 2>/dev/null || true
		wait "$pid" 2>/dev/null || true
	done
}

trap cleanup EXIT
trap 'exit 1' TERM INT

"$(ros2 pkg prefix rmw_zenoh_cpp)/lib/rmw_zenoh_cpp/rmw_zenohd" &
zenoh_pid=$!

ros2 bag play "$1" --loop &
play_pid=$!
wait "$play_pid"
