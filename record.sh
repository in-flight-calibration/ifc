#!/bin/bash

set -e

usage() {
  echo "Usage: $0 [-h|--help] [output_path]"
  echo "  output_path  Bag output directory (default: <script_dir>/records/<YYYYmmdd_HH:MM:SS>)"
}

if [ "$#" -gt 1 ]; then
  usage >&2
  exit 1
fi

case "$1" in
  -h|--help) usage; exit 0 ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT="${1:-$SCRIPT_DIR/records/$(date +%Y%m%d_%H:%M:%S)}"
mkdir -p "$(dirname "$OUTPUT")"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /opt/ros/$(ls /opt/ros | sort -V | tail -n 1)/setup.bash 
source "$SCRIPT_DIR/install/setup.bash"

ros2 bag record \
  -o "$OUTPUT" \
  --topics \
  /stream_telemetry/stream_telemetry
