# Copilot instructions: ifc workspace

ROS 2 **lyrical** colcon workspace for in-flight calibration (C++20, OpenCV 4.10). This repo only holds the workspace root; every package under `src/` is a **git submodule** with its own repo (`in-flight-calibration/<package>`). Commit package changes inside the submodule first, then commit the updated submodule pointer here.

## Build
Always run from the workspace root:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-up-to horizon_detection   # one package + its deps
colcon build --packages-select ifc_tools          # just one package
source install/setup.bash
ros2 run ifc_tools mosaic
```

All C++ packages compile with `-Wall -Wextra -Wpedantic`; keep builds warning-free. There are no linters configured.

## Packages
- `ifc_tools` – header-only `INTERFACE` target (namespace `ifc_tools`) shared by other packages. `ImageReceiver` is an `rclcpp::Node`: subscribes through `image_transport` with `SensorDataQoS`, converts to `bgr8`, forwards `(rclcpp::Time, cv::Mat)` to a callback (the `Mat` is only valid during the callback). `DebugViewer` is a pure-OpenCV fixed-grid mosaic window with per-tile FPS. See `src/ifc_tools/.github/copilot-instructions.md`.
- `horizon_detection` – header-only C++20 detector library plus `horizon_detection_node`. `HorizonDetection` owns the `ImageReceiver`, detector pointer, and ROS publisher; `DetectorFactory` selects the implementation. `Detector` and `HorizonDetection` are ROS nodes. `DetectorAlpha` runs the staged OpenCV Hough pipeline; the horizon convention and stages are in `src/horizon_detection/.github/copilot-instructions.md`. Current work is planned in the root `PLAN.md`.
- `ifc_msgs` – project-specific ROS messages; every `msg/*.msg` is picked up automatically via `file(GLOB ...)` (re-run cmake/colcon after adding one).
- `px4_msgs` – **generated** subset of PX4 messages. Do not hand-edit `msg/`; run `python3 src/px4_msgs/sync.py`, which clones the PX4 fork/branch from `config.yaml` and copies the messages referenced in its DDS topics file (plus transitive dependencies). `.px4_msgs_generated` tracks which files it owns.

## Runtime data flow
FlightGear simulator (separate repo) publishes camera frames on `/flightgear/image` (`bgr8`, `SensorDataQoS`); consumers subscribe with the `compressed` transport (`compressed_image_transport` must be installed). Running GUI examples needs that publisher and a desktop.

`./ifc.sh` builds the workspace and runs `horizon_detection` with root `config.yaml`. `HorizonDetection::publish()` sets `header.stamp` to publication time and forwards the source image timestamp in `image_timestamp`. `ifc_msgs/msg/Horizon` has `header`, `image_timestamp`, `theta`, `rho`, `confidence`, `theta_sd`, and `sigma_rho`.

## Conventions
- `#pragma once`, never `#ifndef` include guards.
- Private members prefixed with `_` (e.g. `_callback_mutex`).
- Allman braces (opening brace on its own line), 4-space indent, blank line before and after every block (`if`/`for`/`try` etc.).
- `Detector` owns an optional `DebugViewer`, enabled by the `horizon.debug_view` parameter (default `false`).
- Avoid unnecessary comments.
- New dependencies must be added to both `CMakeLists.txt` (`find_package`, link, and `ament_export_dependencies` for libraries) and `package.xml`.
