# Copilot instructions: ifc workspace

ROS 2 **lyrical** colcon workspace for in-flight calibration (C++20, OpenCV 4.10). This repo holds the workspace root; the packages registered in `.gitmodules` are **git submodules** with their own repos (`in-flight-calibration/<package>`). Commit package changes inside the submodule first, then commit the updated submodule pointer here.

## Build
Always run from the workspace root:

```bash
source /opt/ros/lyrical/setup.bash
colcon build                                    # all packages
colcon build --packages-up-to horizon_detection   # one package + its deps
colcon build --packages-select ifc_tools          # just one package
colcon build --packages-up-to delay_estimator horizon_detection --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 run horizon_detection horizon_detection --ros-args --params-file config.yaml
ros2 run ifc_tools mosaic
```

All C++ packages compile with `-Wall -Wextra -Wpedantic`; keep builds warning-free. There are no linters configured.

## Test
After building `horizon_detection` and sourcing `install/setup.bash`, run the single bounded, headless synthetic test:

```bash
ros2 run horizon_detection synthetic_horizon --ros-args -p horizon.debug_view:=false -p duration:=30.0
```

It generates images with a horizon and runway, calls the detector directly, and needs no FlightGear publisher. Exit status is nonzero unless detection rate is at least 90%, theta RMSE is at most 1 degree, and rho RMSE is at most 0.01. `duration:=0.0` runs indefinitely. This command uses header defaults; add `--params-file config.yaml` before the `-p` overrides to test workspace configuration while keeping the GUI disabled. See root `PLAN.md` for the recorded baseline and outstanding acceptance criteria.

The delay/provenance checks are headless CTest executables:

```bash
colcon test --packages-select delay_estimator horizon_detection
colcon test --packages-select delay_estimator --ctest-args -R '^delay_estimator_core$'
colcon test-result --verbose
```

## Packages
- `ifc_tools` – header-only `INTERFACE` target (namespace `ifc_tools`) shared by other packages. `ImageReceiver` is an `rclcpp::Node`: subscribes through `image_transport` with `SensorDataQoS`, converts to `bgr8`, forwards `(rclcpp::Time, cv::Mat)` to a callback (the `Mat` is only valid during the callback). `DebugViewer` is a pure-OpenCV fixed-grid mosaic window with per-tile FPS. See `src/ifc_tools/.github/copilot-instructions.md`.
- `horizon_detection` – header-only C++20 detector implementation plus the `horizon_detection` and `synthetic_horizon` executables. `HorizonDetection` owns the `ImageReceiver`, detector pointer, and ROS publisher; `DetectorFactory` selects the implementation. `Detector` and `HorizonDetection` are ROS nodes. `DetectorAlpha` runs the staged OpenCV Hough pipeline; the horizon convention and stages are in `src/horizon_detection/.github/copilot-instructions.md`. Current work is planned in the root `PLAN.md`.
- `ifc_msgs` – project-specific ROS messages; every `msg/*.msg` is picked up automatically via `file(GLOB ...)` (re-run cmake/colcon after adding one).
- `px4_msgs` – **generated** subset of PX4 messages. Do not hand-edit `msg/`; run `python3 src/px4_msgs/sync.py`, which clones the PX4 fork/branch from `src/px4_msgs/config.yaml` and copies the messages referenced in its DDS topics file (plus transitive dependencies). `msg/.px4_msgs_generated` tracks which files it owns.
- `delay_estimator` – compiled C++20/OpenCV core (`delay_estimation`) plus a diagnostic ROS node. Fits a robust projective mapping for each candidate delay, rejects unobservable/ambiguous solutions, and validates with a held-out temporal block. The node snapshots bounded buffers and fits on a worker, not inside image callbacks. See `src/delay_estimator/writing-block.md` for the model and run/configuration contract.

## Runtime data flow
FlightGear simulator (separate repo) publishes camera frames on `/flightgear/image` (`bgr8`, `SensorDataQoS`); consumers subscribe with the `compressed` transport (`compressed_image_transport` must be installed). Running GUI examples needs that publisher and a desktop.

`./ifc.sh` selects the latest installed ROS distribution under `/opt/ros`, builds the workspace, sources the overlay, and runs `horizon_detection` with root `config.yaml`.

`main.cpp` spins only the receiver in a `SingleThreadedExecutor`. Its image callback synchronously invokes `Detector::detect()` and publishes the result. Root `config.yaml` uses the `/**` ROS parameter selector so overrides reach the separately constructed nodes; detector stage parameters live under `horizon.alpha.*`.

`DetectorAlpha` runs prefilter -> Canny -> morphological closing -> Hough segments -> line grouping -> horizon estimation. The estimator owns temporal tracking and confidence, using source-image nanoseconds rather than publication time. Keep stage parameter defaults in headers consistent with root `config.yaml`.

`HorizonDetection::publish()` sets `header.stamp` to publication time and forwards the source image timestamp in `image_timestamp`. `ifc_msgs/msg/Horizon.msg` has `header`, `image_timestamp`, `theta`, `rho`, `confidence`, `theta_sd`, and `sigma_rho`. The internal `Horizon` names its angular uncertainty `sigma_theta`, mapped to wire field `theta_sd`. An empty solution (no current detection; held tracks are not published) is sent with `confidence = inf` and NaN `theta`, `rho`, `theta_sd`, `sigma_rho`.

Run the diagnostic node separately with `ros2 run delay_estimator delay_estimator --ros-args --params-file config.yaml`. It consumes finite horizons and PX4 `VehicleAttitude.timestamp_sample` (microseconds), inverts the body-to-world quaternion, and publishes `ifc_msgs/DelayEstimate`. Explicitly configure `delay.camera_to_attitude_seconds` and `delay.clock_offset_configured`; otherwise it reports `CLOCK_UNCONFIGURED`. The relation is `t_attitude = t_image + configured_offset + residual_seconds`. Only `ACCEPTED` results are usable. Parameters are startup-only, and the node never corrects input timestamps or infers clock epochs from arrival times.

Geometry uses `float` in a normalized image-centred frame: `u` points right, `v` points up, and distances are divided by half the image diagonal. The horizon satisfies `-u*sin(theta) + v*cos(theta) = rho`; theta is in radians. Use `HorizonUtils` for pixel conversion and line alignment: changing theta by pi also flips rho.

## Conventions
- `#pragma once`, never `#ifndef` include guards.
- Private members prefixed with `_` (e.g. `_callback_mutex`).
- Allman braces (opening brace on its own line), 4-space indent, blank line before and after every block (`if`/`for`/`try` etc.).
- `Detector` owns an optional `DebugViewer`, enabled by the `horizon.debug_view` parameter (default `false`).
- Avoid unnecessary comments.
- New dependencies must be added to both `CMakeLists.txt` (`find_package`, link, and `ament_export_dependencies` for libraries) and `package.xml`.
- Register horizon executables with `add_horizon_executable` in the package's `CMakeLists.txt`; it supplies shared dependencies and installs them under `lib/horizon_detection`.
