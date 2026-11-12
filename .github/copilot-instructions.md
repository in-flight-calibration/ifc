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
ros2 run horizon_detection synthetic_horizon --ros-args -p horizon_detection.debug_view:=false -p duration:=30.0
```

It generates images with a horizon and runway, calls the detector directly, and needs no FlightGear publisher. Exit status is nonzero unless detection rate is at least 90%, theta RMSE is at most 1 degree, and rho RMSE is at most 0.01. `duration:=0.0` runs indefinitely. This command uses header defaults; add `--params-file config.yaml` before the `-p` overrides to test workspace configuration while keeping the GUI disabled.

The delay/provenance checks are headless CTest executables:

```bash
colcon test --packages-select delay_estimator horizon_detection
colcon test --packages-select delay_estimator --ctest-args -R '^delay_estimator_core$'
colcon test-result --verbose
```

## Packages
- `ifc_tools` – header-only `INTERFACE` target (namespace `ifc_tools`) shared by other packages. `ImageReceiver` is an `rclcpp::Node`: subscribes through `image_transport` with `SensorDataQoS`, converts to `bgr8`, forwards `(rclcpp::Time, cv::Mat)` to a callback (the `Mat` is only valid during the callback). `DebugViewer` is a pure-OpenCV fixed-grid mosaic window with per-tile FPS. `RingBuffer<T>` is a fixed-capacity, preallocated overwrite-oldest container. `BufferedReceiver<Message, Value>` is a generic node that converts messages (via a `MessageConverter` interface) into a `RingBuffer` of `(timestamp, value)` pairs and clears it when timestamps go backwards; `snapshot()` copies it to a vector. `Interpolator<Container>` (`interpolator.hpp`) decorates any container of ascending `(timestamp, T)` pairs and interpolates for one or ascending timestamps with an `InterpolationMethod` (default `Nearest<T>`), returning `std::nullopt` outside the time range. See `src/ifc_tools/.github/copilot-instructions.md`.
- `horizon_detection` – header-only C++20 detector implementation plus the `horizon_detection` and `synthetic_horizon` executables. `HorizonDetection` owns the `ImageReceiver`, detector pointer, and ROS publisher; `DetectorFactory` selects the implementation. `Detector` and `HorizonDetection` are ROS nodes. `DetectorAlpha` runs the staged OpenCV Hough pipeline; the horizon convention and stages are in `src/horizon_detection/.github/copilot-instructions.md`.
- `ifc_msgs` – project-specific ROS messages; every `msg/*.msg` is picked up automatically via `file(GLOB ...)` (re-run cmake/colcon after adding one).
- `px4_msgs` – **generated** subset of PX4 messages. Do not hand-edit `msg/`; run `python3 src/px4_msgs/sync.py`, which clones the PX4 fork/branch from `src/px4_msgs/config.yaml` and copies the messages referenced in its DDS topics file (plus transitive dependencies). `msg/.px4_msgs_generated` tracks which files it owns.
- `delay_estimator` – header-only C++20 core (Eigen, Ceres, OpenCV `estimateAffine2D`) plus the `delay_estimator` node. `AttitudeUtils` (PX4 `VehicleAttitude`: µs→ns, slerp; `Eigen::Quaterniond`) and `LocalPositionUtils` (PX4 `VehicleLocalPosition` NED `x, y, z` when `xy_valid && z_valid`: µs→ns, linear) are `ifc_tools::MessageConverter` + `InterpolationMethod` implementations driving `ifc_tools::BufferedReceiver` nodes; `Features` holds the pair timestamps and the affine map `x₂ = A·x₁ + b` (`Eigen::Matrix2d A`, `Eigen::Vector2d b`). `SampleSynchronizer(features, horizon, attitude, position)` (`include/alignment/sample_synchronizer.hpp`) wraps attitude and position in `DataShifter`s; `get_flow(delay)` interpolates both at the unique feature timestamps `t + delay` and returns `std::vector<FlowSample>` (`features`, `attitude[2]`, `position[2]`), dropping pairs whose attitude or position falls outside the buffers; `get_horizon(delay)` returns `std::vector<HorizonSample>` (`horizon`, `attitude` at `t + delay`) for the horizon method; all samples are passed to the method, which does its own filtering. `FlowMethod::fit(samples)` (Method 2) computes per pair the body rotation `ω_b = Log(q₁⁻¹q₂)` and body translation `t_b = q₁⁻¹(p₂ − p₁)`, fits `s = Θ·(ω_b, t_b)` with `s = (b_x, b_y, A₀₁, A₁₀)` (plus `A₀₀−1, A₁₁−1` when `delay_estimator.flow.expansion`, default `false`; `Θ` is 4×6 or 6×6) with Ceres + `HuberLoss` and returns a single fitness `double` J (0 = perfect; NaN when samples, excitation or fit cost are insufficient), and the generic `GridSearch<T, Cost>` (integral `T`, not a node; configured with `setParameters(SearchParameters<T> {min, max, steps, neighbours})`) runs the coarse-to-fine search via `run(evaluate)` and then also evaluates `±neighbours` finest steps around the best value (`delay_estimator.search.neighbours`, default 5), returning `SearchResult<T, Cost> {value, cost, evaluations}` with `evaluations` a sorted `std::vector<std::pair<T, Cost>>` (`cost` is the cost at `value`; invalid when NaN). `HorizonMethod::fit(samples)` (Method 1, `include/method/horizon_method.hpp`) fits a camera mount rotation and focal so that gravity `q⁻¹·(0,0,1)` predicts the horizon line `(θ, ρ)` (residuals scaled by the robust θ spread; detector `theta_sd`/`sigma_rho` unused) and returns J the same way. Both methods run every update; `estimate()` (`include/estimation/fusion.hpp`) turns each search into `Estimate {valid, delay, sd}` by fitting the parabola `J = a + b·x + c·x²` (`x = delay − best`) to the evaluations within `±neighbours` finest steps: the delay is the vertex `best − b/(2c)` and, since J is a scaled sum of squares, `sd = √(J_min / (dof·c))` with `J_min` the fitted minimum and `dof = N − p` from the method's `degrees_of_freedom(samples)` at the best delay (flow `S·(N − 6)`, horizon `2N − 4`); it is invalid when `c ≤ 0`, `J_min ≤ 0`, fewer than 3 points are finite or the vertex lies outside the window. `fuse()` publishes their inverse-variance average and `sd = 1/√Σw`. `delay_estimator.debug_view` (default `false`) draws both search curves in one window (`SearchPlot`, `include/debug.hpp`: `setAxis(min_x, max_x, min_y, max_y)` once with the search range and `[0, max_cost]`, `add(name, x, y)` per curve, `draw()` after the update renders each with `cv::plot::Plot2d` into one image with a 10-line grid, marking and labelling only the minima, `show()` refreshes the window); a `std::jthread` inside `DelayEstimator` (declared after the plot, so it is stopped and joined first) calls `show()` every 30 ms because Qt highgui needs one GUI thread.

## Runtime data flow
FlightGear simulator (separate repo) publishes camera frames on `/flightgear/image` (`bgr8`, `SensorDataQoS`); consumers subscribe with the `compressed` transport (`compressed_image_transport` must be installed). Running GUI examples needs that publisher and a desktop.

`./ifc.sh` selects the latest installed ROS distribution under `/opt/ros`, builds the workspace, sources the overlay, and runs `horizon_detection` with root `config.yaml`.

`main.cpp` spins only the receiver in a `SingleThreadedExecutor`. Its image callback synchronously invokes `Detector::detect()` and publishes the result. Root `config.yaml` uses the `/**` ROS parameter selector so overrides reach the separately constructed nodes; detector stage parameters live under `horizon_detection.alpha.*`.

`DetectorAlpha` runs prefilter -> Canny -> morphological closing -> Hough segments -> line grouping -> horizon estimation. The estimator owns temporal tracking and confidence, using source-image nanoseconds rather than publication time. Keep stage parameter defaults in headers consistent with root `config.yaml`.

`HorizonDetection::publish()` sets `header.stamp` to publication time and forwards the source image timestamp in `image_timestamp`. `ifc_msgs/msg/Horizon.msg` has `header`, `image_timestamp`, `theta`, `rho`, `confidence`, `theta_sd`, and `sigma_rho`. The internal `Horizon` names its angular uncertainty `sigma_theta`, mapped to wire field `theta_sd`. An empty solution (no current detection; held tracks are not published) is sent with `confidence = 0.0` and NaN `theta`, `rho`, `theta_sd`, `sigma_rho`.

Run the node separately with `ros2 run delay_estimator delay_estimator --ros-args --params-file config.yaml`. It uses a `MultiThreadedExecutor` with two nodes: the attitude `BufferedReceiver` (`delay_estimator_attitude`) and the estimator, where the `Features` and `Horizon` subscriptions and the 1 Hz fitting timer each have their own mutually exclusive callback group. The timer copies the mutex-guarded buffers before computing; a backwards attitude timestamp (simulation reset) clears the attitude buffer. Image and PX4 stamps are both UTC (`timestamp_sample` µs → ns), and `t_ap = t_frame + delay` with delay in `[−search.max_delay_ms, 0]` = [−500, 0] ms. Each update uses the whole attitude and pair buffers and publishes invalid until flow (pair buffer holds `feature_buffer_size` pairs, attitude and position non-empty) or horizon (horizon buffer holds `horizon_buffer_size`, attitude non-empty) is ready. The node prints nothing to the console unless `delay_estimator.debug` (default `false`) is set; then every update logs, without interpretation, the fused delay ± sd (`RCLCPP_INFO`) or failure (`RCLCPP_WARN`) and per method either the buffer fill (while waiting) or its delay ± sd / invalid with the best delay, J and fitted delays. It publishes `ifc_msgs/Delay` (`valid`, `delay` and `delay_sd` as `int64` ns); only `valid` results are usable. An `ifc_tools::PerformanceMonitor` times each fit (search) and publishes on `delay_estimator.performance_topic` (`/delay_estimator/performance`). Parameters under `delay_estimator.*` are startup-only; header defaults equal root `config.yaml`.

Geometry uses `float` in a normalized image-centred frame: `u` points right, `v` points up, and distances are divided by half the image diagonal. The horizon satisfies `-u*sin(theta) + v*cos(theta) = rho`; theta is in radians. Use `HorizonUtils` for pixel conversion and line alignment: changing theta by pi also flips rho.

## Conventions
- `#pragma once`, never `#ifndef` include guards.
- Private members prefixed with `_` (e.g. `_callback_mutex`).
- Allman braces (opening brace on its own line), 4-space indent, blank line before and after every block (`if`/`for`/`try` etc.).
- `Detector` owns an optional `DebugViewer`, enabled by the `horizon_detection.debug_view` parameter (default `false`).
- Avoid unnecessary comments.
- New dependencies must be added to both `CMakeLists.txt` (`find_package`, link, and `ament_export_dependencies` for libraries) and `package.xml`.
- Register horizon executables with `add_horizon_executable` in the package's `CMakeLists.txt`; it supplies shared dependencies and installs them under `lib/horizon_detection`.
