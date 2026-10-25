Use pragma once instead #ifndef
Avoid unessesery comments
Start names of private fields with underscore.
Formatowanie: stosuj bloki kodu zaczynające się w nowej linii, puste linie wokoł bloku.

## Horizon detection

- `horizon_detection` is a C++20 header-only detector library plus the `horizon_detection_node` executable. `HorizonDetection` owns the receiver, detector, and publisher. `Detector` and `HorizonDetection` inherit from `rclcpp::Node`.
- `DetectorFactory` selects the configured detector. `DetectorAlpha::detect()` runs the OpenCV Hough pipeline.
- `HorizonDetection::publish()` sets `header.stamp` to publication time and preserves the source image timestamp in `image_timestamp`. Read `src/horizon_detection/PLAN.md` before changing the pipeline.
- `Horizon.msg` is the published wire contract. It has `header`, `image_timestamp`, and float fields for `theta`, `rho`, `length`, `confidence`, `sigma_theta`, and `sigma_rho`; `confidence == 0` denotes an invalid result.
- Use `./ifc.sh` from the workspace root to build all packages and run with `config.yaml`.
- Build with `source /opt/ros/lyrical/setup.bash && colcon build --packages-up-to horizon_detection`.