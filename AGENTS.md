Use pragma once instead #ifndef
Avoid unessesery comments
Start names of private fields with underscore.
Formatowanie: stosuj bloki kodu zaczynające się w nowej linii, puste linie wokoł bloku.

Każdy moduł (za wyjątkiem `ifc`) w `src` jest repozytorium w organizacji `https://github.com/in-flight-calibration` i  submodułem w repozytorium `ifc`.

## Horizon detection

- `horizon_detection` is a C++20 header-only detector library plus the `horizon_detection_node` executable. `HorizonDetection` owns the receiver, detector, and publisher. `Detector` and `HorizonDetection` inherit from `rclcpp::Node`.
- `DetectorFactory` selects the configured detector. `DetectorAlpha::detect()` runs the OpenCV Hough pipeline.
- `HorizonDetection::publish()` sets `header.stamp` to publication time and preserves the source image timestamp in `image_timestamp`. Read `src/horizon_detection/PLAN.md` before changing the pipeline.
- `Horizon.msg` is the published wire contract. It has `header`, `image_timestamp`, `theta`, `rho`, `confidence`, `theta_sd`, and `sigma_rho`; an empty solution (no current detection) is published with `confidence = 0.0` and NaN `theta`, `rho`, `theta_sd`, `sigma_rho`; consumers must skip it. Internal `sigma_theta` maps to `theta_sd`.
- Use `./ifc.sh` from the workspace root to build all packages and run with `config.yaml`.
- Build with `source /opt/ros/lyrical/setup.bash && colcon build --packages-up-to horizon_detection`.