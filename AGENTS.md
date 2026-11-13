# Working style

## Code style

- `#pragma once`, never `#ifndef` guards.
- Allman braces, 4-space indent, a blank line before and after every block (`if`/`for`/`while`/`try`/`switch`).
- Private members and private helper-state start with `_` (`_mutex`, `_flow_method`); protected virtual hooks too (`_process`).
- Avoid unnecessary comments; let names explain the code. No commented-out code, no closing-namespace comments.
- Header-only C++20 inside a per-package namespace; `main.cpp` only creates nodes, adds them to an executor and spins.
- Include order, one blank line between groups: project/ifc headers (`<...>` form), then third-party (OpenCV, Eigen, rclcpp), then the standard library; each group alphabetical. Include what you use.
- Prefer `const` locals, explicit types for domain values (`int64_t` ns timestamps), `std::optional` for "maybe" results and NaN for invalid numeric outputs.
- Small structs for configuration and data (`AffineConfig`, `FlowSample`); parameters are read in the node constructor with dotted names (`delay.flow.huber_k`), and header defaults must equal root `config.yaml`.
- Extensibility via an abstract base plus a `*Factory::create(name, ...)` that throws `std::invalid_argument` for unknown names; implementations live in `include/<role>/<name>/`.
- Name things after what they are: one type per concept (`FlowSample`, `HorizonSample`) and one function per output (`get_flow`, `get_horizon`) rather than generic `Sample`/`get`. Follow the surrounding file's case (camelCase methods in older code, snake_case in `delay_estimator`).
- Keep the code minimal: delete unused options, gates and parameters instead of disabling them; do not consume data the user excluded (e.g. horizon sd fields); keep side concerns (smoothing, filtering) out of the core when the user says so.
- Shared, reusable pieces go to `ifc_tools`; files in a package are placed by role (`include/`, `include/method/`; `ifc_tools/include/ifc_tools/{common,processing,receiver,filter,msg}/`); message converters live in `ifc_tools/msg/<message_in_snake_case>.hpp`.
- Builds must be warning-free (`-Wall -Wextra -Wpedantic`). Tests are plain executables printing `PASS`/`FAIL` registered with CTest.

## Workflow

- Ask before design decisions with several reasonable options; then implement exactly the chosen variant.
- Validate with the real data: build, run tests, then check against the bag the user is playing in the background.
- Update `.github/copilot-instructions.md` (and an existing `PLAN.md`) together with behaviour changes; remove a `PLAN.md` once its work is done.
- Do not commit; the user commits (messages like `add: ...`, `update: ...`, `fix: ...`, `refactor: ...`, `rename: ...`).

## Repository

Każdy moduł (za wyjątkiem `ifc`) w `src` jest repozytorium w organizacji `https://github.com/in-flight-calibration` i  submodułem w repozytorium `ifc`.

## Horizon detection

- `horizon_detection` is a C++20 header-only detector library plus the `horizon_detection_node` executable. `HorizonDetection` owns the receiver, detector, and publisher. `Detector` and `HorizonDetection` inherit from `rclcpp::Node`.
- `DetectorFactory` selects the configured detector. `DetectorAlpha::detect()` runs the OpenCV Hough pipeline.
- `HorizonDetection::publish()` sets `header.stamp` to publication time and preserves the source image timestamp in `image_timestamp`.
- `Horizon.msg` is the published wire contract. It has `header`, `image_timestamp`, `theta`, `rho`, `confidence`, `theta_sd`, and `sigma_rho`; an empty solution (no current detection) is published with `confidence = 0.0` and NaN `theta`, `rho`, `theta_sd`, `sigma_rho`; consumers must skip it. Internal `sigma_theta` maps to `theta_sd`.
- Use `./ifc.sh` from the workspace root to build all packages and run with `config.yaml`.
- Build with `source /opt/ros/lyrical/setup.bash && colcon build --packages-up-to horizon_detection`.