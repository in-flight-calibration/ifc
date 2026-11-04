# IFC architecture

In-flight calibration of the camera from the camera stream and autopilot telemetry only. Estimated one after another:

1. delay between camera stream with respect to autopilot telemetry,
2. camera orientation and intrinsics
3. lens distortion.

Node and topic graph: [DIAGRAM.md](DIAGRAM.md).

Inputs (not nodes of the system):

- **Camera stream** – video from the camera rigidly mounted on the UAV; every frame stamped with its capture time. Camera orientation on the UAV, intrinsics and distortion are unknown at start.
- **Autopilot telemetry** – sensor data (angular velocity) and navigation results (attitude, local and global position); every sample stamped with its sample time.

## Messages

| Message | Package | Description |
|---------|---------|-------------|
| `Image` | `sensor_msgs` | Frame and its capture time. |
| `VehicleAttitude` | `px4_msgs` | Quaternion body → world (FRD → NED) and sample time. |
| `VehicleAngularVelocity` | `px4_msgs` | Body angular rate [rad/s] and sample time. |
| `VehicleLocalPosition` | `px4_msgs` | Position and velocity in local NED [m, m/s] and sample time. |
| `VehicleGlobalPosition` | `px4_msgs` | Latitude, longitude, altitude and sample time. |
| `Horizon` | `ifc_msgs` | Horizon line (angle θ, distance ρ) in the normalized image frame, their standard deviations, confidence and source image time. |
| `FeatureMatches` | `ifc_msgs` | Matched pixel pairs between two subsequent frames with feature ids persistent across frames, the fundamental matrix `F` (3×3, raw pixel coordinates) fitted to them, both frame times and image size. |
| `Delay` | `ifc_msgs` | Delay [s], its standard deviation and estimation status. |
| `StreamTelemetry` | `ifc_msgs` | Per frame: frame time, interpolated attitude, local and global position, horizon and feature matches. |
| `CameraOrientation` | `ifc_msgs` | Rotation body → camera (quaternion) and its covariance. |
| `CameraIntrinsics` | `ifc_msgs` | Focal lengths fx, fy, principal point cx, cy [px], image size and their covariance. |
| `Terrain` | `ifc_msgs` | Local elevation grid: origin, resolution and heights [m]. |
| `Distortion` | `ifc_msgs` | Distortion model and coefficients with their standard deviations. |
| `CalibrationResults` | `ifc_msgs` | Accepted delay, camera orientation, intrinsics and distortion, each with its uncertainty and status (not estimated, estimating, accepted). |


## Nodes

| Node | Description |
|------|-------------|
| `horizon_detection` | Detects the horizon line in every frame from the image only, so it works before any calibration. Publishes the line in a normalized image frame with its uncertainty and confidence; frames without a horizon are published as invalid. |
| `feature_tracker` | Detects features in every frame and matches them with the previous frame. Only the ground below the horizon is used, since sky and clouds give no reliable features. Outliers are rejected by the epipolar constraint (RANSAC on the fundamental matrix), which needs no intrinsics. Publishes the inlier pixel pairs in raw image coordinates with feature ids, so a feature keeps its id while it is tracked, together with the fundamental matrix. |
| `delay_estimator` | Estimates the constant delay between camera stream and autopilot telemetry. Method 1: the horizon is the image of the plane perpendicular to gravity, so it must follow the attitude; the delay that best fits horizon to attitude is chosen. Method 2: frame-to-frame motion from the inliers must follow the angular velocity; the delay that best fits them is chosen. In both methods the unknown camera parameters are a fitted linear transform, so no calibration is needed. Requires roll and pitch changes (method 1) or any rotation (method 2). |
| `stream_telemetry` | Assigns telemetry to every frame. Telemetry is interpolated at the frame time shifted by the accepted delay from `CalibrationResults`, so attitude and position correspond to the moment the frame was captured. Bundles them with the horizon and feature matches of the same frame into one message used by all later stages. |
| `camera_estimator` | Jointly estimates the camera orientation relative to the UAV body and the intrinsics (focal length, i.e. FOV, and principal point). The horizon constrains roll and pitch of the camera; the inliers, compared with the vehicle rotation and translation between frames, constrain focal length, principal point and camera yaw. Solved by nonlinear least squares with a robust loss. |
| `terrain_model` | Builds a local elevation map around the vehicle from its global position and an elevation source. Gives `structure_from_motion` the ground geometry, so ground points can be located without long feature tracks. |
| `structure_from_motion` | Tracks ground points over many frames. Camera poses are known from `StreamTelemetry` and the accepted orientation and intrinsics from `CalibrationResults`, so the remaining reprojection error, growing towards the image edges, is caused by the lens. Estimates distortion coefficients and small corrections of orientation and intrinsics in one bundle adjustment. |
| `calibration_manager` | Governs the calibration. Runs the stages in order: delay, orientation and intrinsics, distortion. Filters every estimate over time, accepts it when it is stable and its uncertainty is small enough, and publishes all accepted values in `CalibrationResults` to the nodes that depend on them. |

## Node details

### horizon_detection

- The line is given in a normalized, image-centred frame: `u` right, `v` up, distances divided by half the image diagonal. The line is `-u·sin θ + v·cos θ = ρ`.
- The alpha implementation of pipeline is: prefilter → Canny → morphological closing → Hough segments → line grouping → horizon estimation with temporal tracking.
- Keeps the source image time in `Horizon`, so later nodes can pair it with telemetry.
- When there is no detection, publishes `confidence = 0.0`

### feature_tracker

- Detects features (e.g. ORB) in every frame and matches them with the previous frame.
- Uses only the region below the horizon. Sky and clouds give no reliable, static features.
- Rejects outliers with RANSAC on the fundamental matrix `F`. This needs no intrinsics.
- Publishes the inlier pixel pairs in raw image coordinates. The ids persist while a feature is tracked, so multi-frame tracks can be built. Fundamental matrix `F` is published in the same `FeatureMatches` message.

### delay_estimator

- Estimates one constant latency between the camera stream and autopilot telemetry
- Uses no camera orientation and no intrinsics; at this stage nothing about the camera is known.
- **Method 1 – horizon ↔ attitude.**
  - The horizon is the image of the plane perpendicular to gravity: `ℓ ∝ A · g_b(t + Δt)`.
  - `g_b` is the gravity direction in the body frame, taken from the attitude.
  - `A` (3×3) absorbs the unknown intrinsics and camera orientation.
  - For each candidate `Δt`, `A` is fitted robustly, and the `Δt` with the best fit is selected.
  - Short windows give a rough estimate; it becomes accurate over a long time.

- **Method 2 – frame motion ↔ angular velocity.**
  - For every frame pair, the image motion from the inliers (shift `du`, `dv`, rotation `dα`) follows the body rate: `[du, dv, dα] ≈ M · ω(t + Δt)`.
  - `M` is an unknown 3×3 matrix, solved by linear least squares.
  - `Δt` is found by a 1-D search.
- Publishes `Delay` with its standard deviation and status.

### stream_telemetry

- Builds one `StreamTelemetry` message per frame.
- Interpolates attitude and position at the frame time shifted by the accepted delay from `CalibrationResults`. Attitude is interpolated on rotations, not per component.
- Adds the horizon and feature matches of the same frame.
- Is the single source of time-aligned data for all later stages, so the delay is applied in only one place.

### camera_estimator

- Jointly estimates two things:
  - the rotation body → camera;
  - the intrinsics: focal length (i.e. FOV) and principal point.
- They are estimated together because they are coupled: neither the horizon nor the inliers separate them alone.
- Horizon: with the attitude known, it fixes the camera roll and pitch relative to gravity, i.e. the line `ℓ ∝ K⁻ᵀ · R_cb · g_b`.
- Inliers:
  - The vehicle rotation and translation between frames are known from telemetry.
  - With `R_cb` and `K` they predict the epipolar geometry of every pair.
  - This constrains focal length, principal point and camera yaw.
- Solved by nonlinear least squares with a robust loss over many frames.

### terrain_model

- Uses the global position to build a local elevation map around the vehicle.
- The elevation source is DEM.
- Exists because the terrain is not planar.

### structure_from_motion

- Runs last, when the delay, orientation and intrinsics are accepted.
- Temporarily reads raw video; if possible, it should use the `FeatureMatches` tracks (persistent ids) from `feature_tracker` instead.
- Camera poses are known from `StreamTelemetry` and `CalibrationResults`. Ground points are tracked over many frames and constrained by `Terrain`.
- The remaining reprojection error, which grows towards the image edges, is attributed to the lens. One bundle adjustment estimates:
  - the distortion coefficients;
  - small corrections of orientation and intrinsics.
- Publishes `Distortion`, plus `CameraOrientation` and `CameraIntrinsics` as corrections.

### calibration_manager

- Governs the whole process and runs the stages one by one:
  1. delay;
  2. orientation and intrinsics;
  3. distortion.
- Filters every incoming estimate over time and accepts it when it is stable and its uncertainty is small enough.
- Publishes all accepted values in `CalibrationResults`, the only feedback path:
  - `stream_telemetry` uses the delay;
  - `structure_from_motion` uses orientation and intrinsics.
- Merges the corrections from `structure_from_motion` into the accepted orientation and intrinsics.

