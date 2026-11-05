# IFC architecture

In-flight calibration of the camera from the camera stream and autopilot telemetry only. Initial estimates become available in dependency order:

1. delay between camera stream in respect to autopilot telemetry,
2. camera orientation and intrinsics
3. lens distortion.

Calibration is continuous and iterative: accepted estimates are updated throughout flight, including delay when stream latency changes. Distortion estimation is a fine-tuning stage for modest lens distortion; large-distortion camera models are outside the primary scope.

Node and topic graph: [DIAGRAM.md](DIAGRAM.md).

Inputs (not nodes of the system):

- **Camera stream** – video from the camera rigidly mounted on the UAV, typically at 30 Hz. Capture, rendering, GStreamer and other processing make it arrive later than autopilot telemetry; observed relative latency is about 50–300 ms. Camera orientation on the UAV, intrinsics and distortion are unknown at start.
- **Autopilot telemetry** – sensor data (angular velocity) and navigation results (attitude, local and global position); every sample stamped with its sample time. Attitude is available at about 200 Hz; position topics may have different rates.
- **Time contract** – preserve the source frame timestamp and identify whether it denotes capture or a later pipeline stage. Arrival latency is not automatically a timestamp offset: correctly capture-stamped frames on the same clock need buffering, not a second latency correction. The alignment model is `t_ap = t_frame + clock_offset + delay`, where `clock_offset` maps clock domains and `delay` is the signed residual correction estimated from motion. Its sign depends on timestamp semantics, not on arrival order.

## Messages

| Message | Package | Description |
|---------|---------|-------------|
| `Image` | `sensor_msgs` | Frame and its source timestamp, interpreted according to the time contract. |
| `VehicleAttitude` | `px4_msgs` | Quaternion body → world (FRD → NED) and sample time. |
| `VehicleAngularVelocity` | `px4_msgs` | Body angular rate [rad/s] and sample time. |
| `VehicleLocalPosition` | `px4_msgs` | Position and velocity in local NED [m, m/s] and sample time. |
| `VehicleGlobalPosition` | `px4_msgs` | Latitude, longitude, altitude and sample time. |
| `Horizon` | `ifc_msgs` | Horizon line (angle θ, distance ρ) in the normalized image frame, their standard deviations, confidence and source image time. |
| `FeatureMatches` | `ifc_msgs` | Matched pixel pairs between two subsequent frames with persistent feature ids, per-match quality and inlier/outlier flags, the fundamental matrix `F` (3×3, raw pixel coordinates) fitted to the inliers with its validity status, both frame times and image size. Flagged outliers remain available to downstream consumers. |
| `Delay` | `ifc_msgs` | Delay [s], its standard deviation and estimation status. |
| `AutopilotTelemetry` | `ifc_msgs` | Body attitude and camera-centre local and global position evaluated for a source frame using the current accepted delay and configured camera lever arm; includes frame time, target autopilot time, calibration revision and per-component validity/extrapolation status. |
| `StreamTelemetry` | `ifc_msgs` | Per frame: frame time, interpolated attitude, local and global position, horizon and feature matches. |
| `CameraOrientation` | `ifc_msgs` | Rotation body → camera (quaternion) and its covariance. |
| `CameraIntrinsics` | `ifc_msgs` | Focal lengths fx, fy, principal point cx, cy [px], image size and their covariance. |
| `Terrain` | `ifc_msgs` | Local elevation grid: origin, resolution and heights [m]. |
| `Distortion` | `ifc_msgs` | Distortion model and coefficients with their standard deviations. |
| `CalibrationResults` | `ifc_msgs` | Current delay, camera orientation, intrinsics and distortion, each with its uncertainty and status (not estimated, estimating, accepted), plus calibration revision and validity time. Acceptance permits use, not permanent freezing. |


## Nodes

| Node | Description |
|------|-------------|
| `horizon_detection` | Detects the horizon line in every frame from the image only, so it works before any calibration. Publishes the line in a normalized image frame with its uncertainty and confidence; frames without a horizon are published as invalid. |
| `feature_tracker` | Detects features below the horizon and matches them with the previous frame. Optional attitude information from `AutopilotTelemetry` adapts the search range to motion magnitude; tracking also works before delay acceptance. RANSAC on the fundamental matrix flags outliers without requiring intrinsics. Publishes matched raw pixel pairs with persistent ids and quality flags. |
| `delay_estimator` | Continuously estimates camera-to-telemetry time alignment, treating delay as approximately constant within each fitting window. Compares horizon motion with attitude, or frame-to-frame image motion with relative attitude over the same interval. Fits unknown camera-dependent mappings without requiring prior calibration, and rejects windows with insufficient excitation or ambiguous delay minima. |
| `telemetry_synchronizer` | Buffers autopilot telemetry and interpolates attitude, local and global position at each frame's target autopilot time using the current accepted delay from `CalibrationResults`. Publishes `AutopilotTelemetry`; it is the only place where accepted delay is applied to downstream data. |
| `stream_telemetry` | Bundles already interpolated `AutopilotTelemetry` with the horizon and feature matches of the same frame into one message used by later stages. Does not apply delay or interpolate telemetry again. |
| `camera_estimator` | Jointly estimates the camera orientation relative to the UAV body and the intrinsics (focal length, i.e. FOV, and principal point). The horizon constrains roll and pitch of the camera; the inliers, compared with the vehicle rotation and translation between frames, constrain focal length, principal point and camera yaw. Solved by nonlinear least squares with a robust loss. |
| `terrain_model` | Builds a local elevation map around the vehicle from its global position and an elevation source. Gives `structure_from_motion` the ground geometry, so ground points can be located without long feature tracks. |
| `structure_from_motion` | Tracks ground points over many frames, using poses from `StreamTelemetry`, current calibration and terrain as uncertain constraints. Estimates modest lens distortion and small corrections of orientation and intrinsics in one bundle adjustment without attributing every reprojection residual to the lens. |
| `calibration_manager` | Governs continuous, iterative calibration. Bootstraps delay, then orientation and intrinsics, then distortion; subsequently updates all stages as data permit. Checks observability, uncertainty and independent validation before publishing revised accepted values in `CalibrationResults`. |

## Node details

### horizon_detection

- The line is given in a normalized, image-centred frame: `u` right, `v` up, distances divided by half the image diagonal. The line is `-u·sin θ + v·cos θ = ρ`.
- The alpha implementation of the pipeline is: prefilter → Canny → morphological closing → Hough segments → line grouping → horizon estimation with temporal tracking.
- Keeps the source image time in `Horizon`, so later nodes can pair it with telemetry.
- When there is no detection, publishes `confidence = 0.0`
- The calibration flight plan is designed to keep the horizon visible, including during excitation manoeuvres. This is an operating assumption, not a guarantee of detection in every frame.
- Primarily supplies the ground mask and a motion signal for delay estimation. At approximately constant altitude, a small, stable horizon offset is less problematic for temporal alignment than a changing skyline; it must not be treated as an exact gravity constraint for camera calibration.

### feature_tracker

- Detects features (e.g. ORB) in every frame and matches them with the previous frame.
- Uses only the region below the horizon. Sky and clouds give no reliable, static features.
- Optionally uses the relative attitude from `AutopilotTelemetry` to distinguish small and large rotations and adapt the feature search range. This is a heuristic, not an exact pixel-motion prediction before intrinsics and camera orientation are known. Without it the tracker works on the images alone.
- Flags outliers with RANSAC on the fundamental matrix `F`. This needs no intrinsics. A degenerate or unreliable fit is marked invalid rather than presented as reliable geometry.
- Publishes matched pixel pairs in raw image coordinates, including flagged outliers so downstream stages can reconsider them. The ids persist while a feature is tracked, so multi-frame tracks can be built. Fundamental matrix `F` and its validity are published in the same `FeatureMatches` message; geometric estimators select suitable matches using the flags.

### delay_estimator

- Continuously estimates the signed residual delay in the time contract. Delay is approximately constant within a fitting window, not necessarily throughout flight; windows spanning a latency change must be rejected or shortened.
- Requires no camera orientation or intrinsics.
- **Method 1 – horizon ↔ attitude.**
  - Uses the ideal gravity horizon model: `ℓ ∝ A · g_b(t_ap)`, where `t_ap = t_frame + clock_offset + Δt`.
  - `g_b` is the gravity direction in the body frame, taken from the attitude.
  - `A` (3×3) absorbs the unknown intrinsics and camera orientation.
  - For each candidate `Δt`, `A` is fitted robustly, and the `Δt` with the best fit is selected.
  - Approximately constant altitude limits changes in horizon offset. A stable offset need not prevent delay estimation, but is not in general absorbed exactly by the projective model; changing terrain silhouettes or altitude can bias the fit.
  - Longer windows help only while delay remains approximately constant and motion supplies independent information.

- **Method 2 – frame motion ↔ relative attitude.**
  - For each candidate delay, interpolate the roughly 200 Hz `VehicleAttitude` samples at the two shifted timestamps of a roughly 30 Hz frame pair. Interpolate on rotations, not quaternion components.
  - Compare image motion over that pair with the relative rotation between those attitudes. Integrated angular velocity over the same interval is an alternative; do not compare frame displacement directly with an instantaneous gyro sample.
  - An unknown fitted mapping relates image-motion summaries to relative rotation. A linear mapping is a local approximation whose validity depends on motion and feature distribution; translation and depth variation remain nuisance terms.
  - Search delay using buffered raw telemetry, independently of the currently accepted delay. The synchronizer provides interpolation for downstream consumers, not the candidate-dependent alignment used during this search.
- **Observability and acceptance.**
  - Mere motion is insufficient: regular single-axis or periodic motion can let the fitted camera mapping absorb a time shift or produce several delay minima.
  - A planned AP calibration mode will excite roll and pitch with a distinctive changing-frequency signal, such as a logarithmic sine sweep, within the aircraft's flight limits. Multi-axis excitation and varying frequency should reduce temporal ambiguity, but do not replace observability checks.
  - Require sufficient excitation, a well-separated delay minimum and validation on a held-out temporal block. Low residuals or stable repeated estimates alone do not establish correctness.
- Publishes `Delay` with its standard deviation and status.

### telemetry_synchronizer

- Subscribes to frame timestamps from the camera stream, `VehicleAttitude`, `VehicleLocalPosition`, `VehicleGlobalPosition` and `CalibrationResults`.
- Buffers each telemetry topic at its native rate. For every frame, evaluates `t_ap = t_frame + clock_offset + accepted_delay` and interpolates attitude, local and global position at that time. Attitude interpolation operates on rotations.
- Takes the known camera position as configuration: a lever arm `r_b` in metres, expressed in body FRD axes from the telemetry position reference point to the camera optical centre. It is not estimated by calibration.
- After interpolation, accounts for this lever arm using the body-to-world attitude: `p_camera = p_reference + R_wb · r_b`. Published local and global positions refer to the camera centre, with the global correction performed through the corresponding local navigation frame; attitude remains body-to-world. Downstream nodes must not apply the lever arm again.
- AP telemetry usually arrives ahead of the corresponding image, so bracketing samples are normally already available. If not, wait for a bounded interval; forward extrapolation is a last resort with a bounded horizon and explicit per-component quality flags. Missing or stale components remain invalid.
- Publishes `AutopilotTelemetry` keyed by the original frame timestamp, with the target autopilot time and calibration revision. Both consumers receive the same aligned result.
- Is the only place where accepted delay is applied to downstream data. Until the delay is accepted it does not publish; the image-only tracker and delay estimator remain operational.
- Applies revised delay estimates to subsequent frame outputs without silently retiming previous outputs. Consumers must not mix calibration revisions in an estimation window without realigning its data.

### stream_telemetry

- Builds one `StreamTelemetry` message per frame.
- Joins already interpolated `AutopilotTelemetry` by source frame timestamp; does not repeat interpolation or delay correction.
- Adds the horizon and feature matches of the same frame.
- Is the single source of per-frame data for all later stages.

### camera_estimator

- Jointly estimates two things:
  - the rotation body → camera;
  - the intrinsics: focal length (i.e. FOV) and principal point.
- They are estimated together because they are coupled: neither the horizon nor the inliers separate them alone.
- Horizon: provides an additional, uncertainty-weighted constraint through `ℓ ∝ K⁻ᵀ · R_cb · g_b`. Its suitability depends on how closely the observed skyline represents the gravity horizon; orientation and intrinsics remain coupled.
- Inliers:
  - Body rotation and camera-centre translation between frames are obtained from synchronized telemetry, with their uncertainty; the configured lever arm is already accounted for.
  - With `R_cb` and `K` they predict the epipolar geometry of every pair.
  - This constrains focal length, principal point and camera yaw.
- Solved by nonlinear least squares with a robust loss over many frames.

### terrain_model

- Uses the global position to build a local elevation map around the vehicle.
- The elevation source is DEM.
- Exists because the terrain is not planar.

### structure_from_motion

- Starts when usable delay, orientation and intrinsics are available, then continues refining them as calibration evolves.
- Camera poses from `StreamTelemetry` and `CalibrationResults`, and ground geometry from `Terrain`, are uncertain constraints. Ground points are tracked over many frames.
- Reprojection error may also reflect timing, pose, terrain or feature errors; it is not automatically lens distortion. Weight these constraints by uncertainty and use a robust loss. One bundle adjustment estimates:
  - the distortion coefficients;
  - small corrections of orientation and intrinsics.
- Publishes `Distortion`, plus `CameraOrientation` and `CameraIntrinsics` as corrections.

### calibration_manager

- Governs the whole process and bootstraps the stages in dependency order:
  1. delay;
  2. orientation and intrinsics;
  3. distortion.
- Subsequently runs continuous refinement: acceptance never freezes a parameter, and delay can change during flight.
- Filters incoming estimates and checks observability, uncertainty and held-out validation before acceptance. Accounts for shared data in overlapping windows rather than treating repeated fits as independent evidence.
- Owns delay-update filtering: distinguishes noisy estimates, gradual drift and latency jumps before publishing an update. A detected jump invalidates fitting windows spanning the change and temporarily pauses camera-parameter updates until alignment is reliable, so timing errors are not absorbed into camera calibration.
- Publishes versioned updates with validity times. If an accepted result becomes unreliable, marks it as estimating again and pauses dependent estimation until usable alignment/calibration is restored.
- Publishes all accepted values in `CalibrationResults`, the only feedback path:
  - `telemetry_synchronizer` uses the delay;
  - `structure_from_motion` uses orientation and intrinsics.
- Merges jointly estimated corrections from `structure_from_motion` as one consistent revision of orientation, intrinsics and distortion.
