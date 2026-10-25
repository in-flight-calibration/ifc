# PLAN: minimal HorizonEstimator redesign

## Goal
`HorizonEstimator` chooses the horizon from the up to 5 `LineGroups` candidates produced by `LineGrouping`.
- Confidence accumulates while we keep choosing the candidate closest to the moving track.
- A wrong line is worse than reporting "not detected" (`confidence = 0`).

## Design
`HorizonEstimator::estimate(int64_t timestamp_ns, const LineGroups& groups)` returns a `Horizon` directly. The estimator owns its track: `_track` (a `Horizon`), `_track_timestamp_ns`, and the accumulated confidence `_confidence`.

1. **Score each candidate.** Skip groups with `coverage < minimum_coverage`. For the rest, `score = mean(μ_cov, μ_top, μ_prev)`:
   - `μ_cov` = `group.coverage`.
   - `μ_top` = fraction of the other groups' segment length that lies below this line (signed distance `n·p − ρ < 0`, where `n = (−sinθ, cosθ)` points image-up). Equals 1 when there are no other groups.
   - `μ_prev` = closeness to `_track`, limited by how fast the horizon can move. With `dt = max(t − t_track, 50 ms)`:
     - `d = max(|Δθ| / (max_theta_rate·dt), |Δρ| / (max_rho_rate·dt))`, with θ aligned by `HorizonUtils::alignLine`;
     - `μ_prev = max(0, 1 − d)`.

     It is left out of the mean when there is no track or the track is older than `stale_seconds`. Candidates with `d > 1` are rejected while a track reference exists, including after it becomes stale; an impossible jump cannot reset the reference.
2. **Choose** the best-scoring candidate. If `score < minimum_score`, or the best and second-best scores differ by less than `ambiguity_margin`, there is no new detection.
   - While the track is fresh, switch away from the closest candidate only when the challenger leads it by at least `ambiguity_margin` for `switch_confirmation_frames` consecutive frames; until then, hold the tracked horizon.
3. **Accumulate confidence:**
   - **Continue:** the chosen candidate is the closest to the non-stale track (smallest `d`) and `d ≤ 1`. Then `C ← C + (1 − exp(−dt/tau_rise_seconds))·(score − C)`, and `_track` takes the new θ and ρ.
   - **Switch:** after the challenger clears the score margin and consecutive-frame confirmation above, update `_track` without resetting `C`, then apply the same update.
   - **New track:** when there is no fresh track, start with `C = 0` and apply the same update.
   - **No detection:** while the track is fresh, return the last tracked horizon for short candidate dropouts. Once older than `stale_seconds`, output is invalid and confidence resets; the last line remains only as a rate reference.
4. **Publish** only when `C ≥ minimum_confidence`, with `Horizon.confidence = C`. Otherwise output `confidence = 0`.
   - θ, ρ, and `length` (= covered length) come from the group.
   - `sigma_rho = residual/√n` and `sigma_theta = residual/(√n·covered_length/√12)`, where `n` = number of segments in the group.

**Debug tile 5** draws grouped lines on a black canvas sized to the source image. **Debug tile 6** overlays only the detected horizon line in red on the original source image.

## Parameters (`horizon.alpha.estimator.*`)
| name | default |
|---|---|
| `minimum_coverage` | 0.1 |
| `minimum_score` | 0.5 |
| `ambiguity_margin` | 0.1 |
| `stale_seconds` | 0.1 |
| `max_theta_rate_degrees_per_second` | 90.0 |
| `max_rho_rate_per_second` | 1.0 |
| `tau_rise_seconds` | 0.5 |
| `minimum_confidence` | 0.3 |
| `switch_confirmation_frames` | 2 |

All old estimator parameters are removed:
- `support_radius_px`
- `sample_step_px`
- `minimum_continuity`
- `minimum_length_support`
- `temporal_*`

Header defaults must equal `config.yaml`.

## Synthetic test: runway
`examples/synthetic_horizon.cpp` `HorizonRenderer::render()` also draws an endless runway, which adds two strong edges that converge on the horizon:
- a gray (≈(150,150,150)) triangle filled with `cv::fillConvexPoly`, with its vanishing point on the horizon; horizon and ground use black/white so grayscale edge detection can see the true horizon;
- the vanishing point is the horizon's foot `ρ·n(θ)`, shifted along the line by `0.3·sin(0.2·t)` to simulate yaw;
- its two edges run toward the ground (`−n`), rotated by ±25°, with length 3 (normalized), converted with `toPixel`.

The triangle lies entirely on the ground side, so the true horizon is unchanged. The accuracy metrics stay as they are.

## Todos
1. **Done — `synthetic-runway`:** add the runway and test the original estimator. The first baseline was invalid because the original sky/ground colors became almost identical grayscale values.
2. **Done — `estimator-rewrite`:** implement params, track state, scoring, rate gates, confidence accumulation, result, and debug view.
3. **Done — `detector-wiring`:** `DetectorAlpha` passes `timestamp.nanoseconds()` and groups, declares the new parameters, and no longer uses `_last_horizon`/`_last_update`.
4. **Done — `config-docs`:** update root `config.yaml` and Copilot instructions; remove the obsolete package `PLAN.md`.
5. **Blocked — `validate`:**
   - warning-free `colcon build --packages-up-to horizon_detection`;
   - `ros2 run horizon_detection synthetic_horizon --ros-args -p debug_view:=false -p duration:=30.0` must reach ≥90 % detections, θ RMSE ≤1°, and ρ RMSE ≤0.01.

## Notes
- The rate limits roughly correspond to roll rate (θ) and to pitch rate scaled by `f_px/D` (ρ). The defaults leave a wide margin above the synthetic motion (≈19°/s, ≈0.13/s).
- `stale_seconds` = 0.1 s is about 3 frames at 30 fps. After this, confidence/output reset, but the last line remains as a rate reference to prevent an implausible reacquisition.
- Accumulation delays the first output, and every jump: with `score ≈ 0.8`, C reaches 0.3 after about 0.24 s.
- Out of scope: switch margin, confidence decay, per-criterion weights, and changes to earlier pipeline stages.

## Implementation and validation status
- Implemented the estimator state, direct `Horizon` return, group scoring, rate limits, confidence accumulation, short-gap hold, ambiguity rejection, ROS parameter wiring, and runway renderer.
- `colcon build --packages-up-to horizon_detection`: passed with no warnings.
- 30 s runway test: `82.6%` detections, θ RMSE `0.152°`, ρ RMSE `0.0025`; the executable exits nonzero because detection rate is below the existing 90% gate. The accuracy limits pass. This is an intentional false-negative-heavy result: missing/ambiguous horizons are rejected instead of publishing runway edges.
- The first baseline attempt used the original synthetic colors, whose grayscale values were nearly equal; it did not provide a valid horizon edge and is not comparable. The renderer now uses black/white sky/ground contrast.
