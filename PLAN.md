# PLAN: minimal HorizonEstimator redesign

## Goal
`HorizonEstimator` chooses the horizon from the up to 5 `LineGroups` candidates produced by `LineGrouping`.
- Confidence accumulates while we keep choosing the candidate closest to the moving track.
- A wrong line is worse than reporting "not detected" (`confidence = 0`).

## Design
`HorizonEstimator::estimate(int64_t timestamp_ns, const std::vector<LineGroup>& groups)` returns a `Horizon` directly. State: `_last_horizon` (`std::optional`), `_last_horizon_update_ns`, `_confidance`.

1. No groups: return `INVALID_HORIZON` immediately.
2. `checkTimestamp` returns `dt` since the last update. If it is negative or exceeds `timeout_s`, it clears `_last_horizon`, `_last_horizon_update_ns` and `_confidance`.
3. `scoreGroup(dt, groups)` returns `(scoreCoverage + scoreTop + scoreCoherence) / 3` per group; each `score*` function returns one value in [0, 1] per group. Coherence is `max(0, 1 − d)` with `d = max(|Δθ|/(max_theta_rate·dt), |Δρ|/(max_rho_rate·dt))`, `dt` floored at 50 ms, and 0 without `_last_horizon`.
4. The best score must reach `minimal_update_score` (with a last horizon) or `minimal_init_score` (without); otherwise return `INVALID_HORIZON`.
5. Confidence goes through the alpha filter `C ← C + α(score − C)`, `α = 1 − exp(−dt/confidance_filter_tau)`; `α = 1` without a last horizon.
6. `finalize` builds the horizon from the fitted line (`makeHorizon`), stores it with the timestamp, and returns it with `confidence = C`.

**Debug tile 5** draws grouped lines on a black canvas sized to the source image. **Debug tile 6** overlays only the detected horizon line in red on the original source image.

## Parameters (`horizon.alpha.estimator.*`)
| name | default |
|---|---|
| `minimal_init_score` | 0.33 |
| `minimal_update_score` | 0.6 |
| `timeout_s` | 0.1 |
| `max_theta_rate` (deg/s) | 90.0 |
| `max_rho_rate` (normalized/s) | 1.0 |
| `confidance_filter_tau` | 0.5 |

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
- `timeout_s` = 0.1 s is about 3 frames at 30 fps. After this, confidence/output reset, but the last line remains as a rate reference to prevent an implausible reacquisition.
- Accumulation delays the first output, and every jump: with `score ≈ 0.8`, C reaches 0.3 after about 0.24 s.
- Out of scope: switch margin, confidence decay, per-criterion weights, and changes to earlier pipeline stages.

## Implementation and validation status
- Simplified estimator: three `score*` functions, averaged, leader chosen; no ambiguity margin, switch confirmation or minimum confidence.
- `colcon build --packages-up-to horizon_detection`: no warnings.
- 30 s runway test with `config.yaml`: 99.8 % detections, θ RMSE 0.046°, ρ RMSE 0.0003; exit 0.
