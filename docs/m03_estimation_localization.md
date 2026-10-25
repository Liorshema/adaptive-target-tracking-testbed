# M03 — Estimation Timestamp & Localization Reliability

Completed 2026-10-15. **Verdict: CONDITIONAL.** Timestamp handling and hard-stop
estimation improve measurably; global localization during skid turns and aggressive
joint covariance calibration remain unresolved. No physical limits, M02 actuation,
PID gains, controller strategies or target dynamics were changed for M03. No commit
or push was made.

## Diagnosis and corrections

### Timestamp ordering

The robot filter has one state `[x, y, yaw, v, omega, q, dq]` and one prediction
epoch. Wheel observations select `v, omega`; encoders select `q, dq`; optional IMU
selects yaw rate. Separate observation matrices do not make updates at different
epochs independent: prediction and covariance propagation still require causal
ordering. Previously each arrival advanced the shared watermark immediately and
subsequent older observations were rejected.

Wheel odometry publishes at 50 Hz; joint states are generated at the 0.002 s physics
step, nominally 500 Hz. Actual callback delivery varies with executor/transport load.
In the new matched captures, delivered joints were about 355–362 Hz versus 457–481 Hz
before. A larger acceptance count does not imply lossless transport. No IMU messages
were observed; optional IMU tests use synthetic body-aligned yaw-rate observations.

An earlier saved trace rejected 7,816/12,857 delivered joint observations (60.8%).
The fresh wide baseline rejected 6,420/31,351 (20.48%); the nominal baseline rejected
11,317/30,436 (37.18%). These are different recordings, not contradictory counts.
Wide-baseline wheel/joint receipt-age p95 was 6/16 ms; final wide receipt-age p95 was
14/20 ms. Source stamps within each sensor stream were monotonic in these captures;
cross-source interleaving caused the rejection problem.

Implemented a reusable, bounded timestamp heap in `robot_estimation/common`:

- Admit validated observations with their original timestamps; stable-sort behind
  `ROS now - reorder_holdback`. Drain on the configured timer and publish once per
  batch. No rollback, relabeling, or prediction of a stale observation as current.
- Detect duplicate `(source, stamp)` observations both pending and committed.
  Equal timestamps from different sources are allowed; within-source reordering
  before commitment is also allowed.
- Retain explicit invalid, future, stale, late and overflow rejection. A late wheel
  message cannot rewind the shared filter. Capacity and age bound the queue.
- Reset the queue, filter, freshness and publication watermarks together on ROS
  clock resets. Counters remain cumulative; reset count identifies epoch changes.
- Preserve per-component freshness checks and actual state stamps. Full-state output
  retains the existing 50 Hz maximum; batching can lower its delivered rate. Legacy
  output is now at batch cadence, at most 100 Hz with these defaults.

The nominal added ordering budget is 30 ms plus up to 10 ms timer quantization.
This is not a hard real-time scheduling guarantee: observed fusion-age p95 was
40 ms, with rare maxima 200 ms nominal / 138 ms wide. Observations exceeding the
200 ms age ceiling are discarded. Downstream consumers still see the original
state epoch and enforce their existing freshness policies.

`/estimation/diagnostics` publishes JSON in `std_msgs/String` at 5 Hz. It includes
filter time, queue depth, committed watermark, resets, per-source received/queued/
accepted/delayed counts, rejection reasons, arrival/buffer-delay sums and maxima,
maximum processing age, fusion failures, and raw-joint validity/constraint evidence.
`accepted` means removed from the ordering buffer for fusion; fusion failures are
counted separately. Final pending entries at shutdown are not rejections. A SIGINT
during the final batch can leave the queue watermark slightly ahead of filter time.

### Joint limits and covariance

The previous constant-velocity model extrapolated outward at a mechanical stop.
With low process uncertainty, encoder correction could leave the estimated shoulder
negative even when the physical encoder was legal. M02 had recorded an estimated
shoulder of -0.014045 rad versus a nearby raw sample +0.000193 rad.

The robot-layer `JointBoundsConditioner` reads bounds from the authoritative
`robot_description/config/robot/robot.yaml`; generic EKF code only accepts an optional
state-constraint callback. Bringup supplies the resolved description path. There are
no duplicate hard-coded physical limits in the estimator.

For a marginal mean outside `[l,u]`, use scalar truncated-normal moments, with
`a=(l-mu_i)/sqrt(P_ii)` and `b=(u-mu_i)/sqrt(P_ii)`. If their mean and variance are
`m_c, V_c`, transform the entire correlated Gaussian:

```text
mu_new = mu + P[:,i] * (m_c - mu_i) / P_ii
P_new  = P + outer(P[:,i], P[:,i]) * (V_c - P_ii) / P_ii^2
```

This changes covariance and correlated velocities together, retaining positive
variance for release from contact. Monte Carlo tests independently check the
correlated moments. The final numerical clip removes cancellation roundoff in the
analytic moment only; it is not a mean-only estimator clamp. Degenerate/infeasible
zero-variance priors or invalid moments cause an explicit fusion failure and suppress
that batch's full output.

Eligibility requires causal, fresh raw encoder feedback inside the physical interval
to numerical tolerance (1e-9 rad). A genuine raw excursion disables conditioning for
that joint and suppresses the full robot estimate; receipt-time raw validity also
blocks publication before the reordered sample is fused. Legacy state and raw
feedback/diagnostics remain available. M02 remains responsible for physical recovery.
The 1e-9 tolerance is numerical only; M02's 0.001 rad acceptance policy was not changed.

This is a sequential Gaussian moment approximation, activated on marginal mean
violations. It is not a full non-Gaussian contact dynamics model, nor a proof of
statistically calibrated uncertainty. The approach follows constrained-filter moment
methods discussed in [Simon, NASA report](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20060012173.pdf).

### Process integration and uncertainty

Base motion now uses exact constant-twist SE(2) increments:
`delta_xy = v*dt*sinc(omega*dt/2)*[cos(yaw+omega*dt/2), sin(yaw+omega*dt/2)]`.
The analytic Jacobian includes yaw-rate derivatives and stable small-rate limits.
This removes Euler discretization error; it cannot compensate for wheel slip.

Previously `Q_velocity` scaled as `dt^2`: splitting a fixed interval into more
prediction calls silently reduced total injected uncertainty. Robot process noise
now represents continuous white acceleration density, integrated about the frozen
continuous Jacobian, including base cross terms. Joint blocks use
`sigma^2 * [[dt^3/3, dt^2/2], [dt^2/2, dt]]`. Numeric amplitudes remain unchanged,
but their meaning is now explicitly m/s^(3/2) or rad/s^(3/2), rather than per-callback
acceleration standard deviation. This is a semantics correction, not calibrated
noise identification. The target filter is unchanged. Tests cover stationary time
partition invariance, moving-base covariance quadrature, PSD and finite-difference
Jacobians. Moving nonlinear covariance remains an EKF local approximation.

An initially allocation-heavy implementation was replaced with an algebraically
equivalent closed form: standalone covariance CPU cost fell from 275.02 to 34.66 us
per call (`m03_covariance_cost_before/after.json`). The provisional `m03_wide_after`
capture remains archived; **final wide results below use `m03_wide_final`**.

## Configuration ownership

All new policy values live in `src/robot_estimation/config/robot_estimator.yaml`:

| Parameter | Default | Purpose |
|---|---:|---|
| `reorder_holdback` | 0.03 s | Timestamp ordering horizon |
| `reorder_max_age` | 0.2 s | Admission/drain age ceiling |
| `reorder_rate_hz` | 100 | Batch timer |
| `reorder_capacity` | 256 | Pending observation cap |
| `diagnostic_topic` | `/estimation/diagnostics` | JSON evidence stream |
| `diagnostic_rate_hz` | 5 | Evidence publication rate |
| `joint_bounds_enabled` | true | Robot-layer conditioning and raw-validity guard |
| `joint_bounds_numerical_tolerance` | 1e-9 rad | Floating-point contact tolerance |
| `robot_description_file` | empty | Explicit authoritative path, supplied by bringup |
| `robot_description_package` | `robot_description` | Standalone path fallback |
| `robot_description_config` | `config/robot/robot.yaml` | Standalone description resource |
| `wheel_odom_frame` | `odom` | Wheel pose frame validation |
| `base_frame` | `base_link` | Wheel child/body frame validation |
| `imu_frame` | `base_link` | Optional aligned IMU frame validation |

Existing `future_tolerance=0.01 s` and wheel/joint freshness budgets of 0.2 s are
preserved. The holdback plus timer period must fit component freshness budgets.
Frame-invalid/nonfinite/malformed observations are counted, not silently fused.
Current optional IMU support requires an already aligned frame; it does not invent
an extrinsic transform. SciPy, YAML, ament index and robot description dependencies
are declared for standalone installation.

## Frame, sign and geometry audit

Authoritative wheels have radius 0.15 m, lateral offsets +/-0.34 m (separation
0.68 m), and front/rear offsets +/-0.35 m. Generated DiffDrive SDF agrees exactly
with the radius/separation. Wheel axes/signs yield positive forward motion and
positive yaw for right-faster-than-left. The plugin drives all four wheels but
computes odometry from the first left/right joint angles, as shown in the
[Gazebo Sim 8 implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/diff_drive/DiffDrive.cc).

For steady straight motion all four measured speeds were +1.3333 rad/s. Rotation
gave left -0.9067 and right +0.9067 rad/s; curve gave left +0.3200 and right +1.6800.
Using measured first-pair angles and configured radius/separation reproduced odometry
yaw increments to 8.4e-15 rad. Thus the odometric integration/signs match the wheel
evidence. This does not establish no-slip physical motion.

Spawn is `[0,0,0.25,0,0,0]`; `world -> odom` uses that configured transform. Wheel
odometry starts at zero; body twist uses `base_link`. Audits compare poses in the
same frame at source timestamps, with yaw unwrapping and bracketed interpolation.
Independent native Gazebo model poses agree with the bridged physical-pose stream
within 0.504 mm / 0.000499 rad across the audited captures; interpolation rates
account for small residuals. Neither stream is input to the estimator.

The 38 s drive fixture uses production bringup, the normal safe-command adapter and
unchanged PID/physics, with control `publish_commands=false`. It commands stationary,
straight, stop, rotation, stop, curve, stop; one-second ramps avoid step artifacts.

| Segment, after M03 | Physical motion | Wheel-odometry motion |
|---|---|---|
| Straight, including ramp | 1.471433 m | 1.471433 m; difference 3.16e-8 m |
| Rotation, including ramp | 2.020373 rad yaw; 0.353156 m translation | 2.941666 rad yaw; essentially zero translation |
| Curve, after ramp | 0.995671 m displacement; 1.423882 rad yaw | 0.848157 m; 2.025000 rad yaw |
| All steady stopped windows | zero displacement/yaw change | numerical zero |

The before drive gives essentially the same rotation failure (2.019604 vs 2.941334
rad, 0.352756 m translation). End-to-end drive mean base position error changes only
0.414309 -> 0.408203 m; mean yaw error 45.534 -> 44.555 degrees. This is strong
evidence for four-wheel scrub/skid and the ideal differential-drive assumption,
rather than radius, yaw sign, spawn or integration timestep as the primary cause.
No friction/contact model was identified from these data. Additional independent
yaw-rate/heading/pose information is needed to constrain drift; wheel-derived pose
cannot be fused as an independent absolute measurement. Existing IMU yaw-rate input
is a future integration point, but no sensor or SLAM was added in M03.

## Numerical validation

Raw evidence directories are immutable captures. `m03_metrics.json`, localization
audit JSON and replay JSON are derived analyses. Each capture retains configuration,
generated SDF, callback/commit journal, raw streams, physical poses, CPU, RTF and logs.

### Identical-recording replay

`m03_wide_before/processing.json`, SHA256
`658d3ff73624d301f7c3d3f92f7433136463eb012b212dd58e7c0bc981311f70`, is replayed
through both estimators with identical initial state and **effective deployment
measurement covariance overrides**. Baseline rejection counts reproduce the live
recording. The new replay simulates a 100 Hz timer; executor ordering at a timer tie
is an approximation. It compares filter posteriors, including those that the live
raw-violation guard would suppress, to avoid publication-selection bias.

| Metric | Before | After |
|---|---:|---:|
| Joint accepted / received | 24,931 / 31,351 | 31,351 / 31,351 |
| Wheel accepted / received | 3,239 / 3,258 | 3,242 / 3,258 |
| Remaining late wheel rejects | 19 | 16 |
| Joint RMSE, yaw/shoulder/elbow/wrist, rad | .001169 / .002245 / .004944 / .020709 | .000203 / .000470 / .001032 / .002280 |
| Maximum estimated shoulder excess | .004427 rad | 0 |
| Mean base position error | .259716 m | .264533 m |
| Mean absolute yaw error | 13.207 deg | 13.005 deg |
| Mean joint NEES (4-dimensional) | 107,522 | 1,285 |

Reordering recovers legitimate encoder evidence; it does not eliminate physical
localization drift. NEES improves but remains far above a calibrated four-state
model. No claim of complete covariance consistency is justified.

### Production closed-loop comparisons

Each nominal/wide run lasts 60 simulated seconds, with 10 s settling. Same scenario,
initial configuration and limits are used before/after. Reactive trajectories and
executor timing can diverge, so these are descriptive single-run comparisons, not
phase-locked causal estimates. Physical tracking errors use native Gazebo geometry,
including inactive intervals. Estimation errors use each estimate's own timestamp.

| Metric | Nominal before | Nominal after | Wide before | Wide final |
|---|---:|---:|---:|---:|
| Joint rejected / received | 11,317 / 30,436 | 1 / 23,380 | 6,420 / 31,351 | 2 / 24,540 |
| Wheel rejected / received | 12 / 3,338 | 3 / 3,302 | 19 / 3,258 | 2 / 3,391 |
| Fusion-age p95 | 16 ms | 40 ms | 12 ms | 40 ms |
| Mean base position error | .132782 m | .069516 m | .259684 m | .180886 m |
| Mean absolute yaw error | 8.418 deg | 6.038 deg | 13.204 deg | 12.666 deg |
| Shoulder RMSE | .001480 rad | .000442 rad | .002245 rad | .001009 rad |
| Wrist RMSE | .013863 rad | .004755 rad | .020705 rad | .006746 rad |
| Max published estimated shoulder excess | 0 | 0 | .004427 rad | 0 |
| Mean physical distance error | .106132 m | .077416 m | .393336 m | .207259 m |
| Mean physical pointing error | 10.731 deg | 8.340 deg | 30.997 deg | 24.759 deg |
| RTF | .870248 | .881512 | .885588 | .872229 |
| Estimator CPU, percent of one core | 86.11 | 85.97 | 83.11 | 87.92 |

Final nominal/wide each retain four pending observations at shutdown. Their joint
rejects are future-stamp failures (one/two); wheel rejects are late (three/two).
Both have zero fusion failures and zero backward commits. Average added queue delay
for joints is 22.85/23.18 ms and wheels 27.59/27.79 ms. Final wide performed 20
conditioning operations and retained 34 raw out-of-bound observations in diagnostics.

Raw shoulder excursions remain visible: wide .000220247 -> .000144079 rad maximum;
other joints had no position excess. All are within unchanged M02 acceptance, but
they are real physical excursions, not legalized estimates. Wide controller
joint-position-invalid samples fell from 39 to zero; observed recovery counter
maximum fell 17 -> 4. Both ended active, with no observed permanent stop. Sampled
5/10 Hz diagnostics cannot certify that every short recovery transition was seen.
Nominal had no raw position excursions or position recovery events.

The final wide run also contains 41 stale-target-prediction and 46 waiting-target
samples; these were not removed from physical tracking metrics. The target/prediction
pipeline was not modified. This remains an availability limitation.

M02 physical overspeed persists. Wide peak joint speeds were
`[1.0689,1.2854,2.2425,6.9849] -> [1.0278,1.3877,3.0089,5.3545] rad/s`.
The closed-loop observers return **1** because the measured-speed acceptance check
fails, not because the simulation failed to run. Command bounds, position acceptance
and watchdog checks passed. Both controlled-drive observers return **0**. None of
this upgrades M02 to PASS.

Covariance matrices stayed symmetric/positive in measured output (minimum eigenvalue
at least 3.49e-11 across final nominal/wide; asymmetry below 1e-14). Nevertheless,
mean joint NEES remains 2,204 nominal / 4,850 wide, with only 2.09% / 3.16% under
the 95% four-dimensional chi-square threshold. Aggressive unmodeled PID acceleration,
physical overspeed and the Gaussian contact approximation are not covered by the
current noise prior. Deterministic Gazebo observations and correlated time samples
also preclude treating these empirical fractions as a formal statistical confidence
test. Do not use the covariance as a calibrated safety certificate. No further noise
tuning was performed.

## Tests and acceptance

- Build: `robot_estimation` and `robot_bringup`, success in 7.48 s (`m03_build.log`).
- Final functional regression: **848 passed, 18 style/copyright tests deselected**,
  8.24 s (`m03_regression_final.log`).
- Focused estimator plus analysis: **86 passed**, 0.96 s
  (`m03_targeted_final.log`): in-order/interleaved/delayed/same-source reordering,
  duplicates, invalid/stale/future/frame rejection, capacity/missing streams, reset,
  actual-violation publication suppression, contact/release, correlated moments,
  exact motion/Jacobian/Q, source-time frame analysis and replay override fidelity.
- New estimator modules/tests pass targeted ament lint. Existing files retain legacy
  long-line/docstring lint findings; functional success is not a repository-wide
  style claim. Initial combined pytest collection required disabling the installed
  launch-testing plugins to avoid duplicate test-module-name imports; this was a
  test invocation issue, not a product regression.
- Live evidence: final nominal/wide production launch, controlled stationary/
  straight/rotation/curve drive, independent pose cross-check, archived baseline and
  same-recording replay. No new Gazebo run was required during final closure.

**CONDITIONAL** is deliberate. Timestamp causality, bounded buffering, improved
joint accuracy, authoritative constraints and visibility of raw faults are verified.
Skid-related global localization, aggressive covariance calibration, occasional
target-prediction unavailability, non-real-time scheduling and M02 physical speed
enforcement remain open. These require separate engineering work, not hidden tuning.

## Reproduction

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_estimation robot_bringup
source install/setup.bash

# Working production launch with M03 defaults (GUI optional).
ros2 launch robot_bringup closed_loop.launch.py \
  config:=config/reactive_tracking_oscillatory.yaml \
  strategy_config:=config/whole_body.yaml enable_actuation:=true gui:=true

# Functional regression and focused analysis.
python3 -m pytest src --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros -q \
  -k 'not flake8 and not pep257 and not copyright'
python3 -m pytest src/robot_estimation/test tools/validation/test_m03_analysis.py -q

# Recompute derived audits from preserved evidence, without running Gazebo.
python3 tools/validation/m03_replay.py results/validation/m03_wide_before
python3 tools/validation/m03_localization_audit.py \
  results/validation/m03_drive_before results/validation/m03_drive_after \
  results/validation/m03_nominal_after results/validation/m03_wide_final

# Optional fresh finite physical capture; choose unused output/domain/partition.
ROS_DOMAIN_ID=147 GZ_PARTITION=m03_repeat python3 tools/validation/v2_tracking_gazebo.py \
  --case reactive --config config/reactive_tracking_oscillatory.yaml \
  --strategy-config config/whole_body.yaml --physical-ground-truth --without-markers \
  --shoulder-posture --estimator-trace --duration 60 --settling-time 10 \
  --wall-timeout 240 --output /tmp/m03_nominal_repeat
```

Append `--before-m03-estimation` to select archived pre-M03 estimation only. For
aggressive tracking use `config/reactive_tracking_oscillatory_aggressive.yaml`.
For the safe-command drive use `--drive-fixture tools/validation/m03_drive.yaml
--duration 38 --settling-time 5` and a fresh output; the fixture sets
`publish_commands=false` for the controller. Independent physical pose publication
is enabled only for observing validation. Exact original final-run commands,
domains and return codes are in `validation/m03_remaining_runs.json`.

## Changed files and evidence

The exact M03 implementation/documentation list is
[`validation/m03_modified_files.txt`](../results/validation/m03_modified_files.txt), with final
source hashes in `validation/m03_source_hashes.json`. The machine-readable evidence
file inventory is `validation/m03_evidence_files.txt`; it includes preserved raw
captures, provisional performance evidence and derived analysis. Existing dirty
files from earlier milestones are intentionally excluded from M03 ownership.
