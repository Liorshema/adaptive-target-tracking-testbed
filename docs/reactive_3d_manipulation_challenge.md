# Reactive 3D manipulation challenge

Engineering report, 2026-10-19. The experiment is an acceleration-driven extension
of the existing testbed. Nominal configuration, estimation/prediction algorithms,
WLS/DLS algorithms, robot geometry, tracking radius and M02 native actuator are
preserved. Numerical acceptance and limitations are reported below; larger joint
motion alone is not successful tracking.

## Model and ownership

`robot_simulation/dynamics/target/fight_flight.py` retains the selected saturated
radial interaction, `a_max=.15 m/s²`, transition length `.15 m`, preferred radial
interaction distance `1 m`, damping `.5 s⁻¹`, epsilon `1e-6 m`. These are inherited
from `config/scenarios/target_saturated.yaml`. The controller's independently
owned desired camera-target distance remains `R=1 m`, optical direction camera +X.

New `disturbances/base_guidance.py` adds, in world coordinates,

```
s = target_position - physical_base_position
rho = hypot(s.x,s.y); h = s.z
er = [s.x,s.y,0]/sqrt(rho²+epsilon_rho²); ephi = ez cross er
a_guide = -k_rho*(rho-rho_d)*er - k_h*(h-h_d)*ez + a_phi*ephi
```

This is summed through `CompositeAccelerationField` into the existing derivative
and RK4 engine. There is no extra damping, total-acceleration cap, target-position
assignment, joint animation or controller switch. The radial Fight/Flight bound
is not a bound on combined environmental acceleration. Camera and base changes
therefore change the actual integrated target path.

`disturbances/phase_schedule.py` validates a generic list of named phases, positive
durations, positive later transition windows no longer than their phase, and
finite numeric `[rho,height,tangent]` values. Radius/height are nonnegative. At
each later phase start, cubic `3*tau²-2*tau³` blends the previous values into the
new values. First values hold before time zero; final values hold after 40 s.
Transitions and first derivatives are continuous. Scheduling uses absolute
simulation source seconds; reset returns to the first phase without wall time.

| Phase | Source interval | Transition at start | rho preference | Height above base | Tangent |
|---|---:|---:|---:|---:|---:|
| Low overhead | 0–10 s | 0 s | .55 m | 1.60 m | 0 m/s² |
| Vertical escape | 10–20 s | 3 s | .60 m | 1.85 m | 0 m/s² |
| Orbit | 20–40 s | 3 s | 1.30 m | 1.35 m | .30 m/s² |

Both restoring gains are `1.5 s⁻²`, radial regularization `.02 m`. Initial target
world position is `[.5390366178,.109268132,1.85] m`, zero velocity: nonzero radius
and azimuth `.2 rad` allow circulation without an artificial singularity kick.
Preferences are force equilibria, not guaranteed trajectories. Fight/Flight and
base motion can displace the target from them.

The existing nominal environment is
`robot_simulation/config/scenarios/environment_oscillatory.yaml`: fixed restoring
center `[2.2,0,.85] m`, diagonal gains `.5 s⁻²`, plus sinusoidal amplitudes
`[0,.35,.30] m/s²`, frequencies `[0,.20,.30]`, phases `[0,0,pi/2]`. The challenge
replaces only its own field list with base guidance and selects saturated radial
Fight/Flight without the separate camera-centered tangential field. Historical
scenarios and their fields remain unchanged.

## Physical source-time contract

`state/physical_context.py` represents physical base position, source stamp,
reference frame, provenance and maximum age. Missing, future, expired or wrong
frame data cannot supply guidance. No identity/base-position fallback exists.
Ordinary position/time-only fields retain their API; contextual fields explicitly
advertise their requirements and receive typed context through the composite.

Challenge deployments select `tracking.pose_source: physical`. Bringup's existing
FK broadcaster uses physical-base odometry and named measured joints, applying
the authoritative camera mount once. Nominal deployments retain wheel-odometry
camera TF. Truth is used for simulation interaction/diagnostics only; estimator
and controller localization still use their existing measured wheel-odom path.
Consequently A/B also differ in interaction-point provenance and initial target
state, not solely in one field coefficient.

A reviewed defect was corrected: combined FK is stamped at the **newer** input
stamp, so it cannot claim to precede a measurement used to compute it. Older
inputs are held within the existing age/skew bounds. The challenge caches each
direct world-to-camera TF while target pose-service requests are pending.
`TimeStepper` refreshes causal base/camera samples before each RK4 substep rather
than freezing one sample over a delayed catch-up interval. History retention is
derived from the existing integration maximum catch-up duration plus context
maximum age; freshness limits remain `.3 s`. No interpolation using future
physical observations occurs. Gaps without actual fresh historical samples fail
closed; an unsupported gap requires a world reset, not stale extrapolation.

Clock reset clears both histories and schedule epoch. Old-epoch future odometry
and TF are rejected, and outstanding old-epoch pose-service completions cannot
commit into the new target epoch. Focused tests cover these cases. The live restricted-wrist C reset probe also passed: control recovered after
a real Gazebo reset and both native actuator epochs were complete and safe.
Evidence is `results/reactive_challenge_reset/reset_20261019T160239_195024Z_vp444xry`.

## Kinematic feasibility

[Reproducer](validation/reactive_challenge_feasibility.py) and
[numerical witnesses](validation/reactive_challenge_feasibility.json) use actual
`robot_models` FK/Jacobians and authoritative description. Base world z is `.25 m`,
shoulder world z `.43 m`; camera mount is wrist translation `[.12,0,0]`, zero RPY,
selected optical +X. No second approximate kinematic implementation is used.

102 stored solutions respect physical bounds, with maximum virtual-target
residual `3.14e-10 m`; finite-difference task-Jacobian discrepancy `7.15e-11`.
The following are numerical witness intervals, not proven global extrema.

| Wrist | Low shoulder-camera reach | High reach | Ring reach |
|---|---:|---:|---:|
| Nominal ±1.57 rad | .622–.803 m | .775–.856 m | .749–.851 m |
| ±30° | .622–.709 m | .775–.851 m | .751–.839 m |
| ±15° | .622–.661 m | .785–.830 m | .765–.815 m |

For ±15°, the low/high witness intervals have at least `.124 m` separation.
A low witness has arm yaw about `-2.942 rad`, shoulder `.609 rad`, elbow `1.57 rad`,
wrist `.036 rad`. High witnesses require substantially different shoulder/elbow
configuration. Nominal wrist redundancy also permits a fairly extended low pose,
so folding is not guaranteed there. More directly overhead/lower candidates lose
reachability with restricted wrists. Arm task conditioning is about 7.5 at the
selected low witness and 10–14 at high witnesses; nearly fully extended alternatives
are worse conditioned. Rear targets have valid folded configurations, but a full
continuous orbit requires branch/base coordination around finite arm-yaw travel.

These are collision-unchecked kinematic witnesses. They do not demonstrate
collision-free paths, occlusion-free views, dynamic feasibility or convergence of
the existing local differential controller from the nominal startup posture.

## Configuration and WLS allocation

| Variant | System resource in robot_bringup/config | Target | Wrist | WLS |
|---|---|---|---|---|
| A | system_manipulation_A.yaml | Existing nominal | Nominal | Nominal |
| B | system_manipulation_B.yaml | Challenge | Nominal | Nominal |
| C | system_manipulation_C.yaml | Challenge | ±15° | Nominal |
| D | system_manipulation_D.yaml | Challenge | ±15° | Arm priority |
| E | system_manipulation_E.yaml | Challenge | ±30° | Nominal |

System/profile/scenario/strategy resources use existing relative YAML inheritance.
`system_nominal.yaml` is unchanged. Wrist descriptions are complete authoritative
owner documents because Xacro and existing direct description consumers read plain
YAML. Exact-value tests ensure only wrist position bounds differ. Selected files
flow consistently through model, WLS constraints, final safety, estimator physical
conditioning, reference generation, Gazebo/Xacro and validation. Initial hold values
remain the existing valid raised-reference configuration, with wrist zero. Invalid
selected initial positions are rejected before launch. No independent WLS limit
or actuator limit table was added.

Generalized velocity order is `[base linear, base yaw, arm yaw, shoulder, elbow,
wrist]`. WLS penalizes `damping² * sum(w_i*(u_i/vmax_i)²)` in addition to task,
posture, smoothness and limit-avoidance terms. Nominal motion weights
`[1,2,3,3,3,2]`, velocity scales `[.5,1,1,1,1,1.5]` make base yaw cheaper than arm
yaw before other terms. Their Jacobian columns share rotation but have different
origins and translational effects. Posture, distance geometry, limits, target
motion, and skid-dependent plant response also affect allocation.

Experimental `robot_control/config/whole_body_manipulation_arm_priority.yaml`
changes only base-yaw motion weight `2 → 12`. Base yaw remains available.
[Verified single-mount solver evidence](validation/reactive_challenge_allocation_verified.json)
and [reproducer](validation/reactive_challenge_allocation_verified.py) show centered
base/arm yaw `.11037/.05719 rad/s` nominal versus `.04954/.11797 rad/s` experimental:
arm share `34.13% → 70.42%` in this specific task. At the arm upper bound, the
candidate arm-yaw speed is zero and base yaw is `.16726 rad/s`. These are genuine
optimizer decisions, not special-case switching or universal allocation ratios.
The earlier `reactive_challenge_allocation_audit.json` accidentally applied the
mount twice; it is preserved and explicitly superseded by the verified artifact.

## Evidence policy and execution

M08 retains per-run owner configurations/initial conditions/sensor seed, source
revision, tracked diff hash, relevant tracked/untracked source-tree hash,
component identities, physical state/target/joint/command/safety telemetry,
source and wall times, CPU/RSS and cleanup. `execution.record_actuator_trace`
is an additive boolean, false by default; this study enables run-local full-step
M02 traces without changing holding gains, limits or physics. Trace output paths
are excluded from physical comparison identity. BLAS thread settings are recorded
in metadata; final controlled commands set one BLAS/OpenMP thread per process.

Reactive paths change with wrist/controller behavior even with the same sensor
seed. No target replay was added. M08's existing physical-contract comparability
gate remains unchanged: A/B/C/E cannot be pooled as matched trajectories; C/D
share a physical contract but still have reactive trajectory differences.

An interrupted initial study completed A/B seed42 but left C orphaned after the
recorder stopped. Its partial telemetry and 8.4 GB extended actuator trace remain
preserved and excluded from acceptance. Only its verified private ownership-marker
processes were stopped; [cleanup](validation/reactive_challenge_interruption_cleanup.json)
reports no survivors. Subsequent bounded retry failures are retained separately.
Their timestamp/readiness failures motivated the causal catch-up correction above;
no old failed run is relabeled successful. The final study uses the corrected
implementation for all variants.

Phase metrics use named physical JointState observations, trapezoidal squared
velocity divided by observed duration, explicit gap exclusion, range/travel,
position/speed utilization and unchanged `.001 rad` / `.001 rad/s` acceptance
policy. Rear hemisphere and aft-axis crossings subtract **physical base heading**;
angle unwrap and angular coverage reset across gaps. Shoulder-camera reach uses
authoritative FK, not wrist angle. Startup is separated at first valid control;
phase windows use absolute source time and are clipped to actual active coverage.
Nominal A uses the same time windows for comparison, without claiming its target
has the challenge phases. Full physics traces independently check every joint at
every recorded physics iteration; missing motor torque remains unavailable.

## Reproduction

From the workspace, with ROS Jazzy and built installation:

```bash
cd /home/user/ros2_ws
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

# Original nominal behavior / restore nominal selection:
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_nominal.yaml enable_actuation:=true gui:=false

# Challenge B, nominal wrist/weights:
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_manipulation_B.yaml enable_actuation:=true gui:=false

# Choose one of the other exact installed selections:
ros2 launch robot_bringup closed_loop.launch.py system_config:=config/system_manipulation_A.yaml enable_actuation:=true gui:=false
ros2 launch robot_bringup closed_loop.launch.py system_config:=config/system_manipulation_C.yaml enable_actuation:=true gui:=false
ros2 launch robot_bringup closed_loop.launch.py system_config:=config/system_manipulation_D.yaml enable_actuation:=true gui:=false
ros2 launch robot_bringup closed_loop.launch.py system_config:=config/system_manipulation_E.yaml enable_actuation:=true gui:=false

# All five variants, seeds 42 and 917, 40 simulation seconds each:
ros2 run robot_experiments run --config config/reactive_manipulation.yaml --validate-only
ros2 run robot_experiments run --config config/reactive_manipulation.yaml \
  --output results/reactive_challenge_reproduction

# Optional existing read-only visualization wrapper (same selection):
ros2 launch robot_bringup visualization.launch.py \
  system_config:=config/system_manipulation_C.yaml enable_actuation:=true gui:=true rviz:=true

python3 docs/validation/reactive_challenge_feasibility.py --output /tmp/challenge_fk_fresh.json
python3 docs/validation/reactive_challenge_allocation_verified.py --output /tmp/challenge_wls_fresh.json
python3 docs/validation/reactive_challenge_report.py --study STUDY_DIRECTORY \
  --output FRESH_REPORT_DIRECTORY

# Permanent launch compute-only check:
python3 docs/validation/m09_live.py --output results/challenge_compute_reproduction \
  --compute-only --duration 4 --settling-time 2
```

Each variant's exact system selection is listed above; no Python changes are
needed. RViz source and installed defaults already have full TF display disabled,
RobotModel visible and camera axes/optical markers enabled. No transforms or saved
visualization defaults were removed. A manually enabled TF checkbox remains a UI
choice rather than a reason to delete intermediate frames.

## Final measured results

Final study: `results/reactive_challenge_final/reactive_manipulation_20261019T155152_099000Z_nmxd3n7j`. Nine completed runs; E/917 failed readiness after 45 host seconds (simulation clock 32.592 s). All five variants have live evidence; A/B/C/D have both seeds, E has one completed seed. No failed/incomplete trial contributes a successful comparison score. At the E/917 timeout, all required streams had published, but the recorder's last physical joint stamp was 32.408 s versus clock 32.592 s (184 ms lag, exceeding the unchanged 80 ms readiness gate). This does not establish that the target or controller process crashed; the freshness requirement was not weakened.

Analysis uses the common absolute schedule ending at simulation t=40 s. M08 readiness can finish late and its original 40-second measurement window can include the final held phase (for example B/42 extends past t=60). Those original metrics and windows remain retained; they are not silently pooled with the schedule comparison. The earlier full-run figures in `results/reactive_challenge_final_report` remain preserved; the verified figures and tables below explicitly cap phase analysis at t=40.

| Run | Distance error mean/max [m] | Pointing mean/max [deg] | Recorded M08 RTF | Physics limits |
|---|---:|---:|---:|---|
| A_seed42 | 0.080 / 0.263 | 8.7 / 19.9 | 0.785 | PASS recorded |
| A_seed917 | 0.074 / 0.306 | 6.7 / 14.7 | 0.887 | PASS recorded |
| B_seed42 | 0.537 / 1.763 | 58.9 / 144.2 | 0.836 | PASS recorded |
| B_seed917 | 0.553 / 1.757 | 57.3 / 149.0 | 0.858 | PASS recorded |
| C_seed42 | 0.369 / 1.272 | 52.5 / 128.6 | 0.857 | PASS recorded |
| C_seed917 | 0.392 / 1.716 | 44.4 / 148.3 | 0.868 | PASS recorded |
| D_seed42 | 0.415 / 1.502 | 49.3 / 137.4 | 0.858 | PASS recorded |
| D_seed917 | 0.442 / 1.084 | 42.1 / 96.6 | 0.859 | PASS recorded |
| E_seed42 | 0.345 / 1.141 | 48.6 / 119.3 | 0.861 | PASS recorded |

Tracking is substantially worse in the challenge. These are physical-camera +X errors, not errors computed from a convenient alternative axis. No control-acceptance threshold was loosened. RTF belongs to each recorded M08 window; CPU load, late readiness and asynchronous feedback limit causal performance comparison.

| Challenge run | Low region dwell [s] | Rise height min–max [m] | Peak upward speed [m/s] | Orbit rear dwell [s] | Aft crossings | Relative angular excursion [rad] |
|---|---:|---:|---:|---:|---:|---:|
| B_seed42 | 4.05 | 1.46–2.09 | 0.31 | 12.59 | 1 | 4.74 |
| B_seed917 | 5.27 | 1.55–1.92 | 0.13 | 12.80 | 1 | 4.66 |
| C_seed42 | 6.06 | 1.56–2.07 | 0.23 | 7.86 | 1 | 6.52 |
| C_seed917 | 5.37 | 1.55–2.09 | 0.27 | 9.20 | 1 | 6.12 |
| D_seed42 | 4.18 | 1.55–2.09 | 0.26 | 7.89 | 1 | 6.72 |
| D_seed917 | 5.22 | 1.64–2.06 | 0.21 | 7.97 | 1 | 6.25 |
| E_seed42 | 5.79 | 1.60–2.02 | 0.16 | 7.96 | 1 | 6.73 |

Every completed challenge entered the configured low region and physical rear hemisphere. The ring is centered on measured physical base position, and rear detection subtracts physical heading. Angular excursion is the largest independently unwrapped observed segment; it does not invent rotations across missing data or imply every trial completes a full circle.

| Run (seed42) / phase | Angular ranges [yaw, shoulder, elbow, wrist] [rad] | FK reach min–max [m] |
|---|---:|---:|
| A_seed42 / low_overhead | [0.215, 0.214, 0.234, 0.372] | 0.860–0.875 |
| A_seed42 / vertical_escape | [0.181, 0.172, 0.128, 0.283] | 0.866–0.878 |
| A_seed42 / orbit | [0.204, 0.195, 0.097, 0.271] | 0.863–0.876 |
| B_seed42 / low_overhead | [0.420, 0.962, 0.951, 1.570] | 0.744–0.876 |
| B_seed42 / vertical_escape | [0.633, 0.925, 0.608, 1.920] | 0.788–0.877 |
| B_seed42 / orbit | [1.337, 1.497, 1.227, 1.340] | 0.742–0.880 |
| C_seed42 / low_overhead | [0.267, 0.706, 1.844, 0.262] | 0.641–0.877 |
| C_seed42 / vertical_escape | [0.587, 1.339, 0.991, 0.523] | 0.684–0.871 |
| C_seed42 / orbit | [0.156, 0.287, 0.242, 0.090] | 0.863–0.876 |
| D_seed42 / low_overhead | [0.623, 0.705, 1.916, 0.478] | 0.620–0.877 |
| D_seed42 / vertical_escape | [0.887, 0.853, 0.812, 0.501] | 0.628–0.826 |
| D_seed42 / orbit | [0.295, 0.529, 0.558, 0.001] | 0.802–0.869 |
| E_seed42 / low_overhead | [0.253, 0.743, 1.828, 0.524] | 0.620–0.876 |
| E_seed42 / vertical_escape | [0.452, 1.261, 0.951, 1.045] | 0.681–0.865 |
| E_seed42 / orbit | [0.051, 0.357, 0.242, 0.000] | 0.799–0.834 |

Full per-joint, per-phase range, observed travel, time-weighted RMS speed, peak speed, position/speed utilization, near-bound duration, violation count/duration, coverage and gaps for both seeds are in [the joint metrics CSV](validation/reactive_challenge_joint_metrics.csv). Startup and active summaries are separate. Motion is unsmoothed physical feedback resolved by joint name.

| Run | Shoulder / elbow observed travel [rad] | Arm yaw range [rad] | Physical base yaw excursion [rad] | Base planar travel [m] |
|---|---:|---:|---:|---:|
| A_seed42 | 2.28 / 1.87 | 0.223 | 0.193 | 6.04 |
| A_seed917 | 2.20 / 1.73 | 0.215 | 0.190 | 5.91 |
| B_seed42 | 7.35 / 8.57 | 1.556 | 1.305 | 11.52 |
| B_seed917 | 8.04 / 8.19 | 1.530 | 1.218 | 11.27 |
| C_seed42 | 3.48 / 4.82 | 0.601 | 0.175 | 6.60 |
| C_seed917 | 7.67 / 6.85 | 2.058 | 1.630 | 11.03 |
| D_seed42 | 3.18 / 4.62 | 1.280 | 0.108 | 5.72 |
| D_seed917 | 0.89 / 1.89 | 0.312 | 0.016 | 1.68 |
| E_seed42 | 3.58 / 4.80 | 0.560 | 0.134 | 5.91 |

B increases shoulder/elbow travel markedly relative to A. Wrist restriction **does not consistently increase** their participation: C/42 has less shoulder/elbow travel than B/42. D/42 increases arm yaw range relative to C/42, whereas D/917 barely moves the arm yaw and its high-phase effective reach stays near .608 m. The expected high extension therefore fails in that run despite a real target rise. No claim of reliable fold-to-extension behavior across variants is made. None of these live arm-yaw ranges approaches the physical travel bounds closely enough to demonstrate limit-triggered reallocation; only the separate actual-solver boundary experiment establishes the local constrained reallocation response.

## Physical safety, state accuracy and operational limits

All nine completed traces have a complete common physics-iteration grid across all four joints, totaling **799,568 physical joint rows**. Maximum measured speeds are yaw `.720368005`, shoulder `.9999999998`, elbow `1.0000000005`, wrist `.717344955 rad/s`, versus authoritative `[1,1,1,1.5] rad/s`. Maximum position excess is shoulder `6.471531e-5 rad` below zero; wrist excess is approximately `1.6e-12 rad`. Both are below unchanged `.001 rad` policy. Thus recorded physical safety passes; exact nonpenetration and a universal motor/contact speed guarantee are not claimed. Native torque remains unmeasured; bounded effort configuration is not torque telemetry.

| Run | Mean robot yaw error [deg] | Mean robot position error [m] | Control inactive observed [s] | Actuation recovery observed [s] | Latched fault observed [s] |
|---|---:|---:|---:|---:|---:|
| A_seed42 | 5.3 | 0.073 | 0.000 | 0.000 | 0.000 |
| A_seed917 | 6.0 | 0.118 | 0.000 | 0.000 | 0.000 |
| B_seed42 | 62.2 | 5.929 | 0.000 | 0.000 | 0.000 |
| B_seed917 | 51.5 | 1.632 | 0.000 | 0.202 | 0.000 |
| C_seed42 | 20.8 | 0.456 | 0.100 | 17.296 | 0.000 |
| C_seed917 | 55.8 | 0.964 | 0.000 | 0.506 | 0.000 |
| D_seed42 | 10.1 | 0.184 | 0.000 | 18.592 | 0.000 |
| D_seed917 | 14.0 | 0.052 | 0.000 | 33.994 | 0.000 |
| E_seed42 | 20.7 | 0.155 | 0.000 | 17.406 | 0.000 |

State-error means use the separately retained M08 evaluation windows; they are diagnostic context, not a matched-window causal decomposition of the tracking error. B/42 reaches roughly 62° mean yaw and 5.9 m mean position error, while joint estimate errors remain small. This strongly supports existing skid/localization mismatch as a material contributor. Local pose/limit/optimization effects also remain; this experiment does not isolate every cause. The existing spherical-log pointing geometry already has an antipodal escape chart, so a zero-error antipodal bug was not assumed or introduced. Estimator, predictor, Q/R, IMM and WLS algorithms were not changed to improve these scores.

Recovery, inactivity and fault durations describe only observed eligible source-time intervals; missing intervals do not certify normal operation. Actual latched guard reasons are recognized by metrics. Full JSON includes health coverage/counter deltas, prediction availability, startup events and raw log errors; SIGINT cleanup backtraces are retained rather than counted as successful runtime crashes. All final run cleanup records have no owned survivors, including failed E/917.

## Plots and inspectable evidence

[Verified report data](../results/reactive_challenge_verified_report/summary.json) contains every phase, joint, coverage and safety result. Each completed run has four standalone figures: `_trajectory.png`, `_geometry_tracking.png`, `_joints.png`, and `_physical_limits.png`. Geometry plots show scheduled preferences and actual rho/height, physical-heading-relative azimuth, FK reach, base/arm yaw, and tracking errors. Joint plots include all four measured angles/speeds with physical bounds; physics plots include every recorded native sample, including the final hold tail. Gaps are broken rather than smoothed or bridged.

![B42 measured geometry and tracking](../results/reactive_challenge_verified_report/B_seed42_geometry_tracking.png)

![C42 physical joint motion](../results/reactive_challenge_verified_report/C_seed42_joints.png)

![D42 base-relative target trajectory](../results/reactive_challenge_verified_report/D_seed42_trajectory.png)

![B42 physics-step limit evidence](../results/reactive_challenge_verified_report/B_seed42_physical_limits.png)


## Verification and verdict

- Baseline: 1,224 functional tests; ten-package build; nominal isolated Gazebo
  trace accepted. Baseline hashes/status are in
  [the preservation baseline](validation/reactive_challenge_baseline.json).
- Final functional/architecture/integration regression: **1,272 passed**,
  18 existing copyright/style checks deselected. Log:
  `docs/validation/reactive_challenge_verified_regression.log`. The whole style
  suite is not claimed clean; scoped lint of the new guidance, schedule, context
  and phase-metrics modules passes (`reactive_challenge_new_lint_pass.log`).
- Red/green tests cover guidance signs/regularization/determinism, strict YAML,
  smooth source-time scheduling, causal history/catch-up, direct TF source caches,
  reset epoch guards, joint-name order, gap/RMS accounting, heading-relative rear
  detection, latched safety reasons, exact wrist selection, invalid initial holds,
  trace isolation and physical context in comparison identity.
- All **ten ROS packages** built in normal and independent copied installations.
  The final snapshot-only experiment package was copied again after adding the
  new context comparison-identity field; no numerical algorithm changed.
  [Copied verification](validation/reactive_challenge_snapshot_installed.json)
  checks module origins outside the checkout, installed A–E resources, launch
  composition, native motor binary and consistent selected descriptions.
- Both normal and copied-install real Gazebo compute-only probes passed **29/29**
  checks with zero commands. Copied run:
  `results/reactive_challenge_copied_compute_only/headless_20261019T161055_540751Z_d_2nxctz`.
  Physical FK/optical direction, visualization, configured radius and cleanup
  checks passed; this is not a GUI screenshot claim.
- Real C Gazebo world-reset probe: **4/4** checks passed, including recovered
  control, observed clock reset, both complete/safe native actuator epochs and
  cleanup. Reproduce with:

  ```bash
  python3 docs/validation/m02_resolution_reset.py \
    --output results/challenge_reset_reproduction \
    --system-config config/system_manipulation_C.yaml
  ```

- Independent review found and corrected older-input TF stamping, historical
  context starvation, latched-stop reason accounting and a duplicate camera mount
  in an offline WLS probe. Corrected numerical evidence supersedes that probe;
  runtime kinematics never adopted the duplicate mount.
- [Final process audit](validation/reactive_challenge_final_processes.json)
  contains no running Gazebo/simulation/actuation launch processes. No staging,
  commits, Git resets, deletion of historical data or pushes occurred.

| Acceptance area | Verdict | Evidence / limitation |
|---|---|---|
| A: Reactive scenario excitation | PASS for completed challenge trials | Low-region dwell, actual rise, rear entry and aft crossing measured; E/917 incomplete |
| B: Meaningful manipulation | CONDITIONAL | B shoulder/elbow motion rises strongly; consistent high extension fails, especially D/917 |
| C: Wrist restriction | CONDITIONAL | Authoritative limits coherent and physically respected; greater shoulder/elbow participation is not consistently observed |
| D: WLS allocation | CONDITIONAL | Genuine cost-dependent numerical reallocation; live response varies and no live yaw-bound encounter demonstrates boundary transfer |
| E: Physical correctness | PASS for recorded limits; CONDITIONAL generally | Complete native traces pass unchanged tolerances; collision-free feasibility, motor torque and universal contact safety are unproven |
| F: Reproducibility | CONDITIONAL | Installed YAML/immutable inputs/seeds/source identity work; E/917 readiness fails, reactive paths differ and runtime is not bitwise deterministic |
| G: Architecture | PASS | Existing ten packages/factories, owner configuration and dependency boundaries preserved |
| H: Regression | PASS for functional scope | 1,272 tests, ten-package builds, copied installation, live reset and zero-command checks; excluded style checks remain explicit |

**Overall verdict: CONDITIONAL.** This implements and verifies a useful reactive
manipulation challenge, not a successful robust-tracking milestone. Retain the
original nominal selection for the established tracking baseline. The next
engineering action is a separate controlled diagnosis of physical-versus-estimated
base/camera pose and local reachable-branch control under circulation. The measured
localization errors and failed extension must be resolved or bounded before
claiming reliable manipulation. Do not infer that changing one WLS weight or
restricting the wrist solves those issues.

## Exact change inventory and preservation

[Task-only file inventory](validation/reactive_challenge_modified_files.txt)
distinguishes modified existing files, added source/config/tests, engineering
reports and validation evidence. Generated raw results and plots remain in the
separate result directories listed there. [Preservation checks](validation/reactive_challenge_preservation.json)
record no missing baseline files and unchanged protected robot geometry, nominal
manifest, estimator/predictor/control algorithms, native actuator/guard and holding
configuration. Existing M02/M09 work and historical experiment evidence remain.
