# Tracking loss and uncontrolled-looking motion: causal investigation

Task date: 2026-10-22. Primary system: `config/system_manipulation_D.yaml`.
**Final verdict: PARTIALLY CONFIRMED.**

All production source, configuration and the validated DART recovery backend are
preserved. This task adds diagnostic tools, tests and evidence only. No staging,
commit or push. Task baseline: `tracking_loss_baseline.json` (1,633 files).

## Established observations

The controller can regulate a camera that exists in its estimated odometry model
while the physical camera looks elsewhere. `valid: true` establishes input and
solver validity, not physical accuracy. The selected robot estimator has no
absolute planar-pose correction in this deployment: its base motion comes from
wheel velocity measurements, integrated through an ideal differential-drive model.
The physical robot has four fixed-axis wheels and does not obey ideal rolling
kinematics exactly during contact/skid turns. Raw wheel odometry and the final
robot estimate develop almost the same position and heading errors.

The source-aligned historical D/42 reconstruction finds mean internal pointing
error 7.38° versus 73.15° physically. The same pattern exists in the completed
D/917 recording. At 82.05 s, a 0.02457 m displayed desired/actual position gap and
0.02530 task-rate residual coexist with 129.52° physical pointing error; camera
model position differs from the physical reconstruction by 2.525 m. These are
aligned recorded observations, not the user's unsynchronized terminal snapshots.

![Physical versus internal task and pose errors](../../results/tracking_loss_figures_final/tracking_loss.png)

Vector figure: `results/tracking_loss_figures_final/tracking_loss.svg`.
Wrapped yaw jumps at ±180° are coordinate wrapping, not a clock reset. Detailed
stage and intervention results and the final causal verdict follow below.

## Verified runtime architecture

Actual constructed classes are recorded in `tracking_loss_runtime.json` and
corroborated by each launch log. Strategy inheritance overrides legacy scalar
parameters; the raw control YAML's `controller_type: jacobian` and zero lookahead
do not describe the selected Scenario D runtime.

| Boundary | Actual selected representation and consumer |
|---|---|
| Gazebo → wheel odometry | `nav_msgs/Odometry`, `/wheel/odometry`, pose in `odom`, child `base_link`; body `[v,omega]` in m/s and rad/s. Physical encoder displacement drives ideal wheel odometry, not Gazebo world pose. |
| Gazebo → joints | `sensor_msgs/JointState`, `/joint_states`, named wheel and arm q/dq in rad and rad/s. The estimator selects description-ordered arm joints; the actuator independently consumes raw feedback. |
| Measurements → robot estimator | `RobotStateEstimator` / EKF. Wheel sensor measurement is `[v,omega]`; joint measurement contains encoder q/dq. Optional IMU yaw rate is supported but this simulation has no active IMU source. No world-position or absolute-heading update is selected. 30 ms source-time reorder buffer retained. |
| Robot estimator → controller | `StateEstimate`, `/estimation/robot_state_full`, `odom`, mean `[x,y,yaw,v,omega,q(4),dq(4)]`, covariance 13×13. Stamp is filter source epoch. q is the fused joint estimate, not a raw encoder alias. |
| Robot state → model camera | `RobotStateMessageAdapter` → `PlanarKinematicsAdapter` → `WholeBodyComposer` with `DifferentialDriveKinematics` and `SerialArmModel`, then fixed tracking transform. `control.actual_pose` is **FK of estimated base and estimated joint state**, not a physical pose measurement. Planar base has z=0 in odom. |
| Physical target → measurement | `/simulation/target_position` is commanded/acknowledged numerical target position in `world`, `PointStamped`; `/perception/target_position` is noisy synthetic position measurement, in world and metres. No unknown future target is available online. |
| Target measurement → estimate | `TargetStateEstimator` / linear Kalman filter, `/estimation/target_state_full`, `[p(3),v(3)]`, world, 6×6 covariance. |
| Target estimate → forecast | `IMMPredictor` (CV/CA/coordinated-turn mixture) → `ResidualAdapter` → `WeightedCorrection`. `/prediction/trajectory` is the **final corrected forecast**, not raw IMM. Header stamp is target-state reference time; samples use absolute reference+0.1…2.0 s. The residual correction uses prior forecast errors against subsequent estimated observations, not physical future truth. |
| Forecast → selected reference | Fixed world→odom transform at forecast reference time; sample linearly at control `now+0.25 s`. Position and velocity feedforward are selected together. `control.target_position` is this future corrected reference in odom, not current target truth. |
| WLS → safety → actuation | `WeightedLeastSquaresController` → `SafeControlPipeline` → sole `SafeVelocityCommand` publisher → ordinary `VelocityActuationNode` → position references/native velocity servo → repaired DART backend → physical joints. Command order `[base v,base omega,arm dq(4)]`. All physical bounds, authority/watchdogs and PositionGuard modes remain unchanged. |

`world→odom` is the configured static spawn transform, `[0,0,0.25,0,0,0]` in
these experiments. All model/odometry coordinates are transformed through it
before comparison. Translation metrics are planar XY; physical camera FK also
uses the actual full base quaternion and z. The optical axis is tracking/wrist
**+X**, fixed tracking translation `[0.12,0,0]`, zero mount RPY. Desired orientation,
pointing geometry and visualization use that same axis. No evidence supports a
remaining +Z/+X mismatch. Earlier live FK/TF/optical gates remain applicable
because description, geometry and visualization hashes are unchanged; this
investigation reconstructs physical camera pose from world base truth and measured
joints rather than independently recording a camera-link pose sensor.

Control freshness: robot state ≤0.2 s, forecast reference ≤0.5 s, future tolerance
0.01 s, 20 Hz control. These bound age, not estimation bias. The native arm servo
runs at actual 0.002 s physics steps. Positive recorder callback-clock age subsets
are not treated as whole-pipeline latency: asynchronous recorder clocks can lag
source stamps. Analysis uses source stamps, reports coverage, and never combines
backward clock epochs.

## Exact task mathematics

The selected WLS task is three-dimensional: signed range error and a two-component
spherical logarithm of target direction in the camera tangent plane. Its pointing
norm equals optical angular error; free camera roll is not penalized. The task is
neither Cartesian position-only nor full 6D pose control:

`e = [||p_target-p_camera||-R_d, log_axis(direction)_1, log_axis(direction)_2]`.

The geometric Jacobian maps generalized robot velocity to error rate, and
`target_rate` is the target-motion contribution. WLS seeks
`J*u = -K*e - target_rate`, with description-derived feasible position/speed bounds,
normalized motion costs, damping, posture and smoothing regularization. Scenario
D gains are `[1,3,3]`, task weights `[4,12,12]`, motion weights
`[1,12,3,3,3,2]`, damping 0.08. These values are inspected, never retuned.

`task_residual = ||J*u - (-K*e-target_rate)||_2` is an **unweighted instantaneous
rate residual**, with mixed translational/angular units; it is not a pointing
angle. `active_bounds` counts tight feasible generalized-speed bounds; it is not
a singularity or fault count. The desired camera position is the radial nearest
point on the target sphere, so a small desired/actual position gap principally
indicates internal range compliance, independently of optical alignment.

Physical and internal task geometry reconstruction agrees with recorded
`task_error` to 2.35e-13 in historical D/42. There is an explicit antipodal chart
and alternate bounded escape direction. In the recorded deterioration the
**internal** pointing errors are generally small, so an unhandled internal
antipode is not supported as the primary mechanism. The controller cannot see
that its physical camera is behind the actual target when its state estimate
places a different camera in front of its selected reference.

## Measurement contract

`tracking_loss_analysis.py` refuses merged reset epochs and unexpected frames.
At each control source stamp it uses latest causal base/joint/target samples no
older than 80 ms. Camera FK uses the authoritative description exactly once.
The ≤80 ms hold error is a limitation; metre-scale pose error and large pointing
angles cannot be explained by that small delay alone. Localization compares
raw odometry and final estimate to causal world truth at their own source epochs;
CSV includes signed body longitudinal/lateral errors, wrapped signed yaw and
actual truth age. No independent terminal echoes are used.

Future prediction error alone uses offline linear interpolation of recorded
truth within brackets ≤150 ms; this is evaluation, never an online oracle input.
Statistics use equal valid sample weights (MAE/RMSE/max/p95). Compliance is a
left-held time integral on eligible adjacent control intervals ≤150 ms and
requires **both** |range error|≤0.20 m and pointing≤15°. These thresholds were
written in `tracking_loss_experiments.yaml` before live interventions. Missing
coverage is reported separately and is never counted as success.

The absolute-source-time schedule is low-overhead 0–10 s, vertical escape 10–20 s
(with cubic transition 10–13), orbit 20–40 s (transition 20–23). After 40 s the last
orbit settings are held; 40–90 s is extended orbit, not another programmed phase.
Startup readiness excludes some early data. Results preserve actual coverage.

## Prediction and reference consistency

Historical D/42 target measurement position MAE is 0.159 m; target Kalman estimate
MAE 0.0758 m at matched source times. Final corrected forecast position MAE is
0.0883 m at reference+0.1 s, 0.1246 m at +0.3 s, and 0.3374 m at +1.0 s. The
controller-selected target has mean 0.169 m discrepancy from current target and
0.1305 m from actual target at control+0.25 s. The latter is the relevant declared
horizon. Looking ahead is intentional, not evidence by itself of a frame bug.

The offline accepted-epoch predictor replay in
`results/tracking_loss_prediction_replay_verified/summary.json` **does not establish
output identity**: six forecasts lack a matching recorded estimate and maximum
state discrepancy is 0.389. Its CV/raw-IMM/fused comparisons therefore cannot
establish the benefit of live IMM/adaptation. No such benefit is claimed.
Recorded final-output metrics above remain valid; separate internal stages were
not recorded directly.

A smaller mathematical consistency limitation is established independently:
constant `[delta_p,delta_v]` residual offsets applied to every horizon need not
satisfy `d(p_corrected)/dt = v_corrected`. For a constant-velocity baseline and
nonzero velocity correction, the discrepancy is exactly delta_v. Recorded
finite-difference position-rate versus supplied-velocity mismatch averages
0.0101 m/s (max 0.0392). CT discretization also contributes to that recorded
quantity; the full value cannot be attributed to fusion alone. This merits a
focused future regression/repair but is not evidence that prediction causes the
large physical localization discrepancy.

## Separate target-context reliability finding

Previously failed D/917 and E/917 attempts repeatedly lacked physical context;
target output stopped or never bootstrapped, then ordinary prediction freshness
stopped control. Those attempts are not the smooth tracking-degradation cases:
D/42 and the completed oracle cases retain moving target observations and
sustained valid control. No synchronization failure is needed to produce the
large internal/physical pointing gap.

`SimulationNode._on_clock` initializes only after causal base/TF context exists,
then asks `TimeStepper` to advance to the live clock. Each substep refreshes causal
context. On an exception no candidate is committed, so `_state.time` remains at
its old cursor. Histories are bounded; once the required past context has been
evicted (or was never received), later fresh samples cannot repair that old
cursor. `tracking_loss_context_test.py` reproduces this absorbing condition and
verifies future/old-epoch samples are still rejected. Reset clears both histories,
target state and callback epoch; that correct protection is preserved.

The first missing historical TF/base sample's actual arrival sequence was not
recorded, so its initiating loss/delay is **inconclusive**. The stuck-cursor
mechanism is established by source and deterministic test, not guessed from
localization error. No future-state read, stale-validity extension, or time reset
was introduced as a workaround.

## Measured stages and causal interventions

Historical D/42, aligned at control source epochs (absolute distance MAE; pointing MAE in degrees). Raw wheel odometry and final-estimate localization are evaluated separately at their own stamps.

| Stage | Physical distance MAE (m) | Physical / internal pointing MAE (deg) | Final-estimate XY MAE / max (m) | Final-estimate absolute yaw MAE / max (deg) | Compliance / coverage (%) |
|---|---:|---:|---:|---:|---:|
| 0-10 | 0.127 | 14.78 / 16.20 | 0.029 / 0.113 | 4.42 / 15.10 | 76.10 / 72.90 |
| 10-20 | 0.210 | 23.24 / 5.22 | 0.370 / 0.674 | 29.90 / 44.65 | 3.74 / 92.40 |
| 20-40 | 0.477 | 74.63 / 5.67 | 1.327 / 2.429 | 28.98 / 52.88 | 0.00 / 99.73 |
| 40-90 | 0.873 | 90.67 / 7.17 | 1.804 / 2.612 | 49.42 / 179.89 | 0.00 / 98.80 |

Raw wheel odometry versus final estimate (MAE / peak):

| Stage | Raw XY (m) | Final XY (m) | Raw absolute yaw (deg) | Final absolute yaw (deg) |
|---|---:|---:|---:|---:|
| 0-10 | 0.026 / 0.113 | 0.029 / 0.113 | 3.817 / 15.059 | 4.416 / 15.096 |
| 10-20 | 0.369 / 0.674 | 0.370 / 0.674 | 29.848 / 44.624 | 29.900 / 44.650 |
| 20-40 | 1.327 / 2.428 | 1.327 / 2.429 | 28.962 / 52.876 | 28.982 / 52.880 |
| 40-90 | 1.802 / 2.612 | 1.804 / 2.612 | 49.474 / 179.940 | 49.419 / 179.887 |

At first divergence, signed body-frame final-estimate error is −0.206 m longitudinal and −0.327 m lateral. Source-level CSVs retain these signs and matched truth ages. Each source is evaluated at its own stamps, so raw/final averages need not have identical sample weights.

First prespecified pose divergence (XY>0.50 m or |yaw|>30°) occurs at 14.48 s for both raw odometry and final estimate. At that epoch final-estimate XY error is 0.387 m and yaw error −30.04°. Historical D/917 independently shows the same model/physical gap: mean pointing 74.45° physically versus 7.05° internally, with 40–90 s XY MAE 1.824 m and absolute yaw MAE 49.64°. This is confirmation from existing evidence, not a second live oracle replicate.

Four completed 90 s policies share seed 42, spawn/initial target conditions, exogenous settings, Scenario D controller task/weights, joint feedback, physical safety and the repaired motor backend. Reactive target trajectories differ. See `tracking_loss_intervention_design.md` for the scoped ROS remaps and their explicit assumptions. The **robot-only** intervention changes only the localization input; it retains the ordinary IMM/adaptation/0.25 s lookahead. The target-only and combined interventions replace the target pipeline with current/past physical truth and causal velocity, and use zero lookahead. They do not isolate IMM from lookahead.

| Policy | Distance MAE / max (m) | Pointing MAE / max (deg) | Joint compliance / coverage (%) | Internal pointing MAE (deg) | Model-camera position MAE (m) |
|---|---:|---:|---:|---:|---:|
| baseline | 0.766 / 2.023 | 81.09 / 141.48 | 6.29 / 97.22 | 7.17 | 1.893 |
| oracle_robot | 0.397 / 1.000 | 28.53 / 96.73 | 34.06 / 96.77 | 31.77 | 0.035 |
| oracle_target | 0.569 / 1.966 | 64.67 / 139.00 | 25.49 / 96.60 | 3.60 | 1.024 |
| oracle_both | 0.284 / 0.920 | 22.56 / 96.64 | 55.23 / 95.94 | 23.95 | 0.032 |

![Diagnostic intervention comparisons](../../results/tracking_loss_final_comparison/interventions.png)

The robot-only intervention reduces the large physical/internal mismatch without changing the target predictor, controller or physical robot. The target-only intervention leaves a large mismatch. That is causal evidence that the localization input is a major contributor. It is not a guarantee that better localization alone solves the task: robot-only and combined policies retain late errors, and their internal errors now reveal much more of the actual physical error. One completed live replicate per intervention is not a general statistical reliability claim; no confidence intervals are implied.

### Saturation and rate feasibility with accurate states

In the combined case, 10–20 s physical distance MAE is 0.020 m and pointing MAE 1.64° with 100% observed-time compliance. In 20–40 s these are 0.033 m, 5.39°, 90.95%. During 40–90 s the camera-model position error remains only 0.036 m and optical-direction difference 0.97°, yet physical errors rise to 0.469 m and 35.60°, with 26.76% compliance. Mean active bounds rise to 3.57 and task-rate residual to 1.38. Thus localization cannot explain all extended-orbit loss.

`tracking_loss_feasibility.py` reconstructs the actually consumed estimated pose, selected target/velocity, Jacobian, dt and shared feasible bounds at six combined-case epochs. It computes the best **unregularized** least-squares rate residual under the same bounds. This isolates rate feasibility from WLS motion/posture costs; it does not prove global pose unreachability.

| Source time (s) | Selected WLS rate residual | Best residual, same bounds | Interpretation |
|---|---:|---:|---|
| 35.000 | 0.004949 | 0.000000 | Rate feasible; small regularization tradeoff |
| 45.000 | 0.004972 | 0.000000 | Rate feasible; small regularization tradeoff |
| 55.000 | 1.879623 | 1.879567 | Requested instantaneous rate infeasible |
| 65.000 | 0.006710 | 0.000000 | Rate feasible; small regularization tradeoff |
| 75.002 | 2.192166 | 2.189423 | Requested instantaneous rate infeasible |
| 85.000 | 2.445639 | 2.443111 | Requested instantaneous rate infeasible |

Pose reconstruction errors are below 5e-16 and selected-target reconstruction errors below 1.2e-14 m; recomputed selected residuals match recorded residuals. The initial feasibility attempt omitted the live zero-lookahead override and failed this reconstruction check; it is retained and is **not** the evidence above. The verified pass uses the actual launch override.

At 55 s arm yaw is approximately +3.14 rad, shoulder approximately zero and wrist at its +15° bound. Base v=+0.5 m/s and base yaw=+1 rad/s are saturated, with five active optimizer bounds. Similar constraints occur at 75 and 85 s. A nonsingular task Jacobian does not imply feasible motion under those bounds. The minimum recorded scaled task singular value is 0.004602; large residual here is explained by bounded directional authority, not demonstrated rank loss. The physical four-wheel chassis also turns differently from the ideal base Jacobian. No global planner/local-minimum verdict follows from these local solves.

### Wheel geometry, realization and repeated arm motion

The URDF wheel axes/placement, Gazebo drive radius and track width are consistent: wheel radius 0.15 m and separation 2×0.34=0.68 m, with fore/aft wheel offset ±0.35 m. `tracking_loss_wheels.py` compares actual named wheel encoder velocities, ideal differential-drive odometry and physical body velocities reconstructed from adjacent world poses. Across historical D/42, encoder-derived yaw versus wheel-odometry yaw-rate MAE is 0.0199 rad/s, versus physical chassis yaw-rate MAE 0.1172 rad/s. Encoder-derived forward speed versus physical forward-speed MAE is 0.0693 m/s. Physical lateral-speed MAE is 0.0108 m/s. Physical published yaw twist agrees with finite-difference yaw to 0.00441 rad/s MAE. With fixed wheel axes at fore/aft x=±0.35 m, exact lateral no-slip constraints would require both v_y+0.35*omega=0 and v_y−0.35*omega=0; nonzero chassis yaw therefore requires lateral slip/contact compliance. This supports violation of ideal rolling assumptions/contact slip; it does not measure individual contact forces or identify a friction coefficient error. Finite differences and causal encoder timing contribute transient mismatch.

The commanded generalized velocities are not instantaneous physical velocity measurements. The adapter integrates bounded position references; the native governor and effort-limited constraints realize them with lag. The diagnostic CSV includes measured dq and a +0.1 s command/response discrepancy; this is not an assertion that an arbitrary 0.1 s delay models every joint. Native full-step traces and recovery events show motion/resumption rather than sustained legal inward-demand starvation.

Arm oscillation is defined descriptively as velocity sign reversals outside a 0.05 rad/s deadband, connecting samples only across ≤0.15 s gaps, with no smoothing. It is not proof of linear instability. Shoulder reversals are 116 in baseline, 48 with robot oracle, 60 with target oracle and 12 with both, over the nominal 0–90 s window. Changed reactive trajectories and missing sub-deadband crossings prevent interpreting these as matched forcing experiments. The large reduction supports state/reference inconsistency as a contributor; remaining motion under saturation and target circulation is not established to be an independent PID instability.

### Safety, control coverage and timing

| Policy | Valid / recorded control | Longest invalid interval (s) | Sampled recovery events | All release to active | RTF | Compute mean / max (ms) |
|---|---:|---:|---:|---|---:|---:|
| baseline | 1793/1800 | 0.366 | 4 | True | 0.792 | 24.18/108.95 |
| oracle_robot | 1796/1796 | 0.024 | 24 | True | 0.770 | 25.79/94.53 |
| oracle_target | 1785/1792 | 0.258 | 5 | True | 0.645 | 34.49/227.34 |
| oracle_both | 1785/1800 | 0.782 | 20 | True | 0.741 | 27.83/117.95 |

All four completed policies pass complete native physical position/speed acceptance. Maximum raw position excess is 0.000131659709 rad; maximum raw speed excess is 5.00957942e-10 rad/s. Existing 0.001 rad / 0.001 rad/s acceptance is unchanged. There are no sustained inward-demand stationary windows ≥1 s and every sampled recovery releases. Serious >0.001 rad physical fault injection remains unexercised; motor torque remains unavailable, with the inherited effort bound reviewed previously.

In-window command-rejection / timeout / feedback-rejection counter deltas are respectively baseline 0/0/0, robot oracle 0/0/1, target oracle 3/1/6 and combined 1/0/2. Do not mistake startup cumulative counters for in-window event counts. Source-future/stale input rejection and transient holds are retained and fail closed, not disabled. Fresh baseline has a startup TF lookup rejection lasting 0.366 s; combined input wait reaches 0.782 s. No sustained valid-control gap ≥1 s is accepted. No `Physical context unavailable` error occurs in these four completed runs.

RTF is measured, not a matched performance benchmark: read-only offline analyses shared CPU during parts of the trials. No simulation-fidelity, timestep, algorithm, limit or safety change was made to improve timing. Full data, source-time coverage, native safety, component provenance and cleanup are retained in `results/tracking_loss_final_comparison/summary.json` and each source run.

## Hypothesis matrix

| Hypothesis | Evidence for / controlled test | Counter-evidence / limit | Verdict |
|---|---|---|---|
| Robot localization input causes the large physical/internal split | Aligned odom/final-estimate drift; same-stamp body-frame/yaw series; robot-only oracle reduces physical error and collapses model-pose discrepancy with ordinary IMM/WLS retained. | Correct state does not eliminate late errors. | SUPPORTED as a major cause |
| Wrong optical axis or fixed spawn-frame sign | Authoritative +X and spawn transform checked; error-vector reconstruction agrees to numerical precision. | Both oracle inputs restore model/physical agreement without changing geometry. | REJECTED for the measured split |
| Target prediction is the sole/main source of the model-pose discrepancy | Forecast error and 0.25 s lookahead exist; target-only intervention improves some metrics. | Target-only still has ~65° physical pointing; robot-only retains ordinary prediction and greatly improves. | REJECTED as sole explanation; contribution SUPPORTED |
| IMM/adaptation is beneficial in this experiment | Final-output errors measured; code is active. | Replay output identity fails because accepted inputs are incompletely recorded. No pure IMM/adaptation closed-loop ablation. | INCONCLUSIVE |
| Missing pointing objective / misleading position-only error | A small desired/actual position gap can coexist with >120° physical pointing. | Actual selected task explicitly includes two pointing components and correct signs. | REJECTED as missing objective; misleading interpretation CONFIRMED |
| Saturation / insufficient instantaneous task-rate authority | Accurate-state combined run has joint/base bounds active; exact same-state BLS lower bounds at 55/75/85 s nearly equal WLS residual. | This is local rate feasibility, not global pose impossibility or proof a planner cannot recover. | SUPPORTED secondary cause |
| Singular Jacobian or unhandled antipode is the principal cause | Large residual alone is not supporting evidence. | Nonzero recorded singular values, implemented escape charts, small internal pointing during original split. | REJECTED for principal measured mechanism; global local-minimum behavior INCONCLUSIVE |
| Repaired actuator re-enters permanent joint-limit deadlock | Relevant boundary/recovery/native trace inspected. | Full-step safety, bounded motion and all observed release events; no sustained inward-demand stationary window. | REJECTED in tested episodes |
| Target-context synchronization causes ordinary gradual tracking loss | Independent historic source failure demonstrably stops target updates. | All four current policies have zero context errors; historical D/42 degrades with moving target and valid control. | REJECTED as necessary cause; separate reliability defect SUPPORTED |

## Proposed repairs, separated from this diagnosis

No production repair or retuning was implemented.

1. **Estimation/localization input observability first.** Owners: simulation sensor publication/configuration, `robot_estimation/robot/sensors`, `robot/models/measurement_model.py`, estimator factory and timestamp-aware node. Wheel `[v,omega]` alone cannot observe global XY/yaw drift under systematic rolling-model error. Add/configure an independent, realistically noisy pose/heading source through the existing measurement/reorder/frame contracts; an actual IMU yaw-rate source can reduce wheel-turn error but does not by itself make absolute XY/yaw observable. Keep simulation truth strictly diagnostic and never silently feed it to production. Expected effect: reduce the demonstrated camera-model bias. Risks: source/frame covariance errors, correlated measurements, reset ambiguity and overconfidence. Required tests: fixed-transform/sign/spawn cases; wrapped-yaw updates, asynchronous/reordered/delayed sources and reset; no-sensor fallback must honestly remain uncertain. Physical acceptance: repeat D42/D917 with unchanged control/target/physics, prespecified pose-error and joint tracking-compliance gates, full-step safety and no unauthorized ground-truth subscriptions. A covariance retune alone is not this repair.

2. **Qualify task-rate feasibility before tuning WLS.** Owners: `robot_control/controllers/weighted_least_squares.py` diagnostics and experiment analysis. Publish/inspect feasible task-rate residual/slack independently of solver/input validity. The demonstrated nonzero lower bounds show that weights cannot make every instantaneous requested rate attainable under current constraints. Expected effect: expose genuine task infeasibility and distinguish it from regularization or state error. Risks: expensive online diagnostics or mistaking a local infeasibility for global impossibility. Keep lower-bound solves offline initially. Tests: analytical constrained examples and the six exact reconstructed states above; unchanged command output for identical inputs. Only after localization is qualified, design a separately authorized recentering/reference/planning experiment if sustained orbit remains infeasible. Do not increase limits or quietly redefine the target challenge.

3. **Make prediction state correction kinematically consistent.** Owners: `robot_prediction/fusion/weighted_correction.py` and residual correction interface. A constant state bias applied at every horizon can change velocity without corresponding position evolution. Specify the correction reference epoch and propagate delta_p(h)=delta_p0+h*delta_v (or derive velocity consistently from the position correction). Expected effect: coherent position/velocity feedforward; measured scale here is small, so no large tracking gain is promised. Risks: changed calibration, overcorrection at long horizons and doubled bias adaptation. The new `test_proposed_motion_consistent_forecast_correction` is an **expected failing regression** against current production behavior; do not count it as a functional pass. Future repair must turn it into a normal passing test and retain strict timestamp/covariance/frame contracts; compare raw IMM/final output with completely recorded accepted inputs and current/future truth evaluation. No fusion change is applied here.

4. **Causal target integration frontier and history ownership.** Owners: `robot_simulation/nodes/simulation_node.py`, `integration/time_stepper.py`, `state/physical_context.py`, `gazebo/tracking_frame_provider.py`. Advance only to a frontier for which the required causal source context is available; preserve committed substep progress and required history across asynchronous visual-pose requests. Bootstrap at a common valid epoch. Missing irrecoverable past context must produce an explicit fault/controlled restart policy, not a hidden future read or stale extension. Risks: target lag, partial-progress/pose-service transaction mismatch and reset races. Acceptance: the deterministic missing-frontier/eviction reproduction, delayed/omitted context, pose-service delay, startup and world-reset tests; no relabeled timestamps, no target teleport workaround, no future samples, and repeated normal launch without the historic stuck cursor. The initiating lost-sample sequence remains to be captured before selecting the exact patch.

## Failures, validation and remaining independent risks

The first robot-oracle relay published once then exited because its diagnostic JSON logger received a ROS array. Ordinary input validation stopped actuation. That attempt was stopped through its owned launch at clock 62.216 s, retained with error and cleanup, and excluded from every performance comparison. Its original relay source and stop record are in `results/tracking_loss_diagnostic_relay_failure`; the corrected repeat is separately retained. No unsuccessful experiment was silently substituted for a pass. Baseline, target-only and combined source runs remain in the original study; robot-only success is in its new retry study.

Prototype offline failures are also retained: wheel/replay JSON NumPy-scalar serialization logs, CSV plotting parser failures, and the first feasibility reconstruction that omitted zero lookahead. Final evidence uses only the explicitly verified/final artifact roots. The predictor replay completed after serializer correction but still failed its output-identity criterion; its stage attribution is intentionally inconclusive.

Fresh functional regression: **1,279 passed, 18 deselected** (`tracking_loss_functional_regression.log`). No new production test failures. The diagnostic frame/time/axis/coverage suite has four passes and one expected failure for the proposed fusion repair; the context suite has two passes. Existing style/lint debt was not repaired or counted as passing. No new package or production installation change requires a rebuild. Native binary/source pin and all 1,633 pre-existing file hashes are verified in the preservation record.

Independent open issues remain: target-context synchronization; poor extended-orbit tracking despite accurate inputs; unperformed significant physical fault injection; unmeasured torque; Gazebo Physics 7.6.0 / DART 6.13.2 backend pinning and upgrade-maintenance risk. This diagnosis neither reopens nor strengthens the previous conditional physical-safety certification.

## Final causal verdict

**PARTIALLY CONFIRMED.** Localization drift is causally established as a major source of the misleading internal/physical gap and ineffective motion. An additional bounded task-rate limitation is established at exact accurate-state episodes. Current-target substitution helps but does not cure the task. The complete global cause of every extended-orbit oscillation/recovery trajectory is not isolated; pure prediction-stage benefit, global recoverability and the initiating target-context arrival failure remain unresolved.

- Localization confidence: **high** for the measured drift and causal role; proposed production sensor correction still requires implementation/validation.
- Prediction confidence: **moderate** for recorded final-horizon errors and interface semantics; **low** for IMM/adaptation benefit or its independent closed-loop contribution.
- Controller/task formulation confidence: **high** for distance+pointing semantics, reconstruction and demonstrated bounded rate infeasibility; **incomplete** for global planning/recovery optimality.
- Physical actuation confidence: **high within tested motion/recovery/position-speed conditions**; previous fault-injection/torque/hardware limitations remain.
- Synchronization reliability: **conditional**; no current completed-run context failure, but historic failures and absorbing-cursor reproduction remain.

## Reproduction and inventory

Normal production launch remains:

```bash
source /home/user/ros2_ws/install/setup.bash
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_manipulation_D.yaml \
  enable_actuation:=true publish_ground_truth:=true gui:=false
```

Diagnostic tools are not installed and cannot be selected accidentally by normal bringup:

```bash
source /home/user/ros2_ws/install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
python3 docs/validation/tracking_loss_test.py
python3 docs/validation/tracking_loss_context_test.py
python3 docs/validation/tracking_loss_run.py \
  --config docs/validation/tracking_loss_experiments.yaml \
  --output results/tracking_loss_reproduce
python3 docs/validation/tracking_loss_analysis.py \
  --run results/joint_limit_recovery_final_challenge/challenge_recovery_20261021T182453_142101Z_a7qyagur/D_seed42 \
  --output results/tracking_loss_reproduce_analysis
MPLCONFIGDIR=/tmp/tracking_loss_matplotlib python3 docs/validation/tracking_loss_plot.py \
  --analysis results/tracking_loss_reproduce_analysis/0_D_seed42 \
  --output results/tracking_loss_reproduce_figures
python3 docs/validation/tracking_loss_feasibility.py \
  --run results/tracking_loss_interventions/tracking_loss_20261022T054116_446391Z_ipp96q8v/oracle_both \
  --output results/tracking_loss_reproduce_rate_feasibility
python3 -m pytest src -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros \
  -k 'not copyright and not flake8 and not pep257'
```

Use fresh output paths; scripts refuse to overwrite prior result directories. The run YAML prespecifies variant D, seed 42, 90 s, oracle topics, causal maximum age and compliance thresholds. Each run retains materialized owner inputs, sensor RNG seed, selected classes, actual launch command, binary identity, source-time traces and cleanup. `tracking_loss_report.py --case NAME RUN ANALYSIS` accepts repeated triples and rejects incomplete/unsafe/invalid-control cases before assembling comparisons.

Exact task-only additions, generated logs/result roots and preservation checks are in `tracking_loss_modified_files.txt` and `tracking_loss_preservation.json`. No pre-existing source/configuration/result file is modified. No staging, commits or pushes.


Final process audit (`tracking_loss_final_processes.json`): no running ROS/Gazebo nodes; all five owned attempts, including the failed diagnostic relay attempt, have cleanup records with zero survivors.
