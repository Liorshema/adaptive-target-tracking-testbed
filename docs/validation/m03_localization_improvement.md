# Reliable robot localization under wheel slip

Task: attachment `aa63bca2-0b8f-48a8-8f01-2d5ac1a47d64`; ROS 2 Jazzy / Gazebo Sim 8 / WSL2. Task baseline: `m03_localization_baseline.json` (1,657 existing files). The original Scenario D remains selectable and unchanged. The additive `system_manipulation_D_localized.yaml` selects the new source and estimator settings.

**Verdicts: LOCALIZATION CONDITIONAL; TRACKING CONDITIONAL; SAFETY CONDITIONAL; ARCHITECTURE PASS.**

## Root cause, alternatives and selected architecture

Established fact: `tracking_loss_root_cause.md` and `joint_limit_recovery_root_cause.md` distinguish physical tracking from the controller's internal task metric. The deployed EKF integrates wheel-derived forward/yaw velocity using an ideal planar differential-drive model. Four fixed wheels require lateral contact slip during turns. Correct encoder geometry does not make wheel-derived chassis velocity correct. The deployed wheel velocity R was `diag(1e-6,1e-6)`, representing encoder precision rather than chassis-motion uncertainty. The optional IMU adapter has no selected simulated publisher.

Wheel-only calibration cannot make global XY/yaw observable. A yaw-rate IMU would constrain turn rate but still integrates bias and does not observe absolute XY or heading. An independently referenced planar pose supplies the missing observations. The alternatives, observability argument, mathematical contract and acceptance targets were documented **before implementation** in `m03_localization_design.md`.

Selected solution: a simulation abstraction of mapped-fiducial/external-tracker localization. This is **not implemented VIO/SLAM**. Only `robot_simulation.nodes.localization_sensor` consumes the physical base source. It adds random calibration bias, white noise, latency, jitter and dropout and publishes ordinary stamped pose-only `nav_msgs/Odometry`. The existing robot EKF fuses this through its ordinary timestamp queue. No second estimation pipeline, oracle remap or direct estimator/controller truth subscription is introduced. The external coverage and performance assumptions need replacement/qualification for real hardware.

The normal controller still consumes `/estimation/robot_state_full`; `/localization/odometry` has a single publisher, `localization_sensor`, and is consumed by the estimator. Runtime `graph.json` records establish these links and show that the estimator and controller do not subscribe to `/simulation/robot/ground_truth`. Existing simulation consumers of physical context, including reactive-target geometry and feedback TF, remain unchanged. Different robot policies produce different reactive target trajectories; matched seeds do not imply identical trajectories.

## State, measurement and timing contract

State is unchanged: `x=[x_B,y_B,yaw_B,v_B,omega_B,q_1..q_4,dq_1..dq_4]` in odom, SI units. The mean transition retains exact constant-twist planar integration and constant joint velocity. Existing acceleration-noise discretization and Joseph-form EKF update are retained.

| Input | Measurement / H | Frame and sampling | Covariance / policy |
|---|---|---|---|
| Wheel odometry | `[v,omega]`, selects state components 3,4 | odom/base contract; configured 50 Hz | Localized deployment `diag(.02,.04)` in `(m/s)^2,(rad/s)^2`; prior independent body/wheel RMSE ~.129 m/s and .189 rad/s motivates conservative rounded squares. Original deployment unchanged. |
| Joint feedback | `[q,dq]`, selects joint components | authoritative joint names/order; Gazebo physics-step publication | Existing calibrated variances, boundary conditioning and validation unchanged. |
| Optional IMU | yaw rate, selects omega | existing base-frame contract; absent in selected deployment | Existing R and adapter unchanged; no invented absolute-heading observability. |
| New external pose | `[x,y,yaw]`, H selects first three state components | source `world`, child `base_link`; nominal 10 Hz | White sigma `[.04 m,.04 m,2 deg]`, per-epoch constant random calibration-bias sigma `[.01 m,.01 m,.5 deg]`; R is the sum of marginal variances. Non-planar/unusable transforms and non-SPD R rejected. |

For a fixed planar frame transform `T_A_B`, transform XY by its rotation/translation and yaw by its heading, with full covariance `R_A=J R_B J^T`, `J=diag(R_xy,1)`. TF is looked up at acquisition time; world is not assumed equal to odom. The selected spawn has world-from-odom translation `[0,0,.25]`; pure tests also exercise nonzero XY and rotated spawn. Child frames must identify the base; hardware sources for other links require an upstream extrinsic conversion.

Pose innovation wraps heading through `atan2(sin(delta),cos(delta))`. NIS is `innovation^T(HPH^T+R)^-1 innovation`; the configured 3-DOF gate is 11.345 (~99%). Rejected observations do not mutate the filter. Accepted headings are represented in the chart around predicted yaw and passed to the existing Joseph update; state layout and continuous yaw convention remain intact.

Sensor latency is 40–50 ms, 5% independent dropout, configurable complete outages and outliers, and a 150 ms maximum delivery/arrival budget. Source acquisition stamps are preserved, never changed to release/receipt time. The localized reorder holdback is 80 ms; the common queue retains established duplicate, future, stale, committed-time and reset rejection. Existing 200 ms component freshness/safety budgets are unchanged. A pose older than committed state is rejected; there is no retrospective rewind. Disabled/unavailable aiding leaves ordinary wheel/joint propagation active, without imaginary pose corrections. Diagnostics expose transport counts, NIS acceptance/rejection and last accepted source epoch. Their `available` flag uses the source-age budget and may briefly be false between valid 10 Hz observations because holdback and dropout consume that budget; it is not a control enable switch.

The localized profile also changes base acceleration-noise amplitudes to .3 and adds independent isotropic XY contact-slip diffusion .15 m/sqrt(s) and yaw diffusion .20 rad/sqrt(s). With frozen `a=v[-sin(yaw),cos(yaw)]`, additional Q blocks are `Q_xy=.15^2 dt I + .20^2 dt^3/3 aa^T`, `Q_xy,yaw=.20^2 dt^2/2 a`, `Q_yaw,yaw=.20^2 dt`. They model unresolved contact/slip displacement and encoder/body heading discrepancy separately from encoder noise. Defaults are zero and preserve baseline mathematics. Joint Q is unchanged. This is a conservative stochastic approximation over a one-second slip correlation horizon, not an identified contact dynamics model. The disabled-source ablation retains **all** localized R/Q/holdback values.

Important smoke finding: acceleration-only process noise plus wheel-R correction rejected 43/82 valid pose observations by 10.4 s, last acceptance 6.15 s, with ~.40 rad heading error against ~.05 rad reported sigma. That complete failed engineering smoke is retained. The optional contact-slip Q extension was designed and tested before final comparison trials; the gate and acceptance thresholds were not relaxed. The second smoke accepted all 127 observations and passed native position/speed gates.

Repeated calibration bias is temporally correlated. Adding its marginal variance to each observation is an approximation, not a proof of exact consistency. The report therefore evaluates source-time aligned errors, base NEES, marginal coverage and covariance eigenvalues, without claiming that positive-definite covariance alone is calibrated. Existing joint-covariance limitations are not addressed here.

## Configuration ownership and code scope

- `robot_estimation`: optional planar measurement model/transform, wrapped innovation/NIS gate, TF ROS adapter and shared queue integration; optional slip diffusion and parameters. No Gazebo dependency in estimator mathematics.
- `robot_simulation`: pure imperfect sensor model, thin ROS publisher and explicit optional sensor deployment. Sensor noise, bias, frequency, latency, dropout, outages, outliers, seed, topic/frame names and diagnostic rate live in `config/sensors/planar_localization.yaml`.
- `robot_bringup`: inherited localized simulation/calibration/system profiles; preflight validates sensor/source/frame/topic compatibility and rejects direct truth as an estimation input.
- `robot_experiments`: optional sensor materialization with an independent seed derived from the experiment seed, captured owner/configuration hashes, and measurement recording. Target RNG semantics are unchanged.

New estimation parameters: `localization_enabled` (false by default), `localization_topic`, `localization_frame`, `localization_max_age`, `localization_nis_limit`, `localization_transform_tolerance`, `planar_slip_noise_std`, `yaw_slip_noise_std`. Existing parameters overridden only in the localized profiles: reorder holdback, base acceleration amplitudes and deployed wheel velocity variances. Physical description, control/task weights/limits, IMM/adaptation, target physics and native actuator/backend are protected by baseline hashes.

## Validation evidence and results

Prespecified localization gates are XY MAE <=.20 m and yaw MAE <=10 deg over 20–90 source seconds. Tracking gates require improved physical distance/pointing MAE and compliance at unchanged `|e_R|<=.20 m`, `theta<=15 deg`; valid-control fraction >=98% and no sustained invalid interval >=1 s. Physical acceptance remains .001 rad / .001 rad/s, with unchanged effort/position/velocity configuration. Unmet gates remain conditional; no thresholds are retuned after measurement.

Physical camera pose is reconstructed from measured base quaternion and measured authoritative arm joints. Tracking follows the same causal <=.08 s alignment and fixed compliance definition as the previous diagnosis. Localization consistency instead interpolates offline physical XY and unwrapped heading at each estimate's **own source epoch**, accepting only <=.15 s truth brackets. Neither analysis path is a runtime input. Recorder arrival clocks can lag individual callback streams under load and must not be interpreted as sensor latency; estimator receive-age diagnostics are the authoritative runtime timing evidence.

The first original-seed attempts exposed a disabled-source startup regression: reading typed but uninitialized optional ROS parameters. It was fixed and covered by a red/green test. Those unsuccessful attempts remain in the initial study and are excluded from completed comparisons; retries use fresh result directories. No unsuccessful artifact is overwritten or represented as a valid 90-second run.

## Final measured comparison

Localization is source-aligned over 20–90 s; physical tracking and compliance use 0–90 s. Each completed launch ran 90 simulation seconds after readiness, covering the requested source interval. Compliance is time-weighted among observed eligible intervals; its observation coverage is shown explicitly. Control percentages are sampled validity, with no imputation of missing telemetry.

| Seed / policy | XY MAE m | Yaw MAE deg | Physical distance MAE / max m | Physical pointing MAE / max deg | Compliance % (coverage %) | Valid control % | RTF |
|---|---:|---:|---:|---:|---:|---:|---:|
| 42 / original | 1.9632 | 13.401 | 0.4787 / 1.3467 | 71.49 / 117.42 | 6.68 (96.49) | 98.87 | 0.796 |
| 42 / localized | 0.0395 | 3.212 | 0.4229 / 1.0020 | 30.40 / 96.65 | 27.77 (86.14) | 96.76 | 0.640 |
| 42 / disabled | 1.6493 | 91.678 | 0.5887 / 1.9921 | 66.79 / 144.23 | 19.78 (95.99) | 99.66 | 0.789 |
| 917 / original | 2.0142 | 15.842 | 0.5105 / 1.4466 | 73.28 / 122.78 | 5.01 (94.38) | 99.89 | 0.805 |
| 917 / localized | 0.0405 | 2.938 | 0.3979 / 1.0466 | 28.10 / 96.62 | 32.97 (94.28) | 99.54 | 0.761 |
| 917 / disabled | 1.4382 | 58.601 | 0.4483 / 1.7314 | 64.93 / 138.73 | 7.48 (96.05) | 99.94 | 0.727 |

The original and sensor-disabled policies remain unobservable globally. The ablation retains the same slip Q, wheel R and holdback as the localized policy: any remaining drift is exposed rather than corrected through truth. Controller, target, predictor, adaptation and physical contracts are identical across policies. Reactive trajectories differ.

| Localized seed | XY max m / yaw max deg | Base NEES mean | 95% ellipsoid coverage % | Marginal 95% coverage X/Y/yaw % | Camera position disagreement m / optical disagreement deg |
|---|---:|---:|---:|---:|---:|
| 42 | 0.1494 / 17.45 | 2.289 | 99.05 | 99.5/99.7/93.1 | 0.0723 / 2.57 |
| 917 | 0.1247 / 32.02 | 2.074 | 98.50 | 99.6/99.1/95.2 | 0.0726 / 2.44 |

NEES/coverage are descriptive checks on temporally correlated samples, not independent-sample significance tests. XY is conservative; residual yaw undercoverage and the stochastic slip/bias approximation remain limitations. Base covariance stays positive and finite; it is not artificially collapsed. Detailed maxima, standard deviations, pose increments and camera disagreement for all policies are in each analysis `summary.json`.

| Policy / seed | Controller compute mean / max ms | Longest invalid interval 0–90 s / 1.5–90 s | Observed recovery episodes | Unreleased episodes | Longest inward-demand stall s |
|---|---:|---:|---:|---:|---:|
| disabled_D42 | 26.47 / 126.35 | 2.300 / 0.800 | 3 | 0 | 0.000 |
| disabled_D917 | 27.76 / 98.39 | 1.952 / 0.452 | 1 | 0 | 0.000 |
| localized_D42 | 34.19 / 132.00 | 3.150 / 1.650 | 21 | 0 | 0.000 |
| localized_D917 | 27.27 / 74.11 | 2.950 / 1.450 | 22 | 0 | 0.000 |
| original_D42 | 24.34 / 120.61 | 2.250 / 0.750 | 3 | 0 | 0.000 |
| original_D917 | 24.97 / 108.41 | 2.602 / 1.102 | 1 | 0 | 0.000 |

Control gaps and missed sampled-validity gates are retained. Initial waiting and later stale-input holds remain safe holds; they prevent an unconditional tracking acceptance. The first localized seed overlapped the independent copied build and part of functional verification. Offline analysis/regression were later paused to avoid contention in remaining physical trials. These RTF values are measured observations on WSL2, not an isolated causal benchmark of sensor overhead. Estimator CPU time and hardware latency are not measured; source processing-age counters are not compute-time measurements. Best-effort recorder gaps limit tracking/continuity coverage; native physics traces remain complete.

| Joint | Maximum position excess across six cases rad | Maximum measured speed rad/s | Configured speed limit rad/s |
|---|---:|---:|---:|
| base_yaw_joint | 7.06101844e-14 | 0.716929784 | 1 |
| shoulder_joint | 6.7999184e-05 | 1 | 1 |
| elbow_joint | 1.55431223e-13 | 1 | 1 |
| wrist_joint | 9.32032229e-13 | 0.716748037 | 1.5 |

All six completed cases pass native full-step coverage and unchanged .001 rad / .001 rad/s acceptance. Tiny numerical excursions are reported, not hidden by clamping the measurements. No measured motor-torque certification or serious mechanical-fault coverage is added. The repaired DART backend and all native actuator/servo sources remain unchanged.

### Independent verdicts

- **LOCALIZATION: CONDITIONAL** — both nominal mean-error targets pass, but startup-outlier robustness does not. Independent-source observability is demonstrated; a plausible startup outlier can poison the initial estimate and subsequent gating, and abrupt skid episodes still cause a short heading transient. See the fault-probe timeline and proposed closure below.
- **TRACKING: CONDITIONAL** — both physical mean errors and compliance improve at each seed; full control-validity acceptance is missed. Remaining physical task error, infeasible late-orbit motion and input/telemetry gaps prevent a stronger claim.
- **SAFETY: CONDITIONAL** — all measured native position/speed gates pass and observed recovery release is reported; unchanged torque observability and serious-fault-injection gaps preclude real-hardware safety certification.
- **ARCHITECTURE: PASS** — ordinary estimator/controller interfaces, optional source selection, typed configuration, ownership tests, normal/copied installation and runtime ROS graph verified; protected source scopes remain unchanged.

### Evidence and anti-GIGO review

| Fact / hypothesis | Controlled evidence | Conclusion |
|---|---|---|
| Wheel-only pose drifts; independent pose should constrain XY/yaw. | Original/localized/disabled, two fixed seeds, unchanged physical/controller/target inputs. | Aiding benefit is measured physically; wheel/slip uncertainty alone does not create global observability. |
| Near-perfect truth could make metrics misleading. | Sensor errors, nonzero white noise and random calibration bias, dropout/delay, NIS gate, single-source ROS graph. | The production estimator consumes an explicit imperfect external-pose abstraction, not the diagnostic oracle. |
| Internal task error may conceal physical camera error. | Authoritative physical FK from base truth and measured joints; aligned actual target; camera disagreement table. | Tracking improvement and remaining error are assessed from physical geometry. |
| A passing suite might hide unsafe actuation. | Complete 2 ms native traces, unchanged tolerances, recovery/stall audits and unchanged backend hash. | Numerical position/speed conditions pass; unmeasured torque/fault conditions remain conditional. |

Result sources:

- `results/m03_localization_original_retry/m03_localization_20261022T110049_280599Z_56imtoxs/original_D42`
- `results/m03_localization_original_retry/m03_localization_20261022T110049_280599Z_56imtoxs/original_D917`
- `results/m03_localization_comparison/m03_localization_20261022T104901_917919Z_p12k_0ew/disabled_D42`
- `results/m03_localization_comparison/m03_localization_20261022T104901_917919Z_p12k_0ew/localized_D42`
- `results/m03_localization_comparison/m03_localization_20261022T104901_917919Z_p12k_0ew/localized_D917`
- `results/m03_localization_comparison/m03_localization_20261022T104901_917919Z_p12k_0ew/disabled_D917`

## Completed tests, fault probes and limitations

The final uninterrupted functional regression passed **1,298 tests**, with 18 copyright/flake8/pep257 policy tests explicitly deselected (`m03_localization_final_complete_regression.log`). This includes architecture/ownership, configuration/factory, estimation timing, control and actuation regressions. No lint-clean claim is made. Earlier focused checks passed 540 tests; the post-startup-fix focused group passed 170. Missing-module/contract and disabled-parameter red/green logs are retained. An interrupted regression produced one wall-clock process-test failure when the test process was suspended; a subsequent uninterrupted run passed 1,297, and the final run including the startup-outlier limitation regression passed 1,298. That failed attempt is preserved, not presented as a product regression or silently discarded.

Normal and independent copied builds each completed all ten packages. Copied module origins, installed sensor YAML/entrypoint and inherited owner contracts passed from `/tmp` with workspace overlay variables removed. Existing M07/M08/M09/M02/native installation gates passed. The copied Gazebo smoke completed, accepted 106 pose observations with zero NIS rejections, and passed native physical checks. The normal backend retains SHA256 `59c4a3a5481bf9cad4480b9af1307d09d1444ccb1d22a0af1f6aee99e5d0fc5e`.

Nominal source timing: D42 fused 860 poses, with zero statistical rejections; D917 fused 849, with six statistical rejections. Maximum estimator-observed arrival ages were .130/.112 s and processing ages .130/.136 s. Two/one late transport observations respectively were rejected rather than rewound or relabeled. Both had zero fusion errors. The simulated source is acquired from 20 Hz physical base observations and selects nominal 10 Hz acquisition. D42 recorded white-noise standard deviations were [.0415 m,.0389 m,.0351 rad], with nonzero mean offsets [.0096 m,.0060 m,.0101 rad], consistent with the configured imperfect source. Best-effort recording lost some messages; its recorded rate/clock differences are not asserted as actual sensor delivery timing.

Nominal transient audit (`m03_localization_transient_audit.json`): neither seed exceeded .5 m XY error in 20–90 s. D917 exceeded 30 degrees at seven estimate samples, from source 60.080–60.222 s (span .142 s), with a 32.02-degree maximum. Diagnostics show last accepted pose 59.55 s, several legitimate source rejections with NIS 12–14 against gate 11.345, then reacquisition at source 60.25 s (NIS 10.36). The output heading correction reached 29.59 degrees over .030 s. This is an explained **remaining slip/noise-model and innovation-gating limitation**, not evidence of smooth globally reliable localization. D42's largest heading step was 13.03 degrees over .032 s; maximum XY steps were .138/.139 m. The conditional verdict includes these corrections; acceptance thresholds were not relaxed and output was not cosmetically smoothed to conceal them. Subsequent work should validate slip-aware innovation consistency/reacquisition while retaining honest covariance and bounded actuation.

The explicit degraded experiment adds a source outage at 5–7 s and 5% injected `[1 m,1 m,.5 rad]` outliers. There were **zero measurements during the configured outage** and eventual reacquisition; full native safety passed with no fusion errors. However, it exposed an important startup failure: the first accepted pose at source 2.7 s was an injected outlier. A broad, honest initial covariance makes that observation statistically plausible. After the filter commits to it, many good poses fall outside the NIS gate; accepted 4.8/10.3 s observations were also injected outliers, and the first nominal pose was reaccepted only at 15.1 s. Final counts were 50 accepted / 100 statistically rejected. **This probe does not pass robust initialization.** Its complete evidence is retained in `m03_localization_extra_checks.json` and the degraded result root.

Actionable closure: add explicit robust multi-observation startup/source-health consensus before committing a newly available absolute source, and validate recovery after correlated slip/extended loss. That requires a defined initialization/reacquisition contract, not an artificially tight spawn prior, a larger outlier gate, a truth-based reset or acceptance of stale samples. No additional estimator redesign is included in this task. The ordinary nominal 5% dropout cases demonstrate large, repeatable gains, but they do not certify arbitrary startup corruption, long outages, occlusion, map errors or adversarial faults.

Live world reset passed all four existing checks: clock reset observed, control recovered, both native physics epochs complete/safe, and cleanup. Estimator timing recorded one reset and 24 new-epoch pose updates, with last accepted stamp 2.45 s at diagnostic time 2.81 s. Pending old-epoch sensor/filter data were discarded through the existing reset mechanisms. Pure and node tests additionally cover duplicates, future/stale/out-of-order data, disabled source, deterministic bias/noise, covariance, nontrivial frames, heading wrap and outlier non-mutation.

All **13 owned launch attempts** have zero cleanup survivors (`m03_localization_final_processes.json`), including unsuccessful engineering and baseline-startup attempts. A pre-existing unowned sleeping Gazebo server, PID 544334, was left untouched. The initial process audit ran in a restricted process namespace; final host-level auditing makes this visibility limitation explicit. No claim of a completely empty host process table or rigorously isolated host-performance benchmark is made.

## Reproduction

```bash
source /home/user/ros2_ws/install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_manipulation_D_localized.yaml \
  enable_actuation:=true gui:=false
```

The original baseline remains `system_config:=config/system_manipulation_D.yaml`. The localized manifest enables the simulation physical source explicitly for the **simulation sensor**; it does not wire truth into estimation or control.

```bash
source /home/user/ros2_ws/install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
python3 tools/validation/m03_localization_run.py \
  --output results/m03_localization_reproduce --duration 90 --seeds 42 917
python3 tools/validation/m03_localization_metrics.py \
  --study results/m03_localization_reproduce/REPLACE_WITH_CREATED_STUDY \
  --output results/m03_localization_reproduce_analysis
python3 tools/validation/m03_localization_run.py \
  --output results/m03_localization_reproduce_degraded --duration 18 --seeds 42 --modes degraded
python3 tools/validation/m02_resolution_reset.py \
  --output results/m03_localization_reproduce_reset \
  --system-config config/system_manipulation_D_localized.yaml
ROS_LOG_DIR=/tmp/m03_reproduce_tests python3 -m pytest src -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros \
  -k 'not copyright and not flake8 and not pep257'
```

Run scripts refuse to overwrite existing study directories and own only their isolated ROS domain/Gazebo partition. Each attempt retains immutable materialized inputs, launch log, ROS graph, telemetry, complete native actuator trace and cleanup record.

Copied installation (no workspace overlay; regular file copies, not symlinks):

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  bash --noprofile --norc -c '
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/m03_localization_copy/log build \
  --build-base /tmp/m03_localization_copy/build \
  --install-base /tmp/m03_localization_copy/install --packages-up-to robot_experiments'
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  bash --noprofile --norc -c '
source /opt/ros/jazzy/setup.bash
source /tmp/m03_localization_copy/install/local_setup.bash
cd /tmp
python3 /home/user/ros2_ws/tools/validation/m03_localization_verify_installation.py \
  --prefix /tmp/m03_localization_copy/install --copied \
  --output /tmp/m03_localization_installation.json
python3 /home/user/ros2_ws/tools/validation/m03_localization_run.py \
  --output /home/user/ros2_ws/results/m03_localization_copied_reproduce \
  --duration 10 --seeds 42 --modes localized'
```

Validation reused the already independent `/tmp/joint_limit_recovery_copy` build cache; all ten packages were rebuilt/copied, new module origins/configs/entrypoint checked from `/tmp`, and prior M07/M08/M09/native recovery installation gates rerun.

## Inventory and preservation

Exact task-only existing-file modifications and additions are enumerated in `m03_localization_modified_files.txt`; `m03_localization_preservation.json` compares the original 1,657-file baseline, protected scopes, HEAD, staging and native backend hash. Historical experiment roots and reports remain intact. No commit or push is performed.
