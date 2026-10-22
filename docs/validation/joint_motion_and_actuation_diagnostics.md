# Joint motion and actuation diagnostics — M09 refinement

**Verdict: CONDITIONAL.** The analysis is complete for the available recordings; physical speed acceptance remains failed, as documented in [M02](../m02_physical_actuation_safety.md). All four joints participate. Wrist travel and RMS motion are greatest despite modest net angle changes. Several wrist overspeed events are supported by position increments; bounded high-level commands do not guarantee bounded physical speed through the existing effort PID. No controller, safety, PID, physics, limits or estimation configuration was changed, and no new Gazebo run was required.

## Reproducible evidence and alignment

Derived data: [complete JSON, source hashes and event traces](m09_refinement_joint_20261018.json). From the workspace, source `install/setup.bash` and run `MPLCONFIGDIR=/tmp/m09_joint_matplotlib python3 docs/validation/m09_joint_diagnostics.py --output-prefix results/m09_joint_diagnostics_repeat/diagnostics`. Choose a fresh prefix on subsequent runs; the CLI refuses to overwrite existing evidence. The reusable pure math is in `robot_experiments.metrics.joint_diagnostics`; 11 analytical fixtures cover irregular integration, gaps, limits/tolerance, causal alignment, joint-name reordering, quaternion rotation, reset epochs and invalid samples.

Exact recorded directories, relative to workspace:

- `results/experiments/m08_verified_final/paired_prediction_20261017T035322_605496Z_mykcu7wt/runs/{imm_seed42,cv_seed42}`.
- `results/experiments/m08_final_seed917/paired_seed917_20261018T061032_929053Z_750v4ffu/runs/{imm_seed917,cv_seed917}`.

Each run's `metadata.json` provides its evaluation window. The JSON includes hashes of metadata, resolved configuration and all analyzed streams, timestamp/order/frame audits, every available signal in ±0.2 s windows around three separated evaluation wrist peaks plus a larger capture peak when present. Measurements, target estimates and predictions are `world`; controller telemetry is `odom` but its velocity vector has the configured `[v_body, omega_body, dq...]` semantics. SafeVelocityCommand/ActuationTelemetry use `base_link`. Joint angles/speeds are generalized coordinates in rad/rad/s, with an empty JointState frame. No coordinate-position subtraction across these frames is used.

**Coverage policy:** integrate trapezoids of sampled squared velocity only between adjacent source-time feedback samples separated by ≤0.010 s (five nominal 500 Hz steps). Do not extrapolate at either window edge or fill gaps with zeros. RMS divides that integral by **observed duration**, not the nominal 6 s. JSON also exposes the square root of the observed integral normalized by full T as `full_window_normalized_observed_rms_rad_s`. This is a partial sampled integral normalization, not a complete-window RMS or a rigorous lower bound on true physical RMS. p95 is sample weighted. Travel is the requested sum of absolute adjacent position differences; separate JSON travel excludes long gaps. Neither captures unobserved reversals. Trapezoids approximate the sampled integral; they do not reconstruct physics between samples.

This deliberately differs from M08's metric coverage rule (right holds capped at `max_skew=0.08 s`): the stricter interval-based diagnostics can report less covered time without contradicting the preserved M08 results. Violation duration is a left-held estimate over these eligible intervals, neither an exact duration nor a guaranteed lower bound. The unchanged physical acceptance semantics are `abs(dq) > description_velocity + 0.001 rad/s`, and position excess >0.001 rad. Values near 1.500000002 rad/s therefore pass the wrist speed tolerance.

| Run | Source-time evaluation window (s) | Joint samples | Observed s / full T | Coverage | Largest internal / including-edge gap (s) |
|---|---:|---:|---:|---:|---:|
| cv_seed42 | 4.528–10.528 | 2770 | 5.804/6 | 96.73% | 0.042 / 0.154 |
| imm_seed42 | 4.634–10.634 | 3001 | 6.000/6 | 100.00% | 0.002 / 0.002 |
| cv_seed917 | 4.344–10.344 | 778 | 2.960/6 | 49.33% | 0.536 / 0.536 |
| imm_seed917 | 4.738–10.738 | 1366 | 4.316/6 | 71.93% | 0.056 / 0.056 |

Nominal feedback interval is 0.002 s. CV42's final feedback is 10.374 s, before the 10.528 s evaluation end. Seed917 streams have substantial additional missing feedback. Their sampled peaks are valid observations but RMS, p95 and travel are not complete-window population estimates. All available startup samples through each evaluation end are separately audited in `capture_arm`; safety is not restricted to valid controller periods.

## Joint participation and physical safety

All joints are resolved by **name in every JointState sample**. Arm names/configured limits and command array order come from the preserved resolved configuration and its name template, not hard-coded array offsets. SafeVelocityCommand is reordered using its own joint names. The four wheels are independently resolved from configured left/right joint lists: `front_left_wheel_joint`, `rear_left_wheel_joint`, `front_right_wheel_joint`, `rear_right_wheel_joint`. Their range/travel/RMS/peak/p95/coverage are in JSON; no unsupported wheel velocity limit is invented.

| Run | Joint | Range (rad) | Sampled travel (rad) | Observed RMS (rad/s) | Peak abs (rad/s) | p95 abs (rad/s) | Speed violation estimate (s) |
|---|---|---:|---:|---:|---:|---:|---:|
| cv_seed42 | base_yaw | 0.2020 | 0.4594 | 0.0909 | 0.2148 | 0.1512 | 0.000 |
| cv_seed42 | shoulder | 0.1718 | 0.4971 | 0.1133 | 0.7422 | 0.2086 | 0.000 |
| cv_seed42 | elbow | 0.1006 | 0.4507 | 0.1134 | 1.1312 | 0.2072 | 0.008 |
| cv_seed42 | wrist | 0.3637 | 0.9594 | 0.3483 | 1.5035 | 0.8330 | 0.002 |
| imm_seed42 | base_yaw | 0.1911 | 0.4676 | 0.0916 | 0.2457 | 0.1659 | 0.000 |
| imm_seed42 | shoulder | 0.1474 | 0.5919 | 0.1259 | 0.6394 | 0.2514 | 0.000 |
| imm_seed42 | elbow | 0.0840 | 0.5554 | 0.1296 | 1.0000 | 0.2545 | 0.000 |
| imm_seed42 | wrist | 0.3584 | 1.0314 | 0.3524 | 1.5508 | 0.8916 | 0.004 |
| cv_seed917 | base_yaw | 0.2118 | 0.5082 | 0.1089 | 0.2971 | 0.2103 | 0.000 |
| cv_seed917 | shoulder | 0.3746 | 0.7161 | 0.2217 | 1.0886 | 0.6382 | 0.002 |
| cv_seed917 | elbow | 0.1481 | 0.4251 | 0.1506 | 1.2687 | 0.2686 | 0.002 |
| cv_seed917 | wrist | 0.6622 | 1.0568 | 0.5141 | 3.4761 | 1.5000 | 0.024 |
| imm_seed917 | base_yaw | 0.1997 | 0.4889 | 0.0946 | 0.3028 | 0.1820 | 0.000 |
| imm_seed917 | shoulder | 0.2901 | 0.7101 | 0.1520 | 1.0000 | 0.2704 | 0.000 |
| imm_seed917 | elbow | 0.1227 | 0.5621 | 0.1494 | 1.5744 | 0.2685 | 0.006 |
| imm_seed917 | wrist | 0.4917 | 1.0774 | 0.3912 | 1.5689 | 1.1538 | 0.004 |

The wrist has about three times yaw/shoulder/elbow RMS in the well-covered seed42 recordings. Elbow range is smaller, but its accumulated travel is similar to shoulder/yaw; small angle range does not imply inactivity. These are motion-participation measures, not marginal task-space attribution: proving each joint's contribution to pointing/position error requires the task Jacobian and controlled counterfactuals.

All received arm positions remain inside their description limits in these recordings (including startup). Speed failures are not confined to startup. Wrist excess above the description limit is 0.00353/0.05082/1.97611/0.06889 rad/s for CV42/IMM42/CV917/IMM917 respectively. The additional 0.001 tolerance remains unchanged. Elbow exceeds tolerance in CV42 and both seed917 runs; shoulder also exceeds in CV917.

| Run | Complete available safety window (s) | Wrist peak abs (rad/s) | Excess above 1.5 (rad/s) | Sampled violation estimate (s) |
|---|---:|---:|---:|---:|
| cv_seed42 | 0.004–10.528 | 2.3919 | 0.8919 | 0.008 |
| imm_seed42 | 0.004–10.634 | 2.7256 | 1.2256 | 0.024 |
| cv_seed917 | 0.004–10.344 | 3.4761 | 1.9761 | 0.030 |
| imm_seed917 | 0.004–10.738 | 2.3919 | 0.8919 | 0.016 |

[CV42 joint time series](m09_refinement_joint_20261018_cv_seed42_joint_activity.png), [IMM42 joint time series](m09_refinement_joint_20261018_imm_seed42_joint_activity.png), [CV917 joint time series](m09_refinement_joint_20261018_cv_seed917_joint_activity.png), [IMM917 joint time series](m09_refinement_joint_20261018_imm_seed917_joint_activity.png). Plot lines break at feedback gaps >0.010 s. Description limits are red; classification uses the additional tolerance.

## Command semantics and velocity discrepancy

Available: raw WLS requested velocity, safety-filtered velocity, **published** SafeVelocityCommand, actuation's held **accepted** velocity snapshot, sampled **position reference**, physical q/dq. Unavailable: every emitted position-reference update, accepted command's source sequence/time inside ActuationTelemetry, PID internal state, actual applied torque and physics-step control ordering. Reported JointState effort is zero throughout and cannot be interpreted as zero PID effort.

The adapter integrates requested arm velocities into bounded position targets for Gazebo's effort PID; its `commanded_velocity` field is the accepted velocity request, **not measured physical dq or an inner velocity-servo output**. It can hold an earlier command while a later publication already exists. Position-reference snapshots are 10 Hz, the reference adapter runs nominally 50 Hz, and physical feedback nominally 500 Hz. A snapshot slope cannot certify every emitted reference step.

Physical dq minus command uses the latest **nonfuture source-time** command, with a deliberately conservative 0.15 s analysis age cutoff (three controller cycles; this is an analysis rule, not a change to the configured 0.25 s command watchdog). No future-nearest samples are used. Actuation snapshots also expire after 0.15 s; held values do not silently span long missing intervals. Source-time alignment does not establish exact Gazebo application order; logger arrival time and its asynchronously received `/clock` are not substituted for source time.

| Run | Yaw dq−published RMS | Shoulder RMS | Elbow RMS | Wrist RMS (all rad/s) |
|---|---:|---:|---:|---:|
| cv_seed42 | 0.0333 | 0.0884 | 0.0897 | 0.3094 |
| imm_seed42 | 0.0545 | 0.1058 | 0.1111 | 0.3192 |
| cv_seed917 | 0.0357 | 0.1206 | 0.1182 | 0.3405 |
| imm_seed917 | 0.0544 | 0.1236 | 0.1409 | 0.3690 |

Every one of the 120 valid evaluation controller samples in each run has **raw=safe** on all six axes; all 120 corresponding SafeVelocityCommand vectors match the safe output. Existing WLS box bounds can already constrain raw requests; equality here does not mean constraints were absent. No extra safety-filter discontinuity explains these evaluation peaks. Command gaps peak at 0.052/0.052/0.066/0.066 s in CV42/IMM42/CV917/IMM917, with nominal 0.050 s spacing. No evaluation timeout or command-rejection counter increments occur. CV917 has one evaluation feedback rejection away from its largest event; IMM917's one feedback rejection and one command rejection precede evaluation. Sparse snapshot differences cannot establish loss of command delivery.

## Wrist events and estimation-to-control coupling

| Run | Largest evaluation event t (s) | Reported wrist dq (rad/s) | Preceding q interval (s) | q-difference velocity (rad/s) |
|---|---:|---:|---:|---:|
| cv_seed42 | 6.368 | -1.503528 | 0.002 | -1.503528 |
| imm_seed42 | 9.568 | -1.550822 | 0.002 | -1.550822 |
| cv_seed917 | 6.802 | -3.476109 | 0.002 | -3.476109 |
| imm_seed917 | 6.286 | -1.568894 | 0.004 | -0.771963 |

CV42 at 6.368 s and IMM42 at 9.568 s agree with their immediately preceding 0.002 s position increment essentially exactly. Their overspeed cannot be dismissed as a velocity-only reporting spike. IMM42's larger **pre-evaluation** peak at 2.318 s is +2.72562 rad/s and also matches its preceding 0.002 s q difference. CV42 and IMM917 have a startup peak −2.39189 rad/s at 0.008 s, before target measurements or accepted tracking commands, likewise supported by q increments. Startup behavior alone disproves a universal explanation based on jagged target estimates.

The largest event is **CV917 at 6.802 s: −3.47610875 rad/s**. Its q changes from −0.1717244716 rad at 6.800 s to −0.1786766891 rad at 6.802 s, giving exactly −3.47610875 rad/s. It is physical sampled motion, with 1.97610875 rad/s excess over the 1.5 rad/s description limit. The exact duration and torque mechanism remain unobserved. Wider windows contain gaps up to 0.012 s, so their integrated feedback should not be treated as an exact physics trace.

Source-time sequence around CV917:

| Stage | Source time (s) | Observed value |
|---|---:|---|
| Measurement | 6.626 → 6.680 | Position changes [2.2095,−0.3027,1.0137] → [1.8822,0.0007,0.7456] m |
| Target estimate | 6.680 → 6.738 | Position [2.0682,−0.2618,0.8380] → [2.0341,−0.2137,0.8396] m |
| Prediction | 6.738 | First future point [2.0026,−0.1957,0.8326] m; complete horizon in JSON |
| Raw/safe/published wrist request | 6.700 → 6.750 → 6.800 | −0.054915 → −0.951027 → −1.192387 rad/s |
| Actuation snapshot | 6.800 | Held accepted wrist request −0.951027 rad/s; reference −0.200214 rad; held feedback −0.147604 rad |
| Physical feedback | 6.800 → 6.802 | q increment −0.0069522175 rad; dq peak −3.476109 rad/s |

The actuation and command snapshots at the same 6.800 source time have different held requests, consistent with callback ordering and sparse snapshot observation. They do not prove a delivery error. The selected control target is a reference derived from a predicted horizon, not necessarily its first future point; full prediction states, target positions, desired poses and robot-state estimates are retained. No logged input lineage ID establishes which estimate/prediction was consumed in a given WLS cycle.

No timeout, command rejection or feedback rejection counter increments occur within [6.602,7.002] s. `reference_limit_events` rises 0→12, while the shoulder target leads the snapshot's measured shoulder by 0.080 rad at 6.800/6.900/7.002 s. This supports activity of the existing lead limiter, but that aggregate counter does not identify every affected joint or prove a wrist reference discontinuity. The wrist reference snapshots move continuously over the observed values; unobserved intermediate steps and PID output remain uncertain.

The direction-changing target observations, estimate/prediction response and bounded wrist-command changes are temporally related. Across the six-second windows, Pearson correlations between controller-grid target-position step norm and raw-wrist command step magnitude are **0.037 (CV42), −0.028 (IMM42), 0.064 (CV917), 0.271 (IMM917)**. This descriptive test uses latest nonfuture estimates ≤0.15 s old; an estimate step is neither an actual Kalman correction nor an innovation. It cannot separate target motion, robot feedback, reference geometry or plant effects. The data support a potential upstream trigger for some requested transients and downstream physical amplification; they do not prove that estimator jaggedness causes every wrist spike or that Q/R retuning would solve overspeed.

[CV917 event and all available pipeline signals](m09_refinement_joint_20261018_cv_seed917_wrist_event.png), [CV42 event](m09_refinement_joint_20261018_cv_seed42_wrist_event.png), [IMM42 event](m09_refinement_joint_20261018_imm_seed42_wrist_event.png), [IMM917 event](m09_refinement_joint_20261018_imm_seed917_wrist_event.png). The JSON retains additional events near ±1.5 rad/s that do **not** exceed the existing acceptance tolerance. IMM917's largest reported evaluation magnitude spans a 0.004 s feedback interval; the averaged q-difference is smaller, so that individual peak is less independently resolved. Its 5.844 s peak −1.53053 rad/s does match a contiguous 0.002 s position increment.

## Base versus arm allocation

`base_truth` is world-referenced pose with body/child-frame twist, as implied by its Odometry contract. We rotate twist by the recorded xyzw orientation for comparison with world position changes, and compare body-forward velocity/body z angular rate with body-frame commands. For CV917, world position finite differences versus rotated twist have RMSE 0.00877 m/s (versus 0.01988 if body twist is incorrectly treated as world). Residual differences include finite differencing and published twist estimation. Body/world angular z are almost equal for these nearly level runs, but the coordinate transform is still applied for world metrics.

| Run | Requested / physical forward RMS (m/s) | Requested / physical yaw RMS (rad/s) | Horizontal sampled path / excursion (m) |
|---|---:|---:|---:|
| cv_seed42 | 0.226 / 0.237 | 0.193 / 0.086 | 1.154 / 0.504 |
| imm_seed42 | 0.256 / 0.246 | 0.185 / 0.057 | 1.253 / 0.486 |
| cv_seed917 | 0.277 / 0.293 | 0.219 / 0.102 | 1.496 / 0.521 |
| imm_seed917 | 0.272 / 0.278 | 0.210 / 0.082 | 1.306 / 0.511 |

Requested RMS here is sample weighted at controller cadence; physical RMS integrates observed 20 Hz ground-truth twist. Ground-truth coverage is 5.95/6 s (99.17%) for each run, largest gap 0.050 s. This is not a complete physics-step trajectory. Physical base forward peaks are 0.506/0.529/0.543/0.562 m/s, whereas all requested forward peaks are 0.5 m/s. Those numbers are observed plant overshoot, not proof of a hard velocity bound. Physical yaw motion is consistently smaller than requested; plant/contact response, wheel slip and signal latency are plausible contributors, without enough torque/contact instrumentation to isolate them.

The base travels **1.15–1.50 m** across six seconds despite only ~0.49–0.52 m maximum displacement from its first evaluation pose. It is not stationary in these motion-enabled runs. All wheels rotate: paired left/right RMS speeds are CV42 1.712/1.246, IMM42 1.859/1.576, CV917 2.224/1.546, IMM917 1.984/1.464 rad/s; front/rear values agree within each side. WLS requests substantial base motion, and raw/safe equality rules out additional high-level safety clipping as the explanation for low apparent base displacement in these intervals. Reversal/scenario requirements and imperfect physical tracking both matter. Angular/linear velocity components must not be combined into an unweighted norm to claim a percentage allocation.

[CV917 base commands/feedback](m09_refinement_joint_20261018_cv_seed917_base_allocation.png), [CV42](m09_refinement_joint_20261018_cv_seed42_base_allocation.png), [IMM42](m09_refinement_joint_20261018_imm_seed42_base_allocation.png), [IMM917](m09_refinement_joint_20261018_imm_seed917_base_allocation.png).

## Supported conclusions and next action

| Hypothesis | Supporting evidence | Counter-evidence / remaining uncertainty |
|---|---|---|
| Abrupt controller output contributes | Bounded wrist request jumps before CV917's largest physical event | Initial peaks occur before target tracking; request magnitude alone is not physical dq |
| Extra safety-filter discontinuity | No supporting evaluation evidence | Raw=safe and safe=published at all evaluation controller cycles |
| Effort PID / coupled Gazebo dynamics amplify motion | Actual q overspeed while requests remain bounded; existing M02 plant mechanism | No measured torque/PID state, so cannot separate derivative kick, coupling, contact and other plant effects |
| Corrupted/stale velocity alone | Sparse seed917 sampling leaves several peak magnitudes unresolved | Several exact 2 ms position increments confirm physical overspeed, including the 3.4761 rad/s event |
| Timing/delivery fault is principal trigger | Sparse snapshots and some historical feedback rejections exist | No rejection/timeout increments around the largest event; no accepted source ID or per-update publication trace to prove exact application order |
| Estimate jaggedness explains every spike | Some observation/estimate/prediction/command changes precede an event | Weak/modest descriptive correlations; physically corroborated pre-tracking startup overspeed |

**Next corrective action owner: `robot_simulation`.** Follow the already documented M02 actuator investigation: instrument physics-step q/dq and actual PID/commanded torque, and validate coupled plant response and any future effort-limited actuator safety layer. Preserve existing effort limits, physics and PID until that separate engineering work is justified. The smallest observability addition is accepted command source stamp/session/sequence plus timestamped emitted position references, then physics-step torque/state access; avoid another unsynchronized effort writer. Add prediction/estimate source lineage to control diagnostics only if needed to resolve the upstream trigger (`robot_control`/`robot_prediction`, without changing algorithms). `robot_experiments` owns gap-aware evidence and repeatable evaluation.

Remaining limits: short, non-phase-locked reactive scenarios; no torque/contact trace; startup telemetry incompleteness; substantial seed917 feedback loss; snapshot aliasing; no causal counterfactual; no certified physical-speed guarantee. Successful diagnostic calculation does not turn physical acceptance into PASS.
