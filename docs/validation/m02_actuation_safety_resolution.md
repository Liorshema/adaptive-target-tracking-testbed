# M02 physical actuation resolution — 2026-10-19

**CONDITIONAL overall; final nominal trials demonstrate simulation-level compliance.**
The permanent simulation now uses a governed, effort-limited native motor. All
four final CV/IMM × seed42/917 trials have complete physics-step feedback and zero
observed speed/position violations at the unchanged 0.001 tolerances. This is
measured compliance in the tested scenarios, not a universal velocity guarantee
against arbitrary coupled/contact disturbances. Actual native motor torque is
not exposed by the current Gazebo interface. Historical recordings remain intact.

## Root cause and actual signal flow

`robot_control` WLS → final safe generalized velocity `[v,w,dq...]` → authenticated
`SafeVelocityCommand` → `robot_simulation.velocity_actuation` acceptance/watchdogs
→ bounded 50 Hz position-reference integration / PositionGuard → Gazebo motor
at 500 Hz → physical q/dq. Base actuation still uses the existing DiffDrive.

The former stock position PID uses `e=q-r` and
`tau=clip(offset-P*e-I-D*(e-e_previous)/h, -effort, effort)`. Its sign is correct.
At startup physical joints start near zero while targets are shoulder 0.523599,
elbow -0.4 rad. Shoulder P alone requests 104.72 Nm against a 20 Nm limit.
Subsequent 50 Hz target steps are differentiated at h=0.002 s: a 0.02 rad step
with D=5 contributes 50 Nm. The integral contribution is bounded to ±1 Nm, but
that is not saturation-aware anti-windup. Full traces now show the nominal and
clipped PID requests alongside physical motion. Startup wrist overspeed precedes
tracking inputs, demonstrating that estimator noise cannot explain every event.

Bounded reference slope never implied bounded physical speed under this coupled
effort plant. DART imports the correct position, speed and effort limits. Its
unilateral joint-limit rows and finite motor impulse constraints do not supply
an unconditional simultaneous bound under all other constraint impulses. DART
integrates unconstrained velocity **before** solving constraints; describing the
problem as a mandatory one-timestep delayed speed check would be incorrect.

Primary implementation references: [Gazebo position controller](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/joint_position_controller/JointPositionController.cc),
[Gazebo PID](https://github.com/gazebosim/gz-math/blob/gz-math7/src/PID.cc),
[DART joint constraints](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/JointConstraint.cpp),
[Gazebo/DART joint command mapping](https://github.com/gazebosim/gz-physics/blob/gz-physics7/dartsim/src/JointFeatures.cc).
The tested installation uses Gazebo Sim 8.11.0, Physics 7.6.0 and DART 6.13.2.

## Implemented mechanism and ownership

One compiled `robot_simulation::ArmActuator` replaces all stock arm PID writers.
It is built inside the existing ament_python distribution using setuptools/CMake;
there is no new package. Python resources and console entry points are preserved.
The installed binary is resolved explicitly, including in a copied installation.

The default `velocity_servo` backend sends **JointVelocityCmd**, never a state
reset. In DART this selects a SERVO motor constrained by the unchanged description
effort limits, ±20/20/20/10 Nm, through impulse bounds ±effort*h. Native gravity
holding replaces explicit PID holding; the old PID remains selectable as
`legacy_pid`, and its instrumented equivalent as `instrumented_pid`. PID gains,
geometry, physics timestep, WLS/DLS, estimation, prediction and target dynamics
remain unchanged. PID configuration applies only to the PID backends.

Each physics step computes:

```
goal = clip((reference - measured_q)/T, -description_speed, description_speed)
slewed = clip(goal, previous_demand - a*h, previous_demand + a*h)
lower_demand = max(-description_speed, (description_lower - measured_q)/h)
upper_demand = min( description_speed, (description_upper - measured_q)/h)
motor_demand = clip(slewed, lower_demand, upper_demand)
```

If already outside position bounds with no feasible one-step interval, demand is
bounded inward. Position protection takes precedence over demand slew in a
conflict. The acceleration limit constrains **motor demand**, not measured
physical acceleration. Startup uses the same motor toward the configured raised
posture; no joint is teleported. There is no servo integral to wind up. Missing
physics feedback commands zero motor velocity with effort-limited braking;
reset explicitly clears owned commands/PID state/demand history, restores the
configured posture target and opens a new trace epoch. ROS freshness, wall
watchdog, ownership, replay/session protection and latched position faults remain.
Scalar Gazebo position messages lack timestamps, so native receipt alone cannot
authenticate an epoch; the existing ROS adapter remains that authority.

The direct-error/h native prototype was **not accepted**: three of four longer
trials had shoulder peaks 1.058209/1.240765/1.316543 rad/s. At 7.174 s in IMM917,
elbow/wrist motor demands jumped from approximately zero to -1/-1.5 rad/s in one
step while shoulder demand stayed +1; shoulder physical speed became 1.316543.
The final governor removes that abrupt coupled excitation. These failed runs are
preserved under `results/m02_resolution_matched`, alongside the PID baselines.

## Configuration and instrumentation

Owner: `robot_simulation/config/gazebo/joint_position_hold.yaml`.

| Setting | Default / meaning |
|---|---|
| `backend` | `velocity_servo`; selectable `legacy_pid` / `instrumented_pid` |
| `servo.position_time_constant` | 0.1 s; outer position-error response rather than artificial 1/h gain |
| `servo.acceleration_limit` | 4 rad/s²; demand slew, not a physical acceleration guarantee |
| `trace_directory` | omitted/empty disables logging; otherwise absolute fresh output directory |

Unknown backends/governor keys, missing/nonfinite/nonpositive settings and boolean
governor values fail preflight. All physical bounds come from authoritative robot
description → rendered SDF joint axes, including effort; no parallel limit table.
The plugin rejects competing existing motor components and builder-recognized
joint controllers. Multiple configured one-axis joints are supported by name.

Optional local `actuator.csv` records every physics iteration: source time/dt,
iteration/epoch, joint name, pre/post q/dq, reference/error, motor velocity demand,
demand acceleration, PID P/I/D contributions, requested/written effort and limiting
events. `saturation` means torque clipping in PID mode and demand limiting in
servo mode. CSV `error` is q_pre-reference, matching the PID convention; ROS
`position_errors`, tables and plots use reference-measured_q instead.
Native motor effort is NaN/unavailable; PID written effort is a command,
not independently measured torque. JointState effort zeros and structural joint
wrenches are not substituted. PostUpdate retains the writer's pre-step PID values
because Gazebo clears force commands after application. Traces flush each step
and refuse overwriting; reset rotates `actuator_epoch_N.csv`.

Existing `ActuationTelemetry` now appends joint names, dq, feedback source stamp /
freshness, qref-q, and the last accepted active command's session/sequence/stamp.
An inactive or rejected packet cannot impersonate an accepted command. Reset
clears accepted identity. Snapshot time remains distinct from source and physics
receipt time. Existing fields remain; ROS interfaces/dependents must be rebuilt.
Historical normalized JSON remains readable, tested without changing normalization.

## Evidence, alignment and claim limits

The initial historical wrist maximum **3.47610875 rad/s** remains independently
confirmed by q increments. New matched instrumented-PID baselines reproduce the
failure at 2.392–3.054 rad/s; these are not claimed to reproduce the exact historical
trajectory. Both methods use the nominal scenario, unchanged configuration/limits,
same predictor and sensor seed. Reactive trajectories and launch/readiness epochs
differ, so matching configuration/seed does not mean identical input trajectories.

Baseline directory:
`results/m02_resolution_matched/m02_actuator_20261018T203017_596057Z_ifw8d1qe`.
Final directory:
`results/m02_resolution_governed/m02_actuator_20261019T035546_061962Z_oda65sm7`.
Each contains immutable inputs, plugin hash, telemetry, launch log, local physics
trace and results. [Full recomputed metrics and source pointers](../../results/validation/m02_resolution_final_figures/summary.json)
include all four joint tables, startup/tracking splits, RMS/p95, violation samples
and duration, reference errors, physical-vs-command discrepancy and base motion.

Physical safety covers **all available physics iterations including startup and
shutdown**, not only valid control or the tracking window. All four joints must
share the same complete increasing iteration/time grid; missing first steps,
gaps, unequal final horizons, invalid values or absent joints prevent acceptance.
The final trials have 7,812–8,086 samples per joint, first physical post-step
sample 0.002 s, last 15.624–16.172 s, largest gap 0.002 s and 100% sampled-interval
coverage. No missing intervals are filled. Time-weighted RMS uses trapezoidal
squared velocity; p95 is sample weighted. Violation durations use left holds over
eligible intervals. Actual q-increment velocities agree with final reported dq
to <2.4e-13 rad/s. This certifies discrete observed simulation steps, not unseen
continuous real-hardware dynamics.

There are **zero final speed-violating samples and zero observed violation
duration** at speed_limit+0.001 rad/s, and zero position-policy violations at
0.001 rad. The shoulder nevertheless reaches about -6.4716e-5 rad, a small actual
excursion inside the existing tolerance; it is not reported as exact geometric
nonpenetration. Other arm positions remain within description bounds. ROS-only
M08 safety remains unknown in three final runs due to delivery coverage; its gates
were not weakened. Complete local physics traces provide the separate acceptance.

Maximum final speeds across the four trials are yaw **0.625272**, shoulder
**1.000000000**, elbow **1.000000001**, wrist **0.689062 rad/s**, versus description
limits **1/1/1/1.5**. The retained tolerance covers the indicated numerical residue.

## Tracking, activity, performance and verification

Tracking uses 12 simulated seconds after two settling seconds/readiness. Mean
distance errors before→after are CV42 .09587→.10158, CV917 .10697→.11094,
IMM42 .10762→.10017, IMM917 .09147→.09507 m. Mean pointing errors are
12.249→12.466°, 6.636→6.830°, 7.728→8.266°, 6.336→6.821° respectively.
The largest mean pointing increase is 0.54°; the largest mean distance increase
is 0.00571 m. No new tracking acceptance threshold was invented or tuning applied.
Final maximum distance error is .296–.305 m, maximum pointing 14.94–23.70°.

Final RTF is .944/.846/.900/.796 versus .946/.852/.930/.886. These sequential
headless runs include equal full-physics logging, but host scheduling is uncontrolled;
they are not evidence of a performance improvement. Physical fidelity is unchanged.
Base sampled travel is 2.51–3.13 m; maximum excursion .829–.946 m. Every arm joint
moves: final full-interval ranges are yaw .204–.208, shoulder .651–.739, elbow
.426–.486, wrist .397–.461 rad. Actuation was not disabled or frozen to pass.
Evaluation invalid-command/watchdog/rejection/feedback-rejection counts are zero.

- Full functional regression: **1,224 passed**, 18 existing style/copyright tests
  excluded; `m02_resolution_final_regression.log`.
- Focused actuator/architecture/native/interface-recording tests: **137 passed**,
  `m02_resolution_closure_focused.log`; eight analytical trace-acceptance fixtures
  and four native law cases are included, covering missing data/common grids,
  reset epochs, generic limits, invalid input and demand-governor reversals.
- New numerical/robot-builder/native-test focused lint passed at 99 columns.
  Existing unrelated whole-file lint debt remains distinct from functional results.
- Normal and independent copied builds: **all ten packages passed**;
  `m02_resolution_closure_build.log`, `m02_resolution_closure_copy_build.log`.
- Copied installation outside checkout verifies native binary origin, owner YAML,
  M07/M08/M09 contracts, eight entry points and additive ROS fields.
  A repeated check initially found bytecode generated by the previous installed
  launch inspection. Only that generated file under the owned `/tmp` copy was
  removed; the unchanged check then passed with `PYTHONDONTWRITEBYTECODE=1`.
- Real world reset: both epochs complete/safe, control recovered, zero survivors;
  `results/m02_resolution_reset/reset_20261019T040003_826840Z_dgd949fi/result.json`.
- Normal headless visualization compute-only: **29/29 gates**, zero commands;
  `results/m02_resolution_compute_only/headless_20261019T040034_110368Z_cfpf65ir/result.json`.
  GUI clients/graphics were not changed or used as physical-safety evidence.

All trials cleaned owned processes. Three final runs needed the existing bounded
SIGINT/TERM/KILL escalation (launcher -15); IMM917 exited on SIGINT with code 0.
Shutdown tracebacks/escalation are not presented as clean child exits. Per-step
flush and common-grid validation protect against silently accepting partial traces.

## Reproduction

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robot_experiments
source install/setup.bash
# Permanent simulation default, computation only:
ros2 launch robot_bringup visualization.launch.py \
  system_config:=config/system_nominal.yaml gui:=true rviz:=true
# Motion-enabled:
ros2 launch robot_bringup visualization.launch.py \
  system_config:=config/system_nominal.yaml gui:=true rviz:=true enable_actuation:=true
# The normal standalone simulator also uses the same default holding backend:
ros2 launch robot_simulation simulation.launch.py gui:=false
# Controlled before/after, fresh collision-safe study directory:
python3 tools/validation/m02_resolution_run.py --output results/m02_repeat \
  --duration 12 --settling 2
python3 tools/validation/m02_resolution_reset.py --output results/m02_reset_repeat
```

Do not run deployments in the same ROS domain/Gazebo partition concurrently.
The trial utilities lease isolated routing and clean their own processes. Rebuild
the interfaces and dependent packages together. Diagnostic traces are optional;
normal launch does not perform continuous file logging. The preserved original
PID is selected through an inherited holding YAML using `backend: legacy_pid`;
instrumented comparisons use `backend: instrumented_pid` with a fresh trace path.

## Remaining work and verdict

**CONDITIONAL** overall: observed nominal overspeed is resolved in all final tested
cases, with complete step-level evidence and small tracking tradeoffs. This is
simulation-level compliance for these scenarios and an empirical correction, not
a proved hard velocity bound. Native motor torque/constraint-feasibility telemetry,
adversarial contacts and loads, sustained hard-stop operation and longer aggressive
scenario campaigns remain unverified. Do not infer hardware safety or remove
existing guards. Shoulder tolerance does not replace self-collision avoidance.

No controller/estimator/predictor/target/geometry/limit change, no commit or push.
The exact M02-only changes, preserving M09 separately, are in
[m02_resolution_modified_files.txt](../../results/validation/m02_resolution_modified_files.txt), and the
machine-readable closure is [m02_resolution_final_verification.json](../../results/validation/m02_resolution_final_verification.json).

### Per-joint measured comparison

All speeds are rad/s; reference RMS is rad. Rows include startup. Final speed
violation count/duration is 0 / 0 s in every row, sampled-interval coverage is
100%, and maximum feedback gap is 0.002 s. Published-command discrepancy uses
last nonfuture active command source stamps within 0.15 s on the physical grid;
it does not assert exact motor application order. qref-q pairs the reference used
for an iteration with its physical post-step q.

| Case | Joint | Before peak | After peak | After RMS | After p95 | qref-q RMS | dq-command RMS |
|---|---|---:|---:|---:|---:|---:|---:|
| CV42 | base_yaw_joint | 0.513940 | 0.248763 | 0.0765 | 0.1328 | 0.0076 | 0.0344 |
| CV42 | shoulder_joint | 1.000000 | 1.000000 | 0.1911 | 0.3405 | 0.0701 | 0.0764 |
| CV42 | elbow_joint | 1.581808 | 1.000000 | 0.1571 | 0.1572 | 0.0498 | 0.0325 |
| CV42 | wrist_joint | 2.575772 | 0.689062 | 0.1362 | 0.2442 | 0.0139 | 0.1051 |
| CV917 | base_yaw_joint | 0.239818 | 0.183694 | 0.0734 | 0.1397 | 0.0073 | 0.0349 |
| CV917 | shoulder_joint | 1.000000 | 1.000000 | 0.1915 | 0.3368 | 0.0702 | 0.1029 |
| CV917 | elbow_joint | 1.000000 | 1.000000 | 0.1621 | 0.1840 | 0.0502 | 0.1077 |
| CV917 | wrist_joint | 2.584822 | 0.456503 | 0.1149 | 0.2225 | 0.0117 | 0.0910 |
| IMM42 | base_yaw_joint | 0.895577 | 0.625272 | 0.0882 | 0.1287 | 0.0096 | 0.0879 |
| IMM42 | shoulder_joint | 1.000000 | 1.000000 | 0.1954 | 0.3467 | 0.0716 | 0.1686 |
| IMM42 | elbow_joint | 1.184951 | 1.000000 | 0.1739 | 0.3177 | 0.0517 | 0.1524 |
| IMM42 | wrist_joint | 3.054414 | 0.485018 | 0.1371 | 0.2885 | 0.0145 | 0.1784 |
| IMM917 | base_yaw_joint | 0.416929 | 0.224327 | 0.0757 | 0.1424 | 0.0076 | 0.0511 |
| IMM917 | shoulder_joint | 1.053958 | 1.000000 | 0.2025 | 0.4732 | 0.0710 | 0.1244 |
| IMM917 | elbow_joint | 1.586928 | 1.000000 | 0.1620 | 0.2356 | 0.0503 | 0.1033 |
| IMM917 | wrist_joint | 2.391886 | 0.658881 | 0.1423 | 0.2633 | 0.0150 | 0.1682 |

### Before/after plots

Solid gray/blue are physical PID/governed motor observations; dotted q traces are
their position references. Effort panels show PID requested/written commands,
unchanged effort bounds and explicitly unavailable native torque. Effort uses a
labelled symmetric logarithmic scale to retain startup saturation and later detail.

![CV seed42](../../results/validation/m02_resolution_final_figures/cv_42_comparison.png)
![CV seed917](../../results/validation/m02_resolution_final_figures/cv_917_comparison.png)
![IMM seed42](../../results/validation/m02_resolution_final_figures/imm_42_comparison.png)
![IMM seed917](../../results/validation/m02_resolution_final_figures/imm_917_comparison.png)
