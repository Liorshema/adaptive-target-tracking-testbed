# D4.6 — Model Predictive Control

Implemented `mpc`, displayed as **Model Predictive Control**, alongside unchanged
Jacobian and Weighted Least Squares controllers. No commit or push.

## Acceptance

| Area | Verdict | Evidence and scope |
|---|---|---|
| Mathematical correctness | PASS | Independent scalar normal equations, terminal/smoothness/unit tests, constrained optima; local kinematic model only |
| Genuine prediction horizon | PASS | Identical present states with different futures produce different first commands; interpolated timestamp reference test; real 20-sample IMM forecasts reach MPC |
| Solver performance | PASS | N=8, 20 Hz: full control pipeline mean 14.396 ms, p95 24.265 ms, max 34.434 ms in the scored window; no hard real-time guarantee |
| Physical constraints | PASS | Shared command velocity/rate/position horizon constraints and independent final projection; complete scored physical position/speed coverage with zero violations in this Run |
| Architecture | PASS | Existing Controller/Registry/Factory, owner YAML/schema, task geometry, reference builder and Backend; synthetic dimensions 1/2/5/6 and Cartesian/distance-pointing tasks |
| Runtime integration | PASS | Saved Scenario → authoritative Jobs/Runner → real estimation/prediction/control/actuation → completed Run → Results/Session; cleanup empty |
| Backward compatibility | PASS | 553 broader functional/Builder tests, then 537 final control tests; existing algorithms and defaults preserved |

These verdicts concern this implementation and bounded test configuration. They
are not global tracking-quality, collision-avoidance or real-hardware safety
certification. Pointing performance remains a research limitation.

## Architecture and configuration ownership

`robot_control/controllers/model_predictive.py` owns horizon assembly and the
quadratic solve. Its `ModelRegistration` is discovered normally; no controller-ID
branch was added to the dashboard or ExperimentBackend. The Factory injects the
existing reference builder by declared dependency. Owner metadata supplies the
human-readable display name. The disabled Environmental Disturbance checkbox
was relabeled without enabling it.

Minimal shared extensions:

- Optional `ControlInput.evaluation_time`, supplied from actual orchestrator `now`;
  older controllers/callers can leave it unset. MPC requires it explicitly.
- `PredictedTargetTrajectory.sample()` owns linear interpolation and bounded
  initial-interval backprojection. Existing linear lookahead sampling now reuses
  it, preserving the existing first-future policy and feedforward switch.
- Read-only `SafeControlPipeline.acceleration_limits` exposes the existing owner
  envelope. Physical limits are never copied into MPC tuning.

Task numerical defaults remain in `robot_control/config/control.yaml`. MPC adds:

| Parameter | Default | Meaning |
|---|---|---|
| `horizon` | 8 | Number of optimized future holds, minimum 2 |
| `step_duration` | 0.1 s | Planned hold spacing after the first hold |
| `task_weights` | [4,12,12] | Dimensionless row weights, selected task order |
| `task_scales` | [1,1,1] | Positive row-unit scales: m/rad for distance-pointing |
| `motion_weight` | 0.02 | Squared normalized velocity penalty |
| `smoothness_weight` | 0.01 | Squared normalized command-difference penalty |
| `terminal_weight` | 1 | Additional multiple of last tracking cost |
| `max_iterations` | 60 | SLSQP iteration ceiling |
| `solver_tolerance` | 1e-8 | Convergence tolerance |
| `feasibility_tolerance` | 1e-7 | Maximum residual in normalized velocity / physical linear constraints |

All are exposed through the existing registered schema/forms and immutable
Scenario authoring. Row vectors must match the selected task, including when
switching to another task dimension. Motion normalization derives its dimension
and speed scales from the model/safety envelope; no joint count or joint names
appear in the MPC implementation.

## Mathematics and assumptions

Current-minus-desired task convention is `error_dot = J u + target_rate`.
For times `t_0 = now`, `t_k = now + sum(h_i)`, evaluate unactuated errors `b_k`
using the existing task at the frozen measured tracking pose and each future
target. Freeze the current task Jacobian `J` over this short horizon:

```
e_k = b_k + J sum_{i=0}^{k-1}(h_i u_i)
r_i = (b_{i+1} - b_i) / h_i
e_{i+1} = e_i + h_i (J u_i + r_i)
```

`h_0` is actual positive control `dt`; later holds use `step_duration`. At nominal
20 Hz, the default horizon ends 0.75 s after now. The deployed predictor supplies
20 future samples at 0.1 s spacing (2 s nominal forecast horizon).

Distance and pointing reuse `DistancePointingTask` and its geometry/Jacobian.
Cartesian tasks reuse `CartesianVelocityTask` and TrackingReferenceBuilder.
Distance-pointing does not generate unused Cartesian reference poses.
Reference movement enters **once** through `b_k`; the task's instantaneous
`target_rate` is not added again. Forecast velocities are used only when the
short pre-first-sample interval requires the existing backprojection convention.

Let `D_v = diag(max(abs(v_lower),abs(v_upper)))`, with unit scale for a coordinate
fixed at zero, and `z_k = D_v^-1 u_k`. The condensed integration matrix uses actual
hold durations. Costs are:

```
Q = diag(task_weights / task_scales²)
Qf = terminal_weight * Q
R = motion_weight * D_v^-2
S = smoothness_weight * D_v^-2
```

Minimize all future tracking errors, all normalized command magnitudes and
changes from the actual previous command, plus the additional terminal cost.
Positive motion weight makes the quadratic strictly convex, including redundant
or rank-deficient Jacobians. Angles remain radians, never degrees in the solve.

The principal spherical/SO(3) log chart must stay continuous. Exact antipodes
and detected forecast chart crossings fail explicitly. Cartesian angular checks
inspect the full relative rotation before partial task selection; mixed linear/
angular Cartesian rows are rejected. This is deliberately a local controller,
not a global nonlinear orientation planner or a stability proof.

## Forecast and safety contracts

Forecast samples must be finite, strictly ordered and in the robot-state world
frame, with canonical position or position/velocity layout. MPC interpolates
multiple future samples at actual optimization times. It never extrapolates
past the end; initial backprojection requires six-state data and is limited to
one initial sampling interval. Missing, old-ended, insufficient, incompatible
or corrupted forecasts fail. Prediction reference-epoch and robot-state age
remain checked by the existing upstream InputValidator, including future-time
checks and TF conversion. Direct pure-controller callers must preserve that
upstream freshness contract; the forecast interface does not carry its epoch.

Every hold respects generalized velocity bounds. When configured, command changes
respect shared acceleration bounds using each hold's duration. Integrated joint
positions respect the same authoritative joint limits at every horizon endpoint.
Joint coordinate mapping follows the existing model/adapter configuration-tail
contract and safety velocity indices. First-step feasibility also calls the
shared `feasible_bounds`, preserving current-position violations/recovery behavior.
Only the first optimized command passes to the unchanged final safety pipeline.

No physical acceleration limits are invented when the owner provides none.
These kinematic/reference constraints alone do not guarantee physical actuator
speeds or self-collision avoidance. Effort/holding PID, physical actuation safety,
command timeout/replay protection and clock-reset handling remain unchanged.

## Solver and numerical verification

NumPy solves the SPD unconstrained quadratic first. If feasible, that is already
the exact constrained optimum. Otherwise use SciPy SLSQP with analytic gradients,
linear constraints, bounded iterations, convergence checks and a separate finite/
feasibility check. No other controller is substituted on failure; the existing
node reports failure and stops/inhibits commands.

For ordinary inequality envelopes, Cholesky whitening `H=L Lᵀ`,
`z=z_free + L^-T y` makes the **same** QP objective `0.5 ||y||²`, up to positive
scale/constant. All box and horizon inequalities are transformed exactly. This
avoids SLSQP spending tens of iterations learning curvature. Degenerate fixed/
equality envelopes retain SciPy's normal bound/equality handling for the same QP.

Independent tests cover hand-derived two-step scalar solutions, terminal and
smoothness terms with nonunit scales, timestamp interpolation using control time
rather than stale state time, different futures with the same present, no doubled
feedforward, exact constrained joint/velocity/rate solutions, unequal first hold,
fixed/inactive actuators, zero acceleration, dimensions, deterministic output,
antipodes/partial angular tasks, and explicit infeasible/failed/nonfinite results.
Final focused MPC suite: **28 passed**.

A fresh read-only code review found one Important issue: a partial Cartesian
angular selection could hide an SO(3) antipodal discontinuity. Reproducing tests
failed before correction; full-rotation checks now pass them. The reviewer
otherwise confirmed the condensed mathematics and shared horizon constraints.

## Measured integration

Both real checks used the same saved `EXP-MPC-D46`, revision 1, seed 42, nominal
IMM/fight-flight composition, six scored simulation seconds after three seconds
settling. Explicit Backend actuation opt-in was used. No GUI was needed.

| Measurement | Initial implementation | Final implementation |
|---|---:|---:|
| Run ID | `9e551941-11fa-46b6-9f72-48a16423d5bf` | `cd18c83c-33cf-4ff4-beb0-805d0b60fa08` |
| Genuine execution | Completed | Completed |
| Existing overall acceptance | false | true |
| Scored full control time mean / p95 / max | 114.764 / 241.974 / 300.640 ms | 14.396 / 24.265 / 34.434 ms |
| RTF | 0.567 | 0.942 |
| Distance error mean / max | 0.17375 / 0.32860 m | 0.08987 / 0.26258 m |
| Pointing error mean / max | 20.519 / 35.934° | 30.002 / 46.110° |

The first Run exposed an unacceptable timing problem. Profiling isolated unused
pose generation and 27–35 optimizer iterations. After the exact preconditioning
and removal of unused work, 81 recorded actual state/forecast contexts replayed
with zero failures, generally two constrained iterations. Profiled full-pipeline
replay mean/p95/max: 10.65 / 13.18 / 15.30 ms. Diagnostic `solver_seconds` includes
MPC horizon assembly; recorded ROS `computation_time` includes the entire control
pipeline, conservatively bounding optimizer time against the 50 ms period.

The first live check initially overlapped functional tests; these were paused
before completion. Final live performance was measured with regression tests
paused throughout. Different readiness origins and reactive trajectories make
these two Runs unsuitable for a controlled tracking-quality comparison. No
tracking improvement or ranking is claimed. Both Runs and their evidence remain.

Final scored window: **5.178–11.178 s**, stopped at **11.178 s**. It contains:

- 120 valid control samples / active commands, zero invalid scored control samples.
- 102 genuine prediction messages, each carrying 20 future states.
- Maximum raw versus final-safe command difference `7.55e-15`.
- 0.164 m net planar base displacement (maximum excursion 0.271 m).
- Arm joint spans, description order: 0.2791, 0.2604, 0.2589, 0.2531 rad.
- Maximum measured joint speeds: 0.6808, 0.7170, 0.7166, 0.7173 rad/s,
  versus the unchanged configured 1 rad/s limits.
- 3,001 scored joint observations; zero position/speed violation samples;
  full six-second coverage, maximum observation gap 0.002 s. Existing tolerance
  remains 0.001 rad / rad/s; raw reported maximum overshoot is zero.
- Prediction RMSE 0.53428 m over 1,681 forecast/truth comparisons; recorded target
  estimation RMSE 0.08476 m. These are observations, not quality acceptance claims.
- Cleanup returned zero with no owned survivors.

Final Session: `8115ed13-346e-4c4e-9a35-394725861e22`. Backend Results access
returns the immutable Run and real time series (504 tracking/pointing samples,
1,681 prediction errors, 120 control/command samples, plus recorded estimation,
safety, RTF and actuation data). The Session references the existing Run directory;
no second result store was created.

## Regression, build and preservation

- `functional_tests.txt`: **553 passed**, existing Dash DataTable deprecation
  warnings only; includes all control functional tests and 22 Builder tests,
  including MPC Factory/Scenario save/load, immutable revisions and UI label.
- `final_control_tests.txt`: **537 passed** after the optimizer/chart changes,
  including all 28 MPC cases. Counts overlap the broader run.
- `final_targeted_tests.txt`: **53 passed** for MPC, shared contracts and
  orchestrator sampling after final whitespace cleanup.
- `component_tests.txt`: **2 passed**, existing public component API checks.
- `final_new_file_lint.txt`: new controller and focused tests pass the package's
  Flake8 configuration. Repository-wide historical lint was not used as a
  functional gate or reformatted.
- `build_verified.txt`: final selected-package build (robot_control,
  robot_dashboard). Final robot_control installation sync follows in
  `build_install_sync.txt`; installed-module/source equality checks are in
  `installed_verification.json` and `install_sync.json` (isolated import from `/tmp`).
- `verification.json`: all **7,239** pre-existing result JSON/YAML documents
  fingerprinted before implementation are byte-identical, including historical
  configuration/provenance. No evidence deletion or relocation.

Evidence root: `results/mpc_controller/`. `final/runs/…/runs/imm_seed42/` owns
metrics, raw ROS telemetry, frozen inputs, launch logs, status and cleanup.
Generated validation scripts/evidence stay outside production packages.

## Reproduction

From `/home/user/ros2_ws`, after building:

```bash
source install/setup.bash
# Uses saved immutable revision 1, a NEW output folder, explicit actuation opt-in.
build/dashboard_venv/bin/python results/mpc_controller/final_run_check.py \
  --output-dir results/mpc_controller/reproduction_01 --enable-actuation
```

The output folder must not exist. Omitting `--enable-actuation` keeps physical
actuation off. The helper uses the existing Backend/Jobs/Runner, bounded outer
cancellation, original duration/settling/readiness predicates and cleanup.

To inspect the saved Scenario and Runs in the dashboard, use a verified free port:

```bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir /home/user/ros2_ws/results/mpc_controller/workspace \
  --catalog /home/user/ros2_ws/results/mpc_controller/workspace/catalog.yaml \
  --result-root /home/user/ros2_ws/results/mpc_controller/final/runs
```

Select `MPC integration`, revision 1. New drafts can select **Model Predictive
Control** and edit registered parameters normally. Run requires the existing
explicit physical-actuation checkbox. No dashboard/lifecycle redesign was added.

Targeted verification:

```bash
PYTHONPATH="$PWD/src/robot_control:$PYTHONPATH" \
  build/dashboard_venv/bin/python -m pytest src/robot_control/test \
  --ignore=src/robot_control/test/test_flake8.py \
  --ignore=src/robot_control/test/test_pep257.py \
  --ignore=src/robot_control/test/test_copyright.py -q
build/dashboard_venv/bin/python results/mpc_controller/verify_results.py
```

## Remaining limitations and review decisions

- Local frozen-J dynamics and finite horizon do not establish global stability,
  long-duration tracking quality or performance for arbitrary horizon/robot sizes.
  Degenerate equality envelopes are tested numerically, not benchmarked in Gazebo.
- Antipodal/branch-crossing orientations fail safely instead of using a global
  orientation planner. Current distance-pointing and continuous Cartesian tasks
  remain supported.
- Reviewer Minor, documented/deferred: existing orchestrator `target_position` /
  `desired_pose` telemetry follows the configured single-lookahead reference
  (0.25 s here), whereas MPC's `task_error` starts at `evaluation_time` and uses the
  full horizon. These are distinct diagnostic instants, not the scored physical
  pointing metric. MPC does not use the single-lookahead target as its horizon
  origin. No cross-controller telemetry redesign was introduced.
- No matched WLS/DLS tracking benchmark or tuning sweep was performed. The observed
  30° mean pointing error warrants future experiment work; it is not hidden by
  successful execution or physical-limit acceptance.
- Physical-limit observations passed in this bounded simulation only; hardware
  safety, collision avoidance, slip/localization limitations and actuator-model
  assumptions remain outside this milestone.

Review rulings: the read-only reviewer deferred runtime, physical safety and
historical integrity to measured evidence; those checks are now reported above.
The Important angular issue was fixed and tested; the diagnostic timestamp Minor
is explicitly retained with the stated interpretation and no scientific metric
redefinition.

## Exact production/documentation changed files

1. `src/robot_control/robot_control/controllers/model_predictive.py` — MPC and registration.
2. `src/robot_control/config/control.yaml` — numerical MPC defaults only.
3. `src/robot_control/robot_control/interfaces/control_data.py` — evaluation time and shared forecast sampling.
4. `src/robot_control/robot_control/pipeline/tracking_control_orchestrator.py` — timestamp propagation, reuse sampling.
5. `src/robot_control/robot_control/safety/safe_control_pipeline.py` — expose existing acceleration envelope.
6. `src/robot_control/robot_control/factory/control_factory.py` — declared reference-builder injection.
7. `src/robot_control/test/test_model_predictive.py` — independent numerical/constraint/failure tests.
8. `src/robot_dashboard/robot_dashboard/app.py` — registry display name and disabled label.
9. `src/robot_dashboard/test/test_builder.py` — real MPC Scenario/UI/immutable revision test.
10. `docs/validation/mpc_controller_d46.md` — this design, evidence and verdict record.

No production files removed. Existing untracked reference images/PDF were left
untouched. No M03 changes, physics/controller-baseline changes, commit or push.

Implementation ledger: inspection → failing import/independent tests → MPC and
shared integration → Factory/Builder regression → first live timing failure →
profiled exact preconditioning → angular review correction → final bounded Run →
final functional/build/history/results checks. All three implementation-plan
steps are complete; validation evidence is retained under its dedicated root.
