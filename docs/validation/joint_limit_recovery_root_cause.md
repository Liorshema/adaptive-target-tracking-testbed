# Joint-limit recovery: causal diagnosis and verification

Task date: 2026-10-21. Working-tree baseline is
`joint_limit_recovery_baseline.json`; pre-existing M09, M02 and challenge work is
preserved. No staging, commits or pushes. **Final verdict: CONDITIONAL.** The
reproduced physical motor deadlock is corrected and physically verified. Serious
physical fault-injection coverage and intermittent target-context synchronization
remain open; tracking accuracy is a separate limitation. Completed evidence is below.

## Cause established before correction

The failure is physical motor-row starvation, rather than missing recovery
publication. The active engine is Gazebo Physics 7.6.0 DART with DART 6.13.2,
Gazebo Sim 8.11.0. `<physics type="ode">` does not choose that engine. The actual
selected deployment step is 0.002 s. The hypothesis/falsification table is in
`joint_limit_recovery_plan.md`; exact versioned source and installed-library
checks are in `joint_limit_recovery_physics_audit.md`.

In DART's combined `JointConstraint::update`, a position-limit row takes priority
over its SERVO motor row. The velocity-limit branch also precedes the motor.
Each selected branch continues to the next degree of freedom. A joint a tiny
floating-point distance outside its hard stop can therefore receive only a
boundary correction: no motor row acts on its legal inward velocity demand.
Outward unconstrained gravitational velocity can sustain the competing limit row.
Solver force mixing and floating-point residuals keep this state persistent.
This is a constraint-construction interaction; replacing the boxed LCP solver or
changing a ROS reference limiter does not remove it.

Historical C/D/E wrist traces show contiguous stalls of 18.736, 18.494 and 19.160
simulated seconds, respectively, with demand reaching +0.6 rad/s and physical
velocity near 1e-12 rad/s. Feedback stays fresh, active safe commands continue and
timeout counters do not increase. `accepted_stamp` is historical accepted active
identity, not recovery heartbeat. `joint_limit_recovery_authority_audit.md`
records the orchestration audit: no cyclic dependency prevents release once
physical position actually crosses the existing interior margin.

## Physical RED and causal control

The native probe launches the normal `robot_simulation/simulation.launch.py` with
run-local configuration and the real robot, gravity, bridge and native motor. It
commands authoritative hard stops, dwells, then requests the existing 0.06 rad
inward offset. The standalone motor stage deliberately has no adapter: authority
and guard release are tested separately through the real adapter.

| Stock engine fixture | Boundary q (rad) | Inward displacement in 4 s (rad) | Release |
|---|---:|---:|---|
| C wrist lower | -0.261799387800174 | 0 | FAIL |
| C wrist upper | +0.261799387754613 | 0.0600000000022 | moves |
| C shoulder lower, loaded | -3.99485e-13 | 3.67e-19 | FAIL |
| C shoulder upper | 1.56999999999728 | 0.0600000000013 | moves |

Original evidence roots:

- `results/joint_limit_recovery_red/native_boundary_20261021T034132_558095Z_rxzw_s1u`
- `results/joint_limit_recovery_shoulder_red/native_boundary_20261021T104748_225793Z_fuj3c7ju`
- `joint_limit_recovery_pre_fix_measurements.json` (full-step safety and continuous windows)

A diagnostic using the existing effort PID at the same wrist lower stop moves
inward. This falsifies an insufficient-effort explanation for that wrist posture.
It also demonstrates severe physical overspeed (wrist approximately 12.35 rad/s)
so it is **not a correction** and is not selected for deployment. Its isolated
results are under `results/joint_limit_recovery_pid_diagnostic`.

## Focused physics integration

The design and risks were documented before implementation in
`joint_limit_recovery_backend_proposal.md`. All production changes belong to
`robot_simulation`. The new explicitly selected backend is built offline from
pinned official Gazebo Physics DART source using the installed public SDK, with
upstream license, exact commit/archive digest and retained-file hashes. It has a
distinct namespace/plugin class; it does not interpose private installed C++ ABI.

The native actuator's position-to-velocity governor, authoritative robot bounds,
effort and speed limits, gains, PositionGuard thresholds, heartbeat/replay/timeouts,
control strategies, estimator, prediction and target dynamics remain unchanged.
The only motor integration change separates the existing bounded motor impulse
row from the automatic physical-limit row. Passive ordinary dynamics prevent a
velocity command being misinterpreted as ordinary torque. Inherited motor math
retains the actual-step impulse bound `tau_min*h <= lambda_motor <= tau_max*h`.
The automatic unilateral limit row remains free to supply physical stop reaction.
These are distinct quantities: a stop reaction is not motor torque.

The first integration candidate was correctly rejected by real Gazebo tests.
Although its inherited `update()` computes motor rows in PASSIVE mode, inherited
`ServoMotorConstraint::isActive()` separately requires SERVO. Thus it registered
but excluded motor rows and the arm drifted under gravity. The candidate results
are preserved in `results/joint_limit_recovery_candidate`. A minimal derived
constraint must explicitly enable its independently owned nonempty motor row;
no tolerance, limit or gain change addresses that gate.

Motor ownership covers force-mode switching, joint detach/model destruction and
world reset; physical tests and code review determine the limits of those claims.
The `world.physics.engine` selector accepts `dartsim` (stock) or
`dartsim_recovery` (packaged plugin). The world builder writes Gazebo's supported
Physics-system `<engine><filename>` field and resolves the binary from the active
installation. Unknown selectors fail preflight. Omitting the selector preserves
Gazebo's legacy engine selection.

## Evidence and acceptance

The native boundary and real-adapter recovery gates passed, together with reset,
compute-only, normal build and copied-install checks. Challenge attempts are
evaluated separately for physical safety, motor mobility, sustained valid control
and tracking accuracy below. A safe stationary joint is not proof of successful
closed-loop tracking. Failed and incomplete attempts remain preserved.

## Primary source references

- [DART 6.13.2 combined limit/motor row](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/JointConstraint.cpp)
- [DART independent bounded motor and activation gate](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ServoMotorConstraint.cpp)
- [Gazebo Physics 7.6.0 velocity command](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.6.0/dartsim/src/JointFeatures.cc)
- [Gazebo Sim 8.11 engine selection](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/systems/physics/Physics.cc)

## Reproduction commands

From `/home/user/ros2_ws`, with one controlled simulator at a time:

```bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
# RED: explicit stock engine, same configured robot/native actuator.
python3 tools/validation/joint_limit_recovery_probe.py \
  --output results/joint_limit_recovery_reproduce_red --variants C \
  --joints wrist shoulder --sides lower upper --engine dartsim
# Physical GREEN and independent starts; no direct state edits.
python3 tools/validation/joint_limit_recovery_probe.py \
  --output results/joint_limit_recovery_reproduce_green --variants A C E \
  --joints wrist --sides lower upper --engine dartsim_recovery --starts 2
python3 tools/validation/joint_limit_recovery_probe.py \
  --output results/joint_limit_recovery_reproduce_shoulder --variants C \
  --joints shoulder --sides lower upper --engine dartsim_recovery --starts 2
# Real ROS authority -> adapter -> native physics -> recovery release.
python3 tools/validation/joint_limit_recovery_authority_probe.py \
  --output results/joint_limit_recovery_reproduce_authority --variants C D E \
  --joints wrist --sides lower upper --seeds 42 917 --engine dartsim_recovery
# Unchanged challenge definitions; longer observation, not a tuned scenario.
python3 tools/validation/joint_limit_recovery_challenge.py \
  --output results/joint_limit_recovery_reproduce_challenge \
  --variants C D E --seeds 42 917 --duration 90
# Existing real reset and compute-only gates.
python3 tools/validation/m02_resolution_reset.py \
  --output results/joint_limit_recovery_reproduce_reset \
  --system-config config/system_manipulation_C.yaml
python3 tools/validation/m09_live.py \
  --output results/joint_limit_recovery_reproduce_compute_only \
  --compute-only --duration 4 --settling-time 2
```

ROS/Gazebo needs normal local DDS/UDP access. The execution sandbox does not
provide that; physical verification uses the permitted external execution
context. All scripts lease isolated ROS domains/Gazebo partitions and retain
owned-process cleanup evidence. They never stop unrelated user processes.

## Constraint interpretation and scope

For one lower stop, the linearized solve combines the free velocity with motor
and stop impulses: `v_next = v_free + M^-1 (lambda_motor + lambda_stop)`.
The stop is unilateral and the motor impulse obeys the configured effort bound
`|lambda_motor| <= tau_max * h`. Inward motor demand can therefore act while the
stop prevents outward motion; an effort cap may still make a requested velocity
unachievable under a sufficiently large load. No new minimum inward torque or
unbounded "recovery impulse" is introduced. The untouched demand governor still
limits requested speed, position approach and acceleration; measured full-step
feedback, rather than that reference alone, determines physical acceptance.

The backend applies to configured joints without a wrist/shoulder name check.
Independent motor ownership is removed before force-mode changes, detachment,
model removal and backend destruction. Unregistered skeleton root joints are
excluded from the owned-motor lookup. Membership is checked through the public
constraint count/index API; the native actuator's fresh demand on every physics
step re-registers a cleared manual row after reset. No claim is made that a motor
will re-register without another velocity command.

The source-built backend is pinned to the installed Gazebo Physics 7.6.0 and DART
6.13.2 SDK. An SDK upgrade requires an explicit source/patch review and repetition
of the physical gates. This maintenance cost is preferable to relying on the
private binary layout of an installed plugin. Hardware safety certification,
universal contact/collision guarantees and elimination of existing tracking or
localization limitations are outside this correction.

## Regression evidence

- `joint_limit_recovery_focused.log`: **105 passed**, covering guard, real-adapter
  callback safety tests, native demand math, model/world construction and selector
  validation.
- `joint_limit_recovery_engine_red.log` / `...engine_green.log`: the new selector
  tests first failed on the missing contract, then **21 related tests passed**.
- The initial complete suite reported 13 failures, 1,278 passes and six skips.
  One was a newly introduced numerical-layer import of a Gazebo resolver. That
  regression was fixed by keeping the pure selector contract in
  `factory/physics_configuration.py`; the runtime resolver remains in `gazebo`.
  `joint_limit_recovery_architecture_fix.log`: **55 passed**.
- `joint_limit_recovery_regression_functional.log`: **1,279 passed, 18 deselected**
  after the architecture fix and permanent engine selection. The deselections
  are copyright/style checks, not functional cases.
- The remaining 12 complete-suite failures are the repository-wide `test_flake8`
  and `test_pep257` checks in `robot_bringup`, `robot_control`, `robot_description`,
  `robot_models`, `robot_prediction` and `robot_simulation`. Those checks scan
  existing style debt, historical snapshots and validation scripts. They are
  not functional passes. Newly added production selector modules and the new
  selector test file pass their focused flake8 check
  (`joint_limit_recovery_new_file_lint.log`).

Existing tests still verify that serious measured excursions latch, good feedback
cannot clear the latch, stale/replayed/competing commands are rejected, timeouts
stop motion, and clock resets retire transport authority. A real significant
physical excursion greater than the unchanged 0.001 rad guard threshold has not
been deliberately induced. No fake feedback, enlarged physical bounds, reset
teleports or weakened fault policy are used to manufacture such a demonstration.
This physical fault-injection coverage gap must remain explicit in the verdict.

Motor torque/constraint impulse is not exposed by the current native recorder.
Its effort fields remain unavailable for velocity motors. The motor effort bound
is established from the inherited DART constraint implementation and unchanged
SDF limits, **not a claimed torque measurement**. Full-step physical position and
velocity evidence is evaluated separately with the existing 0.001 rad and
0.001 rad/s acceptance tolerances; these are acceptance thresholds, not enlarged
URDF bounds or altered PositionGuard numerical tolerance.

## Completed focused physical gates

- **12/12 final normal-install native wrist cases passed:** nominal, ±15° and ±30°,
  lower and upper stops, each on two independent process starts. Every boundary
  was reached within the unchanged guard numerical band before the inward test.
- **2/2 copied-install native cases passed:** wrist lower and loaded-shoulder lower,
  using the final copied plugin from `/tmp`, with no workspace package imports.
- **12/12 real-adapter cases passed:** C wrist/shoulder at both stops on seeds 42
  and 917; D/E wrists at both stops on seed 42. Fresh validated heartbeats,
  measured inward motion, return to `normal`, new accepted ordinary commands and
  resumed physical base/arm movement were all observed. Sampled fresh-authority
  recovery windows were 0.120–0.136 source seconds. Physical base displacement
  during the one-second resume phase was 0.04592–0.04862 m. There were zero watchdog
  timeout events and one fail-closed combined input-validation rejection during
  seek; subsequent valid input resumed normally. No authority rules were weakened.
- The final focused native and adapter traces all passed full-step position and
  speed acceptance, including startup. See `joint_limit_recovery_native_measurements.json`
  and `joint_limit_recovery_authority_measurements.json`, with direct raw paths.
- **Reset: 4/4 checks passed:** real world reset observed, control recovered,
  both native epochs complete and physically safe, and owned-process cleanup.
- **Compute-only: 29/29 live gates passed in each installation**, including
  `publish_commands=false`/zero command policy, physical FK/optical agreement and
  cleanup. Startup gravity holding remains owned by simulation.
- **Builds: all 10 packages passed** in both normal symlink installation and a
  fresh copied installation. An initial normal build encountered task-created
  regular notice files from a prior `install_data` check; these were preserved in
  `results/joint_limit_recovery_install_resource_backup`, then the normal symlink
  build passed. No source correction was needed for that generated-file conflict.
- Copied M02/M07/M08/M09 checks, all five challenge variants, binary origin and
  installed notices passed outside the checkout. The old validation helper
  assumed a merged-install CLI path; its two-line prefix lookup now supports
  isolated and merged layouts. The original helper is preserved in
  `results/joint_limit_recovery_validation_utility_backup`.

Run roots and full commands are retained in each result's identity/cleanup files.
The copied backend's license/provenance/README are installed under
`share/robot_simulation/third_party/dartsim_recovery`. Local source hashes were
refreshed after the final lifecycle correction and prominent change notices;
`joint_limit_recovery_backend_verified_identity.json` records the exact final
normal binary and source digests.

Final matching lower-stop measurements make the mechanism particularly clear:
the corrected C wrist still dwells just outside its lower bound
(`q=-0.2617993877996618`, bound `-0.2617993877991494`) and then physically moves
0.0599999999484 rad inward. The final copied shoulder likewise dwells at
`q=-1.99742e-13` against bound zero and moves 0.0599999999800 rad inward.
The correction does not depend on moving the hard stop, erasing the tiny
excursion, or editing feedback before allowing recovery.

## Normal launch after installation

The bringup default keeps actuation disabled. Request closed-loop actuation
explicitly; the selected simulation configuration inherits the corrected engine:

```bash
source /home/user/ros2_ws/install/setup.bash
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_manipulation_C.yaml \
  enable_actuation:=true publish_ground_truth:=true gui:=false
```

Use `system_manipulation_D.yaml` or `system_manipulation_E.yaml` for the other
unchanged challenge variants. `gui:=true` enables the usual Gazebo GUI; the
physical validation above is headless. Standalone permanent simulation/holding:

```bash
ros2 launch robot_simulation simulation.launch.py gui:=false
```

No recovery-specific ROS node or manual transport command is required.

## Hypothesis verdicts

| Hypothesis | Evidence-based result |
|---|---|
| H1: hard-stop constraint/motor interaction | Supported: stock lower-stop RED in wrist and loaded shoulder, versioned row-priority source/binary evidence, and matching physical GREEN with independent bounded motor rows. |
| H2: insufficient motor effort | Not the primary cause in the reproduced wrist posture: existing effort PID moves it; the corrected bounded motor also releases the loaded shoulder. Arbitrarily heavier loads are not covered. |
| H3: target/governor pathology | Excluded for the saved failures: native internal reference and inward demand update correctly for thousands of contiguous steps. The same untouched governor works after the backend correction. |
| H4: recovery authority deadlock/starvation | Excluded for captured long freezes: fresh feedback/validated commands continue without timeout increments. Twelve real adapter cases demonstrate release and new accepted ordinary commands. |
| H5: physical failure plus persistent watchdog deadlock | No supporting evidence in the captured failures. The single observed new-fixture input rejection fails closed and is followed by successful recovery; it does not persist. |

## Full-step physical safety measurements

`joint_limit_recovery_final_physical.json` aggregates 37 distinct trace epochs: final native, copied native, adapter, reset, nominal smoke and challenge attempts (including the safe incomplete startup). Each arm joint has 488,565 measured samples at the actual 0.002 s step, with complete native coverage. This is independent of lower-rate ROS recording gaps.

| Joint | Configured speed (rad/s) | Peak measured speed (rad/s) | Maximum raw position excess (rad) | Violating position/speed samples |
|---|---:|---:|---:|---:|
| base_yaw_joint | 1.0 | 0.723766442846 | 1.30118e-13 | 0/0 |
| shoulder_joint | 1.0 | 1.000000000200 | 6.47153e-05 | 0/0 |
| elbow_joint | 1.0 | 1.000000000501 | 0 | 0/0 |
| wrist_joint | 1.5 | 1.500000000137 | 1.78513e-12 | 0/0 |

Violation counts use the existing 0.001 rad and 0.001 rad/s acceptance policy. The maximum raw speed excess is 5.00958e-10 rad/s; shoulder startup position excess is 6.47153e-5 rad. These small raw excursions are reported, not rounded into a claim of exact hard-bound compliance. No physical bound, guard tolerance or acceptance tolerance was increased. Motor effort is analytically bounded by inherited DART impulse limits; measured torque remains unavailable.

## Configuration ownership and implementation scope

The only new runtime configuration parameter is `robot_simulation/config/launch.yaml: world.physics.engine`. Its permanent default is `dartsim_recovery`; `dartsim` explicitly selects the stock backend for reproduction. An omitted parameter preserves legacy Gazebo selection. Validation belongs to the pure simulation factory contract, binary resolution and SDF generation to simulation Gazebo integration, and motor/constraint ownership to the simulation native backend. No new control strategy, ROS actuator writer or recovery node was added.

Existing `position_recovery` values are unchanged. The recovery inset comments and one legacy PID test name/docstring now identify their historical PID rationale; PID gains do not govern the active velocity-servo backend. The default engine change is necessary because the same stock-backend physical RED otherwise remains present in normal launch.

Nominal A passed a short 10 s closed-loop smoke with physical safety and zero sustained inward-demand stalls. Its control stream had 198 valid samples out of 201, with maximum invalid interval 0.148 s. This is a smoke test, not a long-duration nominal reliability guarantee.

## Final deployment and functional verification

`joint_limit_recovery_copied_verified_installation.json` and `joint_limit_recovery_copied_verified_check.log` are the final checks after provenance refresh. They verify all ten copied packages, M02/M07/M08/M09 contracts, all five unchanged challenge configurations, both new module origins inside the copied prefix, the actual recovery binary and installed upstream notices. Copied native and compute-only runs separately demonstrate actual Gazebo loading, not just importability.

The final normal plugin SHA-256 is `59c4a3a5481bf9cad4480b9af1307d09d1444ccb1d22a0af1f6aee99e5d0fc5e`. Every recorded final source digest matches `joint_limit_recovery_backend_verified_identity.json`. The copied binary can differ because build paths/debug information differ; its own digest and loaded path are recorded.

Build and functional regression commands:

```bash
source install/setup.bash
colcon build --symlink-install --packages-up-to robot_experiments
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python3 -m pytest src -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros \
  -k 'not copyright and not flake8 and not pep257'
python3 -W error::ResourceWarning tools/validation/joint_limit_recovery_analysis_test.py
```

The saved functional regression is **1,279 passed, 18 deselected**, plus 105 focused passes and 55 architecture passes. The validation continuity helper has three passing tests (short invalid intervals, a stale tail after one valid sample, and missing coverage); the helper is used only to prevent an incomplete validation run being promoted to success. The unfiltered suite still has the documented 12 style failures; these are not reported as passing.


## Unchanged C/D/E comparisons and sustained control

Final comparison: `results/joint_limit_recovery_verified_comparison/summary.json`. The earlier `results/joint_limit_recovery_comparison` is retained: its D/917 metrics exposed the stale-only run. Historical sources are `results/reactive_challenge_verified_report`. The same physical-FK metric and first-valid-control-through-schedule-time-40 s policy are used before/after; independent launch origins differ, so the intervals and reactive trajectories are not identical. These are descriptive comparisons, not a controlled claim of tracking improvement. E/917 has no complete historical baseline.

Values are mean / maximum. Distance is absolute radius error in metres; pointing is the configured optical-axis angular error in degrees.

| Case | Before distance | After distance (to 40 s) | Before pointing | After pointing (to 40 s) |
|---|---:|---:|---:|---:|
| C_seed42 | 0.369 / 1.272 | 0.367 / 1.720 | 52.507 / 128.627 | 44.702 / 144.921 |
| C_seed917 | 0.392 / 1.716 | 0.236 / 0.755 | 44.406 / 148.312 | 44.500 / 93.702 |
| D_seed42 | 0.415 / 1.502 | 0.340 / 0.709 | 49.346 / 137.371 | 49.792 / 99.623 |
| D_seed917 | 0.442 / 1.084 | 0.247 / 0.686 | 42.062 / 96.625 | 47.892 / 96.624 |
| E_seed42 | 0.345 / 1.141 | 0.484 / 1.786 | 48.589 / 119.341 | 54.595 / 146.387 |
| E_seed917 | unavailable | 0.490 / 1.810 | unavailable | 50.561 / 149.308 |

Full completed 90 s evaluation follows. RTF and compute time use the existing M08 evaluation window; physical-FK tracking uses the recorded active interval and can differ slightly from the M08 nearest-sample metric. Base travel is measured world-frame physical travel, not wheel odometry. Recovery event counts/durations are observed at the diagnostic sampling rate; they can miss shorter events.

| Case | RTF | Compute mean / max (ms) | Distance mean / max (m) | Pointing mean / max (deg) | Base travel (m) | Observed recovery windows (s) |
|---|---:|---:|---:|---:|---:|---|
| C_seed42 | 0.803 | 23.67 / 81.05 | 0.751 / 1.949 | 83.942 / 151.279 | 36.668 | 0.200 |
| C_seed917 | 0.815 | 24.59 / 106.67 | 0.666 / 1.975 | 80.795 / 147.913 | 36.154 | 0.100, 0.202, 0.092 |
| D_seed42 | 0.816 | 23.26 / 85.39 | 0.653 / 1.863 | 73.965 / 138.808 | 35.702 | 0.200, 0.202 |
| D_seed917 | 0.826 | 20.68 / 78.28 | 0.548 / 1.810 | 73.889 / 142.403 | 36.218 | 0.200 |
| E_seed42 | 0.813 | 23.50 / 79.71 | 0.781 / 1.971 | 87.185 / 149.413 | 36.620 | 0.188 |
| E_seed917 | 0.831 | 20.57 / 82.93 | 0.796 / 1.983 | 85.864 / 150.283 | 37.358 | 0.200, 0.202 |

All six completed repeated cases passed full-step position/speed acceptance and had no inward-demand/stationary window at the original diagnostic thresholds, including no window lasting at least one second. Every observed recovery interval released to active actuation. All four arm joints had nonzero physical excursion. This demonstrates the repaired causal chain in the tested conditions, not indefinite reliability.

| Case | Valid control samples / total | Longest invalid control interval (s) | Inactive actuation observed (s) | Fault observed (s) | Timeout counter delta | Feedback-rejection counter delta |
|---|---:|---:|---:|---:|---:|---:|
| C_seed42 | 1800 / 1800 | 0.036 | 0.200 | 0.000 | 0 | 1 |
| C_seed917 | 1795 / 1795 | 0.134 | 0.484 | 0.000 | 0 | 2 |
| D_seed42 | 1798 / 1801 | 0.168 | 0.502 | 0.000 | 0 | 0 |
| D_seed917 | 1800 / 1800 | 0.006 | 0.302 | 0.000 | 0 | 0 |
| E_seed42 | 1798 / 1798 | 0.146 | 0.292 | 0.000 | 0 | 0 |
| E_seed917 | 1800 / 1800 | 0.026 | 0.402 | 0.000 | 0 | 0 |

The added validation-only continuity check rejects runs with no valid control, an invalid-control interval of at least one second, or a source-sample gap of at least one second. This diagnostic gate is not a new runtime safety timeout and changes no controller/estimator behavior. Three focused tests verify that a single early valid sample cannot hide a stale tail. Original run artifacts remain immutable; retrospective continuity results are in `joint_limit_recovery_control_continuity.json`.

### Incomplete attempts retained, not promoted

- First E/917: readiness failed after 60 wall seconds (clock 45.868 s), no target or prediction stream. Log repeatedly reports `Physical context unavailable at t=1.854000`. Control waits for prediction and actuation holds safely. Its first fresh repeat completed 90 s with valid control.
- First D/917: clock ran for 90 s, but target truth stopped at 2.444 s and prediction at 2.424 s. The integration cursor repeatedly reports `Physical context unavailable at t=2.454000`; 1,798 of 1,800 in-window control messages reject stale prediction. There is one watchdog timeout, approximately 89.8 s inactive actuation, no PositionGuard recovery interval, and no sustained native inward-demand stall. Distance/pointing metrics are unavailable. Physical safety alone did not make this an end-to-end pass. Its fresh repeat is the D/917 row above.

There are six completed sustained-control C/D/E cases from eight launch attempts. Both incomplete attempts and their logs are excluded explicitly in the verified comparison JSON. The target-context failure is distinct from the reproduced motor-row starvation; it occurs in unchanged target/context code. Whether backend timing changes its incidence has not been established. No localization, prediction, target dynamics or startup algorithm was modified to hide it. Follow-up should establish valid physical context at numerical-target bootstrap and prevent its integration cursor falling outside available history, with reset/startup and delayed-context regressions. That is an actionable remaining reliability investigation, not part of this actuator patch.

### Timing and resource evidence

Headless RTF for the six sustained-control runs is recorded above. Historical RTF and compute metrics are preserved as `before_timing` in each comparison file; historical 40 s and new 90 s workloads/launch timing differ, so no causal performance improvement is claimed. The first six-attempt job used 3,057.21 user CPU seconds plus 321.02 system CPU seconds over 11:36.70 wall time (484% job CPU, 761,076 KiB maximum resident set, zero swaps). E/917 repeat used 511.75 + 53.04 CPU seconds over 2:07.21 wall time (443%, 778,692 KiB maximum resident set, zero swaps). These `/usr/bin/time` values include launch, recording and offline analysis; maximum RSS is the largest process peak, not simultaneous total memory. The final D repeat was not separately resource-profiled.

Recorder `streams` retain source publication gaps and callback-associated clock freshness. Many records have a source stamp ahead of the recorder callback clock; negative ages are excluded by the existing metric and are not treated as valid latency measurements. Do not read its positive-age subset as a whole-pipeline latency distribution. In contrast, sustained source-time valid control, actuation feedback validity, actual native targets/demands and full-step physical movement establish the relevant recovery freshness. Input rejection and timeout events remain inspectable and fail closed.


Source-time feedback and publication gaps for the completed cases (seconds):

| Case | Maximum valid feedback age at actuation stamp | Maximum robot-estimate source gap | Maximum prediction source gap |
|---|---:|---:|---:|
| C_seed42 | 0.020 | 0.056 | 0.092 |
| C_seed917 | 0.020 | 0.056 | 0.086 |
| D_seed42 | 0.020 | 0.050 | 0.098 |
| D_seed917 | 0.020 | 0.058 | 0.088 |
| E_seed42 | 0.020 | 0.064 | 0.102 |
| E_seed917 | 0.020 | 0.048 | 0.082 |

Feedback age uses actuation and measured-feedback source stamps from the same simulation epoch, rather than the recorder callback clock. Gaps are publication-source gaps, not end-to-end arrival latency.

## Final acceptance and remaining risks

| Requirement | Verdict | Evidence / limitation |
|---|---|---|
| Physical RED, root cause and matching inward-motion GREEN | PASS | Stock wrist and loaded shoulder remain stuck; corrected backend moves inward with the same limits, gravity, native governor and effort bound. |
| Recovery authority, guard normal and ordinary base/arm resumption | PASS | 12 real-adapter cases; no weakened watchdog, replay or ownership policy. |
| Repeated nominal/restricted wrists, both stops and loaded shoulder | PASS | 12 normal native cases, 2 copied native cases and 12 adapter cases, multiple starts/seeds. |
| Observed physical position/speed policy | PASS for tested traces | Complete native feedback, unchanged acceptance, raw excursions disclosed; not universal contact/hardware certification. |
| Significant real physical excursion still latches | CONDITIONAL | Existing serious-fault tests pass, policy unchanged; no real >0.001 rad fault deliberately induced. |
| C/D/E sustained-control repeats | CONDITIONAL overall | Six completed cases without reproduced actuator deadlock; two distinct target-context failures retained. |
| Tracking quality | CONDITIONAL | Large long-window distance/pointing errors remain; motor recovery is not a localization/controller accuracy fix. |
| Reset, zero-command compute-only, normal/copied deployment | PASS for recorded gates | Four reset checks; 29 compute-only checks per installation; ten-package builds and copied provenance/module checks. |
| Functional regression / style | PASS / existing style debt | 1,279 functional passes plus focused gates; 12 documented style failures remain. |
| Measured motor torque | UNAVAILABLE | Inherited motor impulse effort bound reviewed; recorder does not expose motor torque. |

**Final verdict: CONDITIONAL.** The physical hard-stop deadlock is resolved in the reproduced and repeated tested conditions. A PASS for every requested safety/reliability criterion is not justified because significant physical fault-injection evidence is absent and target-context synchronization is intermittent. Maintain the SDK pin and rerun these physical gates after backend/SDK changes. No further control tuning or unrelated milestone work is included.

Copied installation can be rebuilt and checked without a workspace overlay:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  bash --noprofile --norc -c '
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/joint_limit_recovery_copy_reproduce/log build \
  --build-base /tmp/joint_limit_recovery_copy_reproduce/build \
  --install-base /tmp/joint_limit_recovery_copy_reproduce/install \
  --packages-up-to robot_experiments'
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  bash --noprofile --norc -c '
source /opt/ros/jazzy/setup.bash
source /tmp/joint_limit_recovery_copy_reproduce/install/local_setup.bash
cd /tmp
python3 /home/user/ros2_ws/tools/validation/joint_limit_recovery_verify_installation.py \
  --prefix /tmp/joint_limit_recovery_copy_reproduce/install \
  --output /home/user/ros2_ws/results/joint_limit_recovery_reproduced_installation.json'
```

## Exact changes and preservation

`joint_limit_recovery_modified_files.txt` lists every task-modified pre-existing file and every task-added source, test and report separately from historical dirty-tree work. `joint_limit_recovery_preservation.json` records the 1,564-file baseline comparison, expected 11 modified pre-existing files, zero missing files, source/binary identity checks, unchanged HEAD and empty staging. Generated logs and isolated result roots are listed separately. Native actuator/governor, PositionGuard implementation, robot description/limits, hold gains and posture, control, estimation, prediction and target dynamics retain their baseline hashes.

Final comparison reproduction (substitute fresh result roots for new experiments):

```bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 python3 tools/validation/joint_limit_recovery_compare.py \
  --study \
  results/joint_limit_recovery_final_challenge/challenge_recovery_20261021T182453_142101Z_a7qyagur \
  results/joint_limit_recovery_final_challenge_retry/challenge_recovery_20261021T183754_411014Z_5hc_cy0z \
  results/joint_limit_recovery_final_D917_retry/challenge_recovery_20261021T190032_455393Z_33sul6li \
  --output results/joint_limit_recovery_reproduce_comparison
```


Final process audit: `joint_limit_recovery_final_processes.json` found no running simulation/ROS nodes, and all 60 retained cleanup records had zero survivors. No unrelated processes were stopped.
