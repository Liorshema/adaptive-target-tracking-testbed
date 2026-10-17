# M07 — Unified system configuration and component selection

2026-10-16. Baseline `421b773` on `feature/m07-unified-configuration`.
M07 extends the existing `closed_loop.launch.py`; no parallel launch stack,
experiment-runner CLI, plugin framework or new robot family was introduced.
No commit or push was made.

## Launch and examples

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robot_bringup
source install/setup.bash
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_nominal.yaml gui:=false enable_actuation:=true
```

Omit `enable_actuation:=true` for compute-only operation. All supplied valid
manifests default to disabled actuation and command publication. An explicit
manifest `runtime.enable_actuation: true` also enables actuation; it is never
inferred from a controller or scenario choice. `publish_commands:=false` retains
computation while preventing safe-command publication. Physics holding stays with
robot_simulation.

| Installed bringup resource | Actual selection |
|---|---|
| config/system_nominal.yaml | Robot EKF, target KF, IMM, residual correction, WLS Distance + Pointing, oscillatory reactive scenario |
| config/system_cv.yaml | Inherits nominal; real ConstantVelocityPredictor, no adaptation/fusion |
| config/system_dls.yaml | Inherits nominal; real JacobianController with its existing Cartesian reference/task |
| config/system_invalid.yaml | Intentionally requests standalone CA; rejected before process actions |

The DLS example preserves the existing DLS task; it does not pretend DLS implements
WLS's Distance + Pointing formulation. Both still use the independent final safety
pipeline. Existing nominal commands remain supported:

```bash
ros2 launch robot_bringup closed_loop.launch.py \
  config:=config/reactive_tracking_oscillatory.yaml \
  strategy_config:=config/whole_body.yaml enable_actuation:=true gui:=false
```

## Schema and ownership

Version 1 has required `schema_version`, `robot`, `estimation`, `prediction`,
`control`, `simulation` sections; `runtime` is optional. Unknown keys and wrong
primitive types are errors. The actual nominal manifest is:

```yaml
schema_version: 1
robot:
  type: mobile_manipulator
  description: {package: robot_description, path: config/robot/robot.yaml}
  model: {base: differential_drive, manipulator: serial_arm}
estimation:
  robot:
    type: robot_ekf
    config: {package: robot_estimation, path: config/robot_estimator.yaml}
  target:
    type: kalman
    config: {package: robot_estimation, path: config/target_estimator.yaml}
prediction:
  type: imm
  config: {package: robot_prediction, path: config/prediction.yaml}
  adaptation: residual
  fusion: weighted_correction
control:
  type: weighted_least_squares
  config: {package: robot_control, path: config/control.yaml}
  strategy: {package: robot_control, path: config/whole_body.yaml}
simulation:
  config: {package: robot_bringup, path: config/reactive_tracking_oscillatory.yaml}
runtime:
  enable_actuation: false
  gui: false
```

This is a composition manifest, not a second parameter store:

| Section/reference | Owner and responsibility |
|---|---|
| robot.description | robot_description's authoritative joints, axes, geometry, limits and mount; must match the simulator's selected description |
| robot.model | Existing model identifiers, checked against owner control configuration and physical composition |
| estimation.*.config | robot_estimation noise, initial distributions, state handling and publication settings |
| prediction.config | robot_prediction timestep/horizon, IMM, monitoring and correction parameters |
| control.config / strategy | robot_control operational parameters and existing strategy/task settings |
| simulation.config | Existing bringup profile composing simulation, environment, integrator, measurements, actuation and TF resources |
| runtime | Bringup GUI, actuation, publication and diagnostic-visualization choices |
| Parsing/resource primitives | robot_configuration, without ROS imports or robotics policies |

There are no joint angles, physical limits, gains or dynamics constants in the new
manifests. Every pre-existing owner YAML/Xacro/message asset is preserved. Existing
profile calibration remains explicit: the profile's `estimation_overrides` applies
on top of the selected owner estimator files.

## Resource resolution and inheritance

A resource is either a path string or a complete `{package, path}` mapping.
Examples use installed-package resources, without environment-dependent absolute
paths or an unimplemented `package://` syntax.

- A relative manifest resource resolves from the document **declaring that
  resource**, including inherited parent documents in other directories.
- `extends` names one parent file relative to the child. Nested mappings merge;
  lists/scalars/null replace. Cycles, malformed/non-mapping YAML and recursive
  aliases are rejected. A resource override must supply the complete resource
  value; partial `{path: ...}` package references are not supported.
- Package references resolve through an injected package-share lookup. Absolute
  package paths and lexical traversal outside the package share are rejected.
  Normal colcon installed-resource symlinks are followed; the package-share check
  is not a filesystem sandbox. Both symlink and copied installs are verified.
- Existing scenario/profile resource strings retain their owner's package/resource
  base conventions. The manifest does not redefine those existing schemas.
- Owner estimator/control ROS YAML is resolved and validated before launch, then
  delivered as typed parameter mappings. ROS's raw YAML parser is not asked to
  understand `extends`. DOUBLE scalars/arrays are normalized to floats without
  changing numeric values. Derived command joint names resolve the empty owner
  placeholder before ROS launch tries to infer its array type.
- Control strategies and prediction configuration use the same inheritance-aware
  loader in preflight and runtime. Actuator configuration loading does too.

## Override precedence

1. Explicit **nonempty** existing launch arguments override manifest runtime or
   resource choices. `false` is explicit and wins over a manifest `true`.
2. The manifest selects owner files and algorithms. Its prediction/estimator
   identifiers override corresponding owner/default selections.
3. Within owner resources, child YAML overrides parent YAML. Existing profile
   estimator calibration overrides selected estimator parameters.
4. Deployment metadata is derived from the authoritative description and simulator
   configuration. Conflicting configured joints, order, DOF, frames or physical
   description references fail validation; they are not silently repaired.
5. With no manifest, legacy defaults and owner files retain their existing behavior.

`config` overrides the existing bringup profile; `strategy_config` overrides the
strategy. `strategy_config:=none` explicitly selects the owner control file's DLS
configuration. An empty strategy argument means no override under a manifest and
retains the legacy DLS default without one. `lookahead` and `feedforward` keep their
existing meanings. `gui`, `enable_actuation`, `publish_commands`,
`visualize_tracking`, `publish_ground_truth` support explicit boolean overrides.
The existing launch defaults use empty sentinels to distinguish absence from an
explicit false; effective legacy GUI/actuation defaults remain false.

`publish_commands=true` with disabled actuation is rejected. Visualization with
explicitly disabled physical ground truth is rejected. Optional publication and
watchdog settings are checked before process creation, not only by running nodes.

## Genuine component selection and compatibility

| Domain | Supported deployed choices | Deliberately unsupported |
|---|---|---|
| Robot | mobile_manipulator; differential_drive + serial_arm, current four-joint description | Other robot families, incompatible DOF/layouts/mounts |
| Robot estimation | robot_ekf → RobotStateEstimator with existing nonlinear process/measurement models | A nominally similar KF substituted for the nonlinear robot contract |
| Target estimation | kalman → TargetStateEstimator, 3D [p,v] | Other dimensions or unsupported filter/sensor combinations |
| Prediction | imm → IMMPredictor; constant_velocity → ConstantVelocityPredictor | Standalone CA requires [p,v,a]; CT requires turn-rate state; learned adapters are not deployed choices |
| Adaptation/fusion | residual + weighted_correction; none + none | RLS/learned/AdaptiveFusion library components are not automatically wired selections |
| Control | jacobian → JacobianController; weighted_least_squares → WeightedLeastSquaresController | Unknown solvers or incompatible task/model dimensions |
| Simulation | Existing fight_flight / saturated_fight_flight registrations, linear/sinusoidal fields, rk4, existing compatible profiles | New state dimensions, unregistered dynamics/integrators, ignored stochastic-disturbance configurations |

CV preserves [position, velocity], finite outputs, absolute timestamps and the
source epoch. It is a distinct implementation; its selection survives clock reset.
IMM retains its existing internal CV/CA/CT ensemble. Residual correction and fusion
objects are absent when disabled. Factory defaults remain IMM plus residual
correction for legacy callers.

Compatibility preflight invokes the **actual owner factories**, rather than merely
reading YAML keys. It validates estimator distributions/state sizes; authoritative
joint names/order/count; physical description agreement; model/adapter/controller
construction; independent safety; publication/freshness/rate budgets; prediction
horizon coverage of control lookahead; simulation registry selection, integration
and observation shapes; actuation reference/guard initialization and watchdogs.
The six-state PointStamped target observation contract remains explicit.

Runtime nodes log constructed component types. The unified launch also logs resolved
manifest/profile/description/prediction paths and preflight identities. A preflight
log is not treated as proof of runtime activity; live evidence includes stamped
state, trajectory, control, actuator, clock and physical feedback measurements.

Errors include the manifest/resource path and the offending field or value. For
example `system_invalid.yaml` reports `prediction.type='constant_acceleration'`,
supported predictors `imm, constant_velocity`, and the unavailable richer input
contract. A joint-order mismatch identifies `estimation.joint_names`; invalid
publication/freshness settings name their parameter; unknown fields identify their
owning configuration. No invalid selection is silently routed to IMM or DLS.

## Where to extend

- Generic loading, resource resolution and primitive validation remain in
  robot_configuration; pass a share lookup into pure composition code.
- Estimators register supported contracts in the existing
  `robot_estimation/factory/estimator_factory.py`. Owner parameter types live in
  `parameter_schema.py`, shared with ROS declaration.
- Predictors are constructed by `robot_prediction/factory/system_factory.py`.
  Add a real compatible builder/identifier and trajectory/selection/reset tests;
  richer internal models need an explicit tested [p,v] adapter before exposure.
- Controllers use existing `factory/project_components.py` registries and
  `ControlFactory`; strategies reference existing owner configuration. Extend the
  Controller contract and registry without bypassing final safety.
- Dynamics/fields/integrators use `SimulationFactory.with_defaults()` and its
  existing registries. Select them in referenced simulation YAML; deployment
  validation lives in the simulation factory layer, not the simulation node.
- Pure actuator startup construction lives in
  `robot_simulation/factory/actuation_factory.py`, reused by preflight and the ROS
  adapter. PositionReference/PositionGuard algorithms and all PID settings remain
  unchanged. Session/replay protection, watchdog execution and clock recovery stay
  in their existing runtime owners.

Bringup's `factory/system_configuration.py` orchestrates these contracts. It does
not replace their construction logic. New supported selections require owner
contract tests, composition tests and installed/live verification where applicable.
M08 experiment execution, M09 evaluation infrastructure and M10 dashboards remain
out of scope.

## Verification commands

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src -q --import-mode=importlib -p no:launch_testing -p no:launch_ros \
  -k 'not flake8 and not pep257 and not copyright'
python3 -m pytest src/robot_bringup/test/test_architecture.py \
  src/robot_bringup/test/test_architecture_tree.py \
  src/robot_bringup/test/test_system_configuration.py src/robot_configuration/test \
  src/robot_prediction/test/test_component_selection.py \
  src/robot_simulation/test/test_deployment_configuration.py \
  src/robot_simulation/test/test_actuation_factory.py \
  -q --import-mode=importlib -p no:launch_testing -p no:launch_ros
colcon list --base-paths src
colcon build --symlink-install --packages-up-to robot_bringup
```

Fresh copied installation, checked from `/tmp` without workspace Python/ament paths:

```bash
colcon --log-base /tmp/m07_copy/log build --base-paths src \
  --build-base /tmp/m07_copy/build --install-base /tmp/m07_copy/install \
  --merge-install --packages-up-to robot_bringup
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  bash --noprofile --norc -c '
    source /opt/ros/jazzy/setup.bash
    source /tmp/m07_copy/install/local_setup.bash
    cd /tmp
    python3 /home/user/ros2_ws/docs/validation/m07_verify_installation.py \
      --prefix /tmp/m07_copy/install --output /tmp/m07_install_repeat.json'
```

Bounded live validation; use fresh output directories to preserve prior evidence:

```bash
ROS_DOMAIN_ID=152 python3 docs/validation/v2_tracking_gazebo.py \
  --case reactive --system-config config/system_nominal.yaml \
  --physical-ground-truth --without-markers --shoulder-posture \
  --duration 12 --settling-time 4 --wall-timeout 100 --output results/m07_nominal_repeat
# Repeat with system_cv.yaml for actual alternate prediction.
# Legacy reference: omit --system-config and add
# --config config/reactive_tracking_oscillatory.yaml --strategy-config config/whole_body.yaml.
```

The existing observer retains physical-speed acceptance failures and nonzero exit
status. M07 does not relax criteria to make a run green. Raw streams remain in
ignored `results/`; compact evidence and reusable checks live in `docs/validation/`.
Historical local evidence may be untracked; `git ls-files` distinguishes files
shipped by a commit from local generated captures.

## Final verification and verdict

**M07: CONDITIONAL.** Supported configuration assembly, genuine selection,
backward compatibility, architecture and installation checks pass. All three live
deployments produced active closed-loop output. Physical velocity safety remains
CONDITIONAL under the existing M02 limitation: the unchanged observer returned
exit 1 for measured physical joint overspeed in every run. This is not a
real-hardware safety approval. M07 is complete within its scope; no M08 work began.

| Verification | Actual result | Evidence under docs/validation/ |
|---|---|---|
| Full functional regression | 986 passed; 18 lint/copyright checks deselected | m07_verified_regression.log |
| Focused configuration/selection/architecture | 128 passed, including all 52 architecture tests | m07_completed_focused.log |
| All-nine-package symlink build | 9 finished, exit 0 | m07_completed_build.log |
| All-nine-package copied install | 9 finished, exit 0 | m07_completed_copy_build.log |
| Outside-checkout installation | 7 console entries, 68 resources, nominal/CV/DLS actual identities and invalid example verified | m07_completed_installed_checks.json |
| Real invalid ROS launch | exit 1; CA contract error lists supported choices; no process started | m07_negative_launch.json / .log |
| Unified, legacy and actual CV Gazebo | All produced state, prediction, safe commands and actuator feedback; only measured-speed gate failed | m07_runtime_evidence.json |
| Preservation | 803 preexisting untracked paths retained; 74 owner assets/core algorithms unchanged | m07_preservation.json |

The 18 deselections are existing style/copyright test entry points, not functional
tests. Repository-wide lint success is not claimed. The review findings and fixes
are recorded in [m07_review.md](validation/m07_review.md); the independent reviewer
was interrupted by its usage limit before a final follow-up, so no clean final
independent-review verdict is claimed.

### Live measurements

Runs used the normal installed `closed_loop.launch.py`, sequentially, headless,
with explicit actuation and physical ground truth. No temporary actuator plugin,
alternative launch implementation or tracing fixture was used. Each ran until
12 simulated seconds after target initialization; metrics use the final 8 seconds
after 4 seconds settling. Distance/pointing use Gazebo physical base pose plus
measured joints, the authoritative mount and configured optical axis. These are
short integration checks under varying host load, not controlled performance
comparisons or evidence that one algorithm is better.

| Deployment | Evaluated samples | Mean/max distance error (m) | Mean/max pointing error (degrees) | Mean RTF | Mean/p95 controller compute (ms) |
|---|---:|---:|---:|---:|---:|
| Unified nominal: IMM/residual/WLS | 159 | 0.1193 / 0.3130 | 6.987 / 17.527 | 0.924 | 9.707 / 20.427 |
| Legacy scenario/strategy arguments | 159 | 0.0923 / 0.2864 | 7.489 / 14.144 | 0.991 | 5.930 / 9.661 |
| Unified alternate: CV/none/WLS | 160 | 0.0938 / 0.3294 | 9.602 / 16.268 | 0.944 | 9.509 / 17.246 |

All node logs identify RobotStateEstimator, TargetStateEstimator,
WeightedLeastSquaresController, WholeBodyComposer, SafeControlPipeline,
FightFlightDynamics and RK4Integrator. Nominal and legacy actually construct
IMMPredictor, ResidualAdapter and WeightedCorrection. The alternate node log
actually identifies ConstantVelocityPredictor and `NoneType` adaptation/fusion.
The DLS example constructs JacobianController in both source contract tests and
the copied installation; an additional live DLS run was not performed in M07.

| Deployment | Robot estimate / target estimate / prediction received | Safe command / actuation messages | Maximum measured physical joint speeds, authoritative joint order (rad/s) |
|---|---:|---:|---|
| Unified nominal | 628 / 206 / 206 | 298 / 124 | [0.233, 0.966, 1.938, 2.143] |
| Legacy | 664 / 204 / 204 | 319 / 121 | [0.314, 0.529, 1.000, 1.578] |
| Unified CV | 665 / 203 / 203 | 321 / 123 | [0.253, 0.636, 0.771, 1.531] |

Counts include startup; they are not steady-state publication frequencies.
Target-estimate/prediction counts come from the observer's received-message
counters. Recorded robot means/covariances and commands are finite. Prediction
finite six-state/absolute-timestamp contracts are separately exercised by unit
and installed preflight tests; valid live controller output demonstrates their
integration, without claiming a complete raw prediction capture.

The joint order is base_yaw, shoulder, elbow, wrist, with unchanged physical speed
limits [1.0, 1.0, 1.0, 1.5] rad/s and numerical tolerance 0.001. Across all received
joint positions, maximum position violation is **0.0 rad in all three runs**.
Measured speed peaks use all received joint-state messages after settling, not
only sparse actuator telemetry. Elbow/wrist overspeed in nominal and wrist
overspeed in legacy/CV remain failures. Reference limiting does not prove physical
velocity enforcement.

Actuation is active for every evaluated telemetry sample; watchdog increments
after settling are zero, recorded safe velocities respect bounds, all evaluated
control messages are valid, and feedback/command rejections are zero. Startup
contains waiting/future-stamp control inhibition and watchdog inactivity; lifetime
timeout counters are 55/73/64 and must not be confused with the zero post-settling
increments. No startup data were removed from the raw artifacts.
All runs show nonzero base and four-joint excursions, live TF tracking motion,
nonnegative first-link elevation and positive first-link/base clearance. Minimum
shoulder feedback is 0.000640 rad (nominal), 0.28635 rad (legacy), and 0.06235 rad
(CV). These limited
posture checks do not constitute complete self-collision avoidance.

Raw evidence directories are `results/m07_unified_verified`,
`results/m07_legacy_verified`, and `results/m07_cv_verified`. Each contains the
exact launch command in result.json, runtime.log with actual identities,
configuration_snapshot.yaml, generated_world.sdf, experiment_streams.json,
samples.jsonl, physical pose/posture captures and RTF/CPU samples. The compact
[runtime evidence](validation/m07_runtime_evidence.json) records evaluation windows,
component identities, stream counts, failed gates and measured values. Earlier
failed attempts in `results/m07_unified` and `results/m07_unified_final` are
preserved; they are not counted as completed simulation evidence.

### Corrections completed during resumption

Completed strict validation of owner runtime rates/freshness, simulation resource
schemas, prediction horizon coverage and actuator startup contracts. Extracted
actuator startup construction into its existing owner's factory layer so preflight
and runtime apply the same rules. Merged inherited owner ROS parameters are sent as
typed dictionaries; the runtime strategy loader also honors inheritance. Resolved
two live-startup defects: observer snapshots now serialize resolved paths, and
package traversal validation now permits legitimate colcon-installed resource
symlinks while still rejecting lexical package escapes. Regression tests cover
these corrections. No physical bounds, PID gains, numerical dynamics, filter
tuning or controller equations were changed.

### Remaining limits and working-tree status

- Only the supported deployed mobile manipulator, robot EKF and target KF are
  exposed. CA/CT standalone, learned predictors and arbitrary robot/state families
  remain explicitly unsupported rather than silently substituted.
- Existing skid-turn localization drift, covariance inconsistency and physical
  overspeed remain research-testbed limitations. These short runs do not establish
  long-duration reliability or comprehensive collision/hardware safety.
- No changes were committed or pushed. HEAD remains M06.1 baseline `421b773` on
  `feature/m07-unified-configuration`; M07 remains reviewable in the working tree.
  Historical experiment evidence remains untouched. Validation processes exited;
  no ROS/Gazebo test process remained in the final process check.

The exact tracked modifications, new source/resources/tests/docs, generated logs
and raw artifact paths are listed in
[m07_modified_files.txt](validation/m07_modified_files.txt). The captured Git status
and diff summary are [m07_git_status_short.txt](validation/m07_git_status_short.txt)
and [m07_git_diff_stat.txt](validation/m07_git_diff_stat.txt).
