# M06 — Target architecture and proposed migration (Phase 1)

> Historical M06 state. For current canonical paths and ownership, read
> [ARCHITECTURE.md](ARCHITECTURE.md) and the
> [M06.1 verification report](m06_1_architecture_hardening.md).

2026-10-15 design, approved and implemented 2026-10-16.
See [structural consolidation results](m06_structural_consolidation.md) for actual
verification and [the final tree](validation/m06_final_tree.txt). The proposed tree
below is retained as the approved design record. Implementation additionally added
missing `robot_control/factory/__init__.py` and `pipeline/__init__.py` package markers
after the copied-install check exposed their omission, plus direct `launch` and
`launch_ros` dependencies for description display. Algorithms and YAML values
remain unchanged. The narrow configuration package was included as approved.
Read with [`m06_architecture_audit.md`](m06_architecture_audit.md), which distinguishes
confirmed defects, intentional structure and dormant alternatives. Known M02–M05
CONDITIONAL verdicts remain unchanged.

## Decision and alternatives

**Recommend a focused compatibility migration**, followed by M07 configuration
composition and M08 experiment execution. Keep mathematical algorithms, safety,
physical/configuration values, resource paths and scenario behavior fixed.

Alternatives considered:

1. **Nodes-only cleanup:** smallest immediate diff, but leaves prediction factory
   discovery, shared-loader ownership and undeclared dependencies unresolved.
2. **Focused migration (recommended):** standardize six node locations and two
   assembly locations, name prediction configuration clearly, extract the existing
   YAML inheritance implementation once, fix packaging metadata and archive three
   local backups. Small compatibility shims reduce breakage; no algorithm rewrite.
3. **Universal component/plugin platform:** consolidate all registries/contracts,
   generalize robot state, collapse pipelines and rewrite launch around new schemas.
   This lacks a demonstrated MVP need, increases regression risk and would consume
   M07/M08 scope. Reject for the next prompt.

The only proposed new Phase-2 package is `robot_configuration`: a small pure-Python
owner of the **already shared** YAML mapping/inheritance algorithm. It has genuine
bringup and simulation consumers and a future experiment consumer. It does not own
robot geometry, global registries, ROS parameter semantics or experiment policy.
This extraction removes bringup's dependency on a simulation implementation solely
for parsing. Bringup still legitimately launches simulation. If approved scope must
be smaller, defer this extraction and retain the existing loader unchanged; do not
copy it into bringup as a second implementation.

## Architectural conventions

- ROS `Node` adapters live in `<python_package>/nodes/<existing_basename>.py`.
  `main` and ROS console executable names remain stable. Node files translate
  transport/configuration, orchestrate, validate and publish; numerical algorithms
  remain importable without a ROS runtime.
- Use `interfaces/` for domain contracts only when needed; `adapters/` translate
  representations; `factory/` assembles registered/injected components. Keep the
  singular `factory/` already used by control and simulation. Domain factories such
  as `robot_models/whole_body/factory.py` remain beside their domain.
- Python `configuration/` is for package-specific validated configuration adapters.
  External YAML stays at package-root `config/`, ROS launch at `launch/`, tests at
  `test/`. Avoid a Python `config/` folder that can be confused with installed YAML.
- Preserve estimation `filters`, `robot`, `target`, `common`; prediction `baseline`,
  `fusion`, `residuals`, `confidence`, `history`, `online_adaptation`; control
  `tracking`, `optimization`, `safety`, `pipeline`; simulation `gazebo`, `dynamics`,
  `disturbances`, `integration`, `engine`, `state`, `measurements`.
- `robot_models/common` remains SO(3)/SE(3) numerical utilities. Estimation common
  remains filter data/timing. No cross-project `utils.py` or universal common bin.
- Keep explicit registrations at project assembly boundaries. Control and simulation
  registries retain their different public APIs; no new registry framework is needed.
- An absent implementation is unsupported. Empty namespaces do not prove aerial or
  omnidirectional support. A profile validator must reject rather than substitute.
- Runtime artifacts go to `results/<run_id>/` for future runs; builds/install/log
  stay outside `src`. Historical `docs/validation` evidence and archived sources are
  immutable. An eventual dashboard reads results; it does not become an actuator.

## Target package boundaries

Keep the eight package responsibilities in the audit. Add only:

```text
robot_configuration  → Python stdlib + PyYAML (no project algorithm/ROS dependencies)
robot_simulation    → robot_configuration (mapping inheritance)
robot_bringup       → robot_configuration (mapping inheritance)
```

Description keeps its physical schema/normalization. Control keeps project-to-safety
mapping. Prediction keeps section validation. A generic YAML library must not import
these consumers, look up ROS resources implicitly, or instantiate their components.
Package/file resolution at the composition boundary remains explicit; M07 can add a
resource-reference adapter without changing the verified inheritance algorithm.

### Package-specific exceptions

| Package | Exception and reason |
|---|---|
| robot_interfaces | CMake `msg/`, CMakeLists/package.xml; no Python nodes/interfaces mirror |
| robot_description | Xacro, URDF, RViz, parser and display launch are its real contents; no dummy algorithm factory |
| robot_models | Pure domain interfaces/models; whole-body factory stays under whole_body; no nodes/config loader |
| robot_estimation | No forced new factory in the mechanical migration; current private node builders remain until M07 needs selectable supported estimator kinds |
| robot_prediction | Move factory and typed config wrapper; preserve baseline/fusion/adaptation algorithms and optional learner interface |
| robot_control | Keep all four production pipeline layers; the two state adapters separate ROS parsing from numerical state layout |
| robot_simulation | Keep Gazebo adapter classes and pure actuator guard/reference separate from the relocated ROS actuation node |
| robot_bringup | Deployment model assembly is legitimate here; source description schema and model composition stay in their owners |

## Proposed tree overview

`=` retains a real existing subtree; `→` relocates a file; `+` is a proposed new
nonempty implementation directory/package. Existing data resource paths stay fixed.
The exhaustive per-file tree at the end includes all 422 current nonignored files,
compatibility shims and proposed additions; no implicit deletion is hidden by this
overview.

```text
ros2_ws/
├── README.md                         # actual versus planned capabilities
├── docs/
│   ├── m06_architecture_audit.md
│   ├── m06_target_architecture.md
│   └── validation/                  # = historical captures, probes, archived sources
├── results/                         # future runtime outputs; gitignored
│   └── legacy_backups/              # proposed preserved .bak archive
├── src/
│   ├── robot_configuration/         # + narrow extraction, not a new config system
│   │   ├── robot_configuration/{__init__.py,yaml_loader.py}
│   │   ├── test/test_yaml_loader.py
│   │   └── package.xml, setup.py, setup.cfg, resource/robot_configuration
│   ├── robot_bringup/
│   │   ├── config/, launch/, test/  # =
│   │   └── robot_bringup/
│   │       ├── nodes/{tracking_feedback_tf.py,tracking_visualization.py}  # →
│   │       └── factory/model_factory.py                               # →
│   ├── robot_control/
│   │   ├── config/, test/           # =
│   │   └── robot_control/
│   │       ├── nodes/control_node.py  # → from ros/
│   │       └── adapters/, baseline/, factory/, interfaces/, optimization/,
│   │           pipeline/, safety/, tracking/                           # =
│   ├── robot_description/           # = config/, urdf/, launch/, parser, tests
│   ├── robot_estimation/
│   │   ├── config/, test/           # =
│   │   └── robot_estimation/
│   │       ├── nodes/estimation_node.py                                 # →
│   │       └── common/, filters/, robot/, target/                       # =
│   ├── robot_interfaces/            # = CMake message package
│   ├── robot_models/                # = all mathematical domain folders and tests
│   ├── robot_prediction/
│   │   ├── config/, test/           # =
│   │   └── robot_prediction/
│   │       ├── nodes/prediction_node.py                                 # →
│   │       ├── factory/system_factory.py                               # →
│   │       ├── configuration/prediction_config.py                       # → config_loader
│   │       └── baseline/, confidence/, fusion/, history/, interfaces/,
│   │           online_adaptation/, residuals/, trajectory_prediction.py # =
│   └── robot_simulation/
│       ├── config/, launch/, models/, worlds/, test/                    # = assets
│       └── robot_simulation/
│           ├── nodes/{simulation_node.py,velocity_actuation.py}         # first =, second →
│           └── actuation/{position_guard.py,position_reference.py},
│               dynamics/, disturbances/, engine/, factory/, gazebo/,
│               geometry/, integration/, interfaces/, measurements/, state/  # =
└── build/, install/, log/            # local generated workspace directories
```

Short-lived compatibility modules remain at former public import paths; they
contain no duplicate implementation. No future experiment/dashboard package is
created in Phase 2. `robot_experiments` may become a real nonempty package in M08;
a separate Plotly application is a future results consumer. These are roadmap
boundaries, not empty folders to add now.

## Exact path migration map

All paths below are relative to `src/`. Each old public Python path is retained as
a compatibility module for one migration period, unless a later explicit API
deprecation is approved. New internal callers/tests use canonical paths.

| Current path | Proposed canonical target | Related changes |
|---|---|---|
| robot_control/robot_control/ros/control_node.py | robot_control/robot_control/nodes/control_node.py | setup entry point, node tests, compute-only/safety/profile wrappers |
| robot_estimation/robot_estimation/estimation_node.py | robot_estimation/robot_estimation/nodes/estimation_node.py | setup, node tests, estimator_timing_probe canonical module |
| robot_prediction/robot_prediction/prediction_node.py | robot_prediction/robot_prediction/nodes/prediction_node.py | setup, prediction tests, control-compute validation imports |
| robot_simulation/robot_simulation/actuation/velocity_actuation.py | robot_simulation/robot_simulation/nodes/velocity_actuation.py | setup, actuator tests, actuation trace wrapper; pure guard/reference stay |
| robot_bringup/robot_bringup/tracking_feedback_tf.py | robot_bringup/robot_bringup/nodes/tracking_feedback_tf.py | setup, visualization import, feedback tests |
| robot_bringup/robot_bringup/tracking_visualization.py | robot_bringup/robot_bringup/nodes/tracking_visualization.py | setup and marker tests |
| robot_bringup/robot_bringup/model_factory.py | robot_bringup/robot_bringup/factory/model_factory.py | TF/markers/model tests and validation observer imports |
| robot_prediction/robot_prediction/system_factory.py | robot_prediction/robot_prediction/factory/system_factory.py | prediction node and any factory callers |
| robot_prediction/robot_prediction/config_loader.py | robot_prediction/robot_prediction/configuration/prediction_config.py | factory/node; retain PredictionConfig API and section semantics |
| robot_simulation/robot_simulation/factory/config_loader.py | robot_configuration/robot_configuration/yaml_loader.py | extract unchanged load_yaml implementation; old module delegates; launch/markers/simulation new imports |
| robot_control/robot_control/ros/control_node.py.bak | ../results/legacy_backups/robot_control/control_node.py.bak | preserve original bytes; no import path |
| robot_control/robot_control/ros/control_node.py.before_tf.bak | ../results/legacy_backups/robot_control/control_node.py.before_tf.bak | preserve original bytes; no import path |
| robot_simulation/config/scenarios/environment_oscillatory_aggressive.yaml.bak | ../results/legacy_backups/robot_simulation/environment_oscillatory_aggressive.yaml.bak | preserve bytes; exclude backups from resource installer |

New `__init__.py` files are created only in populated nodes/factory/configuration
directories. Existing `robot_control/ros/__init__.py` stays while its compatibility
module exists. No configuration YAML moves, renames or new inheritance semantics
are included. No file needs an aesthetic rename beyond naming the moved typed
prediction configuration wrapper clearly.

### What breaks if moves are done naively

1. `setup.py` console-script targets; preserve executable names and update only
   module paths. Launch references to executables should remain unchanged.
2. Imports in nodes/factories/tests and source-based validation; search exact old
   module tokens, then review each match. Do not replace substring occurrences in
   archived evidence or raw logs.
3. Monkeypatching/probes: `estimator_timing_probe.py` executes archived source in
   module globals; control/actuation probes wrap class methods and launch source
   prefixes. A `from new_module import NodeClass` shim leaves different module
   globals and can make an archived-baseline test accidentally run the new class.
   Update probes to canonical modules; verify class/module identity where patched.
4. Legacy root imports: compatibility must forward to the same canonical module
   when global patching is supported, not load a second Node implementation. Test
   both public import paths and the installed console entry point. Do not promise
   untested `python -m <legacy_path>` behavior.
5. Relative imports after moving PredictionConfig/SystemFactory; review them rather
   than adding a sys.path workaround.
6. Metadata: new configuration package needs ament resource/setup/build type,
   explicit YAML dependency and declared consumers. Do not create a project-package
   cycle. Add missing direct dependencies listed in audit A03; align models version
   and existing Python buildtool declarations.
7. Package-share installs: current flat globs and simulation recursive installers
   have different coverage. Preserve YAML relative `extends` paths and Xacro
   relative includes. New Python nodes are found by find_packages only with proper
   initializers. Exclude caches/backups in data resources without dropping assets.
8. Documentation links and validation source anchors; update active instructions
   and wrappers, but preserve historical evidence and before-source fixtures.

## Configuration ownership and M07/M08 seams

Keep **one authority per value**, allowing explicit deployment overrides rather
than conflating every difference with duplication:

- Robot description owns joint limits, masses/collisions/axes and tracking mount.
  Runtime component specs derive from it. Named end-frame and joint-order capability
  checks must guard adapters that interpret positional arrays.
- Simulation owns world/contact/physics, target and fields, synthetic measurement
  noise/seed, Gazebo drive deployment and PID/recovery policy. Base command envelope
  currently lives in control YAML and is passed explicitly to actuation; retain this
  for compatibility, and make any later ownership transition explicit.
- Estimation/prediction/control own their algorithm configurations. Bringup owns
  deployment references, ROS wiring, effective ROS parameter order and opt-in
  actuation. Experiment YAML composes these resources and overrides declared fields;
  it must not redefine safety geometry opportunistically.
- `robot_configuration.load_yaml` remains a mapping loader; ROS parameter documents,
  strategy schemas and prediction sections are validated by their owners. Unknown
  component kinds/keys must not be silently ignored or substituted.

### M07: minimal unified composition

1. Introduce a bringup-owned resolved-system specification containing **references**
   to robot physical/deployment, world/scenario, estimator, predictor/adapter,
   strategy and actuator YAML. Reuse existing files and loader. Preserve current
   CLI flags as explicit, recorded overrides.
2. Expose `PredictionNode` configuration resource via a parameter; add explicit
   supported predictor/residual selections at its factory. Existing IMM+residual
   stays the compatibility default. Expose existing CV/CA/CT only with correct
   constructor/update tests. An RLSAdapter is not already a compatible residual
   policy; require an adapter or reject selection until implemented.
3. Give current estimator profiles a truthful `planar_mobile_manipulator_ekf` /
   `target_cv_kf` identity or equivalent named capability; reject unsupported types.
   Add a new abstraction only when a second real implementation requires it.
4. Validate before launching: description/URDF topology, mount parent, joint names
   and order, 5+2n versus consumer layout, 3D target for distance-pointing, fixed
   inertial frame relation, state/trajectory dimensions, holding enabled when
   actuating, compatible rates/freshness, registered kinds, unique output owners,
   paths/schema keys and physical bounds. Report field path and allowed values.
5. Resolve parameter precedence explicitly: owner defaults → referenced deployment
   overlay → allowed experiment overlay → CLI override, with immutable physical
   limits in this MVP. Match existing precedence for legacy launches. Record the
   original files, resolved values, hashes and implementation identities.
6. Preserve reactive interaction's current wheel-odom-derived tracking feedback and
   label it. A physical-feedback variant must be an explicit new scenario with
   comparison evidence, not an incidental config cleanup.

### M08: compose experiments, do not duplicate algorithms

A future `robot_experiments` package should own validation of experiment duration,
settling policy, seed, outputs/run ID, lifecycle and metrics collection, referencing
the M07 system specification. The seed must be propagated to each supported
stochastic component (currently measurement RNG is separately configured) and
document unsupported nondeterminism; do not promise bit-identical Gazebo execution.
No second inheritance implementation, hard-coded scenario if-chain, arbitrary
dynamic Python import, or embedded controller algorithm is needed.

Use a run folder containing original/resolved configuration, version/source hashes,
seed mapping, launch command/domain/partition, raw data, timing source/frame/layout
metadata, physical and valid-only tracking metrics, inactive intervals, acceptance
gates with reasons, return code and resource samples. A failed gate or interrupted
run is a recorded outcome, not a missing row. Prefer existing collectors initially;
remove fragile launch-text rewriting only when the runner has equivalent supported
instrumentation hooks. Historical fixtures remain available.

### Future Plotly dashboard

Read the M08 run manifest/metrics/raw data through a versioned results schema.
Display units, frames, observation source, settling/inactive policy, failed gates,
missing measurements and milestone limitations. Keep plotting/UI dependencies out
of model/estimator/predictor/control/simulation cores. No direct motor-topic publish
or hidden parameter edits from the UI; future experiment requests go through the
runner's explicit lifecycle. No dashboard package/code is part of Phase 2.

## Extension walkthroughs

### A. Add a constant-jerk predictor

- Contract: `Predictor.predict(PredictionInput) -> TrajectoryPrediction`, common
  `[p,v]` output, absolute sample times and requested dimension/horizon. Internal
  `[p,v,a,j]` may be richer; do not change consumers' six-state wire contract.
- Create `robot_prediction/baseline/constant_jerk.py` and focused predictor tests for
  analytic jerk trajectories, dimensions/timestamps, reset and observed-time updates.
- Current required changes: PredictionSystemFactory currently fixes IMM, so exposing
  a standalone new predictor needs factory wiring/config selection and node config
  reachability. Extending IMM's fixed CV/CA/CT ensemble additionally changes its
  model lists/probability dimensions and needs separate tests; not automatic.
- After M07: add one explicit factory registration/spec and a predictor YAML;
  maintain tests for constructor parameters and observation update capability.
  No control, actuation, robot description or simulation edits should be necessary.

### B. Add an MPC strategy

- Contract: `Controller.compute(ControlInput, RobotModel, ModelData) -> ControlOutput`
  in `[v,omega,dq]` for the currently supported robot. Return a finite vector with
  the model's dimension. Final SafeControlPipeline remains mandatory.
- Create `robot_control/optimization/mpc_controller.py`, its numerical/feasibility
  tests and a strategy YAML. Register in `factory/project_components.py`; declare
  any solver dependency and validate supported task/reference configuration.
- If MPC needs horizon/model information absent from ModelData, extend its own
  injected model adapter or a minimal explicit contract with consumer tests; do
  not reach into the ROS node or simulation implementation. Existing scalar-task
  factory injection is not a promise that every MPC constructor already fits.
- Test finite/failed/infeasible outputs, authoritative bounds, warm-start/reset,
  elapsed dt, independent final projection and compute-only behavior. No PID or
  target-dynamics changes. No MPC implementation in M06.

### C. Add a quadrotor

- This is **not supported by swapping YAML today**. RobotModel assumes base plus
  manipulator; description parsing expects wheels and serial links; robot EKF,
  state adapters, planar concrete type checks, command layout and DiffDrive/PID
  actuator all encode the current robot family.
- A real implementation needs an aerial physical/deployment description, base/full
  robot model and matching estimator/state-layout adapter, kinematic/control model,
  and actuator transport/safety policy (rotor thrust/rates or a documented flight
  interface). Use demonstrated contracts to design extension interfaces; avoid a
  dummy manipulator solely to satisfy current type checks.
- Files would be added in their owners: description assets, models/base/aerial,
  estimation/robot models, control adapter/factory registration, simulation actuator
  and deployment, then bringup robot profile. Necessary interface changes across
  these packages are legitimate robot-family work, not independent algorithm edits.
- Tests: SO(3)/state/frame conventions, aerial Jacobians/dynamics, actuator bounds,
  gravity/startup/reset, state-layout correspondence and a dedicated physical smoke.
  Target prediction/FightFlight math should not change. Reject this selection until
  these implementations exist. Keep the MVP honest about its current robot family.

### D. Add bounded nonlinear target dynamics

- Contract: `robot_simulation.interfaces.dynamics.Dynamics.derivative(state,
  control, time, context) -> np.ndarray`, as implemented by FightFlightDynamics.
  The engine supplies the target `[p,v]` vector and `context['tracking_position_world']`;
  return its derivative, not a SimulationState object. Reuse field/integrator contracts.
- Create `dynamics/target/<name>.py`, parameter validation, a target YAML and tests.
  Register one constructor in SimulationFactory defaults; choose it via scenario
  references. No new scenario branch belongs in SimulationNode.
- Test bounded acceleration, units, equilibrium/extremes, RK4 step convergence,
  live tracking-position response and invalid parameters. The measurement publisher,
  estimator/predictor/control and Gazebo target pose adapter should not change.
- If a model needs a different state representation, declare that incompatibility;
  the present TargetState contract is position/velocity, not arbitrary latent state.

### E. Define an experiment from existing components

- Today: create a bringup scenario child using `extends`, reference simulation
  target/environment YAML and an existing strategy; use the validation CLI for
  duration/output. Predictor/adapter selection is not fully YAML-reachable yet.
- After M07/M08: create one experiment YAML referring to a supported system profile
  and component files, plus duration/settling/seed and allowed overrides. No Python
  algorithm/launch branch should change for a supported combination.
- Add schema/resolution tests for precedence, relative extends, list replacement,
  unsupported keys/kinds/robot-task combinations, missing resources and seed mapping;
  add a short finite headless run only when composition behavior changes.
- Preserve physical/safety gates, log failed stress scenarios and compare physical
  versus inferred feedback metadata explicitly. Do not copy all component parameters
  into a monolithic experiment file.

## Safe migration sequence for the next prompt

The sequence is ordered by dependency and risk. An approval of this design should
authorize only these structural/packaging changes; it is not M07 implementation.

1. **Freeze and index.** Save the dirty-tree inventory and source hashes, including
   untracked work, outside any rollback operation. Use the exact path map. Verify
   M03/M04 evidence and installed entry points before edits. Do not stash/reset/clean
   the user's existing changes. Preserve baseline scripts and result directories.
2. **Repair direct metadata.** Add manifest dependencies/buildtool declarations and
   align models version; update README's implemented/planned map. Check `rosdep`
   names against installed ROS metadata; don't add unnecessary numerical libraries.
   Keep public package and executable names unchanged.
3. **Extract one YAML owner.** Add the narrow configuration package; move the tested
   loader body unchanged, retain simulation import shim, port its existing tests,
   update explicit consumers. Check parent cycles, recursive aliases, list/null
   replacement and existing scenario equality. No schema/precedence redesign.
4. **Move assembly/config wrappers.** Relocate prediction factory/config wrapper
   and bringup model factory. Update imports and compatibility exports. Check pure
   import boundaries, description-derived model equivalence and installed resources.
5. **Move nodes in small groups.** Bringup, prediction, estimation, control,
   actuation. Move implementation bodies unchanged; add populated package
   initializers, canonical imports and setup entry points. Keep existing
   SimulationNode. Update live probes at each step; test module/class identity.
6. **Preserve backups and isolate resources.** Copy three .bak files to the exact
   archive destinations with SHA verification before removing originals; never
   overwrite an existing different archive. Exclude backups/caches from installation.
   Do not move raw captures, legacy baseline sources or the target SDF asset.
7. **Verify regression and installed behavior.** Run relevant tests per group, then
   full functional regression once; build affected packages/dependents; exercise
   installed entrypoint loading and package-share/Xacro/YAML resources independently
   of source-root imports. Validate both normal and compatibility imports and
   archived baseline instrumentation. Run one short headless normal-launch smoke
   only if runtime files moved, preserving holding and compute-only opt-out. Any
   physical-speed gate remains an expected failed safety criterion, not a pass.
8. **Close migration.** Record exact moves/compatibility policy, new hashes, tests,
   changed files and remaining limitations. Stop before adding M07 selectors or M08
   execution features. If a move exposes a behavior change, split it out rather
   than silently fixing an algorithm during structural verification.

### Regression/build checkpoints

Use the existing environment and essential regression commands:

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Once after the approved structural changes, not run during this audit.
colcon build --symlink-install --packages-up-to robot_bringup
source install/setup.bash
python3 -m pytest src --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros -q \
  -k 'not flake8 and not pep257 and not copyright'

# Separate installed-package check: use a temporary install/build/log base for
# affected Python packages and dependencies; don't delete the working install.
# Confirm share assets and entry points from /tmp with no workspace source path.
```

Add meaningful migration checks for canonical/legacy import identity, console
scripts, all referenced installed resources, the new loader dependency direction,
absence of ROS imports in core mathematics, and archived instrumentation selecting
the intended implementation. Existing control/model equivalence, predictor time,
estimator queue/constraints and M02 guard/watchdog/replay tests must remain green.
Report style failures separately; no blanket style rewrite is authorized.

Rollback is **file-scoped and additive**: retain a pre-migration byte snapshot and
move manifest, then restore only files changed by this migration to that snapshot
if a checkpoint cannot be repaired. Remove only newly created migration paths that
the manifest owns. Restore moved backups from verified copies when needed. Never
use repository-wide git reset/revert/clean or delete the active install/results.
Keep old public imports for compatibility so partial rollback does not break
unmigrated consumers. No commits/pushes unless separately requested.

## Review/acceptance gate

The design is ready for review; it is not implemented or build-verified. Approval
should identify the recommended focused migration (including or deferring the new
configuration package). The next prompt can execute that bounded sequence with
regression evidence. No further confirmation is needed per routine file move once
that scope is approved. Phase 1 ends with these documents and the inventory.

## Exhaustive proposed file tree

This generated appendix maps every current nonignored source/resource/test file.
`[keep]`, `[move]`, `[new]` and `[compat]` denote disposition, not completed work.
Historical docs/evidence remain at their current paths. Three ignored backups are
listed separately in the migration table and intentionally omitted from this
source tree. Generated caches/build/install/log are not source files.

```text
└── src/
    ├── robot_bringup/
    │   ├── LICENSE [keep]
    │   ├── config/
    │   │   ├── closed_loop.yaml [keep]
    │   │   ├── optimal_tracking.yaml [keep]
    │   │   ├── reactive_tracking.yaml [keep]
    │   │   ├── reactive_tracking_3d.yaml [keep]
    │   │   ├── reactive_tracking_3d_aggressive.yaml [keep]
    │   │   ├── reactive_tracking_low_damping.yaml [keep]
    │   │   ├── reactive_tracking_oscillatory.yaml [keep]
    │   │   ├── reactive_tracking_oscillatory_aggressive.yaml [keep]
    │   │   ├── reactive_tracking_oscillatory_aggressive_reference.yaml [keep]
    │   │   ├── reactive_tracking_saturated.yaml [keep]
    │   │   ├── simulation_estimation.yaml [keep]
    │   │   ├── tracking_axes.rviz [keep]
    │   │   └── tracking_visualization.yaml [keep]
    │   ├── launch/
    │   │   └── closed_loop.launch.py [keep]
    │   ├── package.xml [keep]
    │   ├── resource/
    │   │   └── robot_bringup [keep]
    │   ├── robot_bringup/
    │   │   ├── __init__.py [keep]
    │   │   ├── factory/
    │   │   │   ├── __init__.py [new]
    │   │   │   └── model_factory.py [move]
    │   │   ├── model_factory.py [compat]
    │   │   ├── nodes/
    │   │   │   ├── __init__.py [new]
    │   │   │   ├── tracking_feedback_tf.py [move]
    │   │   │   └── tracking_visualization.py [move]
    │   │   ├── tracking_feedback_tf.py [compat]
    │   │   └── tracking_visualization.py [compat]
    │   ├── setup.cfg [keep]
    │   ├── setup.py [keep]
    │   └── test/
    │       ├── test_aggressive_oscillatory.py [keep]
    │       ├── test_copyright.py [keep]
    │       ├── test_distance_pointing_kinematics.py [keep]
    │       ├── test_flake8.py [keep]
    │       ├── test_model_factory.py [keep]
    │       ├── test_model_factory_equivalence.py [keep]
    │       ├── test_pep257.py [keep]
    │       ├── test_reactive_v3_configuration.py [keep]
    │       ├── test_shoulder_safety.py [keep]
    │       ├── test_tracking_feedback_tf.py [keep]
    │       └── test_tracking_markers.py [keep]
    ├── robot_configuration/
    │   ├── LICENSE [new]
    │   ├── package.xml [new]
    │   ├── resource/
    │   │   └── robot_configuration [new]
    │   ├── robot_configuration/
    │   │   ├── __init__.py [new]
    │   │   └── yaml_loader.py [move]
    │   ├── setup.cfg [new]
    │   ├── setup.py [new]
    │   └── test/
    │       └── test_yaml_loader.py [new]
    ├── robot_control/
    │   ├── LICENSE [keep]
    │   ├── config/
    │   │   ├── control.yaml [keep]
    │   │   ├── whole_body.yaml [keep]
    │   │   ├── whole_body_arm_preferred.yaml [keep]
    │   │   └── whole_body_oscillatory_arm_preferred.yaml [keep]
    │   ├── package.xml [keep]
    │   ├── resource/
    │   │   └── robot_control [keep]
    │   ├── robot_control/
    │   │   ├── __init__.py [keep]
    │   │   ├── adapters/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── control_config_adapter.py [keep]
    │   │   │   ├── model_data_builder.py [keep]
    │   │   │   ├── planar_kinematics_adapter.py [keep]
    │   │   │   ├── prediction_message_adapter.py [keep]
    │   │   │   ├── robot_state_adapter.py [keep]
    │   │   │   └── robot_state_message_adapter.py [keep]
    │   │   ├── baseline/
    │   │   │   ├── __init__.py [keep]
    │   │   │   └── jacobian_controller.py [keep]
    │   │   ├── factory/
    │   │   │   ├── component_registry.py [keep]
    │   │   │   ├── control_factory.py [keep]
    │   │   │   ├── project_components.py [keep]
    │   │   │   └── project_configuration.py [keep]
    │   │   ├── interfaces/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── control_data.py [keep]
    │   │   │   └── controller.py [keep]
    │   │   ├── nodes/
    │   │   │   ├── __init__.py [new]
    │   │   │   └── control_node.py [move]
    │   │   ├── optimization/
    │   │   │   ├── __init__.py [keep]
    │   │   │   └── weighted_least_squares.py [keep]
    │   │   ├── pipeline/
    │   │   │   ├── control_execution_pipeline.py [keep]
    │   │   │   ├── control_pipeline.py [keep]
    │   │   │   ├── safe_tracking_controller.py [keep]
    │   │   │   └── tracking_control_orchestrator.py [keep]
    │   │   ├── ros/
    │   │   │   ├── __init__.py [keep]
    │   │   │   └── control_node.py [compat]
    │   │   ├── safety/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── command_limiter.py [keep]
    │   │   │   ├── constraint_bounds.py [keep]
    │   │   │   ├── constraint_projector.py [keep]
    │   │   │   ├── input_status.py [keep]
    │   │   │   ├── input_validator.py [keep]
    │   │   │   ├── joint_position_limiter.py [keep]
    │   │   │   └── safe_control_pipeline.py [keep]
    │   │   └── tracking/
    │   │       ├── __init__.py [keep]
    │   │       ├── pose_reference_generator.py [keep]
    │   │       ├── radial_direction_policy.py [keep]
    │   │       ├── reference_selector.py [keep]
    │   │       ├── task_space_error.py [keep]
    │   │       ├── tracking_frame_kinematics.py [keep]
    │   │       ├── tracking_reference_builder.py [keep]
    │   │       └── velocity_task.py [keep]
    │   ├── setup.cfg [keep]
    │   ├── setup.py [keep]
    │   └── test/
    │       ├── test_command_limiter.py [keep]
    │       ├── test_component_registry.py [keep]
    │       ├── test_constraint_bounds.py [keep]
    │       ├── test_constraint_projector.py [keep]
    │       ├── test_control_config_adapter.py [keep]
    │       ├── test_control_data.py [keep]
    │       ├── test_control_execution_pipeline.py [keep]
    │       ├── test_control_factory.py [keep]
    │       ├── test_control_node.py [keep]
    │       ├── test_control_pipeline.py [keep]
    │       ├── test_controller.py [keep]
    │       ├── test_copyright.py [keep]
    │       ├── test_flake8.py [keep]
    │       ├── test_input_status.py [keep]
    │       ├── test_input_validator.py [keep]
    │       ├── test_jacobian_controller.py [keep]
    │       ├── test_joint_position_limiter.py [keep]
    │       ├── test_model_data_builder.py [keep]
    │       ├── test_pep257.py [keep]
    │       ├── test_planar_kinematics_adapter.py [keep]
    │       ├── test_pose_reference_generator.py [keep]
    │       ├── test_prediction_message_adapter.py [keep]
    │       ├── test_radial_direction_policy.py [keep]
    │       ├── test_reference_selector.py [keep]
    │       ├── test_robot_model_control.py [keep]
    │       ├── test_robot_state_adapter.py [keep]
    │       ├── test_robot_state_message_adapter.py [keep]
    │       ├── test_safe_control_pipeline.py [keep]
    │       ├── test_safe_tracking_controller.py [keep]
    │       ├── test_strategy_configuration.py [keep]
    │       ├── test_task_space_error.py [keep]
    │       ├── test_tracking_control_orchestrator.py [keep]
    │       ├── test_tracking_reference_builder.py [keep]
    │       └── test_weighted_controller.py [keep]
    ├── robot_description/
    │   ├── LICENSE [keep]
    │   ├── config/
    │   │   ├── robot/
    │   │   │   └── robot.yaml [keep]
    │   │   └── robot.rviz [keep]
    │   ├── launch/
    │   │   └── display.launch.py [keep]
    │   ├── package.xml [keep]
    │   ├── resource/
    │   │   └── robot_description [keep]
    │   ├── robot_description/
    │   │   ├── __init__.py [keep]
    │   │   ├── configuration.py [keep]
    │   │   └── model_parameters.py [keep]
    │   ├── setup.cfg [keep]
    │   ├── setup.py [keep]
    │   ├── test/
    │   │   ├── test_copyright.py [keep]
    │   │   ├── test_description_configuration.py [keep]
    │   │   ├── test_flake8.py [keep]
    │   │   └── test_pep257.py [keep]
    │   └── urdf/
    │       ├── base.xacro [keep]
    │       ├── inertial_macros.xacro [keep]
    │       ├── manipulator.xacro [keep]
    │       ├── materials.xacro [keep]
    │       ├── robot.urdf.xacro [keep]
    │       └── tracking_frame.xacro [keep]
    ├── robot_estimation/
    │   ├── config/
    │   │   ├── robot_estimator.yaml [keep]
    │   │   └── target_estimator.yaml [keep]
    │   ├── package.xml [keep]
    │   ├── resource/
    │   │   └── robot_estimation [keep]
    │   ├── robot_estimation/
    │   │   ├── __init__.py [keep]
    │   │   ├── common/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── estimator_interface.py [keep]
    │   │   │   ├── measurement.py [keep]
    │   │   │   ├── state.py [keep]
    │   │   │   └── timestamp_buffer.py [keep]
    │   │   ├── estimation_node.py [compat]
    │   │   ├── filters/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── extended_kalman_filter.py [keep]
    │   │   │   ├── filter_interface.py [keep]
    │   │   │   └── kalman_filter.py [keep]
    │   │   ├── nodes/
    │   │   │   ├── __init__.py [new]
    │   │   │   └── estimation_node.py [move]
    │   │   ├── robot/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── joint_bounds.py [keep]
    │   │   │   ├── models/
    │   │   │   │   ├── __init__.py [keep]
    │   │   │   │   ├── measurement_model.py [keep]
    │   │   │   │   └── process_model.py [keep]
    │   │   │   ├── robot_state_estimator.py [keep]
    │   │   │   └── sensors/
    │   │   │       ├── __init__.py [keep]
    │   │   │       ├── imu.py [keep]
    │   │   │       ├── joint_encoder.py [keep]
    │   │   │       ├── sensor_interface.py [keep]
    │   │   │       └── wheel_odometry.py [keep]
    │   │   └── target/
    │   │       ├── __init__.py [keep]
    │   │       ├── models/
    │   │       │   ├── __init__.py [keep]
    │   │       │   ├── measurement_model.py [keep]
    │   │       │   └── process_model.py [keep]
    │   │       └── target_state_estimator.py [keep]
    │   ├── setup.cfg [keep]
    │   ├── setup.py [keep]
    │   └── test/
    │       ├── test_estimation_node.py [keep]
    │       ├── test_extended_kalman_filter.py [keep]
    │       ├── test_joint_bounds_conditioner.py [keep]
    │       ├── test_kalman_filter.py [keep]
    │       ├── test_robot_exact_motion.py [keep]
    │       ├── test_robot_sensors.py [keep]
    │       ├── test_robot_state_estimator.py [keep]
    │       ├── test_target_state_estimator.py [keep]
    │       └── test_timestamp_buffer.py [keep]
    ├── robot_interfaces/
    │   ├── CMakeLists.txt [keep]
    │   ├── msg/
    │   │   ├── ActuationTelemetry.msg [keep]
    │   │   ├── ControlTelemetry.msg [keep]
    │   │   ├── PredictedTrajectory.msg [keep]
    │   │   ├── SafeVelocityCommand.msg [keep]
    │   │   └── StateEstimate.msg [keep]
    │   └── package.xml [keep]
    ├── robot_models/
    │   ├── LICENSE [keep]
    │   ├── package.xml [keep]
    │   ├── resource/
    │   │   └── robot_models [keep]
    │   ├── robot_models/
    │   │   ├── __init__.py [keep]
    │   │   ├── base/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── aerial/
    │   │   │   │   └── __init__.py [keep]
    │   │   │   ├── differential_drive/
    │   │   │   │   ├── __init__.py [keep]
    │   │   │   │   ├── dynamics.py [keep]
    │   │   │   │   ├── jacobian.py [keep]
    │   │   │   │   └── kinematics.py [keep]
    │   │   │   └── omnidirectional/
    │   │   │       └── __init__.py [keep]
    │   │   ├── camera/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── fov.py [keep]
    │   │   │   ├── geometry.py [keep]
    │   │   │   └── projection.py [keep]
    │   │   ├── common/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── rotations.py [keep]
    │   │   │   └── transforms.py [keep]
    │   │   ├── constraints/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── acceleration_limits.py [keep]
    │   │   │   ├── actuator_limits.py [keep]
    │   │   │   ├── joint_limits.py [keep]
    │   │   │   └── velocity_limits.py [keep]
    │   │   ├── interfaces/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── base_model.py [keep]
    │   │   │   ├── manipulator_model.py [keep]
    │   │   │   ├── model_data.py [keep]
    │   │   │   └── robot_model.py [keep]
    │   │   ├── manipulator/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── fixed_mount/
    │   │   │   │   ├── __init__.py [keep]
    │   │   │   │   └── model.py [keep]
    │   │   │   └── serial_arm/
    │   │   │       ├── __init__.py [keep]
    │   │   │       ├── differential_kinematics.py [keep]
    │   │   │       ├── dynamics.py [keep]
    │   │   │       ├── forward_kinematics.py [keep]
    │   │   │       ├── jacobian.py [keep]
    │   │   │       └── model.py [keep]
    │   │   ├── reference/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── geometry/
    │   │   │   │   ├── __init__.py [keep]
    │   │   │   │   ├── point_reference.py [keep]
    │   │   │   │   ├── reference_geometry.py [keep]
    │   │   │   │   └── sphere_reference.py [keep]
    │   │   │   ├── look_at.py [keep]
    │   │   │   ├── pose_reference.py [keep]
    │   │   │   └── twist_reference.py [keep]
    │   │   ├── target/
    │   │   │   ├── __init__.py [keep]
    │   │   │   └── relative_state.py [keep]
    │   │   ├── task_space/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── distance_pointing.py [keep]
    │   │   │   ├── pose_error.py [keep]
    │   │   │   └── twist_error.py [keep]
    │   │   ├── types.py [keep]
    │   │   └── whole_body/
    │   │       ├── __init__.py [keep]
    │   │       ├── composer.py [keep]
    │   │       ├── factory.py [keep]
    │   │       ├── jacobian.py [keep]
    │   │       ├── kinematics.py [keep]
    │   │       └── twist.py [keep]
    │   ├── setup.cfg [keep]
    │   ├── setup.py [keep]
    │   └── test/
    │       ├── test_base_models.py [keep]
    │       ├── test_configured_model_factory.py [keep]
    │       ├── test_copyright.py [keep]
    │       ├── test_distance_pointing.py [keep]
    │       ├── test_flake8.py [keep]
    │       ├── test_look_at_reference.py [keep]
    │       ├── test_manipulator_models.py [keep]
    │       ├── test_model_pipeline.py [keep]
    │       ├── test_pep257.py [keep]
    │       ├── test_reference_geometry.py [keep]
    │       ├── test_rotation_validation.py [keep]
    │       └── test_whole_body.py [keep]
    ├── robot_prediction/
    │   ├── LICENSE [keep]
    │   ├── config/
    │   │   └── prediction.yaml [keep]
    │   ├── package.xml [keep]
    │   ├── resource/
    │   │   └── robot_prediction [keep]
    │   ├── robot_prediction/
    │   │   ├── __init__.py [keep]
    │   │   ├── baseline/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── constant_acceleration.py [keep]
    │   │   │   ├── constant_velocity.py [keep]
    │   │   │   ├── coordinated_turn.py [keep]
    │   │   │   └── imm.py [keep]
    │   │   ├── confidence/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── performance_monitor.py [keep]
    │   │   │   ├── prediction_confidence.py [keep]
    │   │   │   └── temporal_alignment.py [keep]
    │   │   ├── config_loader.py [compat]
    │   │   ├── configuration/
    │   │   │   ├── __init__.py [new]
    │   │   │   └── prediction_config.py [move]
    │   │   ├── factory/
    │   │   │   ├── __init__.py [new]
    │   │   │   └── system_factory.py [move]
    │   │   ├── fusion/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── adaptive_fusion.py [keep]
    │   │   │   ├── baseline_fusion.py [keep]
    │   │   │   ├── confidence_weighting.py [keep]
    │   │   │   └── weighted_correction.py [keep]
    │   │   ├── history/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── residual_buffer.py [keep]
    │   │   │   ├── trajectory_buffer.py [keep]
    │   │   │   ├── twist_buffer.py [keep]
    │   │   │   └── vector_buffer.py [keep]
    │   │   ├── interfaces/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── prediction_data.py [keep]
    │   │   │   ├── predictor.py [keep]
    │   │   │   └── state_layout.py [keep]
    │   │   ├── nodes/
    │   │   │   ├── __init__.py [new]
    │   │   │   └── prediction_node.py [move]
    │   │   ├── online_adaptation/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── classical/
    │   │   │   │   ├── __init__.py [keep]
    │   │   │   │   ├── adaptive_state_space.py [keep]
    │   │   │   │   ├── residual_adapter.py [keep]
    │   │   │   │   └── rls_adapter.py [keep]
    │   │   │   └── learned/
    │   │   │       ├── __init__.py [keep]
    │   │   │       └── online_sequence_learner.py [keep]
    │   │   ├── prediction_node.py [compat]
    │   │   ├── residuals/
    │   │   │   ├── __init__.py [keep]
    │   │   │   ├── prediction_residual.py [keep]
    │   │   │   └── residual_statistics.py [keep]
    │   │   ├── system_factory.py [compat]
    │   │   └── trajectory_prediction.py [keep]
    │   ├── setup.cfg [keep]
    │   ├── setup.py [keep]
    │   └── test/
    │       ├── test_baseline_predictors.py [keep]
    │       ├── test_copyright.py [keep]
    │       ├── test_flake8.py [keep]
    │       ├── test_pep257.py [keep]
    │       ├── test_prediction_3d.py [keep]
    │       ├── test_prediction_node.py [keep]
    │       └── test_prediction_time_alignment.py [keep]
    └── robot_simulation/
        ├── config/
        │   ├── actuation.yaml [keep]
        │   ├── comparison/
        │   │   └── baseline.yaml [keep]
        │   ├── disturbances.yaml [keep]
        │   ├── environment.yaml [keep]
        │   ├── experiments/
        │   │   ├── flat_baseline.yaml [keep]
        │   │   └── turn_baseline.yaml [keep]
        │   ├── gazebo/
        │   │   ├── joint_position_hold.yaml [keep]
        │   │   └── robot.yaml [keep]
        │   ├── launch.yaml [keep]
        │   ├── measurements.yaml [keep]
        │   ├── scenarios/
        │   │   ├── environment.yaml [keep]
        │   │   ├── environment_oscillatory.yaml [keep]
        │   │   ├── environment_oscillatory_aggressive.yaml [keep]
        │   │   ├── environment_oscillatory_aggressive_reference.yaml [keep]
        │   │   ├── moving_target.yaml [keep]
        │   │   ├── reactive_target.yaml [keep]
        │   │   ├── reactive_target_3d.yaml [keep]
        │   │   ├── reactive_target_3d_aggressive.yaml [keep]
        │   │   ├── reactive_target_low_damping.yaml [keep]
        │   │   ├── reactive_target_oscillatory.yaml [keep]
        │   │   ├── reactive_target_oscillatory_aggressive.yaml [keep]
        │   │   ├── reactive_target_oscillatory_aggressive_reference.yaml [keep]
        │   │   ├── reactive_target_saturated.yaml [keep]
        │   │   ├── target.yaml [keep]
        │   │   ├── target_3d.yaml [keep]
        │   │   ├── target_3d_aggressive.yaml [keep]
        │   │   ├── target_fight_flight_low_damping.yaml [keep]
        │   │   ├── target_geometry.yaml [keep]
        │   │   └── target_saturated.yaml [keep]
        │   ├── simulation.yaml [keep]
        │   ├── target.yaml [keep]
        │   └── terrain/
        │       └── flat.yaml [keep]
        ├── launch/
        │   └── simulation.launch.py [keep]
        ├── models/
        │   └── target/
        │       └── model.sdf [keep]
        ├── package.xml [keep]
        ├── resource/
        │   └── robot_simulation [keep]
        ├── robot_simulation/
        │   ├── __init__.py [keep]
        │   ├── actuation/
        │   │   ├── __init__.py [keep]
        │   │   ├── position_guard.py [keep]
        │   │   ├── position_reference.py [keep]
        │   │   └── velocity_actuation.py [compat]
        │   ├── diagnostics/
        │   │   ├── __init__.py [keep]
        │   │   └── trajectory_probe.py [keep]
        │   ├── disturbances/
        │   │   ├── __init__.py [keep]
        │   │   ├── composite_field.py [keep]
        │   │   ├── gaussian_noise.py [keep]
        │   │   ├── sinusoidal_field.py [keep]
        │   │   └── vector_field.py [keep]
        │   ├── dynamics/
        │   │   ├── __init__.py [keep]
        │   │   └── target/
        │   │       ├── __init__.py [keep]
        │   │       └── fight_flight.py [keep]
        │   ├── engine/
        │   │   ├── __init__.py [keep]
        │   │   └── simulation_engine.py [keep]
        │   ├── factory/
        │   │   ├── __init__.py [keep]
        │   │   ├── config_loader.py [compat]
        │   │   ├── registry.py [keep]
        │   │   └── simulation_factory.py [keep]
        │   ├── gazebo/
        │   │   ├── __init__.py [keep]
        │   │   ├── ground_truth_adapter.py [keep]
        │   │   ├── pose_client.py [keep]
        │   │   ├── robot_builder.py [keep]
        │   │   ├── sdf_plugins.py [keep]
        │   │   ├── target_adapter.py [keep]
        │   │   ├── tracking_frame_provider.py [keep]
        │   │   └── world_builder.py [keep]
        │   ├── geometry/
        │   │   ├── __init__.py [keep]
        │   │   ├── model_exporter.py [keep]
        │   │   └── sdf_generator.py [keep]
        │   ├── integration/
        │   │   ├── __init__.py [keep]
        │   │   ├── rk4.py [keep]
        │   │   └── time_stepper.py [keep]
        │   ├── interfaces/
        │   │   ├── __init__.py [keep]
        │   │   ├── disturbance.py [keep]
        │   │   ├── dynamics.py [keep]
        │   │   └── integrator.py [keep]
        │   ├── measurements/
        │   │   ├── __init__.py [keep]
        │   │   └── synthetic_measurement.py [keep]
        │   ├── nodes/
        │   │   ├── __init__.py [keep]
        │   │   ├── simulation_node.py [keep]
        │   │   └── velocity_actuation.py [move]
        │   └── state/
        │       ├── __init__.py [keep]
        │       ├── simulation_state.py [keep]
        │       └── target_state.py [keep]
        ├── setup.cfg [keep]
        ├── setup.py [keep]
        ├── test/
        │   ├── test_combined_dynamics.py [keep]
        │   ├── test_configuration.py [keep]
        │   ├── test_copyright.py [keep]
        │   ├── test_fight_flight.py [keep]
        │   ├── test_flake8.py [keep]
        │   ├── test_integrator.py [keep]
        │   ├── test_measurements.py [keep]
        │   ├── test_model_exporter.py [keep]
        │   ├── test_oscillatory_experiment.py [keep]
        │   ├── test_pep257.py [keep]
        │   ├── test_pose_client.py [keep]
        │   ├── test_position_guard.py [keep]
        │   ├── test_position_reference.py [keep]
        │   ├── test_robot_builder.py [keep]
        │   ├── test_saturated_fight_flight.py [keep]
        │   ├── test_sdf_generator.py [keep]
        │   ├── test_simulation_factory.py [keep]
        │   ├── test_simulation_pipeline.py [keep]
        │   ├── test_tangential_fight_flight.py [keep]
        │   ├── test_target_adapter.py [keep]
        │   ├── test_target_state.py [keep]
        │   ├── test_time_stepper.py [keep]
        │   ├── test_tracking_frame_provider.py [keep]
        │   ├── test_vector_field.py [keep]
        │   ├── test_velocity_actuation.py [keep]
        │   └── test_world_builder.py [keep]
        └── worlds/
            └── tracking_testbed.sdf.xacro [keep]
```

All original 422 nonignored file paths remain either unchanged or as an explicit
compatibility path. Ten canonical implementation relocations (six nodes, three
assembly/config wrappers and the YAML loader) are proposals, not completed changes.
Tests under existing packages stay in place; loader-specific tests are copied/ported
to the new owner while simulation registry/scenario tests remain simulation-owned.
