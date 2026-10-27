Current scenario composition and direct-launch migration: [experiment composition](experiment_composition.md).

# Canonical component configuration and extension contracts

Updated 2026-10-27. This guide describes Task 1's implemented architecture; the
[experiment guide](experiment_management.md) owns catalog, launch and history
contracts. Large-scale scenario consolidation is reserved for Task 2. No dashboard
or arbitrary YAML-driven code loading is introduced.

## Ownership and YAML

| Owner | Canonical document and top-level groups |
|---|---|
| Estimation | `robot_estimation/config/estimation.yaml`: `robot_estimators`, `target_estimators`, `transport`, `runtime`, `profiles` |
| Prediction | `robot_prediction/config/prediction.yaml`: `prediction`, `monitoring`, `predictors`, `adaptations`, `fusions` |
| Control | `robot_control/config/control.yaml`: `controllers`, `tasks`, `tracking`, `references`, `operating_bounds`, `transport`, `runtime` |
| Simulation | `robot_simulation/config/dynamics.yaml`: `dynamics`; independent physics, sensing, actuation, fields and scenario resources remain |
| Description | Existing `robot_description` resources retain sole authority over geometry, joints, mounts, position/velocity/effort limits and initial physical posture |
| Experiments | Explicit model/task/scenario/runtime selection, calibration, scientific assignments, run overrides, immutable revisions and snapshots |
| Bringup | Validated assembly and deployment adapters; no algorithm defaults or experiment selection policy |

Model groups contain `parameters`, for example:

```yaml
predictors:
  coordinated_turn:
    parameters:
      turn_rate_epsilon: 1.0e-6
adaptations:
  residual:
    parameters:
      correction_gain: 1.0
```

These are defaults, not selected components. The existing explicit composition
selects `prediction.type`, `prediction.adaptation`, `prediction.fusion`,
`estimation.robot/target.type`, `control.type`, task/strategy and reference. A
scenario selects its dynamics. Source defaults stay immutable during experiments.
Mappings merge recursively; lists replace. Empty/false overrides remain meaningful.

Estimation separates numerical robot/target groups from topics, frames, queueing,
freshness and node settings. Disabled pose-aiding numerical options live under
`robot_estimators.robot_ekf.optional_parameters`; the `pose_aiding` profile remains
a selected calibration delta. Control separates DLS/WLS mathematics from Cartesian
and Distance + Pointing tasks, tracking axes/radius/lookahead, reference generation,
command envelopes, transport and runtime flags. Description-derived bounds are
still injected from the selected physical robot, never copied into a controller
schema. Simulation's owner defaults are mathematical dynamics parameters; scenario
initial conditions, centering fields and integration settings retain their owners.
Existing explicit scenario parameter overrides are preserved for Task 2 review.

Prediction sampling (`dt`, horizon) and monitor confidence/history parameters have
independent groups. IMM references the CT group's epsilon rather than duplicating
it. The ensemble order remains CV, CA, CT. RLS and baseline-fusion defaults are
validated even though those components require library-specific adapters.

## Domain interfaces and factories

`robot_configuration.components.ComponentRegistry` is a trusted constructor table,
not a universal algorithm interface. Its public operations are:

```python
registry.register(component_id, builder, description=..., defaults=..., schema=...,
                  capabilities=..., requirements=..., dependencies=(... ,))
registry.names()
registry.describe(component_id)
registry.describe_all()
registry.compatibility(component_id, adapter_context)
registry.validate(component_id, parameter_overrides, context=adapter_context)
registry.create(component_id, parameter_overrides, context=adapter_context,
                **runtime_dependencies)
```

Registration is explicit Python in the owning package. Duplicate IDs, unknown
parameters, nonfinite numbers, booleans supplied as numbers, invalid bounds and
incompatible declared adapter contexts reject before construction. Defaults,
metadata and typed parameter results are detached copies. Runtime dependencies
are declared separately, retain object identity and cannot shadow parameters.
Old untyped internal registrations remain supported by thin API adapters; new
algorithm registrations supply schemas. YAML cannot name arbitrary Python imports.

The supported schema subset is object/properties/required/additionalProperties,
array/items/minItems/maxItems, finite number/integer, boolean/string, enum,
inclusive/exclusive numeric bounds and `oneOf`. `unit` and descriptive metadata
are preserved. This is not a complete JSON Schema implementation. Owners specify
bounds and units through `schema_for` or explicit schemas. Coupled constraints
such as positive-semidefinite covariance, normalized IMM transition rows, matrix
shape, axis geometry and physical DOFs remain in mathematical constructors and
shared deployment preflight. Discovery validation explicitly reports this scope;
it does not certify all physical constraints or real-hardware safety.

| Family | Contract and owner construction path |
|---|---|
| Dynamics | Existing `Dynamics.derivative`; `SimulationFactory` upgrades its existing dynamics registry and checks the returned interface |
| Robot estimation | `RobotEstimatorProtocol`: estimate, predict/update, wheel/IMU/joint/pose operations and health diagnostics; owner factory returns `(estimator, constraint)` |
| Target estimation | Existing `EstimatorInterface`; the target factory validates the actual returned state dimension against an explicit deployment context |
| Prediction | Existing predictor contract; owner `create_prediction_registries` and `PredictionSystemFactory` construct selected predictor/adaptation/fusion |
| Adaptation/fusion | `ResidualCorrection.compute_correction` and `CorrectionFusion.apply`; fusion is injected into the existing trajectory orchestrator |
| Control | Existing `Controller`, `VelocityTask`, reference-policy and geometry contracts; `create_project_registries` upgrades the existing registry/factory path |

There are no algorithm-ID branches in the experiment runner's construction path.
The discovery aggregator names families and delegates algorithm knowledge to their
owners. There is no competing second control or simulation registry engine.

The robot EKF uses continuous white-acceleration noise density (linear units
`m/s^(3/2)`, angular/joint units `rad/s^(3/2)`); the target KF uses discrete
acceleration standard deviation (`m/s^2`). These are deliberately different
contracts. No noise values or discretization equations changed in this migration.

The generic target filter remains usable at other spatial dimensions in the
library. The current PointStamped deployment adapter requires 3D position and a
six-state `[p,v]` target. Robot transport remains a planar mobile manipulator with
`5 + 2*n` state and body-base-plus-joint commands. The selected physical description
provides joint ordering and limits. Fixed deployment assumptions are not claims
that all future implementations must use this layout.

## Discoverable inventory

| Family | Current IDs | Deployment status |
|---|---|---|
| Dynamics | `fight_flight`, `saturated_fight_flight` | Existing numerical/ROS target deployment |
| Robot estimator | `robot_ekf` | Deployed, existing localization/covariance limitations remain CONDITIONAL |
| Target estimator | `kalman` | Deployed `[p,v]`, library dimension generic; runtime limitations remain documented |
| Predictor | `constant_velocity`, `imm` | Existing deployed contracts; IMM runtime acceptance remains CONDITIONAL |
| Predictor | `constant_acceleration`, `coordinated_turn` | Library only: richer state layout required |
| Adaptation | `residual`, `none` | Deployed residual-history contract or explicit disabled selection |
| Adaptation | `rls` | Library only: feature/target adapter and dimensions required |
| Fusion | `weighted_correction`, `none` | Deployed state-bias correction or explicit disabled selection |
| Fusion | `baseline` | Library only: multiple-trajectory fusion, not correction fusion |
| Controller | `jacobian`, `weighted_least_squares` | Existing DLS/WLS adapters and safety validation |
| Task | `cartesian`, `distance_pointing` | Existing tasks |
| Reference | Policy `radial`, geometry `sphere` | Existing 3D geometry contract |

No MPC, LQR or learned model is advertised. `deployable` describes the implemented
transport contract, not a scientific PASS or safety certification. Runtime status
is separate metadata. Generic discovery returns declared dependencies, schemas,
units, defaults, capabilities and requirements.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_experiments run components list
ros2 run robot_experiments run components show predictor imm
ros2 run robot_experiments run components validate controller weighted_least_squares
# Optional YAML mappings: --parameters overrides.yaml --context adapter.yaml
```

The headless Python API is `robot_experiments.components.list_components(family=None)`,
`describe_component(family, component_id)` and
`validate_component(family, component_id, parameters=None, context=None)`.
Without context, validation checks parameters; with context, it additionally checks
all declared requirements. It returns detached JSON-compatible data and explicit
error messages. Library components are rejected by incompatible deployment contexts.

## Adding an implementation

Implement the family's existing interface, add owner defaults/schema/units and
register its builder in the owner factory. Declare runtime dependencies separately
from parameter leaves and declare layout/adapter requirements. Extend only the
relevant deployment adapter if the new algorithm actually requires a new input or
output representation. Select the ID through the existing composition/owner
configuration. Do not add model branches to experiment orchestration.

Tests contain four lightweight extensions, absent from production registrations:

- Dynamics: configured constant acceleration is integrated through the actual
  `SimulationFactory`/RK4 path and checked numerically.
- Estimation: alternate target and robot implementations are discovered and built;
  custom numerical names work with explicit deployment context, and incompatible
  returned interfaces/dimensions reject.
- Prediction: a configured offset predictor produces an actual trajectory; custom
  adaptation/fusion change the computed correction through the real factory.
  Extra owner parameter groups survive run materialization and preflight.
- Control: a test-only `Controller` is registered, configured, validated and built
  through the existing `ControlFactory`, with no runner modification.

See each owner's `test_component_architecture.py`, prediction's
`test_prediction_components.py`, and control's `test_control_factory.py`.

## Compatibility boundaries and reproducibility

| Boundary | Consumers and retained behavior |
|---|---|
| Estimation `load_estimation_parameters(path, group)` | Node defaults, bringup preflight, copied-install checks, tests and reusable probes; projects canonical groups into existing ROS parameter documents and reads historical ROS documents |
| Control `load_control_parameters(path, group)` | Control node, strategy assembly, tests and probes; `parameters` and `distance_pointing` project existing ROS/strategy contracts; old flat strategy documents remain readable |
| Prediction `PredictionConfig` | Node/factory/probes; `.data` is canonical, `.get(old_section)` is a bounded legacy projection, `.legacy_data` is a compatibility read view |
| Prediction `normalize_prediction_overrides` | Experiment owner overrides; translates historical partial keys without inserting defaults and rejects conflicting mixed representations |
| Thin control/simulation registry adapters | Existing `create` signatures and dependency identity; shared schema/registry implementation underneath |
| Package-resource lookup | Source and copied-install owner defaults through `package_config_path`, without requiring ROS in the numerical library |

New run-local prediction snapshots are canonical, retaining all registered model
parameter groups. Historical snapshot documents remain readable. Before/after
scientific comparison normalizes prediction representation and the derived
comparison hash, plus two temporary description paths, without masking numerical
values, identities, selections, seeds or safety settings. Original runtime artifact
hashes and immutable catalog revisions are not rewritten.

When a target scenario omits dynamics parameters, materialization resolves the
owner defaults and freezes the full effective dynamics map in its run input. The
owner defaults file is included in source-hash provenance, preventing later source
edits from silently changing a recorded partial scenario.

Task 2 may review redundant scenario overrides and remaining obsolete composition
resources. It must preserve distinct physical cases, immutable catalog revisions,
legacy saved inputs, independent field/physics responsibilities and scientific
artifacts. None of that cleanup is part of this task.

## Verification and verdict

Final measurements, commands, source-change manifest and limitations are recorded
in [the Task 1 validation report](validation/component_configuration.md).
Local raw evidence is under `results/component_architecture/`; it is ignored and
must accompany scientific results when transferred. It is not installed as code.
