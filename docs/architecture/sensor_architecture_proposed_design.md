# Proposed sensor architecture design

Status: **design proposal only, not implemented**. Based on the [current-tree review](sensor_architecture_review.md), 2026-10-23. Only these two review documents are delivered. Proposed modules, metadata fields, validation policies and example resources below do not exist merely because they are named here.

## 1. Recommendation and intent

Choose **Option B: bounded consolidation within existing packages**, with discoverability first. Engineers should be able to identify the actual sources and fusion assumptions of an M07 system, change existing modeled sensor characteristics using inheritance, and compare intentionally different sensing setups through M08. That does not require moving physics, imperfect generators and EKF mathematics into one package.

Preserve the current M07 manifest as the **only system composition entry**. Keep M08's experiment YAML as the entry for variant/run selection. Initially add a computed, read-only sensor inventory to the existing resolved assembly plan and startup summary. Derive it from owner resources; do not introduce a second editable sensor catalog with duplicated topic, covariance, timing or enable values.

After that, extract the five production message-conversion bodies from the estimator node into one concrete adapter module at its existing ROS boundary. Retain source-time admission, filter prediction, trust policy, publication gating and clock-reset orchestration where they currently belong. Do not create an adapter factory or dynamic plugin registry for five known message contracts.

The experiment gap warrants one explicit M08 comparison policy, not weaker equality checks for all runs. Preserve historical strict comparison behavior by default. Implement none of these changes during this review.

## 2. Alternative comparison

| Criterion | A: ownership + discoverability | B: consolidate within existing packages | C: dedicated `robot_sensors` |
|---|---|---|---|
| Discoverability | Computed inventory/docs expose real call paths; unused wrappers remain clearly labeled | Same inventory plus one production ROS-conversion entry | Attractive package name, but physics/model/fusion paths still distributed |
| Responsibility boundaries | Preserves current correct boundaries | Preserves boundaries, reduces conversion/orchestration mixing | Risks conflating generator, transport and observation math |
| Configuration usability | Existing references/inheritance; show effective overrides | Same; bounded cross-owner preflight and M08 comparison declaration | Still needs M07/M08 changes; package move alone adds no selection semantics |
| Extensibility | New source follows current explicit node/model/factory paths | Concrete conversion location makes extension easier; model/state changes remain explicit | Useful only with independently reusable producer implementations and consumers |
| Reusability | Existing ROS-free generators and models remain usable | Reuses models/Measurement without runtime dependency on legacy wrappers | Would need a deliberate ROS-only or generator-only contract to avoid mixed dependencies |
| Dependencies/cycles | No new package edge | No new package edge; ROS stays under node boundary | A package importing simulation and estimation while both depend on it risks cycles; avoiding that requires additional extraction |
| ROS 2 / Gazebo | Current launch/bridge unchanged | Current transport/QoS/launch preserved | Additional ament resource/dependency/installation migration |
| Tests/results compatibility | Minimal; existing output schemas untouched | Golden adapter equivalence and additive metadata; legacy result default unchanged | Import paths, manifests, tests, snapshots and package provenance all need migration |
| Migration cost in components | Inventory/preflight and documentation; does not resolve conversion duplication by itself | One adapter module; bounded existing composition/benchmark modifications | At least package metadata/build/resource files plus moves and consumer/dependency rewiring |
| New files/layers | No new runtime abstraction required | One production module, one test module; no new package or factory layer | New package layer plus compatibility decisions; exact source split not justified yet |
| Onboarding | Best immediate improvement | Best balance once conversions are documented/tested | Another package to learn before responsibilities are clear |

A is a useful first step and could be delivered alone. B addresses the demonstrated runtime/adapter mismatch as well. C should be reconsidered only when two independently deployed consumers need the same concrete sensor transport/producer implementations and a clear dependency boundary cannot be served by existing ownership. Current evidence does not meet that threshold. Moving files for symmetry is not a requirement.

## 3. Proposed organization and ownership

Only relevant paths are shown; all prefixes below are under `src/`. `[E]` existing, `[P]` proposed new. Existing files remain in place unless a later approved compatibility decision says otherwise.

### Before

```text
robot_estimation/
  robot_estimation/
    interfaces/sensor_interface.py                  [E: raw-array compatibility API]
    robot/sensors/{imu,joint_encoder,wheel_odometry}.py [E: test-used wrappers]
    nodes/estimation_node.py                        [E: conversion + orchestration]
    robot/models/measurement_model.py              [E: wheel/joint/yaw-rate h/H]
    robot/planar_pose.py                            [E: pose transform/model/trust]
    robot/robot_state_estimator.py                  [E: fusion and recovery]
  config/{robot_estimator,robot_localized,target_estimator}.yaml [E]
robot_simulation/
  robot_simulation/measurements/{synthetic_measurement,planar_localization}.py [E]
  robot_simulation/nodes/{simulation_node,localization_sensor}.py [E]
  robot_simulation/gazebo/{robot_builder,tracking_frame_provider}.py [E]
  robot_simulation/state/physical_context.py         [E]
  config/{measurements.yaml,sensors/planar_localization.yaml} [E]
robot_bringup/
  robot_bringup/factory/system_configuration.py      [E: assembled selection]
  launch/closed_loop.launch.py                      [E]
  config/system*.yaml                              [E: composition manifests]
robot_experiments/
  robot_experiments/{configuration,runner,benchmarking}.py [E]
```

### After the recommended bounded migration

```text
robot_estimation/
  robot_estimation/
    nodes/measurement_adapters.py                   [P: concrete ROS conversion]
    nodes/estimation_node.py                        [E: delegates conversion only]
    interfaces/sensor_interface.py                  [E: explicitly compatibility-only]
    robot/sensors/{imu,joint_encoder,wheel_odometry}.py [E: compatibility-only]
    robot/models/measurement_model.py               [E: unchanged]
    robot/planar_pose.py                            [E: unchanged]
    robot/robot_state_estimator.py                  [E: unchanged]
  test/test_measurement_adapters.py                 [P: runtime-contract tests]
robot_simulation/                                   [E: no source move or model change]
robot_bringup/
  robot_bringup/factory/system_configuration.py      [E: computed inventory/preflight]
  launch/closed_loop.launch.py                      [E: additive inventory summary]
robot_experiments/
  robot_experiments/configuration.py                [E: explicit comparison intent]
  robot_experiments/runner.py                       [E: persist/pass comparison intent]
  robot_experiments/benchmarking.py                 [E: bounded comparison policy]
```

The `nodes/` location is intentional: these adapters know ROS wire semantics. No ROS imports should be added to pure `robot/sensors/`, `Measurement`, h/H models or filters just to make the directories look uniform.

| Component | Responsibility after migration | Explicit exclusions |
|---|---|---|
| Simulation builders/generators/nodes | Physical plugin setup; modeled imperfections; generation and delivery; preserve original source time and R | No estimator trust decision or filter observation Jacobian |
| Proposed `nodes/measurement_adapters.py` | Concrete wheel, joint, yaw-rate IMU, planar pose and target message conversion; names/frame/quaternion/value checks and construction of Measurement from explicit inputs | No subscription, timer, filter update, timestamp reorder, sensor RNG, retry or hidden TF lookup |
| `EstimationNode` | Subscription ownership; resolve TF at acquisition time; pass explicit transform to pose adapter; queue/drain; source health; reset; publish gates | No duplicate copy of moved conversion logic |
| Existing pure pose transform and h/H models | Current frame/covariance transformation and observation mathematics | No ROS messages, Gazebo dependencies or implementation selection |
| `RobotStateEstimator` / `PoseAcceptance` | Current fusion, bounds conditioning, NIS/consensus/trust/reacquisition | No sensor-generation noise/delivery behavior |
| Existing `SensorInterface` and wrappers | Preserve tested public imports and old raw-array semantics, clearly documented as not production ROS adapters | Do not silently redefine six-dimensional `ImuSensor` as yaw-only |
| M07 existing assembler | Resolve selected owner resources, validate compatible contracts; expose effective sensing inventory and provenance | No second editable source of settings, no new algorithm implementation |
| M08 existing configuration/materializer/runner/benchmark | Snapshot resolved selection, seeds and declared experimental differences; compare under explicit invariants | No blanket removal of measurement equality or claim of bitwise repeatability |

Extraction should share the existing `Measurement` representation, not add a second sensor sample hierarchy. Localization's pure covariance transform continues to be called once through its existing function. Checks at a public fusion boundary remain; repeated checks should only be removed if equivalence and all direct-call contracts are demonstrated. Retiring the legacy wrappers is a later API decision, not part of this bounded migration.

## 4. Configuration selection workflow

1. Choose an inherited M07 system manifest. Its existing `estimation.robot.config` selects algorithm assumptions/fusion policy, and `simulation.config` selects the existing bringup profile with producer references. Estimator `.type` selection remains algorithm selection, not a sensor type.
2. Use inherited owner files for experimental changes: localization imperfections in simulation's `config/sensors/`; fusion enable/age/assumed noise in estimation config; producer references in simulation deployment; composition references in bringup. Never copy physical geometry/limits into a sensor profile.
3. Preflight the assembled selection once through M07. Read a proposed computed inventory containing producer identity/presence, fusion subscription/enable, topic/type, frame/child, observation fields/dimension, R origin, source-time/latency/age/holdback contracts, seed consumer, mandatory-feedback status and the resource/override that supplied each effective value. These are proposed output fields, **not accepted YAML input keys**.
4. Choose M08 variants by `system_config`, with a common seed list and explicit actuation mode. Preserve resource hashes, all resolved values and the original full comparison contract. Show the sensor inventory in per-run metadata, derived from that same plan.
5. For a new provider, verify its declared message contract before treating it as equivalent. The initial design does not add a dynamic provider registry. Externally managed providers must be described in run provenance; managed provider lifecycle selection requires a separately scoped launch extension.

### Semantics and preflight

| Operation | Authority / present support | Proposed validation behavior |
|---|---|---|
| Enable simulated localization publisher | Existing `localization_sensor.enabled` under referenced simulation config | Inventory must report presence separately from fusion; enabled simulation source requires compatible physical truth input, as today. |
| Enable localization fusion | Existing estimator `localization_enabled` | Fusion off with publisher on is valid. Fusion on with no managed publisher is visibly external/outage-dependent, not automatically a healthy deployment. Do not forbid deliberate outages. |
| Choose equivalent producer | Currently resource choice for built-in generator; external producer starts outside M07 | Initially expose fixed producer identities and external-provider caveat; add managed launch selection only for a concrete replacement, not arbitrary class names. Reject competing publishers at runtime as today. |
| Change generator imperfections | Existing localization sensor noise/bias/delay/dropout fields | Validate modeled maximum delay against delivery age and effective reorder/admission budget. Preserve intentional outages/dropouts; warn/report expected unavailable full state rather than weakening trust. Include timer/transport budget, not just nominal latency. |
| Change assumed observation R | Existing estimator variances; localization transports R | Record generation and assumed R independently. Do not force equality or replace incoming full pose covariance with diagonal defaults. |
| Choose estimator | Existing M07 estimator `.type` plus owner config | Use current implemented types and constructor validation; sensor selection does not promise another filter exists. |
| Disable joint/wheel feedback | No independent supported active-control toggle | Fail preflight for a proposed active deployment that removes required feedback. Preserve actuator and full-state freshness gates; compute-only does not waive estimator requirements implicitly. |

The current schema contains **no top-level `sensors`/`sensing` section**, no general sensor implementation field and no per-source fusion switch for IMU/wheels/joints. A new editable manifest section is not necessary for the first migration. A future optional section should contain only selection references/intent if the computed view and inherited profiles prove insufficient; it must replace, rather than shadow, any previous selection authority. It is outside the counted proposal.

### Conceptual YAML example — proposed resources, not implemented

This one worked hierarchy illustrates selection without a new configuration language. Every key shown is an **existing schema key**, but the four named child files are conceptual and were **not created or executed**. The dropout value illustrates an experimental difference, not a recommended calibration. Source paths and `extends` are relative to the stated destination of each example file.

```yaml
# Proposed: src/robot_bringup/config/system_sensor_dropout_study.yaml
extends: system_manipulation_D_localized.yaml
simulation:
  config: {package: robot_bringup, path: config/reactive_manipulation_dropout_study.yaml}
# Inherited estimation.robot.config remains robot_localized.yaml:
# publisher selection changes; fusion/model/trust stay identical.
---
# Proposed: src/robot_bringup/config/reactive_manipulation_dropout_study.yaml
extends: reactive_manipulation_localized.yaml
simulation_config: config/scenarios/reactive_manipulation_dropout_study.yaml
---
# Proposed: src/robot_simulation/config/scenarios/reactive_manipulation_dropout_study.yaml
extends: reactive_manipulation_localized.yaml
localization_sensor_config: config/sensors/planar_localization_dropout_study.yaml
---
# Proposed: src/robot_simulation/config/sensors/planar_localization_dropout_study.yaml
extends: planar_localization.yaml
localization_sensor:
  enabled: true
  dropout_probability: 0.10
```

These are four separate files, not a multi-document input accepted by the loader. This chain is more verbose than a single toggle, but it keeps each current owner and installed resource convention intact. The computed view should make the resulting selection immediately visible.

For a fusion-only ablation, two analogous child files suffice: an estimator config extending `robot_localized.yaml` with `estimation_node.ros__parameters.localization_enabled: false`, and an M07 manifest overriding `estimation.robot.config` to that file. The publisher can remain on. To remove the publisher too, select a simulation child with `enabled: false`; that is a separate experimental choice. Never use publisher absence alone to claim fusion was disabled.

## 5. Example: add a genuinely new observation

Suppose a future source observes a scalar planar heading and carries covariance. First specify units, circular innovation, frame convention/extrinsic, acquisition epoch, latency, correlation/provenance, covariance validity and whether its availability is optional. Confirm that the existing robot state can express h(x); otherwise this is an estimator/state change outside this example.

Expected changes, each for a real responsibility:

1. Add one observation class to existing `robot/models/measurement_model.py` (or one dedicated pure model module if substantial). Define h, H and wrapped innovation consistently with the selected filter; do not subtract angles naively.
2. Add one concrete conversion function to proposed `nodes/measurement_adapters.py`. Preserve the incoming stamp/R and validate frames and dimensionality.
3. Extend `nodes/estimation_node.py` with subscription, source identifier, shared robot queue dispatch and reset/health bookkeeping. Do not create an independent unordered filter update path.
4. Wire the model/update in existing `robot/robot_state_estimator.py` and `factory/estimator_factory.py`; declare typed fields in `parameter_schema.py` and the selected owner YAML. Adding an h/H alone does not enable production fusion.
5. Extend M07 inventory/preflight in `robot_bringup/factory/system_configuration.py`. The manifest still selects the owner configuration through the existing reference.
6. If a simulated producer is actually needed, add a ROS-free generator under existing simulation `measurements/`, a transport node under `nodes/`, its owner YAML and explicit launch/setup entrypoints. Do not import that generator into estimation. An equivalent external publisher needs none of this synthetic implementation.
7. For benchmarked use, extend `robot_experiments/recording/normalization.py::STREAM_TYPES`, topic/context construction in `configuration.py` and source-appropriate readiness in `runner.py`. Add observation/covariance/frame/timing/reset/integration tests. Record absence explicitly for optional sources.

The minimal mathematical addition touches five existing estimator code files plus one owner config, and adds tests; composition and telemetry changes are separate. This is not a zero-code extension point today. A generic registry would not eliminate the need to define observation mathematics, safety requirements or source timing.

## 6. Example: compare two sensing configurations

Today an M08 experiment can select `system_manipulation_D_localized.yaml` and the conceptual dropout child through variant `system_config` references. It can run, snapshot each owner config and record localization observations. However, the comparison contract includes the complete generator config and will mark these variants incomparable.

Proposed bounded fix within existing M08 files:

- Add an **optional, explicitly versioned comparison intent** in the existing experiment schema. Its name/keys must be chosen during implementation; there are no accepted comparison-intent YAML keys today. Declare exact varied owner paths, such as `localization_sensor.localization_sensor.dropout_probability`, and report before/after values. For fusion ablations, declare estimator `localization_enabled` independently.
- Keep strict matching as the default, including for all historical experiments without that declaration. Keep the original complete contract and its hash in outputs. Derive a second invariant comparison view that masks only validated, declared sensing differences; reject undeclared differences. Do not simply remove all measurements from the contract.
- Never allow this intent to mask physical description, actuator limits/backend/holding, target dynamics/initial conditions, environment, tracking context/provenance, world/terrain, integration fidelity, duration/settling or measurement windows. Changed target measurement noise may be a declared variable; changed truth or time-window semantics may not.
- Snapshot producer identity, managed/external status, source contract and both generation/assumed covariance. Keep existing target measurement seed and independently derived localization seed; equal seeds do not imply identical accepted samples once dropout logic differs.
- Report readiness, source counts/ages, trust withholding and safety independently of tracking means. Never infer an unconditional winner from missing required streams, physical overspeed or failed acceptance.
- In reactive experiments, sensing changes may change robot motion and therefore the target's realized trajectory. Report this coupling; matched configuration/initial conditions are not a claim of identical target trajectories or paired individual samples.

Existing recorder already preserves covariance-bearing localization Odometry and its source/arrival timing. IMU/TF recording and a new external-provider manager are **not** prerequisites for this limited localization comparison, and are not included in its implementation count. They become requirements for experiments whose evidence depends on those streams.

## 7. Migration sequence and compatibility gates

| Priority | Proposed work | Value / risk | Required evidence before completion |
|---|---|---|---|
| 1 | Add derived sensing inventory/provenance to existing M07 plan/startup summary; mark old wrappers' actual status | Immediate discoverability, additive behavior; avoid stale duplicated settings | N/L resolve to identical numerical inputs, current types/topics/frames and physical config; inventory reflects overridden values and absent IMU producer. |
| 2 | Extract concrete conversions; preserve compatibility wrappers/imports | Reduces node responsibilities; greatest risk is altered rejection/state-mutation order | Old vs extracted adapters yield identical z/R/stamp and rejection outcomes; filters, queue, raw-joint checks and publication gates unchanged. |
| 3 | Add explicit M08 sensing comparison intent and report invariant/differing contracts | Enables the demonstrated missing experiment use case | Strict default unchanged; declared source changes allowed; undeclared/physical/time changes rejected; historical inputs/results remain readable and untouched. |
| 4 | Consider wrapper retirement or managed replacement provider only on demonstrated usage | Optional, outside bounded proposal | External API audit and concrete replacement contract/lifecycle tests. No speculative registry. |

### Required tests before any future migration is accepted

These are proposed implementation gates, **not tests claimed to have run during this review**:

1. Wheel/joint/IMU/target/pose adapter equivalence on valid fixtures, nonfinite values, malformed sizes, duplicate/missing joint names, bad frame/child, quaternion and covariance. Explicitly test scalar production IMU versus six-dimensional legacy wrapper.
2. Acquisition timestamps survive delayed delivery, conversion and TF lookup. Mixed-source order, duplicate/future/expired arrivals, buffer overflow and clock reset preserve current `test_timestamp_buffer.py`, joint-bound conditioning and target chronology behavior. Never accept stale data merely to increase counts.
3. Pose transform/Jacobian/covariance tests, yaw wrapping, NIS/consensus/reacquisition and publication trust tests remain unchanged. Existing suites include `src/robot_estimation/test/{test_planar_localization,test_pose_robustness}.py` and simulation `test_planar_localization_sensor.py`.
4. Physical/TF contextual history remains causal and bounded, valid through integration substeps; partial progress, pose acknowledgement, reset epochs and irrecoverable-gap behavior unchanged. Existing `test_tracking_frame_provider.py` and context tests remain mandatory.
5. Full-state mandatory joint/wheel age/raw-bound gates, controller stale/invalid handling, duplicate/older cache preservation, command session/replay/timeout and actuator feedback checks continue to pass. A subscription or enabled publisher must not manufacture source health.
6. M07 inheritance, declaring-origin resource resolution, conflicting selection, unknown keys, external/outage ambiguity and mandatory-feedback cases; old launch arguments and `publish_commands=false` retain behavior. Run existing architecture/dependency tests: no ROS in pure cores, no simulation dependency in estimation, compatibility imports remain available.
7. M08 materialization preserves source resources, inheritance hashes, resolved sensor settings and seed derivation. Old experiment defaults and outputs remain compatible. New comparison tests cover declared-only differences and immutable physical/time contracts. Do not overwrite earlier result directories.
8. Build/isolated-install resource checks, then short fresh headless N and L integration runs and a bounded two-variant localization comparison. Check graph/provenance, acquisition/arrival ages, source acceptance/trust, reset, fail-closed stale-source behavior and owned teardown. Exact replay of current physical-limit tests is required if extraction accidentally changes feedback handling; no physical changes are planned.

Functional, architecture and installation checks are the gates; existing style failures must be identified separately. No new tracking-score threshold or demand that a noisy source improve performance is introduced.

## 8. Contract preservation checklist

| Contract | Migration rule |
|---|---|
| Causal acquisition timing | Keep original header stamp, shared robot reorder buffer, distinct target chronology and per-source ages; preserve delivery/admission differences. |
| Frames/extrinsics | Use description mount, joint ordering and stamped TF; preserve world/odom/base distinctions and selected +X tracking axis. No implicit optical-axis change. |
| Covariance | Preserve full pose covariance transform and production R sources, including deliberate generator/assumed-R differences and PSD/SPD distinctions. No global recalibration. |
| EKF observation math | Keep h/H, process noise, Joseph update, yaw innovation and joint Gaussian conditioning unchanged. |
| Localization reliability | Keep consensus, trust expiry/reacquisition, recovery covariance policy and full-state withholding unchanged. |
| Sensor imperfections | Preserve RNG consumption/reset, calibration bias, latency/jitter, dropout/outages/outliers; seed the same selected owner, no extra draws in adapters. |
| Truth authorization | Estimator/control never subscribe to physical truth as a measurement. Generator truth use and target physical context remain explicit, separately recorded. |
| Feedback/safety | Preserve actuator-level limits/holding, PositionGuard, feedback freshness, command ownership/replay/timeouts; a fusion toggle cannot disable physical safety feedback. |
| Clock/lifecycle | Reset all existing epochs/caches/pending work; retain essential-process failure and verified owned cleanup, including external providers if later managed. |
| Reproducibility | Preserve immutable historical artifacts and strict default comparison; add resolved selection and declared differences without promising bitwise ROS/contact/reactive replay. |
| Physical vs estimated state | Label source provenance in inventory, telemetry and comparison; derived FK/context is not independent fusion evidence. |

## 9. Scope in files/components

The following is a concrete review-sized proposal, not a time estimate or permission to implement it. Counts exclude the two documentation files already created by this review.

| Component | Existing production files proposed to edit | New production files |
|---|---|---|
| Estimator conversion and compatibility labeling | `nodes/estimation_node.py`; `interfaces/sensor_interface.py`; `robot/sensors/imu.py`, `joint_encoder.py`, `wheel_odometry.py` (5) | `robot_estimation/robot_estimation/nodes/measurement_adapters.py` (1) |
| Assembled inventory/preflight/startup display | `robot_bringup/robot_bringup/factory/system_configuration.py`; `robot_bringup/launch/closed_loop.launch.py` (2) | None |
| Declared sensor comparison and persisted selection | `robot_experiments/robot_experiments/configuration.py`, `runner.py`, `benchmarking.py` (3) | None |

**Core total: 10 existing production files + 1 new production module, across three existing packages.** Compatibility-only files receive documentation labels, not changed array semantics. **No new ROS package, registry, ABC or editable configuration layer.** No simulation production changes are needed for the reviewed localization use cases.

Tests: modify four existing suites (`robot_bringup/test/test_system_configuration.py`; `robot_experiments/test/test_configuration.py`, `test_reporting_benchmark.py`, `test_runner.py`) and add one `robot_estimation/test/test_measurement_adapters.py`: **4 existing + 1 new test file**. Keep `test_robot_sensors.py` and architecture import tests passing without rewriting them to pretend the wrappers are production-wired. Further test changes require demonstrated failures or a separately approved scope.

The conceptual dropout example would add **4 optional config resources** at the exact locations in its YAML block. Those are not needed for conversion extraction and were not created. A persisted two-variant M08 experiment using them would add **1 optional experiment YAML** under existing `robot_experiments/config/` (file name chosen when the experiment is authorized). Existing package resource installation already covers those config directories. Thus the bounded example configuration adds at most five files; it is not a mandatory configuration-system migration.

Not counted: a genuinely new observation, IMU producer, IMU/TF recorder streams, external-provider lifecycle selection, API deletion, generated test evidence or a new comparison UI. Those are separate features with separate acceptance needs.

## 10. Changes not recommended

- No `robot_sensors` package now; no wholesale file moves or compulsory sensor plugin abstraction.
- No new competing system/experiment configuration entry, duplicated authoritative sensor catalog or copy of physical limits.
- No conversion of compatibility `ImuSensor` into a scalar adapter under its existing name; no assumption that mounting a visual camera creates images.
- No collapsing generator effects, ROS parsing, h/H and source trust into a single sensor class.
- No extra simulator truth input to filters/control; no treating physical context or derived tracking TF as independent estimated observations.
- No change to PID/servo settings, physical limits, physics timestep/fidelity, estimation/prediction math, safety thresholds or target dynamics.
- No automatic synchronization of generator noise and assumed R, no tuning to improve this review's verdict, no universal covariance validation rule that changes current mathematical contracts.
- No removal of timestamp checks, relabeling old data with receipt time, silent target restart or weakening source trust to make experiments look active.
- No weakening benchmark comparability globally; no rewriting historical evidence or declaring a winner because one configuration has missing feedback.

The existing simulation generators, filter mathematics, localization trust/recovery policy, robot description, target-context histories and actuator safety infrastructure require **no redesign** for the recommended migration. The remaining risks are conversion/rejection equivalence, covariance/time semantics, public wrapper compatibility and over-permissive comparison intent. Those are specific, testable risks rather than reasons to reorganize the whole testbed.
