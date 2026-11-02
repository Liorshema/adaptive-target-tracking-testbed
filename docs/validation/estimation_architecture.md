# Task E — estimation algorithm, model and role separation

This records Task E's acceptance boundary. The later intrinsic-parameter transport
closure is documented separately in [Task E.1](estimation_parameters.md).

Current deployed separation and numerical compatibility are verified. Overall
verdict is **CONDITIONAL**: the architecture supports the bounded deployments and
parameterless compatible extensions below; future intrinsic algorithm settings
need explicit frozen role transport, and runtime acceptance is reported separately.
No Dashboard, new filter/sensor, controller/predictor tuning, physics/limit changes,
commit or push is included.

## Architecture and ownership

The existing numerical `KalmanFilter` and `ExtendedKalmanFilter` were already
role-independent; their source is unchanged. Task E makes that reuse explicit in
`factory/algorithm_factory.py`, using the existing `ComponentRegistry`, trusted
`ModelRegistration` declarations and sorted package-local discovery. Neither
algorithm assumes a role, ROS message, four joints, six states or an experiment ID.
The registry validates initial distributions and model dependencies; callable
outputs are checked when used, without replacing/symmetrizing matrices or evaluating
the robot covariance callback before its filter exists.

`factory/model_factory.py` owns registered physical process/measurement construction
and deployment contracts. `factory/estimator_factory.py` resolves validated bindings
and injects the chosen algorithm into existing adapters. The robot adapter accepts
an optional `filter_factory`; its historical direct-constructor fallback remains.
The target adapter still delegates filtering. Sensor routing, pose acceptance,
recovery, joint conditioning and timestamp policies retain their existing owners.

The ROS node changes are confined to three methods: optional typed binding
parameter declaration and forwarding to each existing factory. Clock resets use
those same calls. Every other node method is AST-identical to the captured baseline.
No ROS messages or experiment-runner branches changed.

| Responsibility | Canonical `estimation.yaml` section |
| --- | --- |
| Numerical method | `estimators.kf.parameters`, `estimators.ekf.parameters`; both currently `{}` |
| Process assumptions and Q | `process_models.robot_motion`, `process_models.target_cv` |
| Measurement mappings and assumed R | `measurement_models.robot_sensors`, `measurement_models.target_position` |
| Mean and covariance initialization | `initialization.robot_state`, `initialization.target_state` |
| Conditioning/pose acceptance settings | `deployment_adapters.robot_state`, `deployment_adapters.target_state` |
| Explicit algorithm-free resource bundles | `bindings.robot_state`, `bindings.target_state` |
| Topics/frames and scheduling/freshness | `transport`, `runtime`; existing `profiles.pose_aiding` delta retained |

The owner schema is version 1. Each binding names its role, process model,
measurement model, initialization resource and adapter. A base composition explicitly
selects that resource group; the package does not select a robot/target algorithm.
Old role-centered owner YAML and flat ROS snapshots remain readable. The public
factory default arguments retain their historical library compatibility; running
systems still take explicit estimator selections from experiment composition.

Robot Q uses continuous white-acceleration density amplitudes with integrated
`dt^3/3`, `dt^2/2`, `dt` blocks and the existing state-dependent planar terms.
Target Q uses its distinct discrete acceleration sigma with `dt^4/4`, `dt^3/2`,
`dt^2` blocks. Q/R values, Joseph updates and discretization were not unified or
retuned. Former factory float conversions are preserved before squaring model
amplitudes, including large integer inputs. Joint bounds remain description-owned.

## Selections, compatibility and authoring

| Public role | Canonical selection | Preserved historical selection | Supported binding |
| --- | --- | --- | --- |
| Robot | `ekf` | `robot_ekf` | `robot_motion` + `robot_sensors` + `robot_state` |
| Target | `kf` | `kalman` | `target_cv` + `target_position` + `target_state` |

Historical aliases resolve explicitly to the same algorithm and physical binding;
no historical IDs, immutable revisions, source hashes or experimental results were
rewritten. Process/measurement dimensions, ordering, available model contracts,
Jacobians, covariance shapes, adapter requirements and existing sensor/frame/time
checks reject incompatible combinations before launch. Robot wheel/joint routing
and optional IMU/pose aiding remain unchanged; absent observations are not invented.

A normal public one-file experiment is:

```yaml
schema_version: 1
experiment:
  id: EXP-ESTIMATION
  name: Canonical estimation
  base_case: EXP-NOMINAL
  components:
    robot_estimator: ekf
    target_estimator: kf
```

It inherits the base model/sensor/initialization binding. Advanced authors may add
`experiment.estimation_bindings.<role>` with required `process_model` and
`measurement_model`, plus optional `adapter`. These become existing typed manifest
parameters and survive flat snapshots and ROS reconstruction. No imports come from
YAML. Changing a compatible algorithm preserves inherited physical-model overrides;
ordinary component changes retain the existing reset policy.

The public registry inventory now has 14 families and 30 entries, including aliases.
Bare `estimator`, `estimation_process`, `estimation_measurement`, and
`estimation_adapter` components require composition; they are not directly deployable.
Supported role facades retain conditional scientific status.

A compatible parameterless algorithm can be declared in one trusted file under
`robot_estimation/filters`, exporting `COMPONENTS`, with one owner
`estimators.<id>.parameters` group. It must satisfy the selected adapter's declared
constructor/update capabilities. Existing discovery, Author, Catalog and M07 create
it without runner changes. Source tests demonstrate this with an isolated local
registration, real filter construction and retained model noise overrides. The
same numerical KF/EKF implementations also operate on independent test-only models
at different dimensions and observation mappings; these tests do not advertise
production robot KF or target EKF support.

## Verification

Evidence is under ignored `results/estimation_architecture/`, separate from source.

| Check | Actual result and source |
| --- | --- |
| Frozen legacy replay | **124,305/124,305 values exactly equal**, maximum difference **0**; `numerical_after.log` |
| Canonical replay | **124,305/124,305 values exactly equal**, maximum difference **0**; `numerical_canonical.log`, final `numerical_closed.log` |
| Numerical coverage | Normal robot, legacy pose gate, consensus/recovery and target sequences; predicted/corrected moments, A/F/H/Q/R, innovations, S/K, independent Joseph candidates, conditioning, accepted/rejected timing, buffer/reset behavior |
| Direct / scientific preparation | **75 / 21 equivalent**, zero differences; `direct_equivalence.json`, `preparation_equivalence.json` |
| Final scoped regression | **186 passed**, `scoped_closed.log`; includes estimation and public authoring |
| Full functional regression | **1,940 passed, 18 style wrappers deselected, 1 existing xfailed**, 1037.81 s; `regression.log` |
| Author/Catalog | Preview/validation, canonical and historical selections, explicit bindings, immutable revisions, reset, provenance, parameter isolation and actual construction passed |
| Historical owner format | Four group projections and six estimation registries equal to modern representation; `historical_owner_verification.json` |
| Builds | Normal and copied **10-package builds passed**; affected-package refreshes passed after final fixes; `build.log`, `copied_build.log`, `build_closed.log`, `copied_build_closed.log` |
| Copied installation | **26 module origins**, 14 families / 30 components, M06–M09 chain, preserved Task B math/Author fixtures, new canonical Author/Catalog workflow passed; `installed_*verification.json` |
| Copied bytes | **8 changed/added production modules and 20 owner YAML resources** identical; `installed_bytes_closed.log` |
| Real ROS node | Six explicit selectors declared, canonical filters constructed, valid reset and invalid reset binding rejection passed from copied prefix; `ros_bindings_verification.json` |
| Style | Eight implementation/test files flake8-clean, two new production modules pep257-clean; 146 inherited findings in four legacy files unchanged, zero new findings there; `style_closed.log`, `docstyle.log`, `lint_existing_verification.json` |
| Preservation | 21 immutable revisions and 14,242 prior artifacts intact; 28 original YAML files byte-identical; only owner estimation YAML and installed authoring example intentionally changed; `change_manifest.json` |

Test counts overlap. The final focused suite covers review corrections and the
last capability-boundary case added after full-suite collection. No failed
functional assertion or numerical tolerance was waived. Review findings and
red/green evidence remain in `review.md`, `review_disposition.md` and `*_red.log`.

## Remaining boundaries

Current production robot/target adapters retain their real state, frame, sensor
and output restrictions. Numerical reuse is broader than ROS deployment support.
New physical model families with new parameter names/sources/adapter protocols
require deliberate schema/registration work; arbitrary layouts are not advertised.

Intrinsic KF/EKF settings are currently empty. Future algorithms with nonempty
intrinsic parameter defaults are **library-only**: their role facades are marked
nondeployable and reject construction until a validated per-role parameter map is
frozen into snapshots and forwarded by the ROS adapter. This preserves scientific
reproducibility rather than silently reading later installation defaults. That
extension transport is not implemented here; current canonical KF/EKF selections
and compatible parameterless registration are verified. This is the remaining
architecture limitation for the full general extension mission.

Existing localization/slip, covariance, IMM/runtime-readiness and hardware-safety
limitations are not closed by this architectural migration. No test-only numerical
example is certified for Gazebo or hardware. Live acceptance follows below.


## Live copied-install result

Exactly **one** headless trial ran after builds/tests exited: isolated
`EXP-ESTIMATION-CANONICAL` revision 1, inherited from unchanged `EXP-NOMINAL` revision
3, seed 42. It selected real `ekf`/`kf` role facades with all six explicit existing
model/adapter IDs. Original physics, actuation, readiness, scientific and safety
gates were retained. **Completed; accepted; exit 0.** No retry was performed.

| Scored observation | Actual result |
| --- | --- |
| Evaluation | Simulation 32.250–38.250 s; 6 s after 3 s settling |
| Absolute tracking-distance error, mean / max | **0.131669 / 0.196243 m**, 419 aligned samples |
| Pointing error, mean / max | **12.1225 / 18.8323°** (0.211578 / 0.328686 rad) |
| Scored RTF | **0.797372** |
| Controller computation, mean / max | **23.806 / 48.507 ms** |
| Physical joint position/speed overshoot | **0 rad / 0 rad/s**; complete observed coverage over the 6 s window |
| Invalid commands / watchdog / rejected commands / feedback rejections | **0 / 0 / 0 / 0** |
| Cleanup | All launch children exited; no survivors; final global process scan clear |

Evidence: `runtime_summary.json`, `runtime_stream_counts.json`, and
`runtime/nominal_20261028T063008_824189Z_xvwjb6fw/` with frozen inputs, telemetry,
metrics, gate decisions, source metadata and logs. Physical observations passed
the unchanged 0.001 rad / rad/s tolerance and coverage policy. This short result
is not hardware certification or evidence that historical timing/localization/IMM
limitations are resolved. Successful availability/safety gates are not a new
tracking-quality threshold. The readiness origin was 29.250 simulated seconds;
startup/source-state variability remains. There is no matched before/after live
tracking comparison; deterministic replay provides the numerical equivalence proof.

**Verdict: PASS for the current KF/EKF separation, numerical/historical preservation,
canonical authoring, and this production integration attempt. CONDITIONAL for the
full general extension mission and research runtime reliability**, because future
nonempty intrinsic settings still require frozen role transport and a single
short run cannot close existing scientific limitations.

## Reproduction

From `/home/user/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -p no:launch_testing -p no:launch_ros --import-mode=importlib -q src/robot_estimation/test src/robot_experiments/test/test_estimation_authoring.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -p no:launch_testing -p no:launch_ros --import-mode=importlib -q src tools/validation -k 'not flake8 and not pep257 and not copyright'
```

The exact retained live selection can be launched again, using a fresh output:

```bash
source /tmp/estimation_architecture_install/setup.bash
ros2 run robot_experiments run --catalog /home/user/ros2_ws/results/estimation_architecture/canonical_catalog.yaml --experiment EXP-ESTIMATION-CANONICAL --revision 1 --output /tmp/task_e_reproduction
```

The catalog and evidence are local generated artifacts; transfer them with the
scientific record. Use a fresh isolated catalog when rerunning author-save checks,
retaining older attempts. All copied-install/source equivalence, numerical replay,
ROS boundary and smoke commands are recorded by the scripts under
`results/estimation_architecture/`; `verification_commands.txt` lists them.
The source installed example `robot_experiments/examples/tracking.yaml` is a separate
canonical-ID CV/DLS authoring example, not the nominal condition measured above.


## Exact Task-E-relative changed files

Relative to the captured completed Task B state, not Git HEAD:

```text
M docs/component_configuration.md
M docs/experiment_composition.md
M src/robot_estimation/config/estimation.yaml
M src/robot_estimation/robot_estimation/configuration.py
M src/robot_estimation/robot_estimation/factory/estimator_factory.py
M src/robot_estimation/robot_estimation/nodes/estimation_node.py
M src/robot_estimation/robot_estimation/parameter_schema.py
M src/robot_estimation/robot_estimation/robot/robot_state_estimator.py
M src/robot_estimation/test/test_component_architecture.py
M src/robot_experiments/examples/tracking.yaml
M src/robot_experiments/robot_experiments/authoring.py
A docs/validation/estimation_architecture.md
A src/robot_estimation/robot_estimation/factory/algorithm_factory.py
A src/robot_estimation/robot_estimation/factory/model_factory.py
A src/robot_estimation/test/test_algorithm_factory.py
A src/robot_estimation/test/test_estimation_bindings.py
A src/robot_experiments/test/test_estimation_authoring.py
```

**11 modified, 6 added, 0 deleted.** Prior Task A/B edits and scientific evidence
remain preserved. `change_manifest.json` records final hashes and the node-method
audit; `before.tar.gz` / `source_after.tar.gz` retain source snapshots. No commit or
push was performed. Task E stops here; Dashboard work is not started.
