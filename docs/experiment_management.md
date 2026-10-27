# Experiment management and configuration ownership

Updated 2026-10-27. This guide supersedes the transitional ownership description
from the catalog milestone. Current migration verification is recorded below;
short integration runs do not replace the historical physical-safety, localization
or R1C scientific-coverage verdicts. No dashboard was implemented.

Canonical model-family structures, schemas, extension interfaces and headless
discovery are described in [the component configuration guide](component_configuration.md).
The counts and paths in the historical migration sections below describe the
earlier ownership migration. Current revision-3 composition, the removal of 42
redundant YAMLs, standalone launch migration and headless workflows are authoritative
in [experiment composition](experiment_composition.md) and its
[verification report](validation/experiment_composition.md). Removed generic
bringup file paths below must first be frozen with `run compose`.

## Ownership and resolution

| Owner | Responsibility |
|---|---|
| `robot_description` | Authoritative physical geometry, joints, limits, mounts, collision and inertial data |
| `robot_estimation/config/estimation.yaml` | Robot EKF and target KF defaults, optional pose-aiding calibration delta |
| `robot_prediction/config/prediction.yaml` | Predictor, adaptation, fusion, history and confidence numerical defaults; no selected model |
| `robot_control/config/control.yaml` | Separated controller, task, tracking, reference, transport and runtime defaults; no selected controller/model/reference |
| `robot_simulation` | Reusable physics, target dynamics/fields, synthetic sensing, transport, PID/holding and actuation integration |
| `robot_experiments` | Explicit component/scenario/runtime selection, calibrations, versioned studies, overrides, execution, metrics and history |
| `robot_bringup` | Shared validated assembly, deployment launch mechanics, TF and read-only visualization; no experiment default |
| `robot_configuration` | Resource lookup, inheritance, trusted registries, schemas and detached overrides |

The single validated boundary is
`robot_bringup.factory.system_configuration.assemble_system_configuration(document,
origin, share, overrides=None)`. `load_system_configuration(path, share, overrides)`
loads files into that same boundary. Both invoke the actual owner factories and
validate physical, dimensional, joint, topic, frame, time and actuator contracts
before launching. `origin` identifies provenance and the base of relative resources;
it need not be a file. Inputs are copied and the plan retains its resolved manifest.
There is no second experiment-specific factory or model interpreter.

Resolution is selected named owner defaults → composition calibration → experiment
owner overrides → explicit variant assignments → seed/runtime wiring. Mappings
merge recursively; lists replace completely. An inherited replacement resource
without a declared group clears the inherited group. Unknown parameters/types,
nonfinite values, incompatible selections and conflicting descriptions fail before
process creation. Owner overrides cannot smuggle component selection into control
parameters. Launch runtime overrides retain precedence; actuation is opt-in.

Experiment selection is explicit:

```yaml
schema_version: 2
# Other required robot, simulation and runtime sections omitted in this excerpt.
estimation:
  robot:
    type: robot_ekf
    config: {package: robot_estimation, path: config/estimation.yaml, group: robot_ekf}
  target:
    type: kalman
    config: {package: robot_estimation, path: config/estimation.yaml, group: kalman}
prediction:
  type: imm
  config: {package: robot_prediction, path: config/prediction.yaml}
  adaptation: residual
  fusion: weighted_correction
control:
  type: weighted_least_squares
  config: {package: robot_control, path: config/control.yaml, group: parameters}
  strategy: {package: robot_control, path: config/control.yaml, group: distance_pointing}
  adapter: planar
  reference: {geometry: sphere, policy: radial}
```

For optional pose aiding select ordered groups `[robot_ekf, pose_aiding]` and the
explicit activation parameter. Description binding derives from the selected robot
unless explicitly supplied; competing physical descriptions are rejected. Numeric
values were retained. WLS controller type now reflects the actual selected factory,
instead of an inert `jacobian` parameter shadowed by a strategy. A JSON strategy
mapping carries the validated selection into the existing control ROS adapter;
standalone strategy files still use the same interpretation and validation.

Current deployed choices are robot EKF / target KF, IMM or CV, residual with
weighted correction or none/none, DLS or WLS, and the existing planar robot/reference
contracts. Standalone CA/CT require richer input than deployed `[p,v]`; RLS and
other library capabilities are not advertised as selectable deployed models.
Prediction construction requires explicit selection, including clock-reset recovery.

## Historical ownership-migration tree and removals

```text
robot_estimation/config/estimation.yaml
robot_prediction/config/prediction.yaml
robot_control/config/control.yaml
robot_bringup/config/
  tracking_visualization.yaml       # presentation only; RViz resources also remain
robot_experiments/config/
  experiments.yaml                  # seven IDs, immutable r1 + canonical r2
  manipulation_analysis.yaml        # independent evaluation policy
  launch_profiles.yaml              # explicit direct-launch cases and allocations
  calibration/
    simulation.yaml
    manipulation_pose.yaml
  systems/
    nominal_tracking.yaml
    cv_prediction.yaml
    dls_control.yaml
    mobile_manipulator_pose_tracking.yaml
    localization_noise_study.yaml
    manipulation_B.yaml
    manipulation_C.yaml
    manipulation_D.yaml
    manipulation_E.yaml
robot_simulation/config/             # 39 meaningful reusable domain resources
```

| Package configuration YAMLs | Task start | Current |
|---|---:|---:|
| Estimation / prediction / control | 3 / 1 / 5 | 1 / 1 / 1 |
| Bringup | 34 | 1 |
| Simulation | 39 | 39 |
| Experiments | 2 | 14 |
| All packages | 87 | 60 |
| All source YAMLs, including invalid test fixture | 87 | 61 |

Forty obsolete source files were removed: 14 bringup manifests, 15 forwarding
profiles, four calibrations, three split estimator defaults and four strategy
wrappers. Nine experiment compositions and explicit case groups replace duplicate
selectors. Four calibrations reduce to two, with physical description binding
derived instead of repeated in wrist wrappers. The invalid production example is
now a noninstalled test fixture. Forty corresponding stale generated symlinks were
removed. Simulation's 39 resources were retained because they represent distinct
physical deployments, sensor models, scenarios, fields or numerical settings;
removing them solely to reduce counts would lose meaningful responsibilities.

The [dependency inventory](../results/ownership_migration/wrapper_audit.md),
[exact removal manifest with replacement and hashes](../results/ownership_migration/removed_files.json),
[generated-link manifest](../results/ownership_migration/removed_cache_links.json) and
[reference scan](../results/ownership_migration/reference_scan.json) preserve the
removal rationale. These local ignored results do not ship with a source clone.
Historical reports describe their original paths; current launch consumers,
installation rules, probes and tests use the new owners.

## Catalog, defaults and immutable run inputs

The shipped catalog is read-only through its API. Each original revision 1 retains
its exact definition and hash; new revision 2 changes only canonical resource
ownership. Thirteen explicitly enumerated old bringup resource URIs relocate old
revision inputs to canonical experiment compositions. This bounded historical
bridge supplies no model defaults or alternate selection logic. The seven old
M08 study entry names likewise import their exact catalog records; arbitrary
external filenames are not guessed. External M08 study documents and old saved
schema-1 inputs remain readers through the same validator, not a second shipped
selection system. New runs emit one schema-2 `inputs/system.yaml`, no forwarding
`inputs/profile.yaml`.

Revisions store self-contained study definitions, owner overrides, UTC creation
time and canonical hashes. Writes use locking, atomic replacement/fsync and
expected-revision checks. Old revisions cannot be overwritten; purged IDs remain
retired. Package defaults never change during edits, reset or execution. Drafts,
prediction defaults and resolved mappings are detached values. Reset removes draft
owner overrides while preserving the explicitly selected composition.

Override owners are `prediction`, `estimation`, `control`, `strategy`, `target`,
`environment`, `integration`, `measurements` and `localization_sensor`. For example,
`prediction: {imm: {likelihood_variance: 0.5}}` replaces the package default `1.0`
in the canonical run-local `inputs/prediction.yaml` (under
`predictors.imm.parameters`); the historical override key is normalized at the
owner boundary. Canonical override keys also work. It writes no source resource. Unknown
keys/types and nonfinite values are rejected, then real owner factories validate.
Assignments remain a separate explicit scientific-comparison contract.

Each run records ID/revision/hash, unique run UUID, effective owners, component
identities, source/environment identity, sensor seeds, timestamps and artifact
references. Description, prediction, strategy, estimator/control, sensing,
target/environment/integration, terrain, holding and actuation values execute
from detached run inputs. Separate external strategy files retain original hashes
and inheritance ancestry as well as effective snapshots. Code, Xacro templates
and native dependencies retain source fingerprints; the snapshot is not a complete
offline software distribution.

## Historical ownership-migration usage (superseded paths)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robot_experiments --executor sequential
source install/setup.bash
ros2 run robot_experiments run catalog list
ros2 run robot_experiments run --experiment EXP-PREDICTION-COMPOSITION --revision 2 --validate-only
ros2 run robot_experiments run --experiment EXP-PREDICTION-COMPOSITION --revision 2 --explain
ros2 run robot_experiments run --experiment EXP-PREDICTION-COMPOSITION --revision 2 --output results/experiments

# Continuous simulation, compute only unless explicitly enabled:
ros2 launch robot_experiments tracking.launch.py \
  system_config:=config/systems/nominal_tracking.yaml gui:=false
# Equivalent old reactive case selection, with physical commands:
ros2 launch robot_experiments tracking.launch.py \
  case:=reactive_tracking_oscillatory controller:=whole_body \
  enable_actuation:=true gui:=false
```

Old bringup `config:=config/<case>.yaml` moves to experiment `case:=<case>`;
old `strategy_config:=config/whole_body*.yaml` moves to the matching
`controller:=whole_body*` allocation name. Explicit manifests contain their own
controller choice; combining `system_config` with `controller` is rejected. The
default direct case remains the existing DLS compute-only scenario. All 15 cases
and four distinct allocations share the same assembler. External flat strategy
paths remain explicit overrides. Generic bringup requires an explicit manifest:

```bash
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=$(ros2 pkg prefix robot_experiments)/share/robot_experiments/config/systems/nominal_tracking.yaml
ros2 launch robot_bringup visualization.launch.py \
  system_config:=$(ros2 pkg prefix robot_experiments)/share/robot_experiments/config/systems/nominal_tracking.yaml \
  gui:=true rviz:=true
ros2 launch robot_simulation simulation.launch.py gui:=false
```

Standalone simulation retains permanent gravity holding and its physics defaults.
Continuous visualization reads the same explicit composition; no second simulation
or experiment policy is hidden in bringup.

```bash
ros2 run robot_experiments run catalog init --catalog results/catalog/experiments.yaml
ros2 run robot_experiments run catalog show --catalog results/catalog/experiments.yaml \
  --experiment EXP-NOMINAL --revision 1
ros2 run robot_experiments run start --catalog results/catalog/experiments.yaml \
  --experiment EXP-NOMINAL --revision 2 --jobs results/jobs --output results/experiments
ros2 run robot_experiments run status JOB_UUID --jobs results/jobs
ros2 run robot_experiments run stop JOB_UUID --jobs results/jobs
ros2 run robot_experiments run history --results results/experiments --experiment EXP-NOMINAL
```

`catalog create/revise` takes a self-contained study `--definition` and optional
owner `--overrides`; revise/delete/purge require `--expected-revision`. Extract
`definition` from a show record rather than using the full record as a study.
A background job freezes its accepted revision; later catalog changes do not alter
it. Stop cooperatively cancels at safe boundaries, preserves partial artifacts,
aborts remaining trials and is idempotent. PID creation-time identity, owned
processes, private domains, timeouts and safety remain intact. No force-kill policy
was introduced. Deleting or purging definitions never deletes run artifacts.

Repeat a revision for another independent run identity; explicit unique seed lists
retain the existing trial policy. History returns stored artifacts without metric
recomputation. Pre-catalog histories remain readable at their original paths and
are not assigned invented identities. Equal sensor seeds do not imply identical
reactive trajectories, and config compatibility is distinct from scientific
acceptance.

## Verification scope

Before changes, the task captured all nonignored sources, dirty state, YAML inventory,
historical artifact stats and 21 complete preparations. Evidence is in
[`results/ownership_migration/`](../results/ownership_migration/); prior
[`results/experiment_architecture/`](../results/experiment_architecture/) evidence
is preserved separately. Compare this task against its saved initial dirty source,
not only Git HEAD.

All 21 preparations retain identities, values, seed consumers and scientific
contracts. Equivalence normalization is limited to the old inert `jacobian` field
under a selected WLS strategy and the new empty strategy transport field; the
comparison invariant hash is recomputed only after those explicit normalizations.
Runtime hashes/metrics/gates are not rewritten. No gains, equations, physical
limits, frame conventions, filter parameters, physics or tolerances were tuned.

| Verification | Actual result |
|---|---|
| Final full functional regression, after review corrections | 1,621 passed; 18 style tests deselected; one expected failure |
| Review corrections | Two failing regressions reproduced, then 108 targeted tests passed |
| Final fixture formatting checks | 74 passed |
| Scientific preparation equivalence | All 21 frozen cases equivalent |
| Normal symlink build | Ten packages built; four affected Python packages refreshed |
| Fresh copied build | Ten packages built from a new build directory; all ten refreshed after final corrections |
| Copied installation verification from `/tmp` | Ten package origins/resources, eight console entries, native backend, Xacro limits, M07/M08 contracts, M09 launch/modules passed |
| Independent review | No critical/important defect; two minor group/provenance gaps reproduced and fixed |
| Preservation audit | 781 baseline files accounted for; 40 deliberate removals; 12,252 historical artifact records and 316 protected files unchanged; HEAD/index unchanged |

The exact [current-task changed-file list](../results/ownership_migration/changed_files.txt)
and [hash/inventory manifest](../results/ownership_migration/manifest.json) distinguish
this cut from pre-existing dirty catalog changes. [Before archive](../results/ownership_migration/before.tar.gz)
contains the initial nonignored sources; restore into a separate checkout rather
than overwriting newer work. [Full regression](../results/ownership_migration/final_review_functional.log),
[equivalence](../results/ownership_migration/equivalence.json),
[normal build](../results/ownership_migration/build.log),
[fresh copied build](../results/ownership_migration/isolated_build.log), and
[installation verification](../results/ownership_migration/installation.json)
retain actual evidence.

All 18 previously excluded style tests were run from their own package directories
against the saved baseline and current tree. Both have **10 failures, two passes,
six copyright skips**. No lint configuration or acceptance policy was relaxed.
Flake8 findings changed as follows:

| Package | Before | After |
|---|---:|---:|
| Bringup | 505 | 419 |
| Control | 76 | 75 |
| Prediction | 53 | 33 |
| Simulation | 1,209 | 1,208 |
| Description | 2 | 2 |
| Models | 7 | 7 |

The 12 core migration modules pass scoped Flake8 and docstring gates; comparison
of the six package logs finds no newly introduced finding signatures. Existing
minified fixtures/runtime adapters and unrelated documentation debt remain; the
repository-wide style suite is not clean. Copyright tests already skip, and owner
license/maintainer provenance is not invented. A narrow documented `Catalog.list`
A003 exception preserves the existing public API; no global rule was disabled.
Temporary formatting tools did not become project dependencies. Scope, logs and
AST checks are retained in the evidence directory.

Reproduce the relevant checks:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 -m pytest -q -p no:launch_testing -p no:launch_ros \
  -k 'not flake8 and not pep257 and not copyright'
# Run each package's existing style tests from that package directory.
(cd src/robot_bringup && python3 -m pytest -q \
  test/test_flake8.py test/test_pep257.py test/test_copyright.py)
# Fresh installation: use new paths to avoid stale CMake installation prefixes.
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH bash --noprofile --norc -c \
  'source /opt/ros/jazzy/setup.bash; colcon build \
   --build-base /tmp/ownership_build --install-base /tmp/ownership_install \
   --packages-up-to robot_experiments --executor sequential'
# From /tmp, after sourcing the copied installation:
python3 /home/user/ros2_ws/tools/validation/m09_verify_installation.py \
  --prefix /tmp/ownership_install --output /tmp/ownership_installation.json
```

## Installed Gazebo results and verdict

**Ownership migration implemented; overall verdict: CONDITIONAL.** The remaining
conditions are existing repository style debt and incomplete runtime observation
coverage, not unfinished default/selector consolidation. No further tuning or
retry-to-PASS was performed. The optional dashboard and unsupported deployed
models remain outside this task.

All attempts are retained. The first prediction study returned **1**: IMM had
active control/actuation but never satisfied the unchanged 80 ms recorder
freshness predicate before the 45 s readiness deadline; CV completed. At its end,
recorded clock/joint time was 30.010 s while base truth was 30.200 s. This is an
observer-stream alignment failure, not evidence of absent control. An equivalence
preparation overlapped its startup, so it is not a clean performance comparison;
that overlap alone does not establish the cause.

A single prediction-pair repeat with no other checks running completed both cases
and returned **3**. IMM's observed position/speed violations were zero, but one
140 ms joint gap left 5.94/6.00 s (99%) safety coverage: acceptance correctly stays
unknown. CV passed. The localized baseline/higher-noise study returned **0** and
both cases passed. No original failure was discarded, no age/coverage threshold
was relaxed, and no unconditional predictor ranking is supported.

| Attempt / case | Distance mean / max m | Pointing mean / max deg | Mean RTF | Acceptance |
|---|---:|---:|---:|---|
| Initial IMM | unavailable: readiness timeout | unavailable | unavailable | unknown |
| Initial CV | 0.1340 / 0.3502 | 10.93 / 18.10 | 0.822 | passed |
| Sensor baseline | 0.1310 / 0.2627 | 7.07 / 12.87 | 0.743 | passed |
| Higher sensor noise | 0.1192 / 0.3501 | 8.72 / 28.01 | 0.805 | passed |
| Prediction repeat IMM | 0.1104 / 0.3061 | 8.14 / 16.51 | 0.852 | unknown: coverage |
| Prediction repeat CV | 0.0982 / 0.3203 | 6.64 / 13.22 | 0.866 | passed |

Both prediction studies use three simulated seconds settling plus six evaluation;
the sensor study uses three plus eight, seed 42. All five completed cases have zero
observed physical position/speed violations and zero invalid-command, watchdog,
rejected-command and rejected-feedback events under their existing evaluation
rules. Four have complete safety coverage; IMM repeat does not. These short runs
verify integration rather than long-duration physical safety or performance gains.
Their actual component startup identities, source stamps, joint dimensions, TF,
commands, actuator feedback and run-local resource inputs are preserved in metadata
and raw telemetry. The simulation/estimation/prediction/control/actuation numerical
code and recorder policy were not changed by this migration.

[All runtime attempts and metrics](../results/ownership_migration/all_runtime_results.json),
[initial study](../results/ownership_migration/runtime/),
[sensor study](../results/ownership_migration/sensor-runtime/), and
[prediction repeat](../results/ownership_migration/prediction-repeat/).
The same catalog commands above reproduce these cases; outcomes remain subject to
the documented asynchronous ROS/Gazebo/WSL observation limitations.

A separate active-command worker executed its accepted revision 1 while the writable
catalog advanced to revision 2. Cooperative cancellation preserved partial history,
aborted the remaining trials, returned cancelled on repeated stop and reaped all
owned processes. [Worker/revision/history evidence](../results/ownership_migration/job_verification.json).

The permanent standalone simulation also passed the updated Control Compute Only
probe: **837 cycles**, `publish_commands=false`, no command publisher, raised
holding posture stable across 6,601 samples from t=5–35 s. Maximum observed holding
position error was `5.20e-11 rad`, maximum joint speed `2.68e-14 rad/s`, and minimum
tracking-frame height above the base origin was `0.43918 m`. Stale joint input
blocked computation while wheel feedback continued; restoring feedback recovered
control. A real Gazebo reset retained the robot and recovered estimation,
prediction, control and raised posture. These are stationary holding measurements,
not a claim about moving-arm speed safety.

```bash
source install/setup.bash
python3 tools/validation/compute_only_gazebo.py
# The probe launches this normal permanent simulation, with no temporary holding plugin:
ros2 launch robot_simulation simulation.launch.py gui:=false
```

[Compute-only result](../results/ownership_migration/compute-only/result.json) and
[full log](../results/ownership_migration/compute_only.log). The final host
[process scan](../results/ownership_migration/final_process_check.txt) found no
surviving ROS/Gazebo launch, node, simulator or worker processes. No commit or push
was performed.

Remaining limitations are explicit: repository-wide style debt, asynchronous
observer readiness/coverage gaps under WSL, short rather than long-duration live
coverage, and no real-hardware certification. All 15 direct cases and nine manifests
were preflighted, but not every historical probe/instrumentation mode or scenario
was run live during this cut. Old schema-1 external snapshots and the enumerated
historical URI imports remain deliberate compatibility readers. Library-only models
remain undeployed until their state/message contracts are implemented. Historical
unversioned runs remain unassigned. These do not justify changing physical safety,
metric definitions, covariance, gains or freshness tolerances.
