# Experiment composition

`robot_experiments/config/experiments.yaml` is the selection authority. Package
configuration supplies physical and numerical defaults; it does not choose an
experiment. The catalog retains immutable historical revisions and adds revision
3 compositions. No dashboard is involved.

The catalog contains one common system mapping, small selection deltas, named
resource recipes, direct-launch cases and controller allocations. Group lists use
the shared loader's ordered mapping merge; lists and null replace inherited values.
A recipe has `base`, `overrides`, optional `selection` and optional `omit`:

```yaml
system_config:
  base:
    package: robot_experiments
    path: config/experiments.yaml
    group: [compositions.common, compositions.selections.nominal_tracking]
  overrides:
    prediction:
      type: constant_velocity
      adaptation: none
      fusion: none
    control:
      type: jacobian
      strategy: null
```

This mapping belongs in an experiment definition's variant. Change component IDs
here, rather than adding a package YAML. Component parameters belong in the
catalog revision's existing owner `overrides`, for example:

```yaml
target:
  target:
    dynamics:
      parameters:
        damping: 0.23
strategy:
  controller:
    parameters:
      motion_weights: [2.0, 4.0, 3.0, 3.0, 3.0, 2.0]
```

`strategy` applies only when a strategy is selected. True synthetic measurement
noise belongs to `measurements` or `localization_sensor`; estimator-assumed noise
belongs to `estimation`. They remain independent. Scientific comparison policies
still authorize and validate explicit experimental variables separately.

A resource recipe can select another dynamics algorithm by replacing the typed
`target.dynamics` slot through `selection`; that operation replaces its complete
`{type, parameters}` mapping before applying parameter overrides. It does not merge
parameters from different equations. Ordinary owner overrides reject unknown
leaves. `omit` explicitly removes existing optional fields; required fields still
fail the shared system preflight. Schema-2 compositions allow structural selection
changes, then invoke the same owner factories and M07 validation as file manifests.

Generated documents are ordinary YAML in a process-lifetime temporary directory.
Relative system references retain their declaration origins, including inherited
parents. Transitive source paths/groups and fingerprints accompany generated
resources. Source changes after preparation reject snapshot materialization;
prepare again to select new values. Stable display labels omit temporary paths.
Scored runs and `compose` freeze effective documents into their own `inputs/`.
Runtime readers do not depend on the temporary directory after that freeze.

## Headless workflows

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential
source install/setup.bash
ros2 run robot_experiments run components list
ros2 run robot_experiments run catalog list
ros2 run robot_experiments run --experiment EXP-PREDICTION-COMPOSITION --revision 3 --explain
ros2 run robot_experiments run --experiment EXP-PREDICTION-COMPOSITION --revision 3 --validate-only
ros2 run robot_experiments run --experiment EXP-PREDICTION-COMPOSITION --revision 3 --output results/experiments
# Repeating this command creates another study and distinct Run IDs.
ros2 run robot_experiments run --experiment EXP-PREDICTION-COMPOSITION --revision 3 --output results/experiments
```

Create a writable catalog once. Export a definition, edit that external file's
component/recipe mappings, then create or revise through validation. Owner overrides
are a separate mapping file outside package config paths.

```bash
ros2 run robot_experiments run catalog init --catalog results/catalog/experiments.yaml
python3 - <<'PY'
from pathlib import Path
import yaml
from robot_experiments.catalog import ExperimentCatalog
catalog = ExperimentCatalog(Path('results/catalog/experiments.yaml'))
draft = catalog.draft('EXP-NOMINAL')
Path('results/definition.yaml').write_text(yaml.safe_dump(draft['definition']))
Path('results/overrides.yaml').write_text(yaml.safe_dump(draft['overrides']))
# Modify the detached draft as needed. Reset discards unsaved owner overrides:
assert catalog.reset(draft)['overrides'] == {}
PY
ros2 run robot_experiments run catalog create --catalog results/catalog/experiments.yaml \
  --experiment EXP-CUSTOM --name Custom --definition results/definition.yaml --overrides results/overrides.yaml
ros2 run robot_experiments run catalog revise --catalog results/catalog/experiments.yaml \
  --experiment EXP-CUSTOM --expected-revision 1 --definition results/definition.yaml --overrides results/overrides.yaml
ros2 run robot_experiments run --catalog results/catalog/experiments.yaml --experiment EXP-CUSTOM --revision 2
ros2 run robot_experiments run history --results results/experiments --experiment EXP-CUSTOM
ros2 run robot_experiments run catalog delete --catalog results/catalog/experiments.yaml \
  --experiment EXP-CUSTOM --expected-revision 2
```

Deleting a definition retains result directories and immutable historical revision
records. Purge is a separate explicit catalog operation; it does not delete runs.
New run execution after a source edit requires a fresh preparation. Editing and
resetting drafts never changes package defaults.

## Direct launch and standalone simulation

```bash
# Continuous tracking; commands require explicit opt-in.
ros2 launch robot_experiments tracking.launch.py \
  case:=reactive_tracking_oscillatory controller:=whole_body enable_actuation:=true gui:=false
# Freeze a continuous-launch case with a chosen sensor seed into a fresh directory.
ros2 run robot_experiments run compose --case standalone --seed 42 --output results/standalone_inputs
ros2 launch robot_simulation simulation.launch.py \
  config:=$PWD/results/standalone_inputs/inputs/simulation.yaml gui:=false
# Generic bringup or visualization accepts the same frozen explicit manifest.
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=$PWD/results/standalone_inputs/inputs/system.yaml gui:=false
```

Use the `system_config` path printed by `compose` for generic bringup (the filename
is authoritative in its JSON output). Choose a tracking case when freezing a full
tracking pipeline. Standalone retains permanent gravity-holding PID and raised
posture. `simulation.launch.py` now requires an explicit deployment `config`; it
cannot silently choose an experiment. The compute-only probe passes this same
permanent launch a materialized standalone config and preserves
`publish_commands=false`.

Old experiment-owned `config/systems/*.yaml`, scenario/wrist variants and former
bringup resource names have a bounded catalog migration map for historical
revisions and experiment entry points. They are no longer installed files. Do not
pass a removed file directly to generic bringup or visualization: freeze the
selection first. Existing standalone no-argument launches must use the explicit
`compose` migration above. Old M08 filenames are explicit catalog imports.

See [the verification report](validation/experiment_composition.md) for exact
removals, equivalence, tests, runtime verdict and limitations.

## One-file authoring

`robot_experiments.authoring.ExperimentAuthor` compiles a small specification to the
same Catalog definition and owner overrides. Start from the installed example
`share/robot_experiments/examples/tracking.yaml` (also in the source package's
`examples/`). It selects CV, no correction, and the existing Cartesian DLS baseline;
WLS with Distance + Pointing remains available. DLS cannot use the velocity-task
interface of Distance + Pointing; actual owner construction rejects this pairing.

```bash
ros2 run robot_experiments run author discover
ros2 run robot_experiments run author preview /path/to/tracking.yaml
ros2 run robot_experiments run author validate /path/to/tracking.yaml
ros2 run robot_experiments run catalog init --catalog /tmp/my_experiments.yaml
ros2 run robot_experiments run author save /path/to/tracking.yaml --catalog /tmp/my_experiments.yaml
ros2 run robot_experiments run --catalog /tmp/my_experiments.yaml --experiment EXP-CUSTOM --revision 1
# After editing the same short file:
ros2 run robot_experiments run author save /path/to/tracking.yaml --catalog /tmp/my_experiments.yaml --expected-revision 1
```

The public document has `schema_version: 1` and one `experiment` mapping:

- Required `id` (Catalog EXP identifier), `name` (display name), `base_case` (existing
  Catalog ID). `base_revision` pins a revision; `base_variant` is required if the
  base contains multiple variants. A schema-1 base supplies all scientific and
  execution settings; schema-2 comparative studies retain the canonical interface.
- Optional `scenario`: an existing deployed case from `author discover`; it selects
  a coherent robot/environment/sensor configuration without internal recipe wiring.
- `components`: any discovered public slot and registered deployable string ID.
  Slots are `target_dynamics`, `robot_estimator`, `target_estimator`, `predictor`,
  `adaptation`, `fusion`, `controller`, `task`, `reference_policy`, `reference_geometry`.
  Adaptation/fusion use the registered string `none`, not YAML null.
- `parameters`: slot-to-parameter mappings. Missing values preserve base settings;
  a changed component starts with its owner's defaults; estimator algorithm changes
  retain compatible inherited model parameters. Lists replace in full.
  Explicit false/zero/empty/null reach the existing validators with their original
  meaning. Reference IDs are selectable; their arbitrary numerical parameters lack
  a deployed owner pathway and are explicitly rejected. Shared tracking values
  remain available through advanced control-owner overrides.
- `execution`: optional `duration`, `settling_time`, `seeds`, or existing M08 execution
  keys. `runtime.enable_actuation` is optional and otherwise inherits the base.
- `resources`: optional `environment` named resource, `robot` pair of `description`
  and `deployment` resources, or `sensing` with `measurements` and/or `localization`.
  Names come from discovery; explicit package/path/group references are accepted.
  Localization null means absent; producer/topic/frame compatibility still passes
  through M07. Prefer a coherent `scenario` for coupled localization changes.
- `advanced.overrides`: the existing owner override mapping for technical authors.
  Existing canonical CLI/catalog APIs remain unchanged.

Python: `ExperimentAuthor(catalog).compile(source)` returns a detached canonical
mapping; `validate(compiled)`, `preview(source)`, and `save(source,
expected_revision=...)` reuse Catalog/M07/M08. `source` is a YAML path or mapping.
Preview reports registry defaults, explicit edits and actual effective values;
`--details` includes the full seeded explanation. Preview never saves or starts ROS.
Explicit parameter edits use Catalog owner overrides, so `Catalog.reset(draft)`
retains selected components and clears those edits. Revisions keep the source SHA256,
base ID/revision/hash in their existing description, original case references, and
canonical values; no new store or revision fields are introduced. Selected inline
owner documents are frozen through the existing materializer's `{document: ...}`
envelope; no process-temporary filename is persisted. External absolute resources
retain the existing installation/path availability requirements.


## Estimator algorithms and bindings

The installed one-file example now selects canonical `robot_estimator: ekf` and
`target_estimator: kf`. Ordinary experiments inherit the base case's verified
process/measurement/initialization and deployment binding:

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

Historical `robot_ekf` and `kalman` IDs remain valid with exactly their previous
meaning. Selecting an algorithm does not silently select different sensors or
reset a compatible base model's process/measurement parameters. Existing estimator
parameter overrides retain their original physical names and units.

Advanced authors may optionally provide `experiment.estimation_bindings`:

```yaml
estimation_bindings:
  robot:
    process_model: robot_motion
    measurement_model: robot_sensors
  target:
    process_model: target_cv
    measurement_model: target_position
    adapter: target_state  # optional; otherwise inherit the existing adapter
```

Each supplied role requires both model IDs. This block is unnecessary for normal
experiments. Registered model/algorithm/adapter compatibility and actual dimensions,
Jacobians, covariance and sensor/frame contracts are validated before launch.
Bindings become existing per-role manifest parameters, then optional typed flat
snapshot fields (`robot_*` / `target_*` model and adapter names); replay revalidates
them. No imports, runner branches or new ROS messages are involved. A generic
numerical EKF/KF does not imply arbitrary ROS deployments; robot KF and target EKF
remain unsupported by current production adapters.

Task E.1 supports intrinsic settings through the existing parameter convention:
`experiment.parameters.<robot_estimator|target_estimator>.algorithm_parameters`.
This is a mapping validated by the selected algorithm's owner schema. Production
KF/EKF have no intrinsic settings, so ordinary experiments omit it. A compatible
registered algorithm with settings inherits owner defaults and base overrides;
local edits merge recursively, while lists/scalars replace. A changed algorithm
starts from its own settings; compatible physical model overrides remain intact.

Preview exposes the effective mapping. Catalog records and prepared run inputs
preserve the same values. Existing `inputs/estimation.yaml` freezes each role's
canonical algorithm identity and full mapping in a JSON ROS STRING parameter
(`robot_algorithm_parameters` / `target_algorithm_parameters`). These are transport
fields, not alternate authoring knobs. Replay and clock-reset reconstruction use
them without merging newer owner defaults. Old empty-setting snapshots and aliases
remain readable. Scientific comparison includes the frozen settings; no additional
snapshot format or estimator result store is introduced. See
[Task E.1 evidence](validation/estimation_parameters.md).


## Headless backend for a future Dashboard

Task C exposes `robot_experiments.backend.ExperimentBackend`. The independent
future `robot_dashboard` / Plotly Dash package consumes this API; the experiments
package has no UI, web-server or Plotly/Dash dependency.

```python
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog

catalog = ExperimentCatalog("/path/to/writable/catalog.yaml")
# Once, for a new writable catalog: catalog.initialize()
backend = ExperimentBackend("/path/to/results/client", catalog=catalog,
                            result_roots=["/path/to/results"])
choices = backend.list_component_options("target_dynamics")
preview = backend.preview_experiment("my_experiment.yaml")
validation = backend.validate_experiment("my_experiment.yaml")
record = backend.save_experiment("my_experiment.yaml")
# Revisions use expected_revision=N; CatalogConflict requires reload.
job = backend.start_run(record["id"], record["revision"],
                        output="/path/to/results/runs", enable_actuation=True)
status = backend.get_job_status(job["job_id"])
# Run IDs appear only after the existing Runner allocates actual attempts.
# When available: backend.get_run_results(status["run_ids"][0])
# Cancel a queued/running job: backend.cancel_job(job["job_id"])
```

`start_run` defaults to `enable_actuation=False`, independently of saved runtime
settings; commands require explicit opt-in. A revision can create several Runs.
Cancelling any Run cancels its owning job and remaining variants/seeds, through
the existing worker's cooperative cancellation and Runner cleanup.

The effective preview uses the existing Author/explain/materialization path.
`explain_experiment(id, revision)` also explains saved single-condition and
canonical comparison studies; those studies remain runnable even where the
one-file Author does not support them as draft bases. It
shows inherited target dynamics (including the model receiving damping changes),
owner defaults, actual effective parameters, schemas/units, known selection origin
and direct authoring overrides. It explicitly identifies unavailable per-leaf
ancestry for inherited/advanced owner settings. New estimator displays group
historical `robot_ekf`/`kalman` aliases under canonical `ekf`/`kf` IDs; immutable
historical records retain their actual identities.

Sessions are ordered UUID metadata files containing only Run IDs and original
artifact directories. `create_session`, `add_run_to_session`, `list_sessions` and
`get_session_comparison` persist references with locked atomic updates; they never
copy metrics, telemetry or inputs. Register the same result roots after restarting
the client. Missing evidence remains unavailable rather than being replaced.

Result queries preserve execution status, safety/acceptance and tracking metrics
separately. Listings read headers, not telemetry; `read_telemetry` pages recorded
JSONL streams. Time-series/frozen-input references point to original Run files.
`get_historical_results(directory)` also reads registered pre-UUID attempts without
inventing IDs. Session comparisons expose original metric availability, coverage,
windows and condition differences, reuse existing scientific checks and never
produce an automatic ranking. A timeout retains its error and unavailable scores.

`gazebo_setup_capabilities()` reports that separate interactive setup and live
robot/world replacement are unsupported. Existing scored jobs own isolated
ROS/Gazebo deployments; a configuration change requires controlled cleanup/restart.
See [Task C validation and D1 contract](validation/dashboard_backend.md) for exact
signatures, verification evidence, limitations and a runnable installed workflow.
