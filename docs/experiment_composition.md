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
