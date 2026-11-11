# D4.0 — Dynamics Library and motion composition

Implemented a workspace-owned mathematical library and a deterministic Cartesian
composition compiler. Both general state-space definitions and composed motion
resolve to the **same D3 `equation_state_space` model**. No new parser, numerical
engine, integrator, Runner branch, dashboard UI or results store was added.

| Acceptance area | Verdict | Evidence and scope |
| --- | --- | --- |
| Generic mathematical composition | **PASS** | Independent contribution arithmetic, isolated namespaces, deterministic compilation, disabling/removing terms, invalid-input rejection, general eight-state preservation. |
| Environmental contribution correctness | **PASS** | Base/interaction/environment combinations and actual SimulationFactory/CompositeAccelerationField test; environmental acceleration applied once. |
| Dynamics Library persistence | **PASS** | Immutable revisions, stable hashes, pinned resolution, concurrent/stale writer rejection, display rename and failed atomic replacement tests. |
| Author/Catalog integration | **PASS** | Two actual user models created through the installed API; composed revision frozen into a saved Experiment and actual installed owner preflight. Later library edits leave it unchanged. |
| Legacy Fight/Flight compatibility | **PASS** | Linear/saturated radial, drag, regularization and optional normalized tangential interaction agree with the retained trusted Python references. |

Mathematical validity, numerical-domain support, deployment compatibility,
scientific comparability, tracking quality and safety acceptance remain separate.
No scientific Run or fabricated metric was created. Earlier Recorder/actuation/
localization limitations are not requalified by this backend milestone.

## Architecture and responsibility boundaries

| Owner/API | Responsibility |
| --- | --- |
| `robot_simulation.dynamics.motion_composition.compile_definition()` | Validate either authoring mode, transform Cartesian contributions into one D3 definition, return its source-symbol mapping. |
| Existing `EquationDynamics`, expression compiler and dimensional algebra | Sole mathematical execution/validation implementation. No alternate expression interpreter. |
| Existing `SimulationFactory`, `CompositeAccelerationField`, RK4 and target adapter | Numerical execution, external acceleration, integration and existing deployment boundary; unchanged by D4.0. |
| `robot_simulation/config/dynamics_templates.yaml` | Data-only editable starting points. CV references the existing owner-default definition instead of duplicating it. |
| `robot_experiments.dynamics_library.DynamicsLibrary` | One user YAML library, identity, immutable revisions, frozen canonical definitions, hashes and source mapping. |
| `robot_configuration.yaml_store` | Catalog's existing lock/temporary-write/fsync/atomic-replace mechanics extracted for reuse. No schemas or version-management policy here. |
| `ExperimentBackend.dynamics` | Thin public access to the workspace library API. |
| `ExperimentBackend.draft_with_dynamics()` | Resolve an explicit revision into a detached ordinary Author draft, including provenance. |
| Existing Experiment Author/Catalog/materializer | Validate and freeze the effective model in existing immutable experiment definitions and inputs. |

The installed `robot_simulation/config/dynamics.yaml` remains unchanged and
authoritative for registered defaults. Legacy IDs and `fight_flight.py` remain
untouched. New user Fight/Flight definitions use the equation engine, not new
Python registration functions.

## Two authoring modes, one canonical representation

**General state-space:** pass a complete D3 model directly. It is validated and
returned without changing state order or dropping states. All D3 supported
scalar/vector dimensions, inputs and unit/domain checks remain available.
The library can save mathematically valid models that are not live-deployable.

**Composed Cartesian motion:** declare two ordered position/velocity 3-vectors,
one base contribution, and zero or more ordered interactions. The result is:

`p_dot = v`

`v_dot = base_acceleration + sum(enabled interaction accelerations)`.

The existing Cartesian D3 adapter subsequently adds the configured environmental
acceleration exactly once. This composes contributions to a derivative function;
it is **not superposition of nonlinear solutions**.

Each contribution has a stable source `id`, `frame`, explicit `output`,
`parameters`, `expression`, and optional `context`, `intermediates`, `enabled`.
Output must be a 3-vector in m/s². Contribution/model frames must agree; context
declarations retain D3 shape/unit/frame rules. Unknown fields, stochastic/noise
declarations, undeclared symbols, duplicate identities, invalid dimensions and
malicious expressions fail validation. Environment declarations are intentionally
not part of this composition schema.

Each contribution is first checked **independently by D3**, with only its own
declared parameters/context/intermediates and the shared state/time. This prevents
an undeclared symbol from accidentally accessing another contribution's generated
parameter. The compiler then renames local parameters/intermediates through the
existing restricted AST, using generated names such as `c0_p3`, `c1_i0`, `c1_a`.
It introduces no attribute access or unrestricted execution. Allocated symbols
avoid shared state/context names. Base precedes interactions; list order and
sorted local declarations determine the mapping and addition order.

Shared context keeps its actual runtime name; simultaneously enabled declarations
of that name must match. Disabled contributions are still validated, but their
terms and dependencies are omitted from the canonical model. Removing/disabling
an interaction returns the exact base derivative without adding a dummy zero
force. Mapping records retain source identity, order, role and inclusion status.
The final generated model is validated again by D3. Composition is bounded to
one base and 31 interactions, and the final model must also satisfy D3's existing
state, expression, symbol and cost limits.

## Complete spiral plus robot-response example

This is a composition **definition**, resolved by the library/compiler before
being selected as a runtime component. It is not another registered dynamics type.

```yaml
schema_version: 1
type: cartesian_composition
id: spiral_with_robot_response
frame: world
state:
  - {name: p, shape: [3], unit: m, initial: [1.0, 0.0, 1.0]}
  - {name: v, shape: [3], unit: m/s, initial: [0.0, 0.2, 0.0]}
base:
  id: spiral
  frame: world
  output: {shape: [3], unit: m/s^2}
  parameters:
    k: {value: 0.22, shape: [], unit: 1/s^2, minimum: 0.0}
    c: {value: 0.12, shape: [], unit: 1/s, minimum: 0.0}
    omega: {value: 0.45, shape: [], unit: 1/s}
    axis: {value: [0.0, 0.0, 1.0], shape: [3], unit: '1'}
  expression: "-k*p-c*v+omega*cross(axis,v)"
interactions:
  - id: robot_response
    enabled: true
    frame: world
    output: {shape: [3], unit: m/s^2}
    parameters:
      a_max: {value: 0.15, shape: [], unit: m/s^2, exclusiveMinimum: 0.0}
      r0: {value: 1.0, shape: [], unit: m, exclusiveMinimum: 0.0}
      length: {value: 0.15, shape: [], unit: m, exclusiveMinimum: 0.0}
      epsilon: {value: 1.0e-6, shape: [], unit: m, exclusiveMinimum: 0.0}
    context:
      tracking_position_world: {shape: [3], unit: m, frame: world}
    intermediates:
      delta: "p-tracking_position_world"
      distance: "norm(delta)"
    expression: "a_max*tanh((distance-r0)/length)*delta/sqrt(dot(delta,delta)+epsilon**2)"
```

The compiler emits the same p/v state, namespaced parameter declarations and
intermediates, an intermediate acceleration per enabled contribution, and one
velocity derivative summing those intermediates. The result includes D3's
`deployment: {adapter: cartesian_target, position: p, velocity: v}`.
The library retains the exact source-to-generated symbol mapping.

Environmental configuration stays with the existing selected environment owner,
for example an existing `linear_restoring` field with `center` and `gain_matrix`.
It is neither copied into this expression nor evaluated by the composition
compiler. D3's existing field dispatch supports ordinary and contextual fields.
The actual Factory composite-field test independently verifies this path.

The radial interaction's sign is the specified Fight/Flight convention: negative
inside the preferred radius, positive outside. The human label “avoidance” does
not make it a universal collision-avoidance or physical-safety guarantee.

## Templates and legacy migration

Available data-only starting points:

- Constant Velocity — reference to D3's owner-default model.
- Circular / Turning — rotational acceleration `rate*cross(axis,v)`.
- Spiral — restoring/drag plus rotational acceleration.
- Spring-Damper — restoring plus drag.
- Saturated Fight/Flight and Linear Fight/Flight — radial interaction and drag.
- Blank State-Space — a valid scalar zero-rate starting point, not deployable.

These are editable definitions, not runtime categories or a Python model allowlist.
Changing equations or adding a new library model requires no registration change.
The legacy templates also contain an independently optional tangential term:

`axis_unit = axis / norm(axis)`

`tangent = cross(axis_unit, p - tracking_position_world)`

`a_tangent = acceleration * tangent / sqrt(dot(tangent,tangent) + epsilon**2)`.

This preserves the trusted implementation's axis normalization, behavior at poles
and coincidence, nonnegative circulation magnitude, and regularization. It is
disabled by default in the template. The radial templates preserve damping;
when appending radial response to a base that already owns drag, the example API
sets the interaction's damping to zero explicitly. No term is silently subtracted
or guessed by the compiler.

## Library persistence schema and API

Default: **`DATA_DIR/dynamics_library.yaml`**. Configure another path with
`ExperimentBackend(..., dynamics_library_path=...)`. The file is created only on
an explicit successful mutation; construction and reading an empty library create
no file. Installed defaults/templates and the Experiment Catalog cannot be used
as the backend's library path.

One YAML document holds:

```yaml
schema_version: 1
generation: 3
models:
  DYN-USER-SPIRAL:
    name: User spiral
    revisions:
      - revision: 1
        created_utc: <actual timezone-aware UTC timestamp>
        created_by: D4.0 installed verification
        compiler_version: 1
        definition: <complete original authoring definition>
        model: <complete validated canonical D3 definition>
        symbol_map: <original contribution to compiled symbol mapping>
        sha256: <deterministic content hash>
```

The source is retained for editing/audit; the canonical model is the frozen
execution authority, produced by an explicit validated transformation. They are
not two independently editable authorities. Both, plus the symbol map, participate
in the content hash. No writable JSON mirror exists. Display labels, creation
time and creator metadata are separate from the mathematical content hash.
Template/library API boundaries detach YAML aliases into finite data trees, so
editing one contribution's parameter cannot mutate another declaration through
shared Python objects. Non-string keys and nonserializable values fail rather
than being silently renamed/coerced. A focused failing alias test was retained
before this correction; numerical parameter namespaces were already separate.

Library IDs are stable `DYN-...` identifiers. Revision numbers are contiguous
positive integers; display renaming does not change revisions, equations or hashes.
Schema/compiler version 1 is explicit; unknown versions fail. Reads verify stored
hashes and validate the stored canonical model. Resolution does **not** recompile
old source through a later composition compiler. Source and mapping are detached
from caller mutations.

`backend.dynamics` exposes:

| Operation | API |
| --- | --- |
| Templates / inspect drafts | `templates()`, `validate(definition)` |
| List / read | `list()`, `get(id, revision=None)` |
| Create | `create(id, name, definition, created_by='local')` |
| Immutable save | `revise(id, definition, expected_revision=..., created_by='local')` |
| Display rename | `rename(id, name, expected_revision=..., expected_name=...)` |
| History / capabilities | `revisions(id)`, `capabilities(id, revision)` |
| Pinned canonical resolution | `resolve(id, revision)` — requires an explicit revision |

Mutations use a stable advisory lock, sibling temporary file, file fsync, atomic
replacement and directory fsync. Compare-and-swap rejects concurrent/stale edits;
duplicate IDs never overwrite history. Fault injection before replacement leaves
the previous bytes intact and removes the temporary file. As with Catalog, an I/O
failure after successful replacement can leave a committed revision despite an
error return; inspect the stored revision before retrying. This is a local
workspace persistence contract, not distributed storage or cryptographic signing.
Deletion/archive is deliberately deferred; no operation removes old revisions.

## Backend selection and scientific freezing

```python
from copy import deepcopy
from pathlib import Path
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog

root = Path('/tmp/my_tracking_workspace')
catalog = ExperimentCatalog(root / 'catalog.yaml')
if not catalog.path.exists():
    catalog.initialize()
api = ExperimentBackend(root, catalog=catalog)
templates = api.dynamics.templates()
spiral = templates['spiral']['definition']
api.dynamics.create('DYN-SPIRAL', 'My spiral', spiral)
combined = deepcopy(spiral)
combined['id'] = 'my_spiral_response'
response = deepcopy(templates['saturated_fight_flight']['definition']['base'])
response['id'] = 'robot_response'
response['parameters']['damping']['value'] = 0.0
combined['interactions'] = [response]
api.dynamics.create('DYN-COMBINED', 'Spiral and response', combined)
draft = api.new_draft('EXP-MY-MOTION', 'My motion', 'EXP-NOMINAL')
draft['experiment']['runtime'] = {'enable_actuation': False, 'gui': False}
draft = api.draft_with_dynamics(draft, 'DYN-COMBINED', revision=1)
assert api.validate_experiment(draft)['valid']
saved = api.save_experiment(draft)
```

The adapter returns a detached draft with the generic component selected and the
complete model under `parameters.target_dynamics.model`. It also attaches
`dynamics_source`: library ID, exact revision, content hash, canonical model hash
and source-symbol mapping. Author checks that the pin matches the authored model,
including advanced owner changes, and retains it in existing Author provenance.
The Catalog and materializer freeze the full model using their existing workflow.
The Runner reads only that immutable experiment configuration; it never asks for
the library's mutable latest revision.

Changing library equations/parameters changes its content hash and produces a new
revision. Active equations and parameters remain in the complete protected target
dynamics document. Existing comparison rules still reject unapproved dynamics
differences; no D4.2 masks/ranking extension was added. A source pin is internally
checked provenance, not a signed authenticity certificate for manually edited
metadata. Saved canonical definitions remain usable independently of the library
file's subsequent changes or availability.

## Genuine installed verification and measured results

Executed the public API from **`/tmp`**, using installed packages and no source
`PYTHONPATH`. Persistent evidence is under `results/dynamics_library/`:

| Created identity | Revision 1 content SHA-256 |
| --- | --- |
| `DYN-USER-SPIRAL` | `848dfb1ffe447cc98d1658106d3d75ec0b3d7c201f0d238345775e99bb9be0b5` |
| `DYN-USER-COMBINED` | `13bc525ffea1930248f0d805682b54b1fa3e6d6ba69ecc50663afe6511191b74` |

The first definition has stiffness 0.22 s^-2, damping 0.12 s^-1 and turn rate
0.45 s^-1. The second adds the saturated response with explicit zero additional
drag. Both complete sources and canonical models are in
`workspace/dynamics_library.yaml`. The second advanced to revision 2 with changed
radial magnitude after experiment save.

Saved Experiment **`EXP-LIBRARY-COMPOSITION`, revision 1**, hash
`8e4353a14a250439f9106f29c800d460a9e7fb3f0b2f8489f0eb06b6181f43f5`,
remains pinned to model revision 1. Actual owner materialization under `prepared/`
contains identical frozen equations/values and selects
`robot_simulation.dynamics.equation_model.EquationDynamics`.
This directory is **configuration preparation**, not a scientific Run.

Measured independent composition derivative error: **0.0** at the retained test
state. Across **20** linear/saturated × tangential on/off × separation cases,
including coincidence, 1e-10 m and circulation poles:

- Maximum derivative difference from trusted Python: **5.551115e-17**.
- Maximum short-trajectory difference: **6.938894e-18** (25 RK4 steps, dt 0.002 s).
- Context and a real `CompositeAccelerationField` were matched between models.

No tracking-quality, RTF, physical safety or Gazebo execution result is inferred.
The installed public API plus actual Catalog/owner preparation covers the changed
deployment wiring; no failure required a new Gazebo smoke test.

## Tests, build and preservation

- **56 focused tests passed**, 29.17 s: composition cases, templates and legacy
  equivalence, library immutability/concurrency/atomic-failure/serialization, and
  real backend/Catalog selection including unsupported eight-state rejection.
  Final output and duration are retained in `focused_final.txt`.
- **165 runtime/D3/built-in/Runner/coverage/ranking regression tests passed**,
  62.80 s (`runtime_regression.txt`).
- **103 shared YAML/Catalog/backend/Author regression tests passed**, 359.19 s
  (`regression.txt`). This includes existing Catalog concurrency and immutable
  revision checks after extracting the shared persistence helper.
- Build: **robot_configuration, robot_simulation, robot_experiments succeeded**,
  approximately 60 s. Existing underlay-override warnings are retained; no physics
  configuration or native actuator source was changed.
- Post-correction installed APIs also verify template alias isolation and both
  unchanged saved hashes from `/tmp` (`installed_final.txt`).
- Focused new production-module lint and scoped whitespace checks pass. No
  repository-wide style-clean claim is made.
- A baseline hash audit covers **2,010** existing files, including **1,261**
  historical evidence files: **zero historical changes**. Only the three intended
  pre-existing Python files changed. D3 mathematics, owner defaults, legacy Python,
  physical constraints, PID gains, readiness, scoring and previous reports remain
  byte-identical. No files were deleted or relocated.

Reproduce tests/preflight without creating Runs:

```bash
cd /home/user/ros2_ws
source install/setup.bash
build/dashboard_venv/bin/python -m pytest \
  src/robot_simulation/test/test_motion_composition.py \
  src/robot_simulation/test/test_declarative_motion_templates.py \
  src/robot_experiments/test/test_dynamics_library.py \
  src/robot_experiments/test/test_dynamics_library_integration.py -q
ros2 run robot_experiments run author validate \
  results/dynamics_library/experiment.yaml \
  --catalog results/dynamics_library/workspace/catalog.yaml
ros2 run robot_experiments run --validate-only \
  --catalog results/dynamics_library/workspace/catalog.yaml \
  --experiment EXP-LIBRARY-COMPOSITION --revision 1
colcon build --packages-select robot_configuration robot_simulation robot_experiments \
  --symlink-install
```

`installed_integration.py` is the retained one-time installed API/equivalence check;
after success it displays its existing result instead of overwriting evidence.
Other artifacts include source models, immutable library/Catalog revisions,
`installed_integration.json`, test/build/lint logs, and preservation manifests.
No separate authoritative library JSON is maintained.

## Exact changed files

New permanent files:

1. `src/robot_simulation/robot_simulation/dynamics/motion_composition.py`
2. `src/robot_simulation/config/dynamics_templates.yaml`
3. `src/robot_simulation/test/test_motion_composition.py`
4. `src/robot_simulation/test/test_declarative_motion_templates.py`
5. `src/robot_configuration/robot_configuration/yaml_store.py`
6. `src/robot_experiments/robot_experiments/dynamics_library.py`
7. `src/robot_experiments/test/test_dynamics_library.py`
8. `src/robot_experiments/test/test_dynamics_library_integration.py`
9. `docs/validation/dynamics_library_composition_d40.md`

Modified:

1. `src/robot_experiments/robot_experiments/backend.py` — workspace library access
   and detached pinned-revision-to-Author adapter.
2. `src/robot_experiments/robot_experiments/authoring.py` — validate/freeze source
   pin provenance and reject inconsistent model rewrites.
3. `src/robot_experiments/robot_experiments/catalog.py` — delegate its existing
   persistence mechanics to the shared helper; same revision/hash policy.

Generated evidence is separate under `results/dynamics_library/`. No installed
default or historical artifact was rewritten. No commit/push.

## Remaining boundaries

This is the Dynamics Studio **backend preparation**, not the dashboard editor.
General output mapping, new live context bindings, non-Cartesian deployment and
user-defined-dynamics comparison policies remain separate D4.1/D4.2 work.
No stochastic model mechanism, automatic solver/stability guarantee, collision
avoidance proof, deletion interface or distributed database is provided.
Library capability inspection is a model/adapter contract; the selected robot,
frame, sensors and full experiment still require existing owner preflight.
Existing live initialization and context freshness policies are unchanged.
Work stops at D4.0.
