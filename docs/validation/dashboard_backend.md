# Task C — Dashboard backend readiness and integration

**PASS for the backend API/integration milestone.** Real lifecycle verdict:
**CONDITIONAL** for the one scored attempt, with a separate cooperative
cancellation check. This is not general research-runtime, localization/covariance
or real-hardware safety certification. No Dashboard D1–D3 implementation was started.

The revised Task C attachment (a9dd2064-a8e6-49e6-987b-4c14d597f075) and current
repository contracts governed this work. The referenced standalone Development
Blueprint document was not present in the supplied workspace/attachments.

## Reuse-first audit and scope

| Capability | Entry state | Owner reused / minimal integration |
|---|---|---|
| Definitions, revisions, hashes, CRUD/CAS | Existing | `ExperimentCatalog`; installed Catalog remains read-only |
| One-file authoring, validation, save | Existing | `ExperimentAuthor`; no second YAML merge or validator |
| Schemas, units, bounds, defaults, capabilities | Existing | Owner component registries; facade groups historical aliases |
| Effective configuration and physical preflight | Existing/partial display | Author → explain → materialization/M07; added known selection origin and client projections |
| Seed/fingerprint/frozen-input contracts | Existing | Materializer, preparation and run-owned snapshots |
| Headless execution and cancellation | Existing | `ExperimentJobs` → Runner; one optional study-created notification exposes real live Run IDs |
| Recorded metrics/history/time series | Existing/partial queries | Read original artifacts; added bounded header queries and JSONL pages |
| Cross-study persistent ordered Session | Missing | Small locked atomic metadata files referencing existing Runs |
| Scientific comparison | Existing | Existing comparison policies, metric/window/coverage guards and original/protected condition metadata |
| Independent interactive Gazebo setup | Unsupported | Explicit capability response; scored jobs own isolated deployments |
| UI/server/charts/database | Deferred | Future separate `robot_dashboard` / Plotly Dash package; no dependency or package added |

Task R was an entry gate, not implemented again: source, normal and prior copied
installations had the canonical description and no old installed copy. The new
copied installation also proves canonical/legacy resource resolution. Physical
SHA256 remains `fd6877be3fb2c8cc20d13c85ddb776e9aad1657d90afa1c6014dce68a0694c76`.
`robot_simulation/config/gazebo/robot.yaml` retains its name and bytes.

## Public Python API and ownership

One facade: `robot_experiments.backend.ExperimentBackend`. No web framework,
server, UI state framework, database, algorithm registry or process manager.
Constructor has no launch/file-creation side effect:

```python
ExperimentBackend(directory, *, catalog=None, result_roots=())
```

`directory` owns job requests and Session metadata. `result_roots` registers existing
result authorities; persisted existing job requests also expose their actual
output roots after client restart. Existing public owners remain available as
`backend.catalog`, `backend.author` and `backend.jobs`, rather than duplicating all
their operations. Structured reads are detached and JSON-compatible.

| Operation | Signature / result |
|---|---|
| Discover saved studies | `list_experiments(include_deleted=False)`; schema, variants and one-file authoring support included |
| Revision history | `list_revisions(experiment_id)`; ordered revision metadata and actual SHA |
| Component metadata | `list_component_options(role, context=None)`; canonical ID, aliases, deployability, requirements, schemas/defaults; compatibility unknown without context |
| New one-file draft | `new_draft(experiment_id, name, base_case, base_revision=None, base_variant=None)`; pins a real base revision |
| Reset unsaved choices | `reset_draft(specification)`; retains identity/base, discards local choices/overrides, never edits saved revisions |
| Preview one-file spec | `preview_experiment(specification)`; existing owner-validated effective values plus display metadata |
| Explain any saved study | `explain_experiment(experiment_id, revision=None)`; canonical display over Catalog explanation, including schema-2 comparisons |
| Validate/save | `validate_experiment(specification)` → valid/errors; `save_experiment(specification, expected_revision=None)` delegates authoritative create/CAS revision |
| Start | `start_run(experiment_id, revision, output=..., enable_actuation=False)` → existing job status |
| Monitor/cancel job | `get_job_status(job_id)`, `cancel_job(job_id, timeout=25.0)` |
| Monitor/cancel actual Run | `get_run_status(run_id)`, `cancel_run(run_id, timeout=25.0)` |
| Read evidence/history | `get_run_results(run_id)`, `get_historical_results(directory)`, `list_runs(filters=None, offset=0, limit=100)` |
| Bounded telemetry | `read_telemetry(run_id, stream, offset=0, limit=100)`; original JSONL records only |
| Session | `create_session(metadata=None)`, `add_run_to_session(session_id, run_id)`, `list_sessions(title=None, offset=0, limit=100)`, `get_session_comparison(session_id)` |
| Setup boundary | `gazebo_setup_capabilities()`; separate interactive setup/live reconfiguration unsupported, controlled restart required |

Keyword-only parameters follow the source signatures. Listings/pages accept up to
1,000 records per request. `get_job_status` includes a bounded first page of actual
Run headers; use `list_runs` with study filter/offset for larger studies. Metadata
JSON reads reject files over 32 MiB and retain original artifact references;
telemetry/time-series files are never loaded for listings. Time-series references
are provided without resampling or metric recomputation.

Job IDs and Run IDs have different meanings. One immutable revision may produce
multiple seed/variant Runs; the backend never invents Run IDs while a job is queued.
The optional Runner callback publishes the study path before its first attempt,
then the existing Runner assigns each actual Run UUID. Cancelling a Run cancels its
owning job and remaining seed/variant attempts. Queued cancellation uses job ID.
Cooperative stop and owned-child cleanup remain in the existing services.

Commands require explicit `enable_actuation=True` at this client boundary; a saved
actuating experiment does not bypass that opt-in. Native readiness/safety limits,
timeouts, physics, holding PID, physical bounds and algorithms are unchanged.

## Effective inherited configuration and YAML follow-ups

Installed `examples/tracking.yaml` preview demonstrates:

- Dynamics ID **`saturated_fight_flight`** is inherited from EXP-NOMINAL, while damping
  is explicitly authored as **0.23**; the registered owner default is **0.5**.
- Preview retains all effective target dynamics parameters (including inherited
  tangential settings), schemas/units, current robot description, process and
  measurement bindings, predictor/adaptation/fusion, controller/task/reference,
  sensing/environment, execution settings and frozen stream seeds.
- Source metadata identifies base ID/revision/hash or explicit scenario/selection.
  Direct authoring overrides are identified. Full per-leaf inheritance ancestry
  does not exist in the owner system and is not guessed; advanced owner overrides
  are conservatively identified as resolved owner/inherited values.
- New displays use `ekf`/`kf`; owner alias metadata exposes `robot_ekf`/`kalman`.
  Effective historical selections are retained in `historical_selection` when
  canonical display changes them. Saved definitions and original comparison
  contracts/hashes are never rewritten for display cleanup.
- One-file drafts require schema-1 bases. Canonical schema-2 comparisons remain
  listable, explainable and runnable via existing Catalog machinery.

The tests exposed one preexisting Author gap: it resolved a historical base resource
but re-embedded the removed filename inside its new recipe. The existing `_freeze`
helper now freezes the already-resolved base, allowing pinned old revisions to be
used safely. This does not introduce another alias resolver or duplicate physical
configuration. Public error presentation labels deleted owned preflight directories
as `<materialized>` while preserving the real field-level validator error.

F2: actual preflight rejected all four measured mismatches: estimator joint count,
estimator ordering, control count and control command ordering. The recorded
`joint_contract_verification.json` shows required description-derived values.
Actuation receives those same authoritative ordered names/limits; holding keys and
initial angles are checked against the description. No independent limits or
required ROS transport fields were removed.

F3: frozen target measurement seed 42 is verified in the installed one-file run.
Existing localized comparison explanation also verifies the distinct deterministic
stream seed **3872720740**. Full regression exercises seed assignment/materialization;
source fingerprints and frozen intrinsic `ekf`/`kf` parameter envelopes survive
installed preparation. Equal seed/configuration does not establish bitwise
physical trajectory determinism under asynchronous ROS/Gazebo/contact/skid dynamics.

## Results, Sessions and scientific integrity

A run result exposes the actual status, metadata, source hashes, seed/variant,
resolved configuration/owner snapshot, recorded comparison document and artifact
references. Execution completion, safety/acceptance and tracking performance are
separate fields. Missing metrics remain unavailable/null; no zero/default score is
substituted. Existing genuine historical readiness timeout Run
`49ef1ed3-1842-4e4e-b261-2171d9e678fd` was read from the copied installation with its
actual `readiness_timeout`, `acceptance=null`, and unavailable scored metrics.
Pre-UUID attempts can be read by registered directory and keep `run_id=None`.

Sessions use one validated JSON file per UUID at
`<directory>/sessions/<session_id>.json` plus a per-session lock:

```json
{"schema_version": 1, "session_id": "<uuid>", "created_utc": "<actual UTC>",
 "metadata": {"title": "...", "notes": "..."},
 "runs": [{"run_id": "<actual Run uuid>", "directory": "<original Run directory>"}]}
```

Creation/addition persists immediately. `flock` serializes additions; the existing
atomic JSON writer commits complete records. Adds validate the actual Run identity,
prevent duplicates and preserve order. A new Python process reopened a Session
containing two distinct genuine recorded Runs. Concurrent-add tests preserve one
association. No telemetry, scores, frozen inputs or result database is copied into
Session storage. Missing/changed referenced evidence stays visibly unavailable;
the backend never substitutes a different Run directory for a saved reference.

Comparison tables retain raw stored metric objects, availability, units, evaluation
window, coverage and per-Run outcome. Existing `comparison_reasons`, metric-evidence
checks, original comparison reconstruction and protected-condition differences
explain limits. Metric definition/unit guards were extracted unchanged from the
existing scientific comparison owner for reuse, including sessions without a
sensor-variable policy. Forged/stale hashes are rejected as comparison evidence.
Different seeds, robot/scenario/noise/windows or protected conditions produce
reasons; the conservative cross-study view may restrict ranking even when a
researcher can interpret side-by-side values. **No ranking or winner is produced.**

## Actual verification

Evidence: `results/dashboard_backend/`. Test counts overlap; do not sum them.

| Check | Actual result |
|---|---|
| Full functional regression | `2012 passed, 18 deselected, 1 xfailed, 3 warnings in 1285.40s (0:21:25)` |
| Initial focused Author/Catalog/backend | `61 passed in 230.63s (0:03:50)` |
| Backend/scientific comparison | `52 passed in 148.91s (0:02:28)` |
| Complete final backend file | `24 passed in 84.32s (0:01:24)` |
| Final interface boundary subset | `4 passed, 20 deselected in 29.81s` |
| Direct equivalence | 75 cases; zero differences |
| Scientific preparation equivalence | 21 cases; zero differences |
| Estimator numerical replay | 124,305/124,305 exact leaves for canonical and historical IDs; max absolute difference 0 |
| Historical frozen reconstruction | 3 real Task B/E/E.1 snapshots, unchanged; Task R history queried directly |
| Normal build | `Summary: 10 packages finished [3min 0s]` |
| Copied build | `Summary: 10 packages finished [8min 18s]` |
| Focused copied refreshes | `Summary: 1 package finished [14.1s]`; `Summary: 1 package finished [7.37s]`; `Summary: 1 package finished [8.94s]` |
| Source-independent installed checks | M06–M09 chain, CLI, real one-file discover/preview/validate/save/revise/freeze/history/Session workflow, saved-revision/schema-2 explain passed |
| Final copied bytes | All 29 experiment production modules and 20 owner YAML resources match source |
| Scoped style | New backend/test files flake8 clean; backend docstrings clean; 567 inherited findings before / 563 after in existing changed Python scope, zero new |
| Preservation | 765 protected source files exact, all 30 existing YAMLs exact, all Catalog revisions/hashes intact; 15,106 historical artifacts retain size/mtime |

The full suite collected before the six final client/explanation guard cases were
added; the final 24-case backend suite and boundary/installed checks cover them.
Its three warnings came from the new concurrency test using `fork()` after earlier
ROS tests had started threads. The test was changed to independent Python
subprocess clients; the two relevant persistence/concurrency tests passed again in
2.27 s without warnings. No production code changed for this test-harness fix.
Style/copyright wrappers are explicitly excluded from the functional suite; no
functional test exclusion or scientific tolerance was added to obtain PASS.

The first raw replay comparison reported three **string path** differences left by
Task R. Those raw logs/results are preserved. `verify_numerical.py` normalizes only
`robot_description_config` in three recorded robot parameter blocks after proving
Task R's physical-file byte identity; all 124,305 numerical leaves remain exact.
The 75/21 comparison scripts retain their established content-based path and
canonical empty-envelope normalization, with the physical SHA guard. No numerical
leaf, condition, safety threshold or acceptance tolerance is masked.

Independent review was partial: its session hit a usage limit after finding two
gaps (metric contract/unit checks without policy and pre-UUID historical access).
Both were reproduced RED and corrected; root completed the requirements/diff audit.
This is not represented as completed independent sign-off. See `review.md`.

## Bounded real ROS/Gazebo lifecycle

Exactly one scored attempt and one different cooperative-cancellation check; no
failed scientific condition was retried. Both used the copied public backend,
existing worker/Runner and saved **EXP-BACKEND-INTEGRATION revision 1**, even though
revision 2 already existed. Source: `runtime_lifecycle.json` and original run files.

Scored job state: **failed**.
Actual Run ID: `7483ae59-2ca2-4655-8e18-0b83f6dccb02`.
Execution: **failed**; acceptance: **None**.
Error: `{'type': 'RunFailure', 'code': 'readiness_timeout', 'message': "readiness_timeout: {'clock': 29.962, 'monotonic_elapsed': 45.00439171900507, 'origin': None}", 'details': {'clock': 29.962, 'monotonic_elapsed': 45.00439171900507, 'origin': None}}`.

| Measurement | Recorded result |
|---|---|
| Distance error | Unavailable: not scored |
| Pointing error | Unavailable: not scored |
| RTF | Unavailable: not scored |
| Control computation time | Unavailable: not scored |
| Physical position violation | Unavailable: not scored |
| Physical speed violation | Unavailable: not scored |

Safety and coverage remain exactly the Runner's recorded result:
`{"accepted":null,"reason":"evaluation unavailable"}`.

Cancellation reached a real launched ROS process, not only a queued mock. The
job ended `cancelled`; repeated cancellation was idempotent.
Its actual Run retains execution failure with `error.code=cancelled`, unavailable
scored metrics and original cleanup evidence. The backend sent a cooperative stop
request; the existing Runner's bounded cleanup escalated SIGINT to SIGTERM and
returned **-15**, with no survivors. This is reported as cancellation, not a clean
completed/scored execution or all-child exit-0 claim. Every observed final Run had cleanup
with no owned survivors. The post-test global launch/server/container/worker scan
found no remaining matching processes. The live Session also associated this new
Run with a different historical experiment without changing either artifact.

The scored attempt did not enter a scoring window: `readiness_timeout` after
45.004 s monotonic, latest recorded clock 29.962. All required streams were
observed (including 580 valid-control records and 290 active-actuation records).
Terminal records were ahead of the latest observed clock beyond the existing
80 ms freshness gate, consistent with an observer/clock-freshness limitation;
these terminal samples do not establish the complete causal callback ordering.
No clear Task C configuration/numerical regression was identified. Read-only
counts are in `runtime_readiness_audit.json`; these are diagnostics, not scored
tracking/safety measurements. No scored distance, pointing, RTF or physical-limit
result is available for this attempt, and no fresh accepted/completed scored Run
is claimed. A genuine accepted historical Run was independently read and verified.
The launch log also records a velocity-actuation rclpy subscription-conversion
exception during SIGINT teardown (child exit 1), despite launch cleanup return 0
and no survivors. This remains runtime debt; no simulator/actuation fix was made.

These short runs demonstrate integration and truthful lifecycle/results handling;
they do not establish all-scenario runtime acceptance, optimal tracking, hardware
safety, localization accuracy or physical trajectory repeatability. No physics,
noise, target dynamics, limits, PID gains, controllers or timeouts were tuned.

## Reproduction and D1 contract

Build/test commands are retained in `verification_commands.txt`. Standard build:

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -p no:launch_testing -p no:launch_ros --import-mode=importlib -q src/robot_experiments/test/test_backend.py
```

Minimal headless workflow, choosing a fresh writable directory:

```python
from pathlib import Path
from tempfile import mkdtemp
from ament_index_python.packages import get_package_share_directory
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog

output = Path(mkdtemp(prefix="backend_demo_"))
catalog = ExperimentCatalog(output / "catalog.yaml")
catalog.initialize()
backend = ExperimentBackend(output / "client", catalog=catalog)
choices = backend.list_component_options("target_dynamics")
source = Path(get_package_share_directory("robot_experiments")) / "examples/tracking.yaml"
preview = backend.preview_experiment(source)
assert backend.validate_experiment(source)["valid"]
record = backend.save_experiment(source)
session = backend.create_session({"title": "Tracking comparison"})
job = backend.start_run(record["id"], record["revision"],
                        output=output / "runs", enable_actuation=True)
# Poll get_job_status(job["job_id"]); the Runner allocates actual run_ids.
# Then inspect get_run_results(run_id), add_run_to_session(session_id, run_id),
# run another saved experiment/revision and inspect get_session_comparison(...).
```

The actual installed builder proof is `verify_installed_backend.sh`; the actual
scored/cancellation proof is `run_lifecycle.sh`. They retain their original
Catalog/revisions/output and must not be rerun into existing evidence to reset it.
A fresh reproduction uses fresh paths as in the example. Actual copied build:

```bash
colcon --log-base results/dashboard_backend/copied-colcon build --build-base /tmp/dashboard_backend_build --install-base /tmp/dashboard_backend_install --executor sequential
```

D1 creates a separate `robot_dashboard` package and consumes this API. It owns UI
controls, polling and rendering, not scientific resolution/validation, ROS processes
or score computation. It should preserve null/unknown states and the three outcome
dimensions, display explicit actuation opt-in and comparison reasons, and retain
trusted result-root configuration across process restarts. Existing Runner live
state is coarse (`preparing` then terminal); no readiness percentage is available.
Independent interactive Gazebo setup/live reconfiguration remains unsupported and
requires a future controlled lifecycle operation. Sessions need real UUID Runs;
older pre-UUID evidence remains directory-readable. Large time series are references,
not a new streaming/chart service. Existing scientific/readiness limitations stay
separate from this API milestone. No D1/D2/D3, web server or Plotly/Dash work here.

## Exact task-relative files

Relative to completed Task R / Task C's captured dirty tree, not Git HEAD:

```text
M docs/experiment_composition.md
M src/robot_experiments/robot_experiments/authoring.py
M src/robot_experiments/robot_experiments/comparison.py
M src/robot_experiments/robot_experiments/jobs.py
M src/robot_experiments/robot_experiments/runner.py
A docs/validation/dashboard_backend.md
A src/robot_experiments/robot_experiments/backend.py
A src/robot_experiments/test/test_backend.py
```

No source files removed. All previous experiments and untracked evidence preserved.
Generated Task C audit/scripts/artifacts are under `results/dashboard_backend/`;
Sessions contain references only. HEAD and staged diff are unchanged. No commit/push.
