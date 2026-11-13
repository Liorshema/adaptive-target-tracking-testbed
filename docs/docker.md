# Docker deployment

This deployment uses the existing Dashboard, ExperimentBackend, immutable Catalog,
Jobs and ROS/Gazebo Runner in one Linux container. No host ROS, Gazebo, Python or
Docker-in-Docker installation is needed. Gazebo runs headlessly; GUI forwarding is
outside this deployment. **Container end-to-end acceptance is not yet verified**;
see [the validation record](validation/docker.md).

## Start from a clone

Install Docker Engine with Compose v2 on Linux, or Docker Desktop using **Linux
containers** on Windows/macOS. On WSL, enable Docker Desktop integration for the
distribution containing the clone. The initial image targets `linux/amd64`.
ARM hosts require Docker emulation and have unverified timing/performance. Allow
several GB of disk space and at least 4 CPU cores / 8 GB RAM as a starting point;
these are operating recommendations, not measured acceptance guarantees.

From a normal Git clone, including a shallow clone:

```bash
docker compose up --build -d
docker compose ps
docker compose logs -f testbed
```

Open **http://localhost:8050**. The container binds `0.0.0.0:8050`; Compose publishes
only `127.0.0.1:8050`. A free host port 8050 is required. The existing
`/dashboard-health` endpoint is the image healthcheck; it verifies service identity
and `/data/catalog.yaml`, not simulator readiness or scientific acceptance.

The default is an unprivileged UID/GID **10001:10001**, an isolated bridge network,
all Linux capabilities dropped, `no-new-privileges`, and Docker's init process.
There is no Docker socket, privileged mode, host networking, GPU or display mount.
The existing single-process Dash server is intended for trusted local use, not an
authenticated public or multi-tenant service.

In Experiment Lab, select or create a Scenario, configure its registered components
and duration, Validate, and save an immutable revision. Select a Session and press
Play/Run explicitly. **Actuation defaults OFF**; check the existing explicit
actuation opt-in only when you intend physical robot movement. Do not select GUI
execution: this image has no display, and backend validation will report that
capability as unavailable. Opening the Dashboard launches no experiment.

## Persistent data

Compose mounts the named Docker volume `adaptive-target-tracking-data` at `/data`.
It contains Catalog/revisions, Dynamics Library, Sessions, Jobs and request/status
records, default `/data/runs` outputs, telemetry, metrics, frozen configurations,
comparison evidence, ROS logs, shutdown receipts and dependency/build manifests.
Keep the Dashboard's output location under `/data`; custom paths elsewhere in the
container are not persistent. Host absolute paths in imported old configurations
must be migrated or explicitly mounted; they do not become container paths.

```bash
docker compose down
docker compose up -d
```

These operations retain data. **Do not use `down -v`, `docker volume rm`, or prune
volumes containing research data.** Do not share this volume between concurrent
Dashboard deployments. `TESTBED_DATA_VOLUME` can select a different named volume;
keep that value consistent when restarting. An optional bind mount must be writable
by UID/GID 10001; the entrypoint fails clearly rather than running as root or
recursively changing the ownership of user files. New named volumes inherit the
image directory's correct ownership without host setup.

Back up the entire `/data` directory while no experiment is running. For example:

```bash
docker compose cp testbed:/data ./testbed-data-backup
```

This is a data backup, not a second results database. No host's existing WSL Catalog
is automatically imported or altered by this deployment.

## Build and source provenance

The Dockerfile pins the official Ubuntu 24.04 image and BuildKit frontend by real
registry digest, Ubuntu packages to snapshot `20260619T000000Z`, and ROS Jazzy to
snapshot `2026-06-18`. Dash 3.4.0, Plotly 7.1.0 and Narwhals 2.28.0 wheels have
exact versions and verified SHA256 hashes. Their remaining Python dependencies,
including numerical libraries, come from the signed apt snapshots and Python 3.12
system packages, shared into a venv. No host Python environment is copied.

The existing manifests declare `ament_python` as a build tool; this is a colcon
build type, not an installable rosdep package. The image supplies colcon/setuptools
explicitly and skips only that key plus the two pip-owned UI packages in rosdep.
The entrypoint's LF checkout rule supports Windows Git without a broken shebang.

The older compatible ROS snapshot is deliberate: the recovery plugin requires
**gz-physics 7.6.0 and DART 6.13.2 EXACT**. Current Jazzy vendor packages have moved
on; replacing the snapshot without reviewing native compatibility may fail the
build. CMake constraints, physics, safety limits and algorithms are unchanged.
Snapshot apt downloads use the archive's HTTP endpoint with signed Release and
package checksums; signature checks are never disabled. TLS verification is not
bypassed. rosdep resolves package.xml dependencies; its online rule/index metadata
is not separately frozen, a remaining rebuild limitation even though apt package
availability is frozen. Snapshot availability and signing-key validity are external
dependencies. Update image/frontend digests, snapshots and wheel locks deliberately,
then repeat the full acceptance checks rather than relying on a floating rebuild.

Every ROS package is built with a normal copied installation. Image build checks
generated message type support, both native libraries with `ldd` and actual
`gz plugin --info` loading/registration, registered components, nominal preflight,
and the real Dashboard layout. Source/configuration/defaults stay inside the image;
only data is mounted. Normal local/WSL build and launch commands remain valid.

The image retains real `.git` objects, index and refs and the copied source, so
Runner records the actual commit, branch, dirty state, tracked diff hash and source
hash. Git config, hooks, reflogs, credentials, local build products and historical
Runs are excluded from the context. Git history is part of the image: do not
publish images built from a repository whose history contains private material.
Archive downloads without `.git`, linked worktrees and alternate object stores are
unsupported; use a normal clone. A dirty build is recorded as dirty, never assigned
a fabricated revision. Workers inherit `/opt/testbed` as their working directory.

The entrypoint archives actual package versions, native binary hashes, component
discovery and source provenance under `/data/deployments/<manifest-hash>/`. Existing
Run metadata is unchanged; it does not automatically include the Docker image ID.
Record that real image ID externally when archiving a study:

```bash
docker image inspect adaptive-target-tracking:local > docker-image.json
docker compose cp docker-image.json testbed:/data/deployments/docker-image.json
```

Retain/tag the tested image or its registry digest for repeatable execution. The
default `:local` tag is a convenience tag that a rebuild replaces; it is not an
immutable release identifier. Dependency snapshots improve rebuild reproducibility
but do not promise bit-identical compiler output or identical host performance.

## Diagnostics and acceptance commands

Exec does not automatically source the ROS overlay. Reuse the entrypoint:

```bash
docker compose exec -T testbed /opt/testbed/tools/docker/entrypoint.sh \
  python tools/docker/verify_install.py
docker compose exec -T testbed id
curl --fail http://localhost:8050/dashboard-health
```

For a short **real** headless experiment using the existing backend, run the Python
block below. It creates one independent immutable IMM experiment and Session;
there are no synthetic observations/results. Duration defaults to six simulation
seconds and the source's existing settling policy is preserved. The poll deadline
is a verification bound, not a replacement for Runner readiness/stall/wall limits.

```bash
docker compose exec -T testbed /opt/testbed/tools/docker/entrypoint.sh python - <<'PY'
import json, os, time, uuid
from pathlib import Path
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog

root = Path('/data')
backend = ExperimentBackend(root, catalog=ExperimentCatalog(root / 'catalog.yaml'))
identifier = 'EXP-CONTAINER-' + uuid.uuid4().hex[:12].upper()
draft = backend.new_draft(identifier, 'Container verification', 'EXP-NOMINAL', base_variant='imm')
draft['experiment']['comparison'] = None
draft['experiment']['execution'] = {
    'duration': float(os.environ.get('TESTBED_DURATION', '6')), 'seeds': [42]}
checked = backend.validate_experiment(draft)
assert checked['valid'], checked
record = backend.save_experiment(draft)
session = backend.create_session({'title': 'Container verification'})
sid = session['session_id']
backend.add_experiment_to_session(sid, record['id'], record['revision'],
    expected_token=backend.get_session(sid)['token'])
job = backend.start_run(record['id'], record['revision'], output=root / 'runs',
    enable_actuation=os.environ.get('TESTBED_ENABLE_ACTUATION') == '1', gui=False, session_id=sid)
deadline = time.monotonic() + 300
while job['state'] in {'queued', 'running'}:
    if time.monotonic() > deadline:
        backend.cancel_job(job['job_id'])
        raise TimeoutError('Verification bound reached; inspect the cancelled Job')
    time.sleep(.5)
    job = backend.get_job_status(job['job_id'])
assert job['state'] == 'completed', job
assert job['run_ids'], job
report = {'job': job, 'session_id': sid, 'runs': []}
for rid in job['run_ids']:
    result = backend.get_run_results(rid)
    backend.add_run_to_session(sid, rid)
    assert result['execution_status'] == 'completed', result['status']
    assert result['metadata']['git']['revision'], result['metadata']
    assert result['metadata']['git']['worktree_source_sha256']
    metrics = result['metrics']['metrics']
    assert any(m.get('available') for m in metrics.values()), metrics
    assert any(s['sample_count'] for s in backend.read_time_series(rid)['series'].values())
    report['runs'].append({'run_id': rid, 'status': result['status'],
        'metrics': metrics, 'artifacts': result['artifacts']})
path = root / 'jobs' / job['job_id'] / 'deployment-verification.json'
path.write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
PY
```

Inspect the saved Run's `launch.log`, `readiness.json`, `metadata.json`, `status.json`,
telemetry JSONL, time-series samples, metrics and cleanup survivors. Confirm the
evaluation origin/window was established, the requested duration plus settling
elapsed, and required ROS streams have actual samples. Confirm Results & History
shows the Run and the Session references it. Execution completion is separate from
tracking quality, physical safety and metric coverage acceptance.

For a representative 60-second actuated experiment, use that same block with
`-e TESTBED_DURATION=60 -e TESTBED_ENABLE_ACTUATION=1` added immediately after
`docker compose exec -T`. This is an explicit actuation opt-in. Do not change the
Runner's 45-second readiness deadline, 0.08-second max skew, or other limits to
force completion. If it fails, preserve/report the actual failure and artifacts.

## Stop, interruption and persistence validation

Normal Stop/Cancel in the Dashboard still calls the existing backend cancellation
API. A deployment shutdown adds opt-in cooperative cancellation of Jobs in `/data`:

```bash
docker compose stop
docker compose up -d
```

Docker's 60-second grace gives the existing Jobs' shared 25-second cancellation
budget time to finish. Requests are sent to all active Jobs before waiting; no PID
is adopted or force-signalled by the added API. Timestamped `/data/shutdown/*.json`
receipts report actual states/errors. An empty error list only means the request
path finished; inspect each Run's own cleanup/survivor evidence. All normal launch
cleanup remains with Runner's `OwnedLaunch`.

If a worker hangs or the container is killed, do not infer successful cleanup or
completion. For a deliberate interruption test, start a disposable validation Job
through the Dashboard, record its ID, then:

```bash
docker compose kill -s SIGKILL testbed
docker compose up -d
docker compose exec -T testbed /opt/testbed/tools/docker/entrypoint.sh python - <<'PY'
from pathlib import Path
from robot_experiments.jobs import ExperimentJobs
jobs = ExperimentJobs(Path('/data/jobs'))
for request in jobs.directory.glob('*/request.json'):
    print(jobs.status(request.parent.name))
PY
```

The interrupted Job must report `worker_lost` (or an actual previously written
terminal failure/cancellation), **not completed**. Raw Run status may retain its
last phase because SIGKILL cannot execute `finally`. That artifact is incomplete;
retain it with the Job's `worker_lost` diagnostics. There is no auto-resume or
fabricated terminal result. Repeat down/up and verify the same Catalog revision
hashes, Session/Run IDs and completed artifact hashes survive. A forced kill cannot
produce a graceful shutdown receipt; absence must not be interpreted as success.

## Rollback without deleting research data

First stop the service and back up `/data`. The task has not committed anything.
Preserve the Docker implementation before switching branches:

```bash
docker compose down
git stash push -u -m 'Preserve Docker deployment work before rollback'
git switch main
git stash list
```

The stash contains source changes, not Docker volumes or ignored Run evidence.
Do not pop it onto main merely to roll back. To resume the feature later:

```bash
git switch feature/docker
git stash apply 'stash@{0}'
```

Review the stash description/ID before applying; it may no longer be the latest
stash. If Docker changes are later committed by an engineer, switch to the previous
known-good Git revision/image instead. Keep the named data volume and its backup;
no Git rollback command here removes or rewrites persisted research data.

References: [ROS/Gazebo pairing](https://gazebosim.org/docs/harmonic/ros_installation/),
[Docker image pinning](https://docs.docker.com/build/building/best-practices/),
[Compose service lifecycle](https://docs.docker.com/reference/compose-file/services/).
