# Docker deployment validation

## Scope and design

The initial deployment is one Ubuntu 24.04 container with ROS 2 Jazzy, Gazebo
Harmonic and the existing Plotly Dash application. The Dashboard, ExperimentBackend,
Registry, immutable Catalog, Jobs, Runner, Recorder and metrics remain authoritative.
No alternative experiment engine, database, physics or controller is introduced.
Headless execution only; physical actuation requires the existing explicit opt-in.

The user explicitly selected a single-container implementation. Separate services
would require new ROS discovery and process-ownership boundaries without a benefit
for this milestone. Host ROS installation would defeat the deployment goal.

## Repository audit

- Started from `main`, commit `c02259a68b76e51abbb141a4ee2b5fd99740d4e6`.
  The initial working tree was clean. Created `feature/docker`; no commit or push.
- Eleven ROS packages: ten Python packages plus `robot_interfaces`, whose CMake
  generates five ROS messages with `rosidl_default_generators`.
- `robot_simulation/setup.py` builds two C++17 setuptools extensions using CMake:
  the actuator System and recovery DART physics plugin. They are Gazebo libraries,
  **not importable CPython extensions** despite their platform suffixes.
- Native CMake requires exactly gz-physics **7.6.0** and DART **6.13.2**.
  Existing pinned upstream source/checksums remain unchanged. Current ROS apt
  candidate `gz_physics_vendor` is 0.0.9 (upstream physics has advanced to 7.8),
  whereas the local verified installation uses vendor 0.0.7. A dated compatible
  ROS package snapshot is required; no `EXACT` constraint is relaxed.
- Numerical Python dependencies are NumPy, SciPy, PyYAML and psutil; MPC uses
  SciPy optimization, not an undeclared external solver. Dash and Plotly are the
  optional UI dependencies. ROS Python must use Noble's system Python 3.12 ABI;
  the image venv includes system packages and does not replace numerical ROS deps.
- `robot_dashboard.cli:main` already accepts host, port and data directory and
  exposes `/dashboard-health`. Browser interaction uses existing backend APIs.
- `ExperimentJobs` persists accepted revisions and starts detached workers.
  Workers call the existing Runner with cooperative `stop.request` cancellation.
  `OwnedLaunch` owns session/descendant cleanup and records signals/survivors.
  Merely killing Dash would not give detached workers time to clean up.
- Jobs already report a dead active worker as `worker_lost`, never completed.
  Graceful container shutdown must request cooperative stops and preserve this
  conservative behavior if Docker ultimately sends SIGKILL.
- Backend data includes Catalog, Dynamics Library, Sessions, job requests/status,
  output roots and Run references. Runner writes immutable inputs, telemetry,
  metrics, readiness, logs, comparison and cleanup evidence beneath its output.
- Runner Git provenance is read from its working directory. Preserve a real Git
  object database/index/refs and source tree in the image, excluding Git credentials,
  hooks and config; never synthesize a commit. Archive checkouts without `.git`
  cannot provide a revision and must fail clearly or be documented as unsupported.

## Implementation and verification plan

1. Pin the Ubuntu image digest, compatible dated apt repositories and Dashboard
   Python dependencies; preserve package signing and native exact-version checks.
2. Build every ROS package, including generated interfaces and both native plugins;
   fail the image build if native load/registration or component discovery fails.
3. Serve as an unprivileged user on `0.0.0.0:8050`, publish `127.0.0.1:8050`, mount
   all persistent data at `/data`, and use the existing health endpoint.
4. Extend only the existing CLI/Jobs ownership boundary for opt-in conservative
   shutdown. Keep the non-Docker default behavior unchanged. Test cancellation,
   lost workers and incomplete cleanup without inventing scientific results.
5. Run focused tests, host build/native checks and Docker Compose validation where
   possible. Record Docker acceptance as UNVERIFIED if the engine is unavailable.
6. Document clean-clone startup, real short/full runs, persistence/restart,
   cancellation/forced interruption, provenance limitations and Git rollback.

## Environment availability

On this WSL2 host, `docker version` and `docker compose version` report that Docker
is not available in the distribution. Windows `docker.exe version` finds client
27.1.1 but fails connecting to `dockerDesktopLinuxEngine` (pipe not found).
There is no Docker Unix socket. Docker Desktop was not started or reconfigured.
Consequently image build, container HTTP, container ROS/Gazebo, volume recreation,
and Docker stop/kill checks cannot be claimed verified here.

The installed Windows Compose 2.29.1 client can parse configuration without the
engine. That check succeeded; it does not establish container execution.

## Implemented

- Pinned official Ubuntu image and Dockerfile frontend digests, dated Ubuntu/ROS
  apt repositories, exact compatible physics vendor packages and hash-locked UI
  wheels. Retrieved real digests/hashes from registry/PyPI; no invented revision.
- ROS snapshot inspection confirmed physics vendor
  `0.0.7-1noble.20260604.105412`, DART vendor
  `0.0.3-1noble.20260225.232143`, Sim vendor
  `0.0.10-1noble.20260604.111001`, and ros_gz
  `1.0.22-1noble.20260616.074726` are available.
- One nonroot, headless service, localhost-only host port, named `/data` volume,
  healthcheck, init/reaping, dropped capabilities and conservative stop grace.
- A minimal exec entrypoint sources the existing overlay, archives real dependency
  manifests under `/data/deployments`, and invokes the existing Dashboard directly.
  `.gitattributes` preserves its LF shebang in Windows checkouts.
- Build-time installation verifier discovers all source packages through the ament
  index, checks all five generated message types, loads/registers both actual native
  libraries, preflights nominal configuration and renders the real Dash layout.
- Only production API extensions: `ExperimentJobs.stop_all` (request all active
  Jobs before waiting within a shared budget), and opt-in CLI
  `--stop-jobs-on-exit`. Existing non-Docker CLI defaults remain unchanged.
  Receipts preserve raw states/errors, including a worker dying during cancellation;
  malformed/lost/timed-out workers are not made completed. No PID force-signalling
  was added. Existing Runner launch cleanup remains authoritative.
- `ament_python` is a declared build type, not a rosdep-installable package. It is
  skipped during image dependency resolution because colcon/setuptools are already
  explicitly installed; pip-owned Dash/Plotly keys are also explicitly skipped.

## Actual verification results

| Check | Actual result and evidence |
|---|---|
| Host workspace build | **11 packages succeeded**, 4 min 5 s; `results/docker/host_build.log` |
| Final reviewed-code build | Both changed packages rebuilt successfully, 9.18 s; `results/docker/reviewed_build.log` |
| Experiments/process regressions | **28 passed**, 9.19 s; `results/docker/experiments_final.log` |
| Dashboard/launcher regressions | **17 passed**, 11.42 s; `results/docker/dashboard_final.log` |
| Native servo/physics regressions | **11 passed**, 2.75 s; `results/docker/native_tests.log` |
| Total focused regression result | **56 passed**; no functional test failure remains |
| Native installed checks | Both actual libraries resolved with `ldd` and loaded by `gz plugin --info`; actuator and recovery physics registrations found, including 93 physics interfaces |
| Installed discovery | 11 packages, 5 generated message types, registered IMM/GP/EDMD/None and other owner components, Jacobian/WLS/MPC, and nominal preflight discovered on host |
| Dashboard HTTP/shutdown | Real isolated host server returned health and layout, received SIGTERM, exited 0 and persisted a shutdown receipt; final `results/docker/dashboard_smoke_reviewed/verification.json` |
| Compose configuration | Windows Compose 2.29.1 parsed the real file, confirming loopback publishing, nonroot user and persistent volume; `results/docker/compose.json` |
| Wheel locks | All three exact PyPI wheels downloaded and passed `--require-hashes`; `results/docker/wheel_check.log` |
| Shell/Python/static | `bash -n`, compilation and scoped `git diff --check` succeeded |

The new shutdown tests cover concurrent stop requests, preservation of completed
evidence, timeouts, existing lost workers, corrupt records, actual child exit during
cancellation, opt-in/default CLI behavior, nonzero error exit and signal-handler
restoration. These are transport/lifecycle fixtures, not fabricated experiment data.
The HTTP smoke used an ephemeral free loopback port and a new isolated data root;
it did not start a Run or access the default user Catalog.
The final native/interface/discovery/layout evidence and real source hash are in
`results/docker/host_installation_reviewed.json`; the two earlier installation
checks are preserved separately.

Failures retained as evidence: initial shutdown tests failed before the new API
(`shutdown_red.log`); final review exposed a real lost-worker receipt race that
failed before correction (`shutdown_race_red.log`). Two attempts at one combined
pytest invocation hit ROS launch-testing's duplicate `test_execution` import-path
collection error, even with `--import-mode=importlib` (`regression*.log`). Running
each package suite in its own interpreter resolved that harness issue without
changing product code or test thresholds. The only final test warnings were 24
existing Dash DataTable deprecation notices. No style/lint overhaul was attempted.

## Commands executed

```bash
git switch -c feature/docker
source install/setup.bash
build/dashboard_venv/bin/python /usr/bin/colcon build --parallel-workers 2 --event-handlers console_cohesion+
build/dashboard_venv/bin/python tools/docker/verify_install.py
```

Focused regression commands (separate processes intentionally):

```bash
export PYTHONPATH="$PWD/src/robot_dashboard:$PWD/src/robot_experiments:$PYTHONPATH"
build/dashboard_venv/bin/python -m pytest -q \
  src/robot_experiments/test/test_job_shutdown.py \
  src/robot_experiments/test/test_execution.py \
  src/robot_experiments/test/test_backend.py::test_job_identity_actuation_opt_in_and_cooperative_stop \
  src/robot_experiments/test/test_catalog.py::test_worker_uses_accepted_revision_after_catalog_changes
build/dashboard_venv/bin/python -m pytest -q \
  src/robot_dashboard/test/test_shutdown.py \
  src/robot_dashboard/test/test_launcher.py src/robot_dashboard/test/test_execution.py
build/dashboard_venv/bin/python -m pytest -q \
  src/robot_simulation/test/test_native_servo.py src/robot_simulation/test/test_physics_engine.py
build/dashboard_venv/bin/python -m pip download --only-binary=:all: --no-deps \
  --require-hashes -r tools/docker/requirements.txt --dest /tmp/testbed-docker-wheels
bash -n tools/docker/entrypoint.sh
git diff --check
```

Host HTTP evidence scripts executed:
`build/dashboard_venv/bin/python results/docker/http_smoke.py` and
`build/dashboard_venv/bin/python results/docker/http_smoke_reviewed.py`.
Compose parser command used from this WSL checkout:

```bash
'/mnt/c/Program Files/Docker/Docker/resources/bin/docker.exe' compose \
  -f '\\wsl.localhost\Ubuntu-24.04\home\user\ros2_ws\compose.yaml' config --format json
```

## Acceptance and remaining limitations

**Verdict: CONDITIONAL — implementation and host checks completed; Docker integration
and end-to-end acceptance remain UNVERIFIED. This is not a deployment PASS.**

| Requested acceptance | Verdict |
|---|---|
| 1. Clean Docker image build | **UNVERIFIED** — no reachable engine; host all-package build succeeded |
| 2. Container Dashboard at localhost:8050 | **UNVERIFIED** — Compose parsed and host HTTP smoke passed |
| 3. Components/configurations discovered | **CONDITIONAL** — actual host discovery/native loads passed; image build includes the check |
| 4. Short real headless container Gazebo Run | **UNVERIFIED** — no Run executed or invented |
| 5. Completed Run metrics/provenance/artifacts | **UNVERIFIED** in Docker — real source provenance retained by design, no mock metrics |
| 6. Representative full experiment | **UNVERIFIED** |
| 7. Persistence through down/up | **UNVERIFIED** — named-volume configuration parsed, no daemon-level recreation test possible |
| 8. Stop/Cancel and forced interruption | **CONDITIONAL** — real host SIGTERM and process/cancellation tests passed; Docker stop/SIGKILL remain unverified |
| 9. Regression/build/native checks | **PASS on host** — 56 tests, all 11 packages and both plugin loads; container checks unverified |
| 10. Existing local workflows | **PASS within focused scope** — full host build, default CLI behavior and workflow regressions preserved; no claim of every historical scenario rerun |

Run these exact Docker commands on an engine-enabled machine, then use the bounded
real experiment and down/up/stop/kill commands in [the deployment guide](../docker.md):

```bash
docker version
docker compose version
docker compose config
docker compose --progress plain build --no-cache
docker compose up -d
docker compose ps
curl --fail http://localhost:8050/dashboard-health
docker compose exec -T testbed /opt/testbed/tools/docker/entrypoint.sh python tools/docker/verify_install.py
```

Archive build output and actual verification Run/Session IDs before updating this
verdict. No existing science thresholds, readiness timeouts, numerical models,
physics, controller logic or safety limits changed. Existing research safety and
localization limitations remain; successful execution alone would not close them.
Initial platform is amd64, GUI is unsupported, snapshot availability is external,
rosdep rule/index metadata is online, and image IDs are not automatically injected
into each Run. Exact dependency manifests and native binary/source hashes are
archived; operators should also retain the actual tested image ID/digest. See the
guide for build pin updates and Git rollback that retains the data volume.

## Exact changed-file list

- `.gitattributes`
- `.dockerignore`
- `Dockerfile`
- `compose.yaml`
- `tools/docker/entrypoint.sh`
- `tools/docker/requirements.txt`
- `tools/docker/verify_install.py`
- `src/robot_dashboard/robot_dashboard/cli.py`
- `src/robot_dashboard/test/test_shutdown.py`
- `src/robot_experiments/robot_experiments/jobs.py`
- `src/robot_experiments/test/test_job_shutdown.py`
- `README.md`
- `docs/docker.md`
- `docs/validation/docker.md`

No existing source files, scenarios, configurations, Catalog data, Results or
historical evidence were removed. Generated validation artifacts are under the
ignored `results/docker/` directory. No merge, commit, push or volume deletion.
