# D2.6 — Controlled Gazebo GUI lifecycle

Verified 2026-10-30, ROS 2 Jazzy / Gazebo Sim 8 / WSL2.

**Verdict: CONDITIONAL.** Real window ownership, configuration freezing, default-OFF
actuation, duplicate/conflict prevention, explicit error handling and controlled
cleanup passed. The actual Gazebo window opened, but its captured 3D viewport was
blank. Native Ogre2 logs contain rendering exceptions. Correct robot/world
rendering is therefore **not verified**. No screenshot or process-readiness check
is represented as proof of a functioning rendered scene.

## Implementation and ownership

The dashboard remains a client of `ExperimentBackend`. Its new
`open_simulation(id, revision, variant=..., seed=...)`, `get_simulation_status()`
and `close_simulation(job_id)` methods extend existing `ExperimentJobs` with an
unscored preview kind. Existing Catalog preflight, immutable saved revisions,
seeded materialization, `DomainLease`, `OwnedLaunch` and
`robot_bringup/closed_loop.launch.py` remain authoritative. No alternate experiment
engine, owner configuration profile, recording store or physics mechanism was added.

- Preview always freezes `gui=true`, `enable_actuation=false` and
  `publish_commands=false`. The UI uses the selected saved variant and first saved
  seed; the backend also accepts an explicit seed belonging to that revision.
- A workspace lock guards preview/Run creation. Repeated Open of the current
  selection returns the same owner. Changing revision/variant/robot/world requires
  Close and proven cleanup. A preview and scored Job cannot overlap in this workspace.
- Opening/Running/Closing/Closed/Error states persist in existing Job artifacts.
  Running specifically means an owned mapped X11 window plus ROS clock observed.
  It does not assert rendered-scene readiness or scientific acceptance.
- Browser refresh observes the existing owner without launching. Browser closure
  does not close the simulator. Preview watches the dashboard server's PID and
  create-time; server exit or cooperative worker signals request cleanup.
- Closing the native GUI triggers the existing launch's essential-process shutdown.
  Cleanup uses the existing bounded native process-tree ownership and signal stages.
  Partial post-spawn startup failure and status-write failure also invoke cleanup.
- Unknown/lost-worker/incomplete cleanup blocks restart. Worker SIGKILL can bypass
  cooperative cleanup; the system reports uncertainty and does not adopt/kill
  arbitrary processes. There is no claim of crash-proof supervision.

### Scored GUI option

`experiment.runtime.gui` is now a validated boolean, default `false`. Catalog runtime
presentation overrides accept `gui`; Author accepts the same field. Materialization
freezes the actual value in the resolved system configuration. The existing Runner
uses that frozen configuration and records GUI/display/Qt platform in metadata.
Headless CLI defaults remain unchanged; existing comparability/provenance protections
are preserved. The dashboard's explicit **Show Gazebo GUI for this Run** checkbox is
OFF and resets after successful Start. It is independent of the actuation opt-in.
An explicit unchecked value overrides a saved GUI-enabled definition. No scientific
Run was executed for this visual/lifecycle task; scored GUI is tested at the request,
validation and frozen-materialization boundaries, not claimed live-scored verified.

### Display support and rendering blocker

The optional GUI path lazily loads `libX11.so.6`, checks real display access and
identifies mapped windows by owned `_NET_WM_PID`. It selects Qt `xcb` for the GUI
job environment. WSLg XWayland is supported; native Wayland-only window verification
is unsupported and returns a precise reason. Display capability does not prove
renderer capability.

The actual capture shows Gazebo chrome/world inspector but a white scene and empty
Entity Tree. The contemporaneous copied native log identifies
`GL_RENDERER = llvmpipe (LLVM 20.1.2, 256 bits)`, missing material warnings,
`RenderingAPIException` stating that the fixed-function pipeline requires shaders,
and GLSL sampler-type validation errors. These are observed native rendering errors;
the exact cause of the blank scene was not isolated. A window capture also cannot
alone exclude native GL capture limitations. SceneBroadcaster is present in the
frozen deployment, and the isolated ROS clock advanced. Neither establishes that
the selected robot/world was visibly rendered.

Actionable follow-up: inspect the live native viewport and Ogre2 material/shader
initialization on this WSL display, then capture a rendered robot/world in the same
owned partition. Do not change physics, algorithms, safety gates or timeouts to
mask this graphics issue. This task used one bounded preview launch and preserved
its failure evidence rather than launching a parameter sweep or changing renderer
settings speculatively.

## Actual integration evidence

The installed application ran at `http://127.0.0.1:8053` with isolated workspace
`results/dashboard_simulation/workspace`. The previous dashboard on port 8052 was
left running untouched.

| Check | Actual result |
| --- | --- |
| Saved selection | `EXP-NOMINAL` r3, `imm`, seed 42; Catalog unchanged |
| Preview owner | `795e3d35-73f4-418d-a353-6e517f320954` |
| Native isolation | ROS domain 92; unique recorded Gazebo partition |
| Visible mapped window | Gazebo Sim, PID 126032, window 6291474; captured at 1000×845 |
| Scientific provenance | Frozen GUI ON, actuation OFF, command publication OFF |
| Physics observation | ROS clock observed through 19.772 simulation seconds |
| Duplicate Open | Same Job returned; exactly one request created |
| Conflict prevention | Different revision and scored Start rejected until Close |
| Refresh | Resumed observation of the same owner; no relaunch |
| Close | `closed`, return code 0, SIGINT only; cleanup 0.992084 seconds |
| Cleanup | No owned live PID identities or windows; domain lease reacquired |
| Invalid saved variant | Backend rejected; UI Error shown; no second launch |
| Browser | No recorded page/console errors; desktop document 1920×1080 |
| Result storage | No scientific Run, metrics or duplicate result store created |
| Rendered robot/world | **Unverified: blank captured viewport and native renderer exceptions** |

Artifacts are under `results/dashboard_simulation/`, including
`live_verification.json`, `final_native_audit.json`, `ogre2.log`, browser script/log,
and the single Job's request/status/launch log/frozen inputs. The script intentionally
asserts an empty workspace before launching: do not rerun it against this preserved
workspace. Use a new explicitly isolated data directory for a new check.

Actual screenshots (not reference images):

- [Running dashboard](../../results/dashboard_simulation/dashboard_running.png)
- [Actual Gazebo window — blank viewport](../../results/dashboard_simulation/gazebo_window.png)
- [Closed dashboard](../../results/dashboard_simulation/dashboard_closed.png)
- [Backend rejection shown as Error](../../results/dashboard_simulation/dashboard_error.png)

## Tests and build

- Core execution/configuration/Runner/Catalog selection: **69 passed** in 178.21 s
  (`core_tests.log`). Later native cleanup fixes were additionally covered by the
  review regression selection: **33 passed** (`review_green.log`).
- Dashboard + backend + preview selection: **67 passed** in 337.00 s
  (`dashboard_backend_verified.log`), including existing Builder, Run/Cancel,
  Results/Session and duration/revision workflows with recorded fixture paths.
- Final focused architecture/lifecycle/UI selection: **67 passed** in 54.52 s
  (`final_focused.log`). Counts overlap the suites above and are not additive.
  The initial architecture check correctly rejected a ROS clock subscriber in the
  domain module; it was moved to the existing `recording/ros.py` adapter, and the
  unchanged architecture gate plus lifecycle tests passed (**13 passed**, previous
  `architecture_lifecycle_green.log`). No architecture acceptance rule was weakened.
- Scoped dashboard flake8: clean (`dashboard_lint_final.log`). No claim is made
  about unrelated repository-wide historical lint failures.
- Installed build: **3 packages succeeded**, 1 min 12 s (`build_verified.log`).
  Colcon emitted the existing selected-underlay `robot_simulation` override warning.
  Final installed Python modules were byte-compared to source before/after the live
  check. HTTP 200 and native cleanup were rechecked without another launch.

Reproduction commands (ROS environment and recorded fixtures already available):

```bash
cd /home/user/ros2_ws
source install/setup.bash
source results/dashboard_execution/evidence.env
export PYTHONPATH=src/robot_experiments:src/robot_dashboard:$PYTHONPATH
build/dashboard_venv/bin/python -m pytest src/robot_dashboard/test \
  src/robot_experiments/test/test_simulation.py \
  src/robot_experiments/test/test_backend.py -q
build/dashboard_venv/bin/python -m pytest \
  src/robot_bringup/test/test_architecture.py \
  src/robot_bringup/test/test_architecture_tree.py \
  src/robot_experiments/test/test_simulation.py \
  src/robot_dashboard/test/test_simulation_ui.py -q
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_simulation robot_experiments robot_dashboard
```

Run existing `robot_experiments/test/test_execution.py` separately from the
Dashboard's same-named test module to avoid the ROS pytest plugin's module-name
collision. The real check uses installed packages and a working native display,
not mocked Gazebo or a screenshot of a fabricated scene.

## Working workflow

The D2.6 server is already listening on 8053. To start it after stopping that server:

```bash
cd /home/user/ros2_ws
source install/setup.bash
ros2 run robot_dashboard dashboard --port 8053 \
  --data-dir /home/user/ros2_ws/results/dashboard_simulation/workspace
```

Open `http://127.0.0.1:8053`, select saved experiment/revision/variant in Experiment
Lab, inspect the saved selection, then Open Simulation. Close before selecting a
new robot/world for launch. For scored execution, close the preview, choose a
Current Session, explicitly choose GUI if wanted, and Start. Actuation remains OFF
unless separately explicitly selected for that scored Start. The rendering blocker
above still applies on this WSL display.

## Exact task-relative changed files

Existing files modified:

- `src/robot_experiments/robot_experiments/authoring.py`
- `src/robot_experiments/robot_experiments/backend.py`
- `src/robot_experiments/robot_experiments/catalog.py`
- `src/robot_experiments/robot_experiments/configuration.py`
- `src/robot_experiments/robot_experiments/execution.py`
- `src/robot_experiments/robot_experiments/jobs.py`
- `src/robot_experiments/robot_experiments/recording/ros.py`
- `src/robot_experiments/robot_experiments/runner.py`
- `src/robot_experiments/test/test_backend.py`
- `src/robot_experiments/test/test_execution.py`
- `src/robot_dashboard/robot_dashboard/app.py`
- `src/robot_dashboard/robot_dashboard/execution.py`
- `src/robot_dashboard/robot_dashboard/assets/dashboard.css`
- `src/robot_dashboard/test/test_builder.py`
- `src/robot_dashboard/test/test_execution.py`
- `src/robot_simulation/launch/simulation.launch.py`
- `docs/dashboard.md`

New files:

- `src/robot_experiments/robot_experiments/simulation.py` — unscored lifecycle adapter.
- `src/robot_experiments/test/test_simulation.py` — configuration/ownership regressions.
- `src/robot_dashboard/robot_dashboard/simulation.py` — thin Dash controls.
- `src/robot_dashboard/test/test_simulation_ui.py` — callback contracts.
- `docs/validation/dashboard_simulation.md` — this report.

Generated artifacts belong only under `results/dashboard_simulation/` (plus normal
ignored build/install/test caches). No source files were removed. Baseline SHA audit
and historical size/mtime audit preserve all 15,825 prior evidence files. No owner
YAML, physics, numerical algorithms, limits, timeouts or safety gates changed. No
commit/push; no D3 work.
