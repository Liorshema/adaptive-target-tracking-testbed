# D2.6b — Paused preview, Play and automatic stop

2026-10-31, installed ROS 2 Jazzy / Gazebo Sim 8 / WSL2 dashboard.

**Verdict: CONDITIONAL.** Paused Open, explicit actuation opt-in, cleanup-gated
handoff, real controller-driven motion and saved failed-Run/Session references
were verified. Successful live completion of the configured evaluation duration
was **not** demonstrated: the single scored attempt hit the unchanged readiness
deadline before entering its evaluation window. Native GUI rendering remains
unverified. No Completed state, accepted safety or successful scored metrics are
claimed for this attempt.

## Causes and changes

Previously, Open inherited `gazebo.start_running=true`, ran an unscored preview
until Close, and deliberately forced actuation/publication OFF. A moving target
and stationary robot were therefore expected preview behavior. Preview had no
Runner evaluation window or automatic duration termination.

Open now materializes the same saved robot/world deployment with the existing
`gazebo.start_running` field set to **false**. This is a frozen preview startup
policy, not a physics modification or another configuration profile. The optional
`materialize_run(..., start_running=None)` argument leaves ordinary materialization
unchanged by default. A supplied value must be boolean. Preview remains unscored,
with `enable_actuation=false` and `publish_commands=false`.

The preview reports Paused once its owned mapped GUI exists. Its status states
that startup is configured paused; an initial clock sample can be absent while
paused. It watches the clock and closes with `preview_advanced` if subsequent
samples advance, directing users to dashboard Play rather than native Gazebo Play.
Paused owners participate in duplicate prevention, Close, worker-loss checks,
server-exit cleanup and restart blocking just like other active owners.

**Play / Run** queries the current preview and explicitly requests the existing
Runner handoff. The UI reports that the preview closes and a fresh scored GUI
opens. Reusing the original GUI/world is unsupported: Runner owns its fresh
isolated deployment, starts its Recorder before launch, verifies a clean domain
and freezes the scientific configuration. Adopting warmed preview state would
change these assumptions.

`ExperimentBackend.start_run(..., preview_id=None)` / existing `ExperimentJobs.start`
now accept the exact optional preview ID. Under the workspace's existing lock:

1. Revalidate saved experiment/revision and runtime before closing anything.
2. Require that ID to be the current Paused owner with the same frozen experiment
   identity. A changed robot/world/revision requires explicit Close first.
3. Require GUI mode, Close the preview, and require `closed` plus confirmed cleanup.
4. Create the ordinary scored Job; record `handoff_from` in its existing request
   and status. Unknown cleanup, validation failure or stale ID cannot create it.

Play requests a visible scored GUI when handing off a paused preview, independently
of the ordinary-Start GUI checkbox. **Physical actuation still requires its own
explicit checkbox**, defaults OFF and resets after Start. Unit tests cover both
opt-in values. Ordinary/headless starts retain their existing behavior. No target
motion or robot commands are fabricated by the dashboard.

Play executes the saved study's complete seed/variant matrix; the preview's
selected variant/first saved seed does not silently replace that definition.
Duration and seeds still belong to immutable Author/Catalog revisions. Existing
Runner `run_window` remains the sole duration owner: it waits for readiness,
sets an origin, settles, evaluates for the saved simulation-time duration, then
cleans up and records results. Stop/Cancel retains its existing cooperative path.
There is no UI wall-time stop timer or duplicate results store.

Preview states are Opening/Paused/Closing/Closed/Error; scored Running/Completed/
Failed are genuine Job outcomes, separately from scientific and safety acceptance.

## Single bounded real workflow

Installed dashboard: `http://127.0.0.1:8053`, workspace
`results/dashboard_workflow/workspace`. Selected **EXP-NOMINAL r3 / imm / seed 42**:
saved duration **6.0 simulation seconds**, settling **3.0 seconds after readiness**.
The saved Catalog was not edited. One preview plus its one scored handoff were
launched; no second experiment or parameter sweep was used.

| Observation | Actual result |
| --- | --- |
| Preview owner | `88fbbe1b-13d9-4323-b046-d90ec760a97d` |
| Frozen preview startup | `start_running=false`, actuation/publication OFF |
| Paused verification | All observed ROS clocks stayed at 0.0 during 4 wall seconds; native world stats `paused=true` |
| Preview command observations | Zero command messages, zero command publishers |
| Play | Actual UI actuation checkbox selected; existing backend/Runner handoff |
| Preview cleanup | Return code 0, no survivors, SIGINT; 1.031546 s |
| Handoff ordering | Preview closed before new scored Job/Run; distinct ROS domain/Gazebo partition |
| Scored Job | `f64cf16e-6622-4936-9301-98dd721ba4de` |
| Controller messages | 436 samples; **385 active nonzero commands** |
| Base physical motion | Maximum XY displacement from first ground-truth sample **0.897851 m** |
| Base measured speed | Maximum ground-truth linear speed **0.531676 m/s**; observation, not safety acceptance |
| Physical arm position ranges | base yaw **0.208188**, shoulder **0.713338**, elbow **0.471627**, wrist **0.748608 rad** |
| Wheels | All four measured wheel positions changed |
| Final outcome | **Failed**, `readiness_timeout` after **45.000864 host seconds**, clock **22.108**, origin/window unset |
| Scored cleanup | Return code 0, no survivors, SIGINT; **1.341939 s** |
| Saved Run | `29491f40-cc22-42ba-ad75-b71b7abaf337` |
| Session reference | `2013c421-8903-46df-9db7-02479d7b16d3`, “D2.6b paused-to-Runner check” |
| Desktop | Actual 1920×1080 document fits 1920×1080 viewport |

The four arm ranges use actual joint names from recorded commands and join
measured joint positions by name. They and base displacement describe the full
recorded **pre-readiness attempt**, not accepted scored-window metrics. No physical
limit, controller, estimator or predictor was adjusted to produce motion.

### Why the duration did not complete in the live attempt

All eleven required streams were recorded. The saved `max_skew=0.08 s` freshness
gate was never observed satisfied at a Runner decision after an observer batch,
so the duration origin was never set. Increasing physical simulation time does
not replace readiness or settling. The automatic failure/cleanup path worked;
**automatic termination after a successful six-second evaluation is unit-tested
but remains unverified live here**. A valid scored result could not be produced.

Offline callback-arrival replay of the unchanged predicate found only **one**
intermediate all-stream-fresh state (maximum absolute age 0.062 s at clock 13.444).
This does not instrument or reproduce the Runner's end-of-batch decisions; it
explains why “streams exist” is insufficient evidence of readiness and prevents
claiming that no intermediate fresh state ever existed. The actual recorded
failure remains `origin=None`. Further work must investigate observer batching
and stream freshness under GUI load before claiming successful GUI-scored runs.
No timeout, skew threshold, safety gate or acceptance criterion was relaxed.

The actual paused capture has a white scene; the early scored GUI capture is
black. Copied Ogre2 logs identify software `llvmpipe` and material/shader
`RenderingAPIException` / GLSL validation errors. Correct visual robot/world
rendering and visual confirmation of tracking remain unverified. These graphics
errors coexist with the freshness failure; causality was not isolated by this
single check. Existing native ROS commands, measured joints and physical ground
truth establish robot motion independently of rendering.

Safety acceptance and scored distance/pointing/prediction metrics are unavailable
because no valid evaluation window exists. This report does not convert execution
or observed movement into a physical-safety PASS.

### Evidence and screenshots

Run artifacts are under
`results/dashboard_workflow/workspace/runs/nominal_20261031T173639_898426Z_10mo4sad/runs/imm_seed42`.
The Session references that immutable Run directory; it does not copy results.
`live_workflow.json` preserves the failed integration assertion. Separate
`post_run_audit.json` records actual motion, readiness replay and Session evidence.
`native_audit.json` confirms no live process in either owned partition, no owned
windows, two requests total, unchanged Catalog and installed/source module equality.

- [Paused dashboard](../../results/dashboard_workflow/dashboard_paused.png)
- [Actual paused Gazebo window](../../results/dashboard_workflow/gazebo_paused.png)
- [Running dashboard](../../results/dashboard_workflow/dashboard_running.png)
- [Early scored GUI capture](../../results/dashboard_workflow/gazebo_running.png)
- [Actual Failed state / Closed preview](../../results/dashboard_workflow/dashboard_failed.png)
- [Saved failed Run in Results & History](../../results/dashboard_workflow/dashboard_failed_results.png)

The final Failed screenshots resume actual saved owner/Session references through
existing backend polling, without another simulator launch. They are not mocked
status or fabricated results. Browser scripts, raw stats, logs and renderer log
remain under the same generated-evidence directory.

## Tests, build and review

- Initial new contracts: seven expected RED failures, then **19 passed** in preview
  suite; the added time-advancement guard separately passed. UI Play regression
  failed before implementation, then the UI execution/lifecycle selection passed
  **12 tests**. See `red.log`, `green.log`, `ui_red.log`, `ui_green.log` and
  `advance_guard.log`.
- Broad core/configuration/Runner/architecture selection: **120 passed, 1 failed**.
  The failure asserted a source path while the factory imported an installed owner
  path. It reproduced using the **pre-task configuration module loaded in memory**
  (`baseline_path_check.log`), without reverting any workspace files. The same case
  passed with all source-package imports aligned (`source_path_check.log`). No
  production code or acceptance condition was changed to hide this environment issue.
- Broad dashboard/backend selection: **56 passed, 1 failed** because the new hint
  dropped the truthful “Unscored preview” label. The label was restored, retaining
  Paused/Play wording, and that exact check passed (`capability_green.log`).
- A larger final focused recheck was terminated with exit 143 before summaries
  (`final_core.log`, `final_ui.log`); it is not counted as a passing suite.
  The replacement minimal contract selection passed **9 tests** in 28.36 s
  (`final_contracts.log`), including paused materialization, exact handoff/failure
  gates, paused-owner tracking, advancement rejection and the unchanged
  readiness/settling/duration window. Final UI contracts: **5 passed** in
  79.56 s (`final_ui_contracts.log`), covering Play opt-in/handoff, callbacks,
  capability display and immutable duration/revision authoring.
- Dashboard scoped flake8: clean (`lint_verified.log`). No repository-wide lint
  cleanliness claim is made.
- Installed build: both changed packages succeeded (`build_final.log`); final UI
  reinstall succeeded (`build_ui.log`). No simulation binary or physics rebuild
  was necessary. Native checks byte-compared installed changed modules to source.
- Independent read-only ownership/provenance/compatibility review: no important
  findings. No files or processes were changed by that review.

Focused reproduction (same-named UI/core execution tests run separately):

```bash
cd /home/user/ros2_ws
source install/setup.bash
export PYTHONPATH=src/robot_experiments:src/robot_dashboard:$PYTHONPATH
build/dashboard_venv/bin/python -m pytest \
  src/robot_experiments/test/test_simulation.py \
  src/robot_experiments/test/test_execution.py \
  src/robot_experiments/test/test_runner.py -q
build/dashboard_venv/bin/python -m pytest \
  src/robot_dashboard/test/test_execution.py \
  src/robot_dashboard/test/test_simulation_ui.py -q
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_experiments robot_dashboard
```

## Launch and use

The verified installed dashboard is currently running on 8053, with both simulator
owners closed. After stopping that dashboard, relaunch using:

```bash
cd /home/user/ros2_ws
source install/setup.bash
ros2 run robot_dashboard dashboard --port 8053 \
  --data-dir /home/user/ros2_ws/results/dashboard_workflow/workspace
```

Select a saved revision/variant, Open Simulation, and confirm Paused. Create/select
a Current Session. Choose **Enable physical actuation** only when deliberately
requesting robot motion. Press **Play / Run**. The preview closes before the fresh
scored GUI opens. Wait for Completed or Failed; inspect actual readiness/cleanup.
Stop/Cancel remains available for an active Job. Ordinary headless CLI/Start does
not require a preview. Do not rerun the bounded browser fixture against the
preserved evidence workspace; it intentionally refuses pre-existing requests.

## Exact task-relative files

Modified:

- `src/robot_experiments/robot_experiments/configuration.py`
- `src/robot_experiments/robot_experiments/simulation.py`
- `src/robot_experiments/robot_experiments/jobs.py`
- `src/robot_experiments/robot_experiments/backend.py`
- `src/robot_experiments/test/test_simulation.py`
- `src/robot_dashboard/robot_dashboard/execution.py`
- `src/robot_dashboard/robot_dashboard/simulation.py`
- `src/robot_dashboard/test/test_execution.py`
- `docs/dashboard.md`

Added: `docs/validation/dashboard_workflow.md` (this report). Generated validation
scripts, screenshots and artifacts are isolated in `results/dashboard_workflow/`.
No source files removed, new production modules, owner YAML changes, Runner
replacement, numerical/physics changes, timeout changes or safety-limit changes.
No commit/push; no D3.
