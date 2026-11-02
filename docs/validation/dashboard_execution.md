# Dashboard execution and current Session verification

Date: 2026-10-29. Scope: D2, extending the installed D1 Plotly Dash client.

D2 execution, recorded-result presentation and persistent Session workflows are
implemented. Runtime acceptance remains **CONDITIONAL**: the single new attempt
completed with overall acceptance unknown. Fresh active live cancellation was
not demonstrated; cancellation semantics and callback behavior passed controlled
regression tests. No simulation, timing, safety or numerical settings were changed.

## Delivered behavior and ownership

- `robot_dashboard` retains the D1 builder and adds explicit saved-revision Start,
  Run & Monitor, Results and Current Session areas. Unsaved drafts cannot launch;
  Start invokes existing backend preflight. Job UUIDs and discovered Run UUIDs are
  separate. Output directory and explicit actuation opt-in are the only exposed
  runtime overrides. Actuation defaults OFF and resets after each successful Start.
- Existing `ExperimentBackend`/Runner own execution, cancellation, cleanup,
  scientific results, Catalog and Session persistence. The dashboard adds no
  process manager, scientific calculations, database or simulator controls.
- Active status polls every two seconds. Terminal, lost-worker and observation
  error states suspend polling; explicit Refresh retries observation. Rejected
  action inputs do not suspend otherwise healthy active polling. Unchanged status
  does not trigger unnecessary Session/result redraws. No readiness percentage is
  inferred. Native stop-request, timeout and cleanup uncertainty stay visible.
- Start freezes revision, launch settings and Session. Repeated Start events are
  guarded in the single local Dash process; refreshing the browser never launches.
  Browser session storage retains execution references. This does not provide
  crash-transaction exactly-once semantics between backend Start and HTTP reply.
- Discovered real Runs attach to the frozen launch Session idempotently, even
  if the user changes the currently selected Session. Stop/Cancel delegates to
  `cancel_job` or `cancel_run`; Run cancellation affects its whole owning Job,
  including remaining seeds/variants. There is no custom process termination.
- Results distinguish execution, overall acceptance, recorded safety and tracking.
  Tables display recorded means/maxima, units and missingness. Three compact
  Plotly marker charts show existing tracking-distance, pointing-error and control
  computation samples without new statistics, interpolation or smoothing.
  Recorded evaluation windows, time bases, coverage and evidence paths remain
  inspectable. Missing/readiness-failed evidence never creates zero-valued plots.
- Current Session comparison uses existing backend scientific guards and reasons,
  displays failed/incomplete Runs honestly, and does not select winners. Persistent
  Session data contains original Run references, not copied metrics or telemetry.
  Current selection persists in browser local storage.

The sole backend API addition is read-only
`read_time_series(run_id, *, offset=0, limit=100)`. Existing results exposed scored
curves only as artifact paths, whereas UI file access would bypass the backend.
The new reader preserves series metadata and exact sample slices, accepts at most
1,000 samples per series per call, and uses the existing 32 MiB JSON-file limit.
It reads a bounded complete JSON document before slicing; it is not streaming.
Raw control telemetry inspection uses existing `read_telemetry` with 20 records.
External result roots can be registered with repeatable `--result-root`; new job
outputs are recovered from existing job requests. Plotly is an explicit optional
UI-package dependency. Headless experiment users do not need Dash.

## Focused verification

All evidence paths below are relative to `results/dashboard_execution/`.

| Check | Actual result | Evidence |
|---|---|---|
| D1 builder + D2 execution/results + existing backend | 47 passed, 250.16 s | `final_regression.log` |
| Final polling event optimization, D2 execution/results subset | 12 passed, 34.16 s | `final_extension_tests.log` |
| Existing architecture / package-tree contracts | 53 passed, 66.90 s | `architecture_regression.log` |
| Dashboard scoped flake8 | Exit 0, no findings | `lint_verified.log` |
| Normal installed two-package build | 2 packages, 12.0 s | `build_final.log` |
| Separate copied installation prefix | 2 packages, 4.52 s | `build_isolated.log` |
| Copied-prefix Flask index/CSS and module identity | HTTP 200/200; installed Python bytes match source | `copied_install.json` |
| Installed console command | `--result-root` and normal options available | `installed_help.log` |
| Installed Chromium workflow | Saved revision, OFF launch, same Job after refresh, genuine results and Session UI actions | `browser_results.json`, `browser_recovery.log` |
| Real server restart + browser local storage | Original selected Session survives despite a newer Session; 3 unique Runs; 1 total job request; no browser errors | `restart_browser.json` |
| Catalog/source/evidence preservation | Exact task file allowlist, original revisions and historical artifact stat checks | `preservation_audit.json` |

The copied-prefix build deliberately uses the existing nine-package workspace
underlay for other dependencies. Colcon reports those locations. This is a copied
installation test of the two affected packages, not an independent full-stack
rebuild. Browser checks use installed modules and a separate localhost port.
The user's existing D1 server on port 8050 was left untouched.

Controlled action/callback tests cover actual API routing, explicit actuation,
duplicate clicks, Job/Run identity, fixed launch Session, idempotent association,
terminal/lost-worker states, cancellation scope and native cancellation timeout.
Result tests read genuine completed and readiness-timeout artifacts; optional
evidence environment variables make unavailable historical data an explicit skip.
Regression tests preserved exact recorded chart coordinates, units and metrics.

Initial integration exposed a duplicate DOM ID shared by the Session section and
dropdown; it was corrected. Independent review found that a rejected cancellation
action incorrectly paused observation; the fix passed a failing-then-passing test.
A separate failing-then-passing test covers unchanged polling events. Historical
failure logs remain in the evidence directory; final green logs are listed above.

## One real lifecycle, without retries

The browser launched saved `EXP-NOMINAL` revision 3 exactly once through the
installed dashboard. No actuation opt-in was selected. Its immutable definition
SHA-256 was
`40d1845815048bbcf37a1b882a657291ed8237b96d8d6a61a3ac2912e29301b2`.
The request records `enable_actuation: false`.

- Job: `da1dc742-ecb3-428f-ae48-36e4b448620d`.
- Run: `24202c0a-fc3a-40c4-8c82-02a6fdd06577`, variant `imm`, seed 42.
- Job state: `completed`; worker exit code: 3. Run execution: `completed`;
  overall acceptance: `null` / Unknown; recorded error: null. These distinct
  existing backend outcomes are displayed as recorded, without reinterpretation.
- Recorded evaluation window: simulation time 24.478–30.478 s; origin 21.478 s;
  stop clock 30.478 s; monotonic evaluation lifecycle elapsed 45.01625 s.
- Cleanup: return code 0, no survivors, native SIGINT phase, elapsed 0.63144 s.
- Refresh retained the same Job. Restart, Session creation and read-only browser
  recovery did not create another request. There remains exactly one request.

Recorded metrics from `get_run_results` (no dashboard recomputation):

| Metric | Mean | Maximum | Unit | Samples |
|---|---:|---:|---|---:|
| Tracking distance error | 0.689967861 | 0.735190545 | m | 473 |
| Pointing error | 0.123312060 | 0.226602894 | rad | 473 |
| Control computation time | 0.0186999073 | 0.0420659370 | s | 120 |
| Real-time factor | 0.844218301 | 0.941251694 | 1 | 14 |
| Physical joint-position violation | 0 | 0 | rad | 1,548 |
| Physical joint-speed violation | 0 | 0 | rad/s | 1,548 |

Recorded safety accepted the covered six-second window, with position/speed
coverage fractions 1.0. This compute-only lifecycle does not establish commanded
actuation safety, general tracking acceptance, or a controlled performance
comparison. Diagnostics timestamp/freshness gaps and unavailable NIS/NEES remain
visible in the stored metric contract. No timeout or acceptance threshold was
adjusted to improve this outcome. See `fresh_results_summary.json` for full values
and availability; `live_job_observed.json` and native job/run files retain provenance.

The original live browser harness timed out selecting a dropdown option before
it clicked Cancel. The job finished naturally. A scoped dropdown locator fixed
the read-only inspection harness; a second Gazebo attempt was deliberately not
made. Thus active cancellation has controlled action/callback coverage, but no
new D2 live interruption/cleanup demonstration. The UI does not claim otherwise.

## Genuine results and persistent Session

Session `f11beca8-dbdf-436a-8342-45d19723d116`, titled
**D2 recorded evidence and controlled lifecycle**, contains exactly three unique
references:

1. `823fa1b3-36cf-4b36-9cc0-6288c02b4213`: historical completed accepted
   `EXP-DESCRIPTION-CANONICAL` r1. Recorded distance mean/max
   0.123900260/0.231098522 m; pointing mean/max 0.151520389/0.347596502 rad.
2. `7483ae59-2ca2-4655-8e18-0b83f6dccb02`: historical
   `EXP-BACKEND-INTEGRATION` r1 readiness failure after about 45.004 s;
   execution failed, acceptance unknown, no scored curves.
3. The new `EXP-NOMINAL` r3 compute-only Run above, automatically attached once.

These Runs have differing protected conditions and incomplete evidence for the
failed Run. The backend reports them incomparable. The dashboard shows descriptive
recorded values and those reasons, without ranking or attributing differences to
a controller improvement. A second Session was created through the UI, the
original was reselected, then the server restarted. A fresh Chromium context with
saved browser storage still selected the original and displayed all three refs.

## Screenshots

Paths link to real installed-browser captures; no synthetic chart data or UI mockups.

- [New job status immediately after Start](../../results/dashboard_execution/live_status.png)
  (actual queued state, not a claim of a separately reported readiness phase).
- [Completed Job status](../../results/dashboard_execution/completed_status.png).
- [New Run results, metrics and charts](../../results/dashboard_execution/fresh_results_complete.png).
- [Three recorded Plotly charts](../../results/dashboard_execution/recorded_charts.png).
- [Current Session comparison](../../results/dashboard_execution/current_session_final.png).
- [Session selection after restart](../../results/dashboard_execution/session_after_restart.png).
- [Historical readiness failure](../../results/dashboard_execution/readiness_failure.png).

## Reproduction

Launch the verified workspace:

```bash
cd /home/user/ros2_ws
source install/setup.bash
ros2 run robot_dashboard dashboard --port 8051 \
  --data-dir /home/user/ros2_ws/results/dashboard_execution/workspace \
  --result-root /home/user/ros2_ws/results/robot_description_naming/runtime \
  --result-root /home/user/ros2_ws/results/dashboard_backend/runtime_complete
```

Open **http://127.0.0.1:8051**. Stop an existing server on that port before
relaunching. Select the original Session UUID above in a new browser profile.
Start is always explicit. Reopening this workspace only inspects existing data.
Keep the external root arguments across restarts to discover historical refs.
See [Dashboard guide](../dashboard.md) for installation and builder workflows.

Focused deterministic test commands, without launching another simulation:

```bash
source install/setup.bash
source results/dashboard_execution/evidence.env
PYTHONPATH=src/robot_dashboard:src/robot_experiments:$PYTHONPATH \
  build/dashboard_venv/bin/python -m pytest src/robot_dashboard/test \
  src/robot_experiments/test/test_backend.py -q
build/dashboard_venv/bin/python -m pytest \
  src/robot_bringup/test/test_architecture.py \
  src/robot_bringup/test/test_architecture_tree.py -q
build/dashboard_venv/bin/python -m flake8 \
  --config src/robot_dashboard/setup.cfg src/robot_dashboard
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_experiments robot_dashboard --executor sequential
```

`browser_restart.py` and `browser_charts.py` in the evidence directory are read-only
installed-browser checks using Playwright. `browser_workflow.py` is the historical
single-launch harness; rerunning it creates a new real job and is unnecessary for
inspection. Generated browser scripts, logs and numerical artifacts remain under
`results/`, not production/test-fixture source.

## Exact task-relative file list

Existing files modified:

- `docs/dashboard.md` — D1+D2 operating guide and launch/recovery instructions.
- `src/robot_dashboard/package.xml` — direct Plotly runtime dependency.
- `src/robot_dashboard/setup.py` — Plotly installation dependency.
- `src/robot_dashboard/robot_dashboard/app.py` — saved identity and D2 integration.
- `src/robot_dashboard/robot_dashboard/cli.py` — repeatable registered result roots.
- `src/robot_dashboard/robot_dashboard/assets/dashboard.css` — status, tables/charts layout.
- `src/robot_dashboard/test/test_builder.py` — preserve D1 and disabled simulation contract.
- `src/robot_experiments/robot_experiments/backend.py` — bounded read-only scored-series API.

New permanent files:

- `src/robot_dashboard/robot_dashboard/execution.py` — thin lifecycle/Session UI.
- `src/robot_dashboard/robot_dashboard/results.py` — recorded evidence presentation.
- `src/robot_dashboard/test/test_execution.py` — lifecycle/Session callback regressions.
- `src/robot_dashboard/test/test_results.py` — genuine-artifact scientific presentation checks.
- `docs/validation/dashboard_execution.md` — this report.

Removed files: **none**. Generated artifacts are confined to
`results/dashboard_execution/` and normal build/install/log outputs. The
pre-existing dirty working tree is preserved; this list is relative to the D2
start snapshot, not a claim that the global Git diff contains only D2 changes.
No owner YAML, estimator, controller, predictor, physics, safety or historical
experiment configuration was edited. No commit or push was performed.

## Remaining limits and D3 boundary

The server is a local engineering client with an in-process click guard, not a
multi-worker deployment or crash-atomic launch protocol. Browser job references
survive refresh; loss of browser storage is not a durable job-history UI.
Readiness failures and unknown overall acceptance remain real runtime limitations.
Fresh active cancellation and actuated physical acceptance were not re-proven in
this task. A successful short compute-only window does not supersede earlier
conditional safety/localization milestones. No changes were made to those systems.

Historical cross-session analytics, advanced filtering and research conclusions
remain D3. No D3 implementation was started.
