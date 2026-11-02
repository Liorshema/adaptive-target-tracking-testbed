# Dashboard D2.5 — dark scientific workspace verification

Date: 2026-10-29. Scope: UI/UX redesign of the existing installed Plotly Dash
application. D1/D2 services and `ExperimentBackend` remain the owners of authoring,
execution, results and Sessions. No backend, numerical algorithm, physics, ROS
lifecycle, timeout, safety gate or production configuration was modified.

## Delivered layout

Two mounted primary tabs replace the long page:

- **Experiment Lab**: select/create, owner-schema component cards, typed parameter
  overrides with defaults/units/bounds/effective values, Duration and Seeds,
  Preview/Validate/Reset/immutable Save, Simulation Setup and Run/Stop. Full JSON
  is under collapsed Advanced Configuration; resolved deployment is collapsed
  separately. Open Simulation stays disabled with the existing capability reason.
- **Results & History**: compact robot/experiment/Run/execution header, five equal
  KPI cards, large side-by-side tracking distance and pointing plots, lower raw
  target XY telemetry, recorded computation-time plot and current-Session table.
  Original Run buttons select results; the selected row has a subtle blue edge.
  Execution, overall acceptance and recorded safety occupy a secondary status
  strip. A recorded error code is visible for failed Runs. Full contracts,
  metric tables, coverage, cleanup, raw control records and evidence paths remain
  available in a bounded details panel.

The header shares persistent Session selection plus a compact create/refresh
menu. Tabs stay mounted, retaining draft controls, execution references and Run
selection during tab changes. Existing browser refresh behavior remains: drafts
are memory-only, execution refs use session storage, current Session selection
uses local storage. There is no new UI database, Catalog or process owner.

The supplied [reference image](../reference%20image%20Dashboard.png) was inspected
and used for panel hierarchy, proportions and colors. The implementation uses
`#07141F` / `#0B1E2D`, muted blue borders, near-white labels, blue/violet primary
plots, green prediction and yellow performance. Reference statistics, curves,
rankings, improvement claims and allocation traces were not copied as evidence.
The Session table is current-Session history; D3 global history is not implemented.

## Authoritative duration and Seeds

Controls initialize from the resolved Author study. No new default duration is
invented. A user may enter 60 simulation seconds and a JSON seed array. These
values enter detached `experiment.execution.duration` / `seeds`, then pass the
existing validation and immutable revision save. The Catalog's canonical compiled
study stores them as `experiment.duration` / `seeds`.

Editing either field invalidates previous validation and disables Save. Full
Author validation handles domain constraints; malformed/nonfinite values are
rejected without silently coercing Seeds. Reset restores the pinned base, including
its execution values. Settling time remains inherited and is additional to the
scored duration. Duration/Seeds are never unsupported Start overrides.

Installed-browser verification created only the following new definitions in the
isolated visual-check Catalog:

| Saved experiment | Revision | Duration [simulation s] | Seeds |
|---|---:|---:|---|
| `EXP-UI-REDESIGN` | 1 | 60 | 42, 43 |
| `EXP-UI-REDESIGN` | 2 | 30 | 42, 43 |

Revision 1 remained unchanged after saving revision 2. Reset of the nominal-derived
draft restored 6 seconds / seed 42. Original shipped experiments/revisions were
preserved. Neither saved definition was executed.

## Scientific display contract

The KPI cards select these existing recorded statistics:

| Card | Metric | Statistic |
|---|---|---|
| Mean tracking error | `tracking_distance` | `mean` |
| Mean pointing error | `pointing_error` | `mean` |
| Prediction RMSE | `prediction_rmse` | `rmse` |
| Target estimation error | `target_position_error` | `mean` |
| Real-time factor | `realtime_factor` | `mean` |

Units stay recorded, including radians. Missing, invalid or nonfinite statistics
remain Unavailable; there is no fallback to a different statistic or zero.
Independent review caught an initial mean-error/RMSE labeling mistake. A real
artifact test reproduced it, then verified the corrected recorded `rmse` selection.
For the selected D2 compute-only Run, the recorded prediction mean error is
0.426361556 m whereas recorded RMSE is **0.525039643 m**; the final card shows the
latter. No scientific calculation was introduced to obtain it.

Primary and computation charts use exact stored scored `(time,value)` samples,
recorded time axes and units, as markers. No new statistics, trendlines, alignment,
interpolation or smoothing are performed. Scored pages remain limited to 1,000
samples per series using the existing backend reader and its 32 MiB JSON bound.
The complete recorded evaluation window is accessible under Evidence & safety.

Raw target XY trajectories use existing `read_telemetry`, up to 200 normalized
messages per stream (`target_truth`, `target_estimate`, `prediction`). The normalized
target-state contract has leading XYZ coordinates; the display projects the
original X/Y values. Prediction uses a single horizon from the last message on
that page, capped at 1,000 points. Only matching recorded frames and metre units
overlay; invalid/missing/incompatible observations stay unavailable. Axes use equal
metre scales. Source/forecast epochs remain in hover data.

These are **independent raw pages**, potentially with differing epochs and outside
the scored window. They are not synchronized trajectory-error comparisons. This
limitation is visible below the plot. No coordinate transforms or metric generation
are performed. Base/arm allocation is not scored in these artifacts, so the
neighboring panel explicitly uses genuine computation time instead.

Readiness failure retains failed execution and unknown overall/safety acceptance.
Its KPI cards and scored charts are unavailable. Partial raw telemetry recorded
before the failure may still appear, explicitly as unscored raw observations.
Session values retain the backend's comparability limitations; there is no ranking
or automatic winner. Session JSON still references original Runs without copying
telemetry or metric storage.

## Verification and screenshots

Evidence is under `results/dashboard_redesign/`.

| Check | Result | Evidence |
|---|---|---|
| Focused dashboard + backend regression | **53 passed**, 298.72 s | `final_regression.log` |
| Final genuine-result/statistic/row-selection regression after RMSE correction and failure-code presentation | **7 passed**, 4.36 s | `results_verified.log` |
| Dashboard scoped flake8 | Exit 0, no findings | `lint_verified.log` |
| Final installed ROS package build | 1 package, **9.56 s** | `build_verified.log` |
| Installed Chromium workflows | No final browser errors, zero job requests | `browser_verification.json` |
| Installed files and HTTP | Source/installed module and CSS byte comparison; index/CSS HTTP checks | `installed_verification.json` |
| Historical evidence and configuration boundary | File hash/stat/Catalog/Session audit | `preservation_audit.json` |

The focused regression includes the previous D1 Builder and D2 explicit Start,
opt-in, duplicate guard, status, native cancel delegation/timeout, Session and
scientific presentation tests, plus duration/revision and row-selection checks.
Existing controlled action transport is used for Run/Cancel; no ROS/Gazebo run
was necessary or launched. Real retained artifacts supply scientific test values.
The seven result tests were repeated after the final presentation correction;
they are a subset, not seven additional distinct tests.

Initial new-feature tests failed for absent controls/helpers before implementation.
The old page-title assertion was updated for the deliberate tab rename. A test
initially assumed the detached Author input shape was the saved Catalog shape;
it was corrected to inspect canonical compiled duration/Seeds. Browser harness
selectors were corrected for actual radio labels and nested summary elements;
completed immutable Saves were preserved rather than repeated. All intermediate
logs remain under the new evidence directory.

Desktop measurements, from the installed browser at device scale 1:

| State | Viewport | Document scroll size | Result |
|---|---|---|---|
| Experiment Lab | 1920×1080 | 1920×1080 | No page overflow |
| Lab + Advanced open | 1920×1080 | 1920×1080 | No page overflow |
| Results & History | 1920×1080 | 1920×1080 | No page overflow |
| Results + evidence open | 1920×1080 | 1920×1080 | No page overflow |
| Failed Run selected | 1920×1080 | 1920×1080 | No page overflow |
| Return to Lab | 1920×1080 | 1920×1080 | Draft retained; no overflow |
| Responsive Results | 1024×768 | 1024×1673 | Vertical scrolling, no horizontal overflow |
| Responsive Lab | 390×844 | 390×2635 | Vertical scrolling, no horizontal overflow |

Checks also assert essential control/panel rectangles lie within the desktop
viewport and Plotly's computed canvas sizes match their actual containers. This
caught and resolved initial lower-chart intrinsic sizing and a Lab footer issue;
page-level clipping is not used to hide scrollbars. Secondary parameter/evidence
panels intentionally scroll internally. The actual final captures were visually
inspected against the supplied reference for spacing, colors, axes and history
row treatment. The shared header Session menu was also exercised in Chromium:
create a new empty Session, reselect the original, and verify its three unchanged
Run references. See `browser_session.json`.

- [Experiment Lab — 1920×1080](../../results/dashboard_redesign/experiment_lab_final.png)
- [Results & History — 1920×1080](../../results/dashboard_redesign/results_history_final.png)
- [Readiness failure](../../results/dashboard_redesign/readiness_failure.png)
- [Responsive Results — 1024px](../../results/dashboard_redesign/responsive_results.png)
- [Responsive Lab — 390px](../../results/dashboard_redesign/responsive_lab.png)

## Launch and reproduce

The installed verification dashboard is served at **http://127.0.0.1:8052**:

```bash
cd /home/user/ros2_ws
source install/setup.bash
ros2 run robot_dashboard dashboard --port 8052 \
  --data-dir results/dashboard_redesign/workspace \
  --result-root results/dashboard_execution/workspace/runs \
  --result-root results/robot_description_naming/runtime \
  --result-root results/dashboard_backend/runtime_complete
```

Stop an existing server on the same port before relaunching. The new workspace
has **Dashboard visual verification**, Session
`30d01daf-7f97-4cc6-8547-441bf41ffb20`, referencing the same three genuine
completed/failed Runs used for D2 checks, plus the two isolated authoring revisions
above. Select that Session in the header if an empty UI-check Session is selected. It contains no jobs.
Existing dashboard processes on other ports were not terminated. Already-running
older application processes need a restart to load the new Python layout.

Normal use remains `ros2 run robot_dashboard dashboard --data-dir YOUR_WORKSPACE`.
No CLI options, package dependency or backend API were added in D2.5.

Focused reproduction, without launching a simulator:

```bash
source install/setup.bash
source results/dashboard_execution/evidence.env
PYTHONPATH=src/robot_dashboard:src/robot_experiments:$PYTHONPATH \
  build/dashboard_venv/bin/python -m pytest src/robot_dashboard/test \
  src/robot_experiments/test/test_backend.py -q
build/dashboard_venv/bin/python -m flake8 \
  --config src/robot_dashboard/setup.cfg src/robot_dashboard
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_dashboard --executor sequential
```

Browser evidence uses Chromium through Playwright in the dashboard environment.
`browser_verification.py` is the final read-only/check-draft harness; it inspects
the already-saved authoring evidence rather than creating duplicate revisions or
starting a job. The earlier authoring harness log retains the actual Save path.

## Exact task-relative changes

Modified:

1. `src/robot_dashboard/robot_dashboard/app.py` — mounted tab composition,
   schema-backed Author execution fields, compact deployment and resize handling.
2. `src/robot_dashboard/robot_dashboard/forms.py` — detached duration/Seeds edits.
3. `src/robot_dashboard/robot_dashboard/execution.py` — named mounted sections,
   shared Session controls, compact monitor, Run row selection/header/highlight.
4. `src/robot_dashboard/robot_dashboard/results.py` — dark figures, five recorded
   KPI statistics, bounded raw XY telemetry, compact status/evidence/history.
5. `src/robot_dashboard/robot_dashboard/assets/dashboard.css` — responsive dark grid.
6. `src/robot_dashboard/test/test_builder.py` — tab/control and duration/revision
   approval/persistence coverage, deliberate label/Author-shape assertion updates.
7. `src/robot_dashboard/test/test_results.py` — recorded KPI/trajectory/frame and
   original Run row selection checks.
8. `docs/dashboard.md` — updated operator guide and scientific display limits.

Added permanent documentation: `docs/validation/dashboard_redesign.md` (this file).
Removed files: **none**. No new production module, wrapper, YAML profile or fixture
was introduced. Generated captures, tests/logs and the isolated verification
workspace remain under `results/dashboard_redesign/` and normal build outputs.
The user supplied `docs/reference image Dashboard.png` and its Windows metadata
sidecar during this task; those are incoming reference files, not generated code.

The audit compares this list against the D2.5 start snapshot, not the much larger
pre-existing Git diff. All **15,675** historical artifact size/mtime records remain unchanged, and all
**21 original revisions in seven shipped experiments** are preserved. No source
changes outside the allowlisted files were found. No commit or push was performed.

## Verdict and limits

**D2.5 UI scope: PASS.** The two-tab layout, Author-owned duration, real scientific
presentation and preserved D1/D2 workflows passed the focused and installed-browser
checks. Runtime readiness, actuated physical safety and localization acceptance
retain their earlier conditional limits; this visual task does not change those
verdicts. No live cancellation or new simulated performance result is claimed.

Trajectory pages are bounded and independent, not a synchronized replay player.
Open Simulation remains disabled. Historical cross-Session analytics, advanced
filters, allocation metrics not already recorded, and research conclusions remain
outside this task. D3 was not started.
