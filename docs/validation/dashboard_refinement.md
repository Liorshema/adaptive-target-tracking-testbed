# Dashboard D2.7 — final UI refinement

Date: 2026-10-31. **Visual acceptance: PASS** for the specified installed-Chromium
views at 100% zoom. This verdict concerns the dashboard presentation and preserved
UI workflows; it does not grant new tracking-quality or physical-safety acceptance.
No new scored Run, experiment or simulation was created for this task.

## Delivered presentation

The two mounted workspaces retain their browser-local draft, execution and Session
references. The Lab uses Catalog scenario names, compact owner-schema model cards,
duration/seeds and prominent revision/execution actions. Target parameters open
initially; a small native-details listener keeps secondary editors exclusive and
retains their input values. Other configurable roles remain accessible. Revision
history, provenance, defaults/bounds, deployment paths and full JSON remain in
Advanced details. Run clearly uses the selected saved configuration; actuation
and scored-GUI flags still default OFF.

Results has exactly four cards and four equal panels. Header, scenario and Session
labels use authoritative metadata; external historical Runs fall back to their
immutable recorded names when absent from the Catalog. UUIDs remain internal or
in diagnostics. Execution, Run acceptance and recorded safety remain separate.
The Session table retains original Run-selection IDs and highlights the selected
row. Control compute time is available only in Advanced / Diagnostics.

The old accumulated stylesheet (1,527 lines) was replaced by a single responsive
stylesheet, including readable typography and bounded secondary scrolling.
Redundant overrides were removed; no production files or historical evidence were
deleted. The only new production asset is the small editor-exclusivity listener,
installed through the existing package-data mechanism. There are no new UI owners,
profiles, backend APIs or dependencies.

Comparison with `docs/reference image Dashboard.png`: the navy palette, restrained
borders, accent colors, readable values, chart hierarchy and selected-row treatment
follow the reference. The explicit D2.7 four-card/equal-2×2 arrangement replaces
its five-card/asymmetric lower row. Plots use genuine markers rather than invented
smooth curves; only two real historical references populate the review table.
No allocation trace, trend, improvement percentage or extra scenario was invented
for visual fullness.

## Scientific presentation

For retained Run `1a0712b4-6226-47f3-a2cf-5a11a2cb3a2a`, the displayed values are:

| Card | Recorded field | Display |
|---|---|---|
| Mean Position Error | `tracking_distance.mean` | 0.086 m |
| Mean Orientation Error | `pointing_error.mean` | 0.201 rad |
| Estimator Accuracy | `target_position_error.rmse` | 0.093 m |
| Predictor Accuracy | `prediction_rmse.rmse` | 0.544 m |

Position means **absolute radial distance-to-reference error**, not general XYZ
position error. Orientation means **target pointing angle**, not full 3D attitude
error. Native mathematical rendering supplies overlines, absolute values, Greek
letters and subscripts without remote math assets. Plotly labels use mathematical
symbols and declared units. Cards and plotted samples retain recorded units;
only the comparison's angular RMSE converts declared radians to degrees.

Estimator KPI alone may fall back to a genuine recorded mean, explicitly labeled
as mean. Table cells never substitute mean for RMSE; unavailable/invalid/unit-
incompatible values remain unavailable. Prediction selects actual `rmse`, distinct
from its mean-error field. Rank uses recorded radial RMSE in metres, lower is
better, with equal values sharing a rank. It requires the backend's explicit
scientific comparability approval and valid radial RMSE for every Run.

The review Session `3e375f1e-48a1-4e1c-8b63-0f6e840a045c`, titled **Recorded tracking
review**, references that Run and historical Run
`823fa1b3-36cf-4b36-9cc0-6288c02b4213`. Their backend comparison is **incomparable**;
both ranks therefore remain unavailable. Positive ranking is verified with focused
presentation tests, not claimed for these two actual Runs. The historical scenario
label `nominal` comes from its immutable recorded configuration.

XY retains matching recorded frames, metre units, equal X/Y scales and original
markers. Its independent pages contain up to 200 raw messages per stream and one
actual final-record prediction horizon. They are not time-aligned or transformed
into scored trajectories. Source/forecast epochs and limitations remain available
through hover/diagnostics. Narrow-height desktop plots have less vertical space;
the equal-scale XY view preserves its geometry rather than stretching the data.

## Installed browser evidence

Real installed Chromium, fresh contexts, device scale 1, default 100% browser zoom;
no zoom-out, page scaling, fullscreen API or page-overflow clipping was used.
Body/controls are 14 px; panel titles 18 px; KPI values 30 px; table/ticks/legends
13 px and axis titles 14 px. Compact provenance/status metadata is 12–13 px.
All primary controls were checked within the viewport. Expanded parameter bodies
end above Save/Preview/Validate/Reset. Plotly canvas dimensions match their graph
containers. Captured primary text contains no UUIDs; browser reported zero JS errors.

| Viewport | Lab / parameters / Results document size | Each Results panel |
|---|---|---|
| 1920×1080 | 1920×1080 | 936×368 px |
| 1600×900 | 1600×900 | 776×278 px |
| 1650×790 | 1650×790 | 801×223 px |

Both primary tabs and the expanded editor have no page-level horizontal or vertical
scrollbars at all three sizes. Historical row selection and its real loading state
also fit at 1650×790. Secondary parameter editors and long histories scroll
internally. At 1024×768, Results reflows to a 1024×1013 document: vertical
scrolling is available and horizontal overflow remains absent.

An actual loading-state probe found an absolute overlay anchored to the viewport,
extending 18 px past the Results grid. Giving the Results workspace its own
positioning context changed measured document width from 1668 to 1650 px during
loading, without hiding overflow. This is a CSS containing-block correction.
Initial browser harness failures were also retained: one attempted selection before
Session options loaded, one captured before result loading completed, and Session
checks initially assumed one Run / checked rows before the asynchronous comparison
response. Assertions now use the actual saved references and wait for their IDs.
A test-only selector quoting error was corrected, with only the missing Session
and responsive checks resumed by `browser_finish.py`. These harness corrections
do not alter readiness, deadlines or backend behavior.

Screenshots:

| State | 1920×1080 | 1600×900 | 1650×790 |
|---|---|---|---|
| Lab loaded | [image](../../results/dashboard_refinement/lab_1920x1080.png) | [image](../../results/dashboard_refinement/lab_1600x900.png) | [image](../../results/dashboard_refinement/lab_1650x790.png) |
| Parameter editor | [image](../../results/dashboard_refinement/lab_parameters_1920x1080.png) | [image](../../results/dashboard_refinement/lab_parameters_1600x900.png) | [image](../../results/dashboard_refinement/lab_parameters_1650x790.png) |
| Real results + history | [image](../../results/dashboard_refinement/results_1920x1080.png) | [image](../../results/dashboard_refinement/results_1600x900.png) | [image](../../results/dashboard_refinement/results_1650x790.png) |

Also retained: [historical Run selection](../../results/dashboard_refinement/historical_row_selected.png),
[loading state](../../results/dashboard_refinement/loading_historical_run.png),
[measurement report](../../results/dashboard_refinement/browser_report.json), and
[source snapshot](../../results/dashboard_refinement/dashboard_before.tar.gz).

## Tests and preservation

- **50 dashboard tests passed** in 303.04 s: authoring, immutable duration/seeds,
  revision conflicts, mounted controls, Run/Cancel transport, explicit opt-in,
  duplicate prevention, preview behavior, recorded metric semantics, paging,
  frame mismatch, row selection and guarded rankings.
- **3 focused chart regressions passed** after the shorter axis labels: exact
  coordinates/units, distinct scalar statistics and grid/advanced placement.
- Dashboard Python/test **flake8 passed** at the package's 99-column convention.
- Installed `robot_dashboard` **colcon build passed**; new JS/CSS are packaged.
- AST/byte comparison to the preserved baseline confirms `ExecutionActions`,
  preview `operate`, draft `forms.py` and CLI are unchanged.
- No Runner, Recorder, readiness, backend, physics, controller, numerical model,
  safety gate, timeout, experiment configuration or results-storage code changed.
- Task-scoped whitespace check passed. Whole-tree `git diff --check` also reports
  a pre-existing trailing blank line in `robot_experiments/recording/ros.py`;
  that unrelated file was preserved.

Logs and reproducible browser scripts remain under `results/dashboard_refinement/`.
Functional browser interaction evidence is recorded separately in
`browser_workflows.json`: owner-schema edits, Preview/Validate, invalidation of Save
after edits, Reset, exclusive editors, actual Session-reference switching and
responsive scrolling passed. The production Catalog hash and Job inventory are
unchanged. Successful authoring assertions are linked to the retained prior log;
the final Session/responsive checks are in `browser_finish.log`.
Launch/cancellation regression uses controlled transport,
while the earlier D2.6c real-Gazebo completion evidence remains authoritative.
No new Gazebo run was required or performed for this UI refinement.

## Reproduction

The installed reviewed workspace is available at **http://127.0.0.1:8060**.
To restart it, close the existing server on that port before running:

```bash
cd /home/user/ros2_ws
source install/setup.bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir /home/user/ros2_ws/results/runner_readiness/workspace \
  --result-root /home/user/ros2_ws/results/robot_description_naming/runtime/nominal_20261028T130412_296933Z_u30m7yel
```

Select **Recorded tracking review**. The explicit historical result root is required
on each launch; Session JSON contains original references, not copied evidence.
Open/Close/Play/Stop continue through the existing paused-preview and scored-Runner
workflow. Actuation stays OFF until explicitly checked. Creating a scenario does
not automatically select it for Play: select the newly saved scenario first.

Focused checks (existing UI environment/build required):

```bash
source install/setup.bash
source results/dashboard_execution/evidence.env
PYTHONPATH=src/robot_dashboard:$PYTHONPATH build/dashboard_venv/bin/python -m pytest -q src/robot_dashboard/test
build/dashboard_venv/bin/python -m flake8 --max-line-length=99 src/robot_dashboard/robot_dashboard src/robot_dashboard/test
build/dashboard_venv/bin/python /usr/bin/colcon build --packages-select robot_dashboard
build/dashboard_venv/bin/python results/dashboard_refinement/browser_verify.py
build/dashboard_venv/bin/python results/dashboard_refinement/browser_workflows.py
```

## Exact task-relative changed files

1. `src/robot_dashboard/robot_dashboard/app.py`
2. `src/robot_dashboard/robot_dashboard/execution.py`
3. `src/robot_dashboard/robot_dashboard/results.py`
4. `src/robot_dashboard/robot_dashboard/simulation.py`
5. `src/robot_dashboard/robot_dashboard/assets/dashboard.css`
6. `src/robot_dashboard/robot_dashboard/assets/dashboard.js` (new)
7. `src/robot_dashboard/setup.py`
8. `src/robot_dashboard/test/test_builder.py`
9. `src/robot_dashboard/test/test_results.py`
10. `docs/dashboard.md`
11. `docs/validation/dashboard_refinement.md` (new)

Generated screenshots, logs, Session-reference evidence and verification scripts
are separate artifacts under `results/`; immutable Run storage is not duplicated.
No commit/push was made. D3 was not started.
