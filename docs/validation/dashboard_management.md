# D2.8 — Dashboard usability and safe management

Date: 2026-11-01. Verdict: **PASS for D2.8's bounded scope**, subject to the
explicitly permitted Scenario-rename deferral and limitations below. No new scored
Gazebo Run, scientific comparison change, commit, push or D3 work.

## Implemented behavior and ownership

The existing dashboard remains a client of `ExperimentBackend`, `ExperimentJobs`,
`ExperimentCatalog` and immutable Run evidence. No new persistence store, process
manager, Scenario entity or experiment engine was added.

- The existing trajectory panel defaults to **2D**, auto-fits genuine samples with
  8% padding and equal physical XY scaling, and has native zoom/reset controls.
  A stationary coordinate receives a finite display range, not fabricated samples.
  Its compact **3D** toggle uses `Scatter3d` and recorded XYZ, data aspect scaling,
  orbit/zoom and native camera reset. Short axis titles and a separate frame label
  keep frame/units readable in the bounded panel. Run/mode-specific UI revisions
  prevent reusing a previous Run's zoom or camera.
- Raw telemetry reads remain bounded to 200 records per stream, and the forecast
  uses only the final prediction record on that page. Original source/forecast
  epochs and horizon metadata remain attached. There is no resampling, alignment,
  smoothing, frame transformation, inferred Z or new scientific score. Nonfinite
  coordinates, missing Z/units/epochs, mixed frames and malformed latest forecasts
  produce omissions with an explanation or an unavailable state.
- The header Session **⋯** menu retains Create/Refresh and adds Rename/Delete.
  Named native confirmation is required before deletion. The backend removes only
  that Session JSON document; referenced Run directories are never deleted.
  Rename changes only the trimmed title. A token covering the entire document,
  existing per-Session locks and atomic JSON writes reject stale edits without
  losing concurrent Run references. Deletion is idempotent; a later append cannot
  recreate the removed document. Symlink documents, directories and locks fail
  closed. The sibling lock file remains to preserve a consistent lock identity.
- `start_run(..., session_id=...)` persists the selected launch Session in existing
  Job request metadata before worker startup. A backend metadata lock serializes
  accepted start/delete/archive operations, and the existing Jobs preview lock
  serializes archive against preview/Job creation. This is association metadata;
  Runner, readiness, duration, settling, physics and lifecycle behavior are unchanged.
  Active or unresolved associated work blocks deletion/archive.
- The Scenario **⋯** menu confirms and delegates archive to revision-guarded
  `ExperimentCatalog.delete()`. Archived entries leave the active dropdown;
  revisions, IDs, definition hashes and historical access survive. UI draft,
  deployment and stale selection state are cleared. There is no purge action.
- New drafts normally require only a display name. `propose_experiment_id()` uses
  NFKC normalization, NFKD ASCII transliteration, uppercase and punctuation-to-dash
  conversion. `Dynamics Comparison` becomes `EXP-DYNAMICS-COMPARISON`. A name with
  no ASCII slug uses the first 12 uppercase hex characters of its normalized UTF-8
  SHA-256; a suffix longer than 71 characters is truncated with a hash suffix.
  The Catalog identifier validator remains authoritative. Active, archived and
  permanently retired IDs are reserved. Collisions block loading, never silently
  suffix/select another experiment. Advanced permits explicit custom IDs only for
  new drafts; revising retains the saved identity. Save rechecks collisions through
  the existing atomic Catalog path.

## Verification and actual measurements

| Check | Result and evidence |
| --- | --- |
| Dashboard regression | **81 passed**, including Builder/revisions/duration, Run/Cancel/preview handoff, Results/Session, management callbacks and trajectory modes; `results/dashboard_management/dashboard_tests_final.log` |
| Final trajectory presentation | **45 passed** (trajectory and existing Results suites) after final frame-label correction; `trajectory_tests_final.log` |
| Backend management | **44 passed**, including concurrent rename/start/archive, stale tokens/revisions, legacy Jobs, missing/corrupt requests, symlinks, immutable evidence and IDs; final formatted source: `backend_tests_formatted.log` |
| Build | Both `robot_experiments` and `robot_dashboard` built successfully; `build_final.log`; final formatting rebuild in `build_verified.log` |
| Lint | Changed dashboard Python/tests and new backend management tests pass configured flake8. Baseline-relative backend/Catalog/Jobs findings are audited in `lint_audit.json`; no broad formatting refactor. |
| Browser | Actual installed Chromium at device scale 1 / 100% zoom; both tabs at **1920×1080, 1600×900, 1650×790**, plus both trajectory modes, no page overflow, body font 14 px; `browser_final_report.json` |
| Layout | Exactly four KPI cards and four equal result panels; panel sizes respectively **936×368**, **776×278**, **801×223 px** |
| Equal XY scale | **49.891804974141394 px/m X**, **49.891804974141344 px/m Y** at 1650×790; directly inspected Plotly's computed axes |
| Real XYZ | **200 Truth**, **200 Estimate**, **20 Prediction** samples; plotted Truth Z and source times exactly equal retained telemetry; real WebGL orbit and native reset verified |
| Metadata browser actions | Rename persisted; cancelling named Session delete kept it; confirming removed only isolated metadata and cleared selection; named Scenario archive retained identical revision/hash and disabled stale Open/Run. `browser_report.json`, `browser_metadata.log` |
| Preservation | **109/109 original files unchanged by SHA-256**: one Catalog, eight Sessions, 100 files across two original Runs; eight installed modules and stylesheet match final source; `original_hashes.json`, `preservation_audit.json` |
| Scientific boundary | Comparison renderer and nine backend scientific-access/cancel/preview methods unchanged by AST comparison against pre-task snapshot; incomparable rows still show **Rank = Unavailable** |
| Independent review | No critical/important correctness or preservation findings in final backend/management review. |

Tests use real retained successful/failed Run fixtures where applicable. The
initial regression found one expected-payload assertion missing the new Session
association; its fixture/assertion was updated and the whole dashboard suite passed.
Browser harness issues involving current Plotly button markup, default camera
semantics and synchronous dialog handling were corrected in validation scripts,
without changing application behavior. Intermediate logs remain available.

### Recorded evidence, not a new experiment

Successful Run: `1a0712b4-6226-47f3-a2cf-5a11a2cb3a2a`, under
`results/runner_readiness/workspace/runs/nominal_20261031T180156_975994Z_7syy67id/runs/imm_seed42`.
Second historical Run: `823fa1b3-36cf-4b36-9cc0-6288c02b4213`, under
`results/robot_description_naming/runtime/nominal_20261028T130412_296933Z_u30m7yel/runs/imm_seed42`.
Screens show original rounded KPI values **0.086 m**, **0.201 rad**, **0.093 m**,
**0.544 m**; none were recomputed or fabricated. Execution completion, safety and
scientific quality remain separate. D2.8 makes no new physical-safety claim.

Management browser tests used `results/dashboard_management/workspace`: a copied
Catalog and a disposable Session referring to the original Run directories. The
Session was renamed then deleted, and copied `EXP-NOMINAL` archived. Original
Catalog/Sessions were never management-test targets; no Job request was created
in the isolated workspace. Source baseline: `before.tar.gz`. Generated scripts,
logs, reports and screenshots remain under `results/dashboard_management/`.

### Actual installed screenshots

| View | 1920×1080 | 1600×900 | 1650×790 |
| --- | --- | --- | --- |
| Experiment Lab | [image](../../results/dashboard_management/final_lab_1920x1080.png) | [image](../../results/dashboard_management/final_lab_1600x900.png) | [image](../../results/dashboard_management/final_lab_1650x790.png) |
| Results 2D | [image](../../results/dashboard_management/final_results_2d_1920x1080.png) | [image](../../results/dashboard_management/final_results_2d_1600x900.png) | [image](../../results/dashboard_management/final_results_2d_1650x790.png) |
| Results 3D | [image](../../results/dashboard_management/final_results_3d_1920x1080.png) | [image](../../results/dashboard_management/final_results_3d_1600x900.png) | [image](../../results/dashboard_management/final_results_3d_1650x790.png) |

[Name-only creation](../../results/dashboard_management/new_scenario_name_only.png).
[Isolated archived selection](../../results/dashboard_management/metadata_archived.png).

## Reproduction

Use the existing dashboard Python environment and recorded fixtures:

```bash
cd /home/user/ros2_ws
source install/setup.bash
source results/dashboard_execution/evidence.env
PYTHONPATH=src/robot_dashboard:src/robot_experiments:$PYTHONPATH \
  build/dashboard_venv/bin/python -m pytest -q src/robot_dashboard/test
PYTHONPATH=src/robot_experiments:$PYTHONPATH \
  build/dashboard_venv/bin/python -m pytest -q src/robot_experiments/test/test_management.py
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_experiments robot_dashboard
```

Working dashboard, using the existing Catalog/Session workspace and original Run
references (stop an existing dashboard on this port before launching another):

```bash
source install/setup.bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir /home/user/ros2_ws/results/runner_readiness/workspace \
  --result-root /home/user/ros2_ws/results/runner_readiness/workspace/runs/nominal_20261031T180156_975994Z_7syy67id \
  --result-root /home/user/ros2_ws/results/robot_description_naming/runtime/nominal_20261028T130412_296933Z_u30m7yel
```

Open `http://127.0.0.1:8060`. Select the existing **Recorded tracking review**
Session, Results & History, and the completed Nominal IMM tracking Run. Use the
header toggle for 2D/3D and native chart toolbar. For fresh verification, the
read-only `results/dashboard_management/browser_final.py` repeats the viewport,
scale, orbit/reset and original-hash checks against that server. Management tests
must use isolated metadata, because their confirmed actions intentionally delete
the disposable Session and soft-archive its copied Catalog entry.

## Limitations

- **Scenario rename is intentionally deferred**, as allowed by D2.8. The display
  name is tied to immutable revision definitions. A future Catalog-level display
  alias contract is needed; the disabled menu explains this. No revision rewriting
  or independent name store was introduced.
- Legacy nonterminal Jobs without a launch Session association conservatively
  block Session deletion even when a specific association cannot be established.
  Unreadable/malformed owned Jobs also fail closed; resolve cleanup/Diagnostics
  rather than bypass the guard. New dashboard starts pin association explicitly.
- Raw trajectories are independent pages, not synchronized comparisons or a scored
  window. Equal scale/data aspect can leave unused panel space, especially for
  nearly planar/stationary data. The smallest viewport has a compact 3D view;
  native zoom remains available. Rendering requires browser WebGL; validation
  used Chromium software WebGL, without a Gazebo rendering claim.
- Safety locking covers the supported backend/Jobs APIs in this local workspace;
  it does not make arbitrary external filesystem edits transactional. Existing
  local Dash development-server/authentication limitations remain unchanged.

## Exact task-relative changed files

- `src/robot_dashboard/robot_dashboard/app.py`
- `src/robot_dashboard/robot_dashboard/assets/dashboard.css`
- `src/robot_dashboard/robot_dashboard/execution.py`
- `src/robot_dashboard/robot_dashboard/forms.py`
- `src/robot_dashboard/robot_dashboard/results.py`
- `src/robot_dashboard/robot_dashboard/management.py` (new)
- `src/robot_dashboard/test/test_execution.py`
- `src/robot_dashboard/test/test_results.py`
- `src/robot_dashboard/test/test_management_ui.py` (new)
- `src/robot_dashboard/test/test_trajectory.py` (new)
- `src/robot_experiments/robot_experiments/backend.py`
- `src/robot_experiments/robot_experiments/catalog.py`
- `src/robot_experiments/robot_experiments/jobs.py` (optional Session association only)
- `src/robot_experiments/test/test_management.py` (new)
- `docs/dashboard.md`
- `docs/validation/dashboard_management.md` (new)

No production files were removed or relocated. Existing unrelated edits and
historical validation artifacts were preserved.
