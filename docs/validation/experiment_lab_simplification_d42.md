# D4.2 — Experiment Lab simplification

Date: 2026-11-05. Scope: the existing Dash client and experiment owners only.
No commit/push, new simulation engine, physical/numerical tuning or Dynamics
Studio redesign. Earlier uncommitted D4.0/D4.3/D4.1 work is preserved.

## Problem and final information architecture

The old Lab exposed a Condition selector, a separate unscored Simulation Setup,
large advanced configuration/policy forms and short nested parameter scrollbars.
Management could reject an actually selected Scenario while its revision callback
was pending. Duplicate → Load → Validate also exposed a real asynchronous race:
the previous draft could be validated before the replacement finished loading.

The Lab now has Scenario selection plus New/Duplicate/Rename/Delete; Models &
Parameters with duration, Seeds and seven directly visible model selectors; and
one compact Play/Stop rail. Selectors sit outside optional parameter accordions.
Expanded EKF/controller bodies have natural height and use the main editor scroll.
Navigation, validation/save and Run actions stay outside that scroll. Actuation
and GUI opt-in start unchecked. There is no Simulation Setup or automatic launch.
New children manage their own future Runs; historical Runs retain their original
parent/variant identity and Session references, resolved to Scenario labels for
display. Deleting a child does not delete unrelated parent/variant history.

Separate simulation presentation and callback registration are removed. The
unused `robot_dashboard/simulation.py` presentation module and its obsolete CSS
were removed after checking imports, entry points, automatic package installation
and affected tests. The existing `robot_experiments` simulation, Jobs, Runner,
Recorder, launch, readiness, preview ownership and cleanup services remain.
Numerical Preview in Dynamics Studio is unchanged. Developer effective-config,
comparison policy and diagnostic access remain in backend/CLI and Results details.

## Scenario migration and identity

Legacy Conditions are variants embedded in immutable Catalog definitions, not a
separate mutable Condition database. Existing single-variant Scenarios keep their
identity. Multi-variant studies project each variant as an individually selectable
Scenario with a deterministic ID derived from parent ID and variant. Listing is
read-only; selection lazily freezes the selected configuration through existing
Catalog validation. The new child stores parent ID, exact revision, content hash
and variant in additive `scenario_origin` metadata outside the immutable revision
hash. Its definition differs only by retaining the selected variant. No parent
revision or historical Run is rewritten. Concurrent freezes check identical origin.

Historical APIs still read and execute original studies. Original Run identities
and Session associations remain intact; display aliases link parent/variant to
its child Scenario. Child deletion/retirement cannot silently resurrect a virtual
Scenario. Existing authoritative deletion scope, confirmation and active-owner
checks remain. New display names use short deterministic collisions (`scenario`,
`scenario 2`, `circular motion 2`, `session 2`); IDs remain immutable and separate.
No aesthetic rename of historical artifacts was performed.

Rename/Delete resolve the actual selected Scenario afresh instead of requiring a
possibly stale `source-revision`. Confirmation still freezes identity/revision and
checks concurrent changes. Load reads the selected immutable revision's schema
and variant, not the latest revision's definition.

**Bounded limitation:** schema-2 sensor-assignment Scenarios migrate, inspect and
execute safely, but their assignment editor remains CLI-only. Lab explicitly
reports Read-only instead of pretending its schema-1 Author can edit them. The
ordinary schema-1 Scenario workflow supports editing and immutable revisions.

## Save, validation and execution

- New/Duplicate return their proposed display name and technical ID atomically,
  avoiding a name-before-ID callback race. Load/Preview/Validate/Save expose a busy state and disable dependent editor
  actions. Validation/save additionally require the loaded draft identity to
  match the user's selected/create intent; a fast Duplicate click cannot validate
  and save the preceding Scenario.
- Editing invalidates the exact validated form digest. Invalid/stale forms remain
  Invalid; an attempted backend write failure reports Save failed. Actual saving
  reports Saving…; success reports only Saved.
- Save explicitly selects the newly saved Scenario and revision for Play. The
  authoritative Author revalidates and Catalog performs optimistic revision CAS.
  Conflicts preserve the draft. No hidden duration, PID or owner-default changes.
- Play takes the actual selected saved ID/revision through existing ExecutionActions
  and Runner preflight. Explicit actuation and GUI flags, polling, cancellation,
  idempotent Session references, readiness and cleanup behavior are unchanged.
  No new physical experiment was required or claimed for this UI milestone.

## Descriptive ranking and actual recorded evidence

The new optional backend descriptive output is separate from the existing stricter
scientific ranking. Four selectable metrics use actual recorded RMSE: Range [m],
Angle [rad, converted to degrees in the table], target-estimation position [m],
and prediction position [m]. The default is Range RMSE, lower is better.

The metric owner defines R error as `abs(||p_target - p_tracking|| - radius)`.
Its RMSE equals the RMSE of the signed radial residual; R is **not** the radius.
The table preserves three-decimal precision and never substitutes means for RMSE.
Headers are Scenario | R (m) | Angle (°) | Estimator (m) | Predictor (m) | Rank,
under **Experiment Comparison · RMSE**.

Eligibility requires completed execution, finite nonnegative selected RMSE, valid
units/sample counts, required metric evidence and the full unchanged scoring
interval. Comparison metadata is reconstructed from original frozen contracts and
checked against independent resolved evidence. Scope identity retains dynamics,
physical/evaluation/sensing/safety configuration, Seed, metric definitions/units and
only explicitly declared compatible non-dynamics variable differences. Merely
declaring a target-dynamics study does not establish equivalent driving difficulty;
unprojected dynamics remain protected. Duplicate immutable Run IDs are excluded.
Only a display/namespace study name is ignored. Scope differences and exclusion
reasons remain inspectable. Multiple compatible cohorts require explicit selection.

At least two distinct comparable Runs are required. Exact ties share competition
rank (1, 1, 3); immutable Run ID provides deterministic tie order. There is no
weighted composite, cross-seed averaging, confidence interval, significance or
universal winner claim. Identical reactive dynamics do not guarantee identical
realized trajectories. Recorded acceptance and safety are never modified. Physical
coverage may be insufficient for qualification even when error metrics exist; only
the separate descriptive path permits this distinction. Existing scientific gates
continue requiring physical coverage/safety evidence.

The isolated `circle tests` Session references six genuine pre-existing Runs in
place, without copying telemetry or generating new metrics. Five share the same
30-second scoring definition; the sixth has a different duration and remains
unranked history. The original scientific ranking is null. Five overall acceptance
values remain null, one remains false; all six recorded safety verdicts remain
null with incomplete physical observation coverage. Descriptive ranks do not cure
these limitations.

Exact UUIDs, all four selected rankings, source metrics, selected scope hash and
Session ID are recorded in
`results/experiment_lab_simplification/recorded_rankings_final.json`.

| Run UUID prefix | Window (s) | R RMSE (m) | Angle RMSE (°) | Estimator RMSE (m) | Predictor RMSE (m) | Range rank |
|---|---:|---:|---:|---:|---:|---:|
| 24efa66c | 6 | 0.388026 | 31.317213 | 0.075145 | 0.461331 | Unavailable |
| 6348d869 | 30 | 0.573274 | 71.914111 | 0.073914 | 0.487693 | 2 |
| 94e3e5d2 | 30 | 0.556979 | 73.232461 | 0.073709 | 0.487256 | 1 |
| 8ba0d023 | 30 | 0.575724 | 75.938867 | 0.074089 | 0.488550 | 3 |
| 2fcec5a1 | 30 | 0.826752 | 97.456358 | 0.080583 | 0.489731 | 5 |
| 5d97ba3f | 30 | 0.816178 | 99.861388 | 0.080109 | 0.489662 | 4 |

## Data integrity verification

The task-start inventory contains 19 historical Catalogs (417 revision records),
119 Run records and 31 Sessions (66 immutable Run references). All loaded through
existing Catalog/Backend readers; there are zero missing Session references.
`historical_loads.json` and `historical_loads.txt` record each attempted read.
The baseline hash audit covers **19,608** pre-existing result files: zero changed,
zero missing. Original Catalogs, Dynamics pins, Run inputs/telemetry/metrics and
Session documents remain byte-identical. Only new isolated D4.2 evidence and
acceptance copies are written under `results/experiment_lab_simplification/`.

The original D4.1 report references an older UUID/path already absent before D4.2;
the current user workspace contains six later circle Runs instead. No missing
artifact was invented or recreated. The preservation assertion is relative to the
captured task-start inventory, not an assertion that every historical experiment
mentioned in every older document still exists.

Early browser attempts and the async-load race reproduction are retained under
`attempt_one_browser/`, `attempt_one_workspace/`, `attempt_two_browser/` and
`attempt_three_browser/`, `attempt_four_browser/`, `attempt_five_browser/`,
`attempt_six_browser/` and `attempt_seven_browser/`. The first race mutated only its isolated copied Catalog,
never the original user Catalog. Focused RED/diagnostic logs are retained rather
than relabeled passing. A stale installed build was rebuilt before the final
installed-browser checks; obsolete generated copies of the deleted Dash module
were removed from build/install and absence verified.

## Verification commands and evidence

All evidence below is under `results/experiment_lab_simplification/`. These are
functional tests, not claims of physical safety acceptance.

| Area | Actual result | Evidence |
|---|---|---|
| Catalog, Backend, composition | 62 passed | `catalog_backend_regression.txt` |
| Current Scenario/descriptive ranking + real component projections | 43 passed | `scenario_ranking_current.txt` |
| Existing scientific comparisons/benchmark/ranking | 64 passed | `comparison_regression_final.txt` |
| Deletion, simulation lifecycle, Runner, execution | 122 passed | `lifecycle_regression_final.txt` |
| Configuration, Author, Scenario integration | 51 passed | `authoring_regression_final.txt` |
| Unified model selection, Studio, mapped output/deployment regression | 34 passed | `d41_regression_final.txt` |
| Final installed-layout unified model/Studio compatibility recheck | 4 passed | `ui_compatibility_complete.txt` |
| Actual-recording Results suite | 23 passed, no skips | `results_current.txt` |
| Builder, immutable duration/revision, identity and stale-form guards | 22 passed | `builder_current.txt` |
| Atomic New identity, layout, selected-identity management and truthful busy/Save feedback | 4 passed | `lab_complete.txt` |
| Official installed console HTTP and cleanup | HTTP 200; exit 0; 8061 free | `installed_verified_complete.txt` |
| ROS package build | robot_experiments + robot_dashboard passed; final dashboard rebuild passed | `build_current.txt`, `build_complete.txt` |

Counts overlap where suites share tests; do not sum them as independent trials.
The earlier dashboard batch recorded 39 passed, 2 failed, 8 skipped. The two
failures were stale test transports lacking the new `list_scenarios` API; the
fixtures now implement that contract, and both pass (`results_labels_final.txt`).
All eight recording-dependent checks subsequently passed with explicit genuine
success/failure artifact paths; the complete Results suite now has no skips.
Partial/interrupted test logs without summaries are not counted as passes.
The selector regression initially ran against an older installed copy and failed;
it passes against the rebuilt current layout. RED tests reproduce duplicate Run
counting, dynamics-policy projection, selected-revision schema, stale loaded
identity, selectors hidden behind accordions and non-atomic new identity.

Dash emits its existing `dash_table.DataTable` deprecation warning. No wholesale
style/lint cleanup was performed or claimed. `git diff --check` is required clean.
A bounded read-only review found and rechecked all three ranking/revision findings;
there were no unresolved review findings from that recheck.

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_experiments robot_dashboard
source install/setup.bash
build/dashboard_venv/bin/python -m pytest -q \
  src/robot_experiments/test/test_scenarios.py \
  src/robot_experiments/test/test_descriptive_ranking.py \
  src/robot_experiments/test/test_component_comparison.py
build/dashboard_venv/bin/python -m pytest -q \
  src/robot_dashboard/test/test_builder.py \
  src/robot_dashboard/test/test_lab_simplification.py
ROBOT_DASHBOARD_RECORDED_RUN="$PWD/results/recorder_diagnostics_d29b/live/nominal_20261102T143918_895978Z_ol7_mqis/runs/imm_seed42" \
ROBOT_DASHBOARD_FAILED_RUN="$PWD/results/scientific_comparison/workspace/runs/nominal_20261102T111309_465057Z_mk0nn8f3/runs/imm_seed42" \
  build/dashboard_venv/bin/python -m pytest -q src/robot_dashboard/test/test_results.py
```

Launch the existing user workspace (8060 must be free):

```bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir results/generic_live_output_mapping/workspace
```

Open `http://127.0.0.1:8060`. Load a Scenario, edit, Validate, Save; select/create
a Current Session; explicitly opt into actuation/GUI if wanted and press Play.
Stop/Cancel uses the existing owner. This milestone's acceptance browser uses
`--data-dir results/experiment_lab_simplification/workspace` and registers only
`results/generic_live_output_mapping/workspace` as the read-only result root, so test
CRUD cannot modify original user Catalog/history. The `serve.py`,
`browser_acceptance.py`, its resume scripts, `recorded_metrics.py`, `load_history.py`,
`verify_installed.py` and `audit_preservation.py` scripts retain the exact bounded
verification operations alongside their generated evidence, not in production.

## Real Chromium verification

Installed Chromium used actual saved Catalogs and Session references. The sequence
was resumed at completed boundaries rather than recreating deleted copies or
rewriting history: `browser_complete.txt` records Duplicate/revision save;
`browser_finish_complete.txt` records cancel/confirmed deletion; and
`browser_new_results.txt` records successful fresh-page New/Save, Play readiness
and all four real-data ranking selections. Earlier failed waits remain archived.
`browser_report.json` records 100% zoom, exact viewport/document equality, natural
component-body sizing, six header measurements and zero page JavaScript errors.

Each of the six table rows was checked against `recorded_rankings_final.json` for
all four metric selections: the four RMSE values with exact three-decimal display,
angle conversion, the backend's rank, and the incomparable Run's Unavailable
state. The browser never computed its own winner. At 1650×790 the history table
uses bounded internal row scrolling; all six headings fit without wrapping. The
1920×1080 screenshot shows all six recorded rows. Lab retains main-editor scrolling
for long parameter bodies; no component-body max-height or overflow scrollbar.

| Desktop | Expanded EKF | Expanded Controller | Recorded Results |
|---|---|---|---|
| 1920×1080 | [Screenshot](../../results/experiment_lab_simplification/lab_robot_estimator_1920x1080.png) | [Screenshot](../../results/experiment_lab_simplification/lab_controller_1920x1080.png) | [Screenshot](../../results/experiment_lab_simplification/results_1920x1080.png) |
| 1600×900 | [Screenshot](../../results/experiment_lab_simplification/lab_robot_estimator_1600x900.png) | [Screenshot](../../results/experiment_lab_simplification/lab_controller_1600x900.png) | [Screenshot](../../results/experiment_lab_simplification/results_1600x900.png) |
| 1650×790 | [Screenshot](../../results/experiment_lab_simplification/lab_robot_estimator_1650x790.png) | [Screenshot](../../results/experiment_lab_simplification/lab_controller_1650x790.png) | [Screenshot](../../results/experiment_lab_simplification/results_1650x790.png) |

Also captured: [seven directly visible selectors](../../results/experiment_lab_simplification/lab_models_1920x1080.png)
and [saved Play readiness with both opt-ins unchecked](../../results/experiment_lab_simplification/play_ready_1920x1080.png).
No scored Run or Gazebo simulation was started by these checks.

## Exact D4.2 changed files

Compared against the captured task-start source hashes, not the Git diff of
all pre-existing milestones:

Modified:

- `docs/dashboard.md`
- `src/robot_dashboard/robot_dashboard/app.py`
- `src/robot_dashboard/robot_dashboard/assets/dashboard.css`
- `src/robot_dashboard/robot_dashboard/assets/dashboard.js`
- `src/robot_dashboard/robot_dashboard/execution.py`
- `src/robot_dashboard/robot_dashboard/management.py`
- `src/robot_dashboard/robot_dashboard/results.py`
- `src/robot_dashboard/test/test_builder.py`
- `src/robot_dashboard/test/test_results.py`
- `src/robot_dashboard/test/test_simulation_ui.py`
- `src/robot_experiments/robot_experiments/backend.py`
- `src/robot_experiments/robot_experiments/benchmarking.py`
- `src/robot_experiments/robot_experiments/catalog.py`
- `src/robot_experiments/robot_experiments/comparison.py`
- `src/robot_experiments/test/test_component_comparison.py`

Added:

- `docs/validation/experiment_lab_simplification_d42.md`
- `src/robot_dashboard/test/test_lab_simplification.py`
- `src/robot_experiments/test/test_descriptive_ranking.py`
- `src/robot_experiments/test/test_scenarios.py`

Removed:

- `src/robot_dashboard/robot_dashboard/simulation.py` (unused Dash simulation presentation only)

Generated scripts, reports, screenshots and isolated acceptance Catalogs live
only under `results/experiment_lab_simplification/`. Prior D4.1 simulation/model/
Author/config changes remain untouched relative to this milestone baseline.

## Preserved deletion safeguards and verification limits

An actual browser Delete attempt against the entire `results/` root was blocked
with **Recorded Run ownership cannot be established**. That broad historical
root includes partial records without the required metadata. This is the existing
conservative owner safeguard, not the former stale-selection error. No deletion
logic or ownership criterion was weakened. Confirmed acceptance CRUD is scoped to
the six complete circle recordings; the copied Scenario has no owned Runs.
Registering broad legacy roots for viewing can still prevent deletion until their
incomplete ownership evidence is resolved. The recommended user launch keeps its
normal workspace scope, rather than registering the entire research archive.

One installed-console smoke attempt used a five-second HTTP request timeout and
repeated requests under concurrent GUI/preflight load; it timed out despite server
HTTP-200 responses. That failure and cleanup evidence are retained. The final
smoke uses one bounded 30-second HTTP request. This changes only the generated
verification script, not any ROS readiness, Runner, safety or production timeout.
Changing an undeclared component/parameter creates a separate comparison scope.
The simplified Lab preserves existing frozen declarations but does not author new
comparison policies; use the existing Author/CLI for a new controlled contrast.
This conservative scope rule is intentional, rather than automatic compatibility.

A New-after-Delete browser sequence exceeded its 90-second form-presentation
wait even though the backend returned a valid new draft. That attempt is retained;
New/Save and Play-readiness subsequently passed in a fresh page with a bounded 180-second
UI wait. These are verification waits, not changes to production execution gates.
Rapid consecutive management responsiveness is not claimed fully resolved.

Owner validation/catalog callbacks can take several seconds; this task does not
introduce a cache or refactor configuration loading for responsiveness.

Final cleanup (`cleanup_final.json`) confirms zero owned verification-process
survivors and both 8060/8061 free. Browser and installed-console processes were
closed through their task-owned lifecycle; no simulator was launched.

## Independent verdicts

| Requirement | Verdict | Evidence / qualification |
|---|---|---|
| Scenario-only workflow | **CONDITIONAL** | Complete schema-1 authoring; schema-2 sensor assignments remain explicitly CLI-editable/read-only in Lab. No Condition selector. |
| Legacy migration and history | **PASS** | Exact variant projections, source-origin checks, 417 revision records/119 Runs/31 Sessions readable; 19,608 prior files unchanged. Partial legacy artifacts remain preserved. |
| Scenario management | **CONDITIONAL** | Identity-safe New/Duplicate/Rename and guarded deletion; confirmed zero-Run copy deletion in complete scope. Ambiguous broader archives intentionally block deletion; one rapid New-after-Delete presentation wait timed out. |
| Removal of separate Simulation Setup | **PASS** | Layout/callback tests; unused presentation module removed; backend lifecycle intact. |
| Play/Stop safety | **PASS** (UI contract) | Explicit opt-ins OFF, selected saved revision, cancellation/ownership/readiness regressions; no claim of physical safety acceptance. |
| Accordion usability | **PASS** | Seven selectors outside parameter accordions; natural expanded bodies, one main scroll; actual EKF/controller screenshots at three sizes. |
| Compact Results comparison | **PASS** | Actual metric values/units and six compact headers; desktop screenshot and recorded-data checks. |
| Scientifically defensible ranking | **PASS** (descriptive only) | Explicit compatible cohort, exact RMSE/ties, full scoring windows and protected dynamics; incompatible result excluded; null/false qualification unchanged. |
| Save/status simplification | **PASS** | Exact validated digest, selected saved identity, atomic new ID, busy feedback, real immutable revisions and conflict/stale-load regression checks. |
| Existing D4.1 regression compatibility | **PASS** | 34 mapped-output/Studio/library checks; pinned model retained through UI duplication; simulation/model source untouched in this milestone. |

Overall: **CONDITIONAL** for legacy schema-2 editing and the observed rapid
management/presentation latency.
The ordinary Scenario workflow and descriptive comparison are implemented within
the existing architecture. Configuration preflight can be slow; archive ownership
and frozen comparison declarations remain strict. Existing physical tracking,
coverage, actuation and localization limitations are neither resolved nor relabeled
accepted by this UI milestone. No commit or push; stop after D4.2.
