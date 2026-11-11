# D4.5 — Dashboard refinement and Robot Response reliability

2026-11-11. Scope: attachment `a63684b5-8b98-4c56-b383-98c4e8d8a3f8`.
Continued the existing uncommitted workspace; no reset, commit, push, physics,
controller, estimator, interaction-law, safety-limit or readiness-policy change.

## Outcome and boundaries

The UI refinements and a demonstrated contextual TF receiver defect are implemented.
The bounded real DDS test now preserves the causal interval lost by the former
five-message best-effort reader. Two post-fix Robot Response ON Runs and one OFF
Run completed through the existing Runner, with zero invalid control samples and
no surviving owned simulator processes. These are compute-only Runs:
`publish_commands=false`, actuation explicitly OFF, and zero motion commands.
Execution completion does not establish tracking-quality or actuated safety acceptance.

Runtime acceptance remains **CONDITIONAL**: the queue-loss mechanism is demonstrated,
and a real failed Run proves the historical gap, but the earliest failed interval
was not independently observed upstream. Not every missing sample in that particular
Run can be conclusively attributed to DDS loss rather than an upstream publication
pause. We do not claim this finite validation rules out every scheduling failure.

## UI implementation

- Lab keeps the existing seven primary model selectors. Each effective model has
  one label, without `Inherited`, permanent `preflight required`, a second selected
  name or selection-provenance paragraph. Expandable **Parameters** remains.
  Secondary resource settings stay secondary. Genuine unavailable selections remain
  disabled; preview-only library models retain their honest deployment status.
- Technical inheritance, defaults, parameter overrides, complete preflight, immutable
  pins and provenance remain with the Author/Catalog and Advanced effective view.
  The presentation says **Valid** after complete validation; the internal validated
  digest and Save gate remain unchanged.
- Robot Response is applied transactionally: candidate preflight/preview must succeed
  before replacing the draft. On rejection the callback immediately redraws cards
  from the authoritative unchanged draft and displays the real error. Expected and
  unexpected exceptions both restore the checkbox; no reload is required.
- Studio removes the requested instructional sentence with no replacement.
  Physical Output Mapping is native `Details`, closed by default, with its original
  fields mounted. Collapse changes visibility only. Equations, parameters and math
  use natural content height in the same two columns, with one workspace scroll for
  long drafts. Six scalar states/seven physical coefficients are covered.
- Studio library-save refresh and initial Lab rendering share `target_options`'s
  inherited-choice presentation. Independent review found and corrected the old
  refresh path that could restore duplicate labels after a Studio save.
- Results displays exactly **Scenario | R (m) | Angle (°) | Estimator (m) |
  Predictor (m)** under **Experiment Comparison · RMSE**. Each row refers to its
  immutable Run, in Session order, with recorded RMSE, original definitions, radians
  converted to degrees, three decimals and honest missingness. Rank, ranking/scope
  selectors and ranking prose are removed. Backend comparison/cohort/ranking APIs,
  eligibility, scientific provenance and immutable Run evidence are unchanged.
- Removed orphan ranking/popover CSS and an unused local variable. No production
  file, YAML scenario, historical Run or scientific metric definition was deleted.

## Robot Response investigation

### Actual failing evidence

Start with [D4.4 evidence](external_influences_d44.md): its ON Run completed at Runner
level but stopped target integration near 3.85 s; 44/80 control samples were invalid.
A new diagnostics-only Run reproduced the problem before the receiver correction:

- Run `9b40fd8d-e023-49b3-a8f5-d97608c45526`, Session
  `8bbfeef8-a9a4-4b0d-a690-68f1d6c2ac2b` (`diagnostic_on/`).
- Target requested 1.212 s, requiring validity through 1.222 s. Retained TF immediately
  before/after the request was **1.022 / 1.450 s**. With unchanged `max_age=0.2 s`,
  the preceding sample could not cover the RK4 interval. Later TF kept arriving but
  could not repair that historical hole without using future/stale data.
- First failure snapshot: history **8 of 5202**, retained since 0.548 s. Thus capacity
  exhaustion or retention pruning did not explain this gap. Direct-receiver counters
  also recorded two ahead-of-local-clock rejections during startup; their individual
  timestamps were not yet captured in this first diagnostic build.
- By clock 10.66 s, latest accepted TF was 10.646 s but the target remained at 1.212 s;
  3868 consecutive failed selections were recorded. The existing bounded-lookback
  guard genuinely failed the Run rather than resetting the model or inventing context.
- The independent observer joined TF after this early interval. Its later continuous
  TF is evidence of a live producer, not proof of every earlier publication.

A second diagnostics-only ON Run (`202fead7-0ef3-46fc-b888-92732a988066`) completed
without a stall, before changing selection behavior. This confirms intermittency;
it is not presented as proof that the previous implementation was reliable.

### Demonstrated receiver defect and correction

`TrackingFeedbackTF` broadcasts canonical measured FK as direct
`world -> tracking_frame` TF. `TrackingFrameProvider` had two subscribers:
normal TF2 (reliable, depth 100) and its contextual direct reader
(`qos_profile_sensor_data`: best effort, depth **5**). The latter is intended to
retain historical source samples while clock callbacks, integration or pose
acknowledgements lag. Latest-data transport can discard the very interval that
historical RK4 catch-up later needs; a large application history cannot restore
samples discarded before its callback.

The bounded **real ROS/DDS** probe publishes 20 known source-time TF samples while
callbacks are held for 0.2 wall seconds. The old reader receives only **5/20**;
the causal requested interval is unavailable. A standard reliable depth-100 reader
receives **20/20**, and the same request returns the actual stamped position
`[2,0,0]`. The post-build probe repeats this with the unpatched installed corrected
provider against an explicitly patched old-QoS control. See `queue_probe.json`,
`queue_verify.json` and their scripts/logs under the evidence directory.

The direct contextual reader now uses **reliable KEEP_LAST** with the existing
configured `context_history_capacity`; standalone callers without a capacity use
TF2's depth-100 default. Production frozen deployments here provide capacity 5202.
No additional YAML parameter or second clock, integrator, TF publisher or Runner
was introduced. Existing TF broadcaster reliability is compatible.

**Unchanged mathematical contract:** select the latest source timestamp no later
than the requested integration time; validate it through the RK4 step; preserve
world frame, maximum age, future/invalid rejection, bounded history, delayed-sample
recovery, pose acknowledgement, clock-reset epochs, full internal model state and
analytic position/velocity output. The frozen response YAML and OFF path are intact.
There is still no valid fallback when real input is missing.

Optional node parameter **`context_diagnostics`**, boolean, default **false**, enables
one JSON snapshot per simulation second plus wait transitions. It can be enabled
through the normal ROS parameter service. It reports request/validity interval,
nearest retained stamps, history bounds/capacity, direct-receiver insertion outcomes,
64 bounded recent arrival tuples, wait duration, committed position/velocity and
pose-ack status. Direct-receiver counters exclude TF2 latest-refresh insertion;
they are not total counts of every history insertion path.

## Actual runtime results

All post-fix Runs use the existing frozen D4.4 A/B revisions, seed 42, four scored
simulation seconds after one settling second, unchanged readiness `max_skew=0.08 s`,
headless Gazebo, and explicit actuation OFF. Functional/browser verification was
also running on this host; RTF below is observed, not a controlled performance
benchmark. Different readiness origins mean these are not a matched-phase
tracking-quality comparison and must not be ranked as one.

| Check | Immutable Run | Scored simulation interval (s) | Distance error mean / max (m) | Pointing error mean / max (°) | Invalid control / samples |
|---|---|---:|---:|---:|---:|
| after_on | `0160d96c-fe64-4667-b525-e2b44406ad0a` | 3.330–7.330 | 0.358083 / 0.686153 | 112.462 / 133.503 | 0/80 |
| after_off | `9df557ea-afa9-4ed0-b423-76824c98d34f` | 18.994–22.994 | 0.309282 / 0.327783 | 138.943 / 144.781 | 0/80 |
| after_on_repeat | `d8552923-f864-4447-a1c3-fe3ca0454ea5` | 11.350–15.350 | 0.306195 / 0.620501 | 170.077 / 173.314 | 0/81 |

- Real Runner readiness decisions were true in all three Runs, evaluation origins
  were established, each scored interval finished, metrics/time series were saved,
  and backend Sessions reference the original immutable Runs. Raw physical tracking
  errors are large because the robot is held with commands disabled; they are not
  represented as tracking acceptance.
- ON scored truth: 197 and 175 samples; maximum source gaps **0.070 / 0.076 s**.
  Target state/velocity continued through completion; no permanent context wait
  after initialization. First ON waited safely for initial physical TF discovery,
  then initialized; it did not substitute a position.
- Observed mean RTF: ON **0.7773**, OFF **0.7253**, repeated ON **0.7048**.
  Scheduling/startup and concurrent validation differ; no improvement claim.
- OFF vs independent exact linear-system matrix exponential: maximum position
  discrepancy **3.8923e-10 m**. Unit tests also preserve bit-exact OFF integration.
- Independent ON continuation from logged committed position/velocity, with measured
  source-time world TF and a separately written response law/SciPy solver, differed
  by at most **3.6141e-11 / 4.2926e-11 m** at logged snapshots. This diagnostic is not
  an exact arrival-order replay: an offline TF set may contain late arrivals.
- Measured velocity finite-difference residual after subtracting the base dynamics
  points away from the measured robot: minimum directional cosine **0.9916 / 0.9430**
  over 5/12 snapshot intervals. This is a finite-interval directional diagnostic;
  independent formula, RK4, bound, zero-distance and hidden-state tests provide the
  mathematical checks.
- Every post-fix Run recorded **zero motion commands**, genuinely reached Completed,
  and cleanup returned code 0 with **no survivors**. Overall acceptance is null for
  these compute-only Runs; completion is not real-hardware or actuated safety PASS.

Detailed source stamps, readiness, state traces, metrics, source-gap statistics,
cleanup and independent-reference limitations are in
`results/dashboard_refinement/runtime_analysis_d45.json`.

## Tests, build and review

| Verification | Actual result | Evidence under `results/dashboard_refinement/` |
|---|---|---|
| Simulation functional suite, including generic equations, Cartesian models, RK4/time stepping, context/reset/recovery | **508 passed** | `simulation_regression_d45.txt` |
| Lab/model-selection/influence/unified-source functional suite | **30 passed** | `lab_regression.txt` |
| Runner/execution/influence authoring | **28 passed** | `experiment_regression_d45.txt` |
| Studio/math/output-mapping/preview suite | **59 passed** | `studio_regression.txt` |
| Results + Lab presentation + cached-deleted-Run refresh | **28 passed**, 14 deselected | `results_final.txt` |
| New reliable-queue/diagnostics/recovery/independent influence checks | **56 passed** (overlap with simulation suite) | `context_queue_green.txt` |
| Studio-save selector refresh, inherited, explicit and frozen-pin cases | RED 2 + pin RED 1, then **3 passed** | `library_refresh_red.txt`, `pinned_refresh_red.txt`, `library_refresh_all_green.txt` |
| `colcon build --packages-select robot_simulation robot_dashboard` | **2 packages built** | `build_final.txt` |
| Dashboard rebuild after removing unused local | **1 package built** | `dashboard_build_verified.txt` |
| Modified production Python, `flake8 --select=F,E9` | **clean** after unused-local removal | `d45_static_check_final.txt` |
| Scoped `git diff --check` | **clean** | `d45_diff_check.txt` |

Counts are per suite, not additive: focused regressions overlap. The 30-test Lab run preceded the last one-line frozen-pin guard; all three affected refresh tests passed afterward. Existing Dash
DataTable deprecation warnings remain. Simulation style/copyright tests were not
included in the functional count; this is not a claim that all repository lint
passes. Colcon's existing underlay override warning is recorded, not a build failure.
RED logs and corrected test-harness failures are retained, rather than counted as
successful verification. No numerical/safety tolerance or readiness deadline changed.

Independent read-only review found the Studio refresh label regression; the shared
helper and callback regression fix it. A second review found no critical/important
runtime defect and explicitly distinguished demonstrated queue loss from inference
about the earliest unobserved producer interval.

## Reproduction

From `/home/user/ros2_ws`, with the existing dashboard virtual environment:

```bash
source install/setup.bash
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_simulation robot_dashboard
ros2 run robot_dashboard dashboard --port 8060
```

Use the existing saved Scenario, explicitly Validate/Save changes, and opt into
actuation only if intended. No dashboard action in this task enables it implicitly.

Exact isolated browser verification server (historical result roots are references):

```bash
source install/setup.bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir "$PWD/results/dashboard_refinement/workspace" \
  --catalog "$PWD/results/dashboard_refinement/workspace/catalog.yaml" \
  --result-root "$PWD/results/robot_description_naming/runtime" \
  --result-root "$PWD/results/runner_readiness/workspace/runs"
```

Bounded runtime/check commands (use a **new** output label; the utility refuses an
existing directory so it cannot overwrite recorded Runs):

```bash
source install/setup.bash
build/dashboard_venv/bin/python results/dashboard_refinement/runtime_check.py on NEW_ON_LABEL
build/dashboard_venv/bin/python results/dashboard_refinement/runtime_check.py off NEW_OFF_LABEL
build/dashboard_venv/bin/python results/dashboard_refinement/tf_queue_verify.py
build/dashboard_venv/bin/python results/dashboard_refinement/analyze_d45.py
```

The first two use `ExperimentBackend.start_run` over the copied frozen D4.4 revisions
and existing Jobs/Runner, not another execution engine. The ROS observer only enables
bounded diagnostics and records TF. Results/Session ownership stays unchanged.
The queue-check script has fixed diagnostic artifact names; retain/copy its prior
outputs before deliberately rerunning it. No original Run is rewritten.

Focused functional tests:

```bash
source install/setup.bash
export PYTHONPATH="$PWD/src/robot_simulation:$PWD/src/robot_experiments:$PWD/src/robot_dashboard:$PYTHONPATH"
build/dashboard_venv/bin/python -m pytest src/robot_simulation/test -q \
  --ignore=src/robot_simulation/test/test_flake8.py \
  --ignore=src/robot_simulation/test/test_pep257.py \
  --ignore=src/robot_simulation/test/test_copyright.py
build/dashboard_venv/bin/python -m pytest \
  src/robot_dashboard/test/test_builder.py \
  src/robot_dashboard/test/test_influence_ui.py \
  src/robot_dashboard/test/test_unified_dynamics.py -q
build/dashboard_venv/bin/python -m pytest \
  src/robot_experiments/test/test_influence_authoring.py \
  src/robot_experiments/test/test_execution.py \
  src/robot_experiments/test/test_runner.py -q
```

## Evidence preservation and exact changed files

SHA-256 audit checked **20,087** task-start historical files outside the evidence
parent: **zero changed, zero missing** (`d45_historical_audit.json`). New immutable
Runs, Sessions, diagnostics and screenshots are isolated under
`results/dashboard_refinement/`; that existing evidence parent is excluded from the
historical baseline inventory, so the audit does not certify all earlier helper
files inside it. Original D4.4 data and Library are untouched; the Catalog/Library
used for this task are working copies. No deletion of scientific evidence.

Task-start source-hash comparison (`d45_changed_files.json`), rather than the much
larger inherited Git diff, identifies these **17** source/documentation files:

```text
src/robot_dashboard/robot_dashboard/app.py
src/robot_dashboard/robot_dashboard/assets/dashboard.css
src/robot_dashboard/robot_dashboard/dynamics_forms.py
src/robot_dashboard/robot_dashboard/dynamics_studio.py
src/robot_dashboard/robot_dashboard/execution.py
src/robot_dashboard/robot_dashboard/results.py
src/robot_dashboard/test/test_builder.py
src/robot_dashboard/test/test_influence_ui.py
src/robot_dashboard/test/test_lab_simplification.py
src/robot_dashboard/test/test_results.py
src/robot_dashboard/test/test_studio_refinement.py          (new)
src/robot_dashboard/test/test_unified_dynamics.py
src/robot_simulation/robot_simulation/gazebo/tracking_frame_provider.py
src/robot_simulation/robot_simulation/nodes/simulation_node.py
src/robot_simulation/robot_simulation/state/physical_context.py
src/robot_simulation/test/test_tracking_frame_provider.py
docs/validation/dashboard_refinement_d45.md                 (new)
```

Generated evidence and bounded diagnostic scripts are separate from production
source. `d45_changed_files.json` enumerates source/docs; runtime analysis records
exact Run directories and Session documents. No package/configuration migration,
new physics/algorithm owner, commit or push.

## Installed-browser verification and screenshots

Real installed Chromium, device scale 1, at **1920×1080**, **1600×900** and
**1650×790**: document width/height exactly matched the viewport in every tab,
with no page-level horizontal or vertical overflow. Lab uses its existing bounded
component workspace; a long six-state/seven-parameter Studio draft uses one natural
workspace scroll at 790 px height, rather than tiny scrolling equation/parameter
tables. Mapping remains accessible and closed. MathJax produced one complete math
container with zero math errors. Both browser reports record **zero JavaScript errors**.

Verified through the real UI/backend:

- Seven primary selectors, one selected model display, parameter disclosures and
  complete preflight with **Valid**/enabled immutable Save.
- Duplicate OFF Scenario, explicitly enable response, Validate, Save revision 1,
  then reload `EXP-D45-BROWSER-ROBOT-RESPONSE`: checkbox remains ON with its frozen pin.
- Real rejected change from an estimated-TF Scenario: backend reports
  **“Derivative influence response requires physical world tracking TF”**;
  checkbox redraws OFF immediately and the error remains visible, without reload.
- Mapping expands/collapses twice without value changes; a six-state/seven-coefficient
  model previews all six internal coordinates with the same saved physical mapping.
  Exact editor/save/reload equivalence is additionally covered by owner tests.
- Results has exactly five requested columns, no ranking controls/prose, and two
  genuine retained immutable Runs. All **eight** numeric RMSE cells were independently
  cross-checked against backend-recorded metrics, including radians-to-degrees
  conversion (`d45_browser/metric_integrity.json`). The source Run/Session data is
  referenced, not duplicated. Final settled captures show all three recorded
  Truth/Estimate/Prediction traces; equal metric aspect is retained.

Primary workflow log: `d45_browser/verify_saved.txt`, with successful earlier
Validate/Save steps retained in `verify_final_attempt.txt`/`verify_resume.txt`.
Report: `d45_browser/browser_report.json`. Fresh final-installed-server visual pass:
`d45_browser/settled_results.txt`, `d45_browser/final_views/browser_report.json`.
Earlier harness attempts raced source revisions/generated IDs or reused a stale
observation across reload. The production guards rejected those operations. The
harness now waits for authoritative completion; no guard was relaxed. An additional
settled-view attempt could not JSON-serialize Plotly internals; its subsequent
primitive-only diagnostic pass completed. All failure logs are retained.

The historical “before” images below are prior real UI evidence, with different
fixture content; this is a visual comparison, not a metric/trajectory comparison.

| Tab | Historical before, 1920×1080 | Verified after, 1920×1080 |
|---|---|---|
| Lab | [D4.4 Lab](../../results/external_influences/influences_1920x1080.png) | [D4.5 Lab](../../results/dashboard_refinement/d45_browser/lab_1920x1080.png) |
| Studio | [Earlier Studio](../../results/dynamics_studio_redesign/studio_1920x1080.png) | [D4.5 Studio](../../results/dashboard_refinement/d45_browser/final_views/studio_1920x1080.png) |
| Results | [Earlier Results](../../results/experiment_lab_simplification/results_1920x1080.png) | [D4.5 Results](../../results/dashboard_refinement/d45_browser/final_views/results_1920x1080.png) |

| Size | Lab | Studio | Results |
|---|---|---|---|
| 1600×900 | [Screenshot](../../results/dashboard_refinement/d45_browser/lab_1600x900.png) | [Screenshot](../../results/dashboard_refinement/d45_browser/final_views/studio_1600x900.png) | [Screenshot](../../results/dashboard_refinement/d45_browser/final_views/results_1600x900.png) |
| 1650×790 | [Screenshot](../../results/dashboard_refinement/d45_browser/lab_1650x790.png) | [Screenshot](../../results/dashboard_refinement/d45_browser/final_views/studio_1650x790.png) | [Screenshot](../../results/dashboard_refinement/d45_browser/final_views/results_1650x790.png) |

[Mapping accessible at 790 px](../../results/dashboard_refinement/d45_browser/final_views/studio_bottom_1650x790.png),
[actual rejected checkbox change](../../results/dashboard_refinement/d45_browser/robot_response_rejected.png).

Runtime Sessions: ON `fe9292fb-fb53-4d9d-82a9-5a0512784917`, OFF
`8a02332a-7da1-43d1-af49-5c270ce8ac17`, repeated ON
`c335703f-b336-4128-8ea0-3fd7c452f182`. Browser comparison Session
`154bdaa0-b27b-4752-89d7-64e576b794ba` references only the existing historical Runs.

Final installed-source audit (`d45_installed_audit.json`) confirms all eight modified
production Python modules and the CSS asset exactly match the installed copies used
for the final process. Final static and whitespace checks are empty/clean
(`d45_static_complete.txt`, `d45_diff_check_final.txt`). The resumed final process
audit found **no owned runtime survivors**, and ports **8060/8061 were not listening**
(`d45_cleanup.json`); the verification server had already exited before this audit.

`analyze_d45.py` is a read-only verifier for the named evidence Runs above. Fresh
output labels use the authoritative Runner's own saved metrics/time series; the
analysis helper is not a replacement general-purpose metrics engine.

## Independent verdicts and remaining limits

| Requirement | Verdict | Basis |
|---|---|---|
| Simplified model selection | **PASS** | Seven primary selectors; simplified initial and Studio-refresh paths; unit/callback/browser checks. |
| Preflight and inheritance integrity | **PASS** | Real complete preflight, immutable save/reload, explicit/inherited/frozen-pin refresh tests; incompatible deployment blocked. |
| Robot Response runtime reliability | **CONDITIONAL** | Demonstrated and corrected receiver queue loss; two actual ON completions with continuous state and zero invalid control. Exact attribution of the earliest unobserved upstream gap remains incomplete. |
| Robot Response checkbox consistency | **PASS** | Transactional rejection and redraw; actual browser rejection plus exception/preflight callback tests. |
| Dynamics Studio layout | **PASS** | Requested sentence removed; six-state/seven-coefficient authoring; MathJax and screenshots at all three sizes. Long drafts use the documented workspace scroll. |
| Physical Output Mapping preservation | **PASS** | Closed native disclosure with mounted values; toggle/preview/browser and exact owner save/reload tests. |
| Results comparison simplification | **PASS** | Five columns, no ranking UI; eight displayed numeric cells independently match immutable recorded metrics. |
| Historical compatibility | **PASS** | Real retained Runs render; comparison APIs untouched; 20,087 inventoried historical files unchanged within the stated audit scope. |

To close the runtime attribution limitation, capture emitted broadcaster stamps and
receiver arrivals from the **first** publication under the failing startup/scheduling
condition. Correlate any remaining source gaps/ahead-of-clock rejection with clock
callbacks before further remediation. Do not replace missing context with future or
stale data, enlarge freshness tolerances or alter the response equation.

This validation does not establish moving-base/actuated Robot Response safety or
tracking-quality acceptance. It does not claim GUI Gazebo rendering reliability,
controlled RTF improvement, exhaustive scheduling coverage, or deployability of a
pre-existing historical custom-fixture revision whose temporary resources are gone.
Authoring operations on this Catalog can take tens of seconds (an observed load
18.5 s plus card construction 28.1 s); no unrelated caching/architecture refactor
was attempted. Backend scientific ranking remains available through its existing
API despite removal from this dashboard presentation.

**Overall D4.5: CONDITIONAL**, due to the explicitly bounded runtime root-cause
attribution. Stop here; no subsequent milestone, commit or push.
