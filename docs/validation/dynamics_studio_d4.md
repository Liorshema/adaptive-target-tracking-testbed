# Dynamics Studio — D4

## Design and implementation plan

The existing dashboard gains exactly three mounted workspaces: Experiment Lab,
Dynamics Studio, Results & History. Studio owns only browser-local structured
forms, presentation and explicit actions. D4.0 remains the composition/library
owner and D3/RK4 remains the sole numerical implementation. A bounded model-only
preview belongs to the mathematical owner and is exposed through DynamicsLibrary.
No scored Run or environment field is created by preview.

The Studio uses a compact library/sidebar, bounded structured editor and a preview
panel. State, parameter, input and intermediate rows are editable tables; equations
are text. A selected contribution exposes its own parameters and namespace.
Switching templates/models requires confirmation for a dirty draft; navigation
between main tabs preserves drafts. Save delegates to immutable library revisions
with optimistic concurrency. Experiment Lab applies a pinned revision through
`draft_with_dynamics`, then uses its existing validation/save flow.

This follows the existing Dash callbacks and data-only template approach. A new
UI server/store or a second parser/solver would duplicate existing responsibilities.
Raw YAML is not an editor. Read-only canonical diagnostics remain under Advanced.

- [x] Test and implement bounded owner preview using D3 and RK4 (initial conditions,
  full state order, explicit fixed inputs, finite domains, resource limits).
- [x] Test and implement structured form conversion and detached library workflow,
  retaining constraints, optional declarations and source identity.
- [x] Add Studio layout/callbacks, dirty-switch confirmation, templates, interactions,
  validation, preview, immutable save/revise/rename and capability inspection.
- [x] Add pinned library selection to Experiment Lab without bypassing preflight.
- [x] Run focused D3/D4.0/dashboard tests, affected builds, installed Chromium
  workflows and layout checks at 1920×1080, 1600×900, 1650×790.
- [x] Record actual evidence, exact changes, limitations and independent verdicts.

Review focus: stale saves preserve drafts; old revisions never overwrite newer
ones; invalid table values remain editable; optional declarations are not silently
lost; invalid numerical stages and unsupported deployment remain distinct errors.

No physics, controller, safety, scoring or historical results changes. No commit
or push. No D4.1 output mapping or D4.2 comparison extension.

## Implementation and ownership

Exactly three main tabs are mounted in the existing Dash app, in this order:
**Experiment Lab → Dynamics Studio → Results & History**. Switching workspaces
changes visibility and dispatches Plotly resize; it does not recreate drafts,
Jobs, Sessions or Results. The existing CSS breakpoint permits document scrolling
on smaller screens, while the three required desktop sizes use bounded editors.

| Responsibility | Implementation / existing owner |
| --- | --- |
| Structured form projection, numeric cell reading, detached source switching | `robot_dashboard.dynamics_forms` |
| Studio layout, explicit actions, presentation and Plotly figures | `robot_dashboard.dynamics_studio` |
| Expression security, units, shapes, domains and capabilities | Existing D3 `EquationDynamics` |
| Contribution compilation and deterministic source-symbol map | Existing D4.0 `compile_definition` |
| Immutable library CRUD/CAS and persistence | Existing `backend.dynamics` / `DynamicsLibrary` |
| Numerical preview orchestration | `robot_simulation.dynamics.motion_preview`, existing D3 + `RK4Integrator` |
| Backend preview boundary | `DynamicsLibrary.preview()` delegates to the mathematical owner |
| Experiment preparation and immutable storage | Existing `draft_with_dynamics`, Author and Catalog |
| Environment fields, live execution, Results, safety and ranking | Existing owners, unchanged |

The UI uses the actual `templates`, `validate`, `list`, `get`, `create`, `revise`,
`rename`, `revisions`, `capabilities` and `resolve` APIs. No new server, parser,
solver, model registry, process owner, database or results store was added.
The existing optional Dash 3 installation supplies the structured DataTables;
no UI dependency was added. There is no Dynamics deletion action.

## User workflow and structured editor

1. Choose a template and **New from template**, or select a saved model and an
   explicit revision, then **Open revision**. Open is disabled until the revision
   is available; a missing pin is also rejected at the form boundary. Merely
   selecting a dropdown value or refreshing the list never loads another model.
2. Edit the model ID/frame and ordered state rows. Individual cells contain
   scalar values or flat arrays, e.g. `0.2`, `[1,0,1]`, `[]`, `[3]`; no YAML or
   complete JSON document is edited. General mode has one derivative per named
   state, optional control/context declarations and the existing adapter selector.
   Parameter rows expose shape, value, unit and ≥/≤/>/< bounds; rows can be added
   or deleted. Intermediates are a compact name/expression table.
3. In composed mode, select the Base or an Interaction to edit its own identity,
   expression, parameters, intermediate/context declarations and output/frame.
   Add a template contribution or custom expression; enable/disable, remove or
   reorder interactions. Source identity and D4.0's map remain visible in Advanced.
   No custom model requires a Python implementation or registration change.
4. The additional-damping checkbox explicitly sets a newly added influence's
   damping to zero by default, leaving the base unchanged. Its resulting value
   remains visible/editable. Unchecking preserves the template's damping; no
   drag contribution is silently inferred or removed.
5. **Validate** delegates to D4.0/D3 and displays distinct mathematical, numerical
   preview and live deployment statuses. Invalid cells stay mounted and editable;
   owner failures appear in the notice and Advanced diagnostics. Save validates
   again and never mutates the library after a validation failure.
6. **Save model** creates an identity or appends a revision using the selected
   revision as the CAS expectation. Old revisions remain read-only in storage;
   detached edits from an old revision cannot overwrite a newer one. **Rename**
   affects only the label, with both old label and revision guards. Changing a
   label and pressing revision Save is rejected with an instruction to use Rename.
7. Dirty source changes use the existing styled Dash dialog pattern: **Keep
   editing** preserves the draft; **Discard edits & open** loads the pinned source.
   Navigating among main tabs preserves drafts without a discard prompt. Browser
   refresh/closure loses unsaved memory-only drafts, as before.

Saving preserves the selected contribution, avoiding a mismatch between an
interaction editor and the underlying Base. The registered-dynamics source is
also preserved: applying Registered without a library pin is a no-op; applying a
library revision retains a detached snapshot of previous registered target edits;
switching back restores them. Without that snapshot it uses the inherited base.
The full candidate preflight passes before replacing the current Lab draft.

## Numerical preview contract

`Preview Motion` evaluates the complete validated model from its declared initial
state through the existing D3 derivative and RK4. The UI displays genuine sampled
position in 3D (`aspectmode=data`) or 2D XY (1:1 axis scale), with native zoom/reset.
Without an explicit supported position declaration it displays named scalar/vector
state-versus-time traces, retaining full state order and units in the legend.
No Cartesian trajectory is invented for a general ODE.

All context/control inputs are explicit **fixed preview assumptions**. The known
tracking-position input starts visibly at `[0,0,0]` m; other declared numeric
inputs require a supplied fixed value. This supports fixed numerical inputs, not
new live robot/sensor bindings. Undeclared inputs and missing fixed assumptions
fail. Existing live deployment preflight still rejects unsupported bindings.

Every preview is labeled **Model-only preview — external environment not
included**. Environment/Disturbances shows the last resolved Lab environment
snapshot and directs configuration to Experiment Lab. No field is attached to
preview; composition still delegates actual external acceleration to the existing
Factory/CompositeAccelerationField exactly once during deployment.

Bounds: duration `(0,60]` s; step `(0,1]` s; at most 2,000 integration steps;
1,000,000 expression-cost×step units; 150,000 output scalars; a five-second
integration deadline checked between steps. Final steps stop exactly at the
requested duration; roundoff cannot append a zero-length step. Tiny finite steps
that overflow the step-count ratio are rejected as a resource error. Nonfinite or
invalid-domain stages report the failing step and its start time. No adaptive
solver, changed step, interpolated data, Run ID, score or safety verdict is created.
This is not an integrability/stability proof or closed-loop prediction.

## Exact pinned Experiment Lab integration

**Use in Experiment** selects the saved model/revision and opens Experiment Lab.
The user loads an experiment draft and explicitly **Applies** the source.
`draft_with_dynamics()` freezes the canonical equations/schema/parameters and
library ID, revision, content/model hashes and source-symbol map. The unchanged
Author preflight checks the complete experiment before normal Validate/Save.
Eight-state/general models may be mathematically valid and previewable while
remaining unavailable to the existing six-state live adapter; no states are dropped.

Actual Chromium found a transport issue missed by Python-only API tests:
JavaScript reserializes `1.0` as `1`. Python numerical equality still holds, but
D4.0's canonical JSON hash differs, so the existing owner correctly rejected the
pin. The dashboard now retains the canonical model as read-only JSON text in the
browser draft and restores its original numeric representations before Author
validation. An exact structural/numerical comparison rejects genuine changes,
including boolean-for-number substitutions; no tolerance or hash rule is relaxed.
Owner pin validation still runs on the restored complete model. This is transient
presentation state, not another persisted model authority or mutable-latest lookup.

## Actual persistence and experiment evidence

Browser actions created two independent models through the real dashboard:

| Model | First revision SHA-256 |
| --- | --- |
| `DYN-UI-SPIRAL-1793690939` | `251e5a433670e6fe92eb2fe5a69ce83cedd6a54f3f68cb6e59f3804e17483777` |
| `DYN-UI-TURN-1793690939` | `2044a7e9c400314c239309cc1ecfb21f5b7441e47eb43690d70f42f0266f4b55` |

The first used edited Spiral parameters/equations. The browser added saturated
Fight/Flight with explicit zero additional damping, previewed enabled and disabled
interaction states, saved revisions, renamed the label and reopened revision 1.
Revision 1 retains damping **0.14 s⁻¹**. The actual selected-interaction save bug
was reproduced and fixed with a regression test before continuing.

The real Lab workflow saved **`EXP-UI-MOTION-1793690939`, revision 1**, hash
`d98dc675c9a68518258ff2ab8be7e4a2b1cc151a6361a2b70c4da3b8a4d332ba`, through
normal Validate/Save. It pins Spiral model revision **3**, content hash
`1fdecb7b474a6b86b7d5b32a7afaebca02f7c6410f6f189c775ca0fd9918b6c9`, canonical
model hash `a3bb9d7d1ca3c7c50ceac7b2df8f7fd2fe5f94f33cdd3b1e026951204ef3498d`.
The library subsequently advanced to revision **4** through the browser. A read-only
installed API check from `/tmp` confirms the saved Experiment is byte-for-byte
equal as a record to its retained post-save snapshot, its frozen model/provenance
still match revision 3, and revision 4 has a different canonical hash.

Evidence is isolated under `results/dynamics_studio/`:

- `workspace/dynamics_library.yaml`, `workspace/catalog.yaml`: actual test workspace
  authorities; historical user workspaces were not retargeted.
- `browser_workflow.py`, `browser_finish.py`, `browser_layout.py`, corresponding
  logs and `browser_report.json`: real installed Chromium interactions. Failed
  diagnostic attempts are retained separately; final successful portions resumed
  from existing saved models rather than rewriting their history.
- `saved_experiment.json`, `saved_verification.json` / `.txt`, `verify_saved.py`:
  read-only exports/assertions, not an additional writable results/model authority.
- `recorded_session.json`: Session **`822372b3-28e9-4efc-9449-c86302941758`** references
  existing historical Runs only. Results screenshots show the actual recorded Run
  `d69bdbd0-49e4-4237-bb1d-81a12d172de4`; its metrics are not D4 preview metrics.

No D4 scored Run or physical tracking/safety acceptance is claimed.

## Real-browser layout acceptance

Installed Chromium rendered at default **100% zoom**, device scale factor **1**.
For every main tab at each size, `documentElement.scrollWidth/scrollHeight` equal
`innerWidth/innerHeight`: no page-level horizontal or vertical scrolling.
Results retains exactly four KPI cards and four panels with equal measured width
and height. The 1650×790 Results panels are each **801×223 px**.

| Size | Experiment Lab | Dynamics Studio | Results & History |
| --- | --- | --- | --- |
| 1920×1080 | [Screenshot](../../results/dynamics_studio/lab_1920x1080.png) | [Screenshot](../../results/dynamics_studio/studio_1920x1080.png) | [Screenshot](../../results/dynamics_studio/results_1920x1080.png) |
| 1600×900 | [Screenshot](../../results/dynamics_studio/lab_1600x900.png) | [Screenshot](../../results/dynamics_studio/studio_1600x900.png) | [Screenshot](../../results/dynamics_studio/results_1600x900.png) |
| 1650×790 | [Screenshot](../../results/dynamics_studio/lab_1650x790.png) | [Screenshot](../../results/dynamics_studio/studio_1650x790.png) | [Screenshot](../../results/dynamics_studio/results_1650x790.png) |

Additional actual screenshots:
[new editable model](../../results/dynamics_studio/new_editor_1920x1080.png),
[composed model with fixed assumptions](../../results/dynamics_studio/composed_editor_final_1920x1080.png),
[2D preview](../../results/dynamics_studio/preview_2d_1920x1080.png),
[general-state preview](../../results/dynamics_studio/state_space_preview_1920x1080.png),
[malicious-expression rejection](../../results/dynamics_studio/validation_failure_1920x1080.png),
[pinned revision in Lab](../../results/dynamics_studio/lab_saved_revision_1920x1080.png).

Images were visually inspected for hierarchy, readability, bounded scrolling,
spatial aspect ratio and retained Results treatment. The final browser report has
**zero page errors**. Real inputs, parameter cells, saves, reopening, revisions,
rename, enable/disable, fixed context, preview, selection refresh, dirty-draft
cancel/discard and cross-tab draft preservation were exercised. Session comparisons
remain unranked when their existing backend says ranking is unavailable.

## Tests, build, review and preservation

| Verification | Actual result | Evidence |
| --- | --- | --- |
| Dashboard (except Builder), D4.0 compiler/templates/library/integration, new numerical preview | **167 passed**, 315.54 s, no skips | `regression_final.txt` |
| Final Builder HTTP save/conflict, duration/revision and three-tab regression | **3 passed**, 214.42 s | `builder_final.txt` |
| Recorded Results / trajectory regressions, genuine retained completed/failed Runs | **58 passed**, 14.21 s; also covered by the final combined suite | `recorded_regression.txt` |
| Post-transport-fix focused owner preflight and registered-source restoration | **2 passed**, 6.86 s; also covered by combined suite | `transport_source_green.txt` |
| Affected installed packages | **3 built**: robot_simulation, robot_experiments, robot_dashboard; 1 min 47 s | `build_final.txt` |
| New production-module flake8 / scoped whitespace | **PASS** | `lint_final.txt`, `git diff --check` |
| Installed public API from `/tmp`, frozen experiment after library edit | **PASS** | `saved_verification.json` / `.txt` |
| Browser / cleanup | **PASS**; zero page errors; owned dashboard stopped and port 8060 released | `browser_report.json`, `cleanup.txt` |

The final disjoint functional checks total **170 passed**. Earlier full Builder
and development red/green evidence is also retained, not added to that total.
The initial combined suite exposed native ConfirmDialog incompatibility with the
existing management UI requirement; Studio now uses styled Dash dialogs and the
unchanged management regression passes. An earlier run skipped optional recorded
fixtures until their explicit historical paths were supplied; the final suite has
no skips. No acceptance tolerance was changed.

Warnings are Dash 3's builtin DataTable future-major-version deprecation and the
existing colcon underlay-override notice. Dash remains pinned below 4 by the
existing package; no new dependency or broad lint cleanup was attempted.

A read-only independent review found that Registered Apply could discard current
registered target edits. The focused test failed before the fix and now passes:
selection is unchanged when already registered and the previous target draft is
restored when leaving a library pin. Browser-derived fixes additionally cover
selected-contribution state after save, explicit revision loading, canonical
number transport and styled discard dialogs. Unit/HTTP negatives cover stale
writes, malicious equations, invalid numeric/table inputs, corruption, missing
preview inputs, domain failures, unsupported deployment and resource limits.
Existing D4.0 negatives retain frame/unit/vector/namespace/solver security checks.

A manifest checked **19,671 pre-existing files**, including **18,835 historical
result/evidence files**. None of those historical files changed or disappeared.
The scan was sequential; the known focused `DynamicsLibrary.preview` addition
landed while it was still hashing historical files and is listed explicitly below.
D3 numerical owners, D4.0 compiler/templates, installed owner defaults, physics,
controllers, PID/limits, readiness, lifecycle and comparison policies were not
changed. No file was deleted or relocated. Pre-existing untracked D4.0 work and
user reference documents were retained. New generated evidence is isolated under
`results/dynamics_studio/`.

## Exact D4 changed files

New permanent files:

1. `src/robot_dashboard/robot_dashboard/dynamics_forms.py` — structured detached
   forms, explicit library operations, source restoration and canonical transport.
2. `src/robot_dashboard/robot_dashboard/dynamics_studio.py` — Studio and Lab source
   presentation/callbacks, model-only plots and revision/discard controls.
3. `src/robot_simulation/robot_simulation/dynamics/motion_preview.py` — bounded
   orchestration over the existing D3/RK4 implementations.
4. `src/robot_dashboard/test/test_dynamics_studio.py` — form, library, HTTP,
   provenance/transport, registered-source and regression tests.
5. `src/robot_simulation/test/test_motion_preview.py` — initial/order/input/domain
   and resource-limit numerical preview checks.
6. `docs/validation/dynamics_studio_d4.md` — this design, implementation and evidence.

Modified permanent files:

7. `src/robot_dashboard/robot_dashboard/app.py` — third mounted tab, pinned library
   card and source application through the existing Builder.
8. `src/robot_dashboard/robot_dashboard/assets/dashboard.css` — bounded Studio grid
   and existing-theme editor/preview styling.
9. `src/robot_dashboard/test/test_builder.py` — third-tab assertion and new callback
   input metadata; existing functional assertions retained.
10. `src/robot_experiments/robot_experiments/dynamics_library.py` — only the focused
    `preview()` owner delegation added to the previously completed D4.0 library.
11. `docs/dashboard.md` — Studio workflow, preview bounds and source/revision usage.

Other Git changes belong to the already completed D4.0 working tree; D4 did not
recreate or revert those implementations. No commit or push.

## Reproduction

Launch the actual saved verification workspace on the verified released port:

```bash
cd /home/user/ros2_ws
source install/setup.bash
ros2 run robot_dashboard dashboard \
  --data-dir "$PWD/results/dynamics_studio/workspace" --port 8060 \
  --result-root "$PWD/results/recorder_diagnostics_d29b/live" \
  --result-root "$PWD/results/scientific_comparison/workspace/runs"
```

Open **http://127.0.0.1:8060**. Use the normal existing `--data-dir` instead to
work in a personal dashboard workspace; no migration or replacement Catalog is
required. Preview does not start ROS/Gazebo, require actuation or create a Run.

Focused checks and installed immutable-artifact verification:

```bash
source install/setup.bash
export ROBOT_DASHBOARD_RECORDED_RUN="$PWD/results/recorder_diagnostics_d29b/live/nominal_20261102T143918_895978Z_ol7_mqis/runs/imm_seed42"
export ROBOT_DASHBOARD_FAILED_RUN="$PWD/results/scientific_comparison/workspace/runs/nominal_20261102T111309_465057Z_mk0nn8f3/runs/imm_seed42"
build/dashboard_venv/bin/python -m pytest \
  src/robot_dashboard/test --ignore=src/robot_dashboard/test/test_builder.py \
  src/robot_simulation/test/test_motion_preview.py \
  src/robot_simulation/test/test_motion_composition.py \
  src/robot_simulation/test/test_declarative_motion_templates.py \
  src/robot_experiments/test/test_dynamics_library.py \
  src/robot_experiments/test/test_dynamics_library_integration.py -q
build/dashboard_venv/bin/python results/dynamics_studio/verify_saved.py
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_simulation robot_experiments robot_dashboard --symlink-install
```

The retained browser scripts document actual input sequences and screenshots.
`browser_workflow.py` creates independent test identities; continuation scripts
resume the retained workspace and should not be used to overwrite old evidence.

## Verdicts and remaining boundaries

| Area | Verdict | Demonstrated scope |
| --- | --- | --- |
| Dynamics Studio UI | **PASS** | Exactly three tabs; structured editable models, templates, library operations and bounded desktop layout. |
| Library create/revise workflow | **PASS** | Two real browser-created models, immutable history, rename, CAS conflicts and read-only old revision reopening. |
| Mathematical validation and preview | **PASS** | Existing D3/RK4, genuine initial/order/units, explicit fixed inputs, real spatial/state plots and domain/resource errors. |
| Composition workflow | **PASS** | Edited Spiral + saturated radial interaction, explicit drag, enabled/disabled preview and owner source-symbol mapping. |
| Experiment Lab integration | **PASS** | Real Author validation/save with revision 3 pin, preserved canonical hash across JavaScript, unchanged saved Experiment after revision 4. |
| Browser UX / regression | **PASS** | Installed Chromium inputs and screenshots; all three required viewports, four KPIs/equal Results panels, retained real Session references and zero page errors. |

These are D4 software/workflow verdicts, not new tracking-quality or physical
safety acceptance. No new Gazebo Run was required or launched. The preview uses
fixed inputs and excludes experiment environment fields; it cannot predict a
moving robot or prove numerical stability. Unsaved work remains browser-memory
only. General non-Cartesian/eight-state deployment, new output adapters and
user-defined-dynamics comparison policies remain **D4.1 / D4.2**, not implemented.
No stochastic mechanism or Dynamics deletion was added. Work stops at D4.
