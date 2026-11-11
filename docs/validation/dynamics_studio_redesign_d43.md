# D4.3 — Dynamics Studio redesign and unified model selection

## Scope and independent verdicts

Completed the existing dashboard redesign and model-selection integration. The
reference is the user-supplied `docs/Studio_reference_screenshot.png` (1536×864),
not a data source. No new application, registry, REST API, solver or experiment
engine. No Gazebo, controller, readiness, Recorder, actuator, safety or physics
changes, commits or pushes. Arbitrary mapped live deployment remains outside scope.

| Requirement | Verdict | Evidence / boundary |
|---|---|---|
| UI fidelity to reference | PASS | Toolbar, three left panels, math/preview right panels; inspected actual Chromium screenshots against reference. Existing branding/tabs/theme retained; not pixel-perfect. |
| Generic equation editing | PASS | Scalar, arbitrary `a/b/c`, vector plus extra states, nonlinear and explicitly declared robot-input models. Same tables; no motion-specific runtime UI branches. |
| Mathematical typesetting | PASS | Actual draft → restricted D3 AST → generated TeX → bundled Dash MathJax SVG; no raw user TeX, eval, SymPy or new dependency. Exact AST bindings avoid name-prefix substitutions. |
| Physical output mapping | CONDITIONAL | Genuine validated position/analytic velocity for numerical preview. Live adapter remains the existing ordered six-state Cartesian contract; no generic extra-state or orientation adapter. |
| Numerical preview | PASS | Existing RK4 and resource bounds, full internal state, genuine 3D/XY or state/time traces, explicit fixed inputs and no external field. |
| Validate & Save | PASS | Generated IDs, owner validation, atomic immutable revisions/CAS; label-only change preserves math hash and version. Browser negative/stale cases verified. |
| Immediate Lab availability | PASS | Studio save immediately refreshes the existing Model selection options. Same workspace persists across browser/dashboard restart. |
| One-selection pinned integration | PASS | One selection freezes the exact canonical model, revision, mapping and hashes; full Experiment Validate/Save remains separate. Unsupported models remain visible and blocked. |
| Existing experiment / historical compatibility | PASS | Older pinned experiment reloads revision 1 after library revision 2; frozen provenance works without mutable latest; registered edits and composition sources preserved. Historical hash audit unchanged. |
| Browser layout / regression | PASS | All three tabs at 1920×1080, 1600×900, 1650×790, scale 1; no page scroll. Real recorded Results retain four equal panels and four KPIs. |

## Before and after

Previously Studio used a permanent library sidebar, technical IDs and revisions,
a tall technical form and separate Validate/Save/Use actions. Lab then required
source radio, library, exact revision and Apply. The default now has one toolbar:
Model Name, Saved Models, New Model, Preview, Validate & Save. The two columns
contain State-Space Equations / Parameters / Physical Output Mapping and
Mathematical Representation / Motion Preview.

Units/shapes, bounds, inputs, intermediates and adapter declarations remain under
Advanced. A blank general state-space system is the primary starting point.
Optional D4.0 templates and exact history remain in a compact library section.
Existing compositions retain editable original contributions in Advanced; no
flattening or loss of symbol maps. Unused parameters are identified after owner
validation; no unused `R` was copied from the reference.

The reference and actual screenshots were inspected side by side. Layout,
proportions, dark panels, table hierarchy and toolbar follow it. Differences are
intentional: Adaptive Tracking header, existing blue Save accent, editable Dash
cells, native Plotly controls, faithful elementwise `*` notation and explicit
preview-only capability text. Reference decorative badges, unused parameters and
invented trajectories were not adopted. At the shortest viewport, final inspection
caught clipped row actions / a tall velocity vector; compact rows and inline
verified velocity notation corrected those. Advanced popovers were also anchored
above their own toggles so users can close them.

## Mathematical ownership and contract

The dashboard edits the existing D3 state list, initial values, derivative
expressions, parameters, context/control declarations and optional output:

```yaml
outputs:
  schema_version: 1
  frame: world
  position: '[a,b,c]'
```

`robot_simulation.dynamics.position_output.PositionOutput` validates that
`p_world = h(t,s;theta)` is a three-vector in metres in the explicit matching world
frame. It derives `v_world = Dh*f + h_t` using the existing bounded AST and typed
D3 compiler. State-rate bindings call the actual compiled derivative nodes. No
second numerical evaluator, finite-difference production velocity or solver.
Supported rules include sums, products, quotients, constant powers, scalar/vector
construction, indexing, sin/cos/tanh/exp/sqrt/norm/dot/cross. Radian trig arguments
use a typed constant `1 rad` to preserve strict D3 angle dimensions. Both rate and
unit bindings are exposed in diagnostics and formatted by exact AST name.

Generated expressions must pass existing source/node/unit bounds and total model
cost. Derivation also has a 32-level dependency limit and 4096-work expansion
budget, before recursive construction can explode. Preview retains its existing
2000-step, sample/cost and five-second wall budgets, including mapped output work.
Nonfinite values and singular derivative domains fail explicitly. Variable
exponent differentiation and mappings directly dependent on context/control inputs
are explicitly rejected; declared inputs may still influence `f`. No implicit
frame conversion or orientation/angular-velocity derivation.

`motion_preview.py` integrates the full state with the existing RK4, then evaluates
position/velocity at actual retained samples. Extra states are never discarded or
reinterpreted as a six-state target. Existing mapping-free definitions and live
Cartesian behavior retain their original equations. A supplied explicit live
mapping must equal the existing position state; it cannot redefine physical
tracking behind the adapter's back. Three-state and extra-state systems remain
mathematically valid / preview-only and cannot pass full Author live preflight.

## Saving, selection and immutable provenance

Technical model/library IDs are generated once per new draft. The sole storage is
workspace-owned `dynamics_library.yaml`; no installed dynamics defaults are edited.
The existing library revision API accepts an optional display-name update with
revision and expected-label CAS under the same lock/write. Invalid math or stale
metadata leaves history intact. A label-only save uses existing rename semantics
and does not append changed mathematical content.

Target Dynamics → Model selection now combines registered implementations and
library entries. Library option values contain an actual immutable revision,
never `latest`. Saving refreshes these options immediately. A single selection
applies the exact canonical model and source metadata and runs full Author
validation. Preview-only entries remain visible; unsupported live selections are
kept as an invalid unsaved draft, with Save blocked. Advanced exact older-version
selection is optional. Switching back restores the previous registered target
selection and parameter overrides.

The existing Author's frozen canonical model remains authoritative. When loading
an inherited saved experiment, internally consistent library provenance is
recovered from its existing frozen Author description and checked against the
actual model hash. No lookup of mutable library latest is needed. The UI retains
canonical JSON text alongside its structural value: JavaScript `1.0 → 1` transport
is restored only after exact structure/numeric equality (booleans distinct), then
normal owner hash validation. No tolerance or hash checks were weakened.

## Actual browser and numerical evidence

Generated evidence lives in `results/dynamics_studio_redesign/`; historical
artifacts are referenced, not duplicated. Installed Chromium used scale 1, actual
Dash callbacks, library, Author and Catalog. No injected UI model state or fake
preview samples. Browser scripts and failed-attempt diagnostics are retained.
Some intermediate acceptance scripts required corrected selectors / prerequisites:
hidden duplicate labels, clearing a selected cell, and selecting a recorded Run
before asserting Results panels. These were test issues, not bypassed production
validation. Final screenshots and reports supersede those attempts.

Verified browser flows:

- Blank scalar `x/y/z`, omega `0.5 1/s`, `[x,y,z]` output: typeset, preview, save,
  immediate Lab availability, one selection, honest live-deployment rejection.
- Generic six-state Cartesian model with explicit `p` output: single selection,
  full Experiment Validate, immutable Save; model revision 2 then original
  experiment reload at revision 1, including after dashboard restart.
- Vector `a[3]` plus `extra[2]`: all five coordinates and vector declarations
  survive integration/save. Arbitrary scalar `a/b/c` is also tested by the owner.
- Dimensionless Lorenz states through identical controls, explicit `0.1 m` scale
  for spatial output, actual typesetting and bounded RK4 preview.
- Declared robot-position context: missing fixed assumption rejects preview;
  explicit fixed `[0,0,0] m` succeeds. No inferred context or fake live robot.
- Unsafe expression, missing units, invalid mapped units, dirty replacement/cancel,
  tab draft preservation, label-only rename/hash, concurrent stale-save rejection.

`numerical_evidence.json` records actual owner previews of the browser-saved
models. Scalar 5 s / 0.02 s: 251 samples, maximum Cartesian analytic error
`1.659e-10 m`, maximum radius error `1.735e-12 m`, Z error `0 m`. Vector final full
state is approximately `[1.5,3,4.5,7,8]`; extra states remain unchanged. Lorenz
0.5 s / 0.01 s has 51 finite samples; final dimensionless state approximately
`[1.198565,-8.867139,32.454933]`, with metres only supplied by explicit scaling.
These are model-only numerical checks, not Gazebo or tracking-quality results.

Saved experiment: `EXP-MAPPED-MODEL-1793725630`, revision 1.
Pinned model: `DYN-CA4EF7717BAD4F069473B48CE0568B1E`, revision 1, with later revision
2 retained independently. Revision content SHA-256:
`3e2f6389a4286b9b1eb8a14641b40414fcdca91ff6a2ad243ddf65cbddcb6d37`.
Canonical model SHA-256:
`0cec252d90da2fac8108fefab8720689cc7fd465000c02525658012a1e83df90`.

Recorded Results regression uses original Run
`d69bdbd0-49e4-4237-bb1d-81a12d172de4` via Session
`f3e35f12-a009-47d6-98dc-64558595e683`. Only that Session reference is new; no scored
Run was created. The existing charts, four KPIs, forecast and comparison table
render actual immutable recorded telemetry. Completed execution, recorded safety
and performance remain separate; no ranking or acceptance was invented.

## Screenshots

All paths are under `results/dynamics_studio_redesign/`:

| Size | Studio | Experiment Lab | Recorded Results |
|---|---|---|---|
| 1920×1080 | [studio_1920x1080.png](../../results/dynamics_studio_redesign/studio_1920x1080.png) | [lab_1920x1080.png](../../results/dynamics_studio_redesign/lab_1920x1080.png) | [results_1920x1080.png](../../results/dynamics_studio_redesign/results_1920x1080.png) |
| 1600×900 | [studio_1600x900.png](../../results/dynamics_studio_redesign/studio_1600x900.png) | [lab_1600x900.png](../../results/dynamics_studio_redesign/lab_1600x900.png) | [results_1600x900.png](../../results/dynamics_studio_redesign/results_1600x900.png) |
| 1650×790 | [studio_1650x790.png](../../results/dynamics_studio_redesign/studio_1650x790.png) | [lab_1650x790.png](../../results/dynamics_studio_redesign/lab_1650x790.png) | [results_1650x790.png](../../results/dynamics_studio_redesign/results_1650x790.png) |

Additional vector, nonlinear and declared-input screenshots are under `examples/`.
`browser_report.json` includes viewport/document equality, scale, typeset math
count/errors, row-action geometry and four-panel dimensions.
`controls_complete.txt` additionally records the full velocity-row height and
its position inside the panel at every requested size. The supplied reference
is preserved. Smaller screens retain responsive page scrolling.

## Tests, review and build

- Combined dashboard, library/integration, equation-owner, composition, template,
  preview and mapping suite: **245 passed, 38 recording-dependent skipped**
  (`final_tests.txt`, 496 s). The skipped tests were subsequently exercised using
  actual existing completed/failed recordings: **58 passed** across their two
  files (`recorded_regression.txt`, 21.69 s; includes other tests in those files).
- Final exact-symbol/compact-presentation/owner mapping check: **30 passed**
  (`final_presentation.txt`). Final UI smoke **10 passed**; existing live-adapter
  deployment regression **5 passed** (`deployment_regression.txt`). No new functional failures.
- Earlier structured UI/Builder regression: **32 passed**; unified pinning,
  presentation and redesign tests: **11 passed**. RED/GREEN evidence retained
  for mapping, safe math, unified selection, radian derivative and exact-symbol
  substitution. Independent review reproduced and checked the derivative-work
  and radian fixes; owner/math recheck **28 passed** before the final display edge.
- Affected `robot_simulation`, `robot_experiments`, `robot_dashboard` symlink build:
  **3 packages finished**, 1 min 23 s (`build.txt`). Dashboard build repeated after
  final presentation changes (`final_dashboard_build.txt`). Existing colcon
  underlay-override warning retained; no physics/library sources redesigned.
- Focused production-file flake8 (`--max-line-length=99 --ignore=E203,W503`) and
  `git diff --check` pass. This is not a whole-repository lint-clean claim.
  Dash DataTable's existing future-deprecation warning remains; no replacement
  dependency or broad table refactor was introduced.

## Preservation and exact task-relative changed files

Task-start SHA-256 manifests captured 842 source/document files and 19,018 prior
experimental files. `historical_audit.json`: **19,018 checked, zero changed,
zero missing**. No source files deleted. Existing D4/D4.0 modifications and
untracked evidence remain. `source_audit.json` distinguishes these D4.3 edits from
work already present. User-supplied image / Zone.Identifier and ignored build
extension artifacts are not authored source changes.

Modified:

- `docs/dashboard.md`
- `src/robot_dashboard/robot_dashboard/app.py`
- `src/robot_dashboard/robot_dashboard/assets/dashboard.css`
- `src/robot_dashboard/robot_dashboard/dynamics_forms.py`
- `src/robot_dashboard/robot_dashboard/dynamics_studio.py`
- `src/robot_dashboard/test/test_builder.py`
- `src/robot_dashboard/test/test_dynamics_studio.py`
- `src/robot_experiments/robot_experiments/authoring.py`
- `src/robot_experiments/robot_experiments/dynamics_library.py`
- `src/robot_experiments/test/test_dynamics_library.py`
- `src/robot_simulation/robot_simulation/dynamics/equation_model.py`
- `src/robot_simulation/robot_simulation/dynamics/motion_composition.py`
- `src/robot_simulation/robot_simulation/dynamics/motion_preview.py`

Added:

- `docs/validation/dynamics_studio_redesign_d43.md`
- `src/robot_dashboard/robot_dashboard/mathematics.py`
- `src/robot_dashboard/test/test_mathematics.py`
- `src/robot_dashboard/test/test_studio_redesign.py`
- `src/robot_dashboard/test/test_unified_dynamics.py`
- `src/robot_simulation/robot_simulation/dynamics/position_output.py`
- `src/robot_simulation/test/test_position_output.py`

Removed behavior: old Lab source radio / exact-revision / Apply workflow, Studio
permanent technical sidebar and separate Validate/Use actions, obsolete CSS for
those layouts. Retained history, contribution editing, owner validation and safety.
Generated browser scripts/logs/manifests/screenshots are contained in the evidence
root, separate from production source and permanent documentation.

## Reproduction

```bash
cd /home/user/ros2_ws
source install/setup.bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir "$PWD/results/dynamics_studio_redesign/workspace" \
  --result-root "$PWD/results/recorder_diagnostics_d29b/live" \
  --result-root "$PWD/results/scientific_comparison/workspace/runs"
```

Open `http://127.0.0.1:8060`. Choose the saved Mapped model experiment, then Load
Draft to inspect its frozen version 1; Studio Saved Models opens current library
version 2. Select the recorded-result regression Session for actual Results.
The owned verification dashboard was closed after testing; port 8060 was verified
free (`cleanup.txt`). Actuation remains explicit opt-in; no Run is started by these
UI checks.

```bash
source install/setup.bash
build/dashboard_venv/bin/python -m pytest -q \
  src/robot_dashboard/test/test_dynamics_studio.py \
  src/robot_dashboard/test/test_unified_dynamics.py \
  src/robot_dashboard/test/test_studio_redesign.py \
  src/robot_dashboard/test/test_mathematics.py \
  src/robot_simulation/test/test_position_output.py \
  src/robot_simulation/test/test_motion_preview.py \
  src/robot_experiments/test/test_dynamics_library.py

export ROBOT_DASHBOARD_RECORDED_RUN="$PWD/results/recorder_diagnostics_d29b/live/nominal_20261102T143918_895978Z_ol7_mqis/runs/imm_seed42"
export ROBOT_DASHBOARD_FAILED_RUN="$PWD/results/scientific_comparison/workspace/runs/nominal_20261102T111309_465057Z_mk0nn8f3/runs/imm_seed42"
build/dashboard_venv/bin/python -m pytest -q \
  src/robot_dashboard/test/test_results.py src/robot_dashboard/test/test_trajectory.py

build/dashboard_venv/bin/python /usr/bin/colcon build --symlink-install \
  --packages-select robot_simulation robot_experiments robot_dashboard
```

`browser_views.py` repeats viewport checks against the evidence workspace;
`browser_workflow.py` and `browser_examples.py` retain actual authoring steps and
resume flags. Fresh authoring creates new immutable test entries. These are
validation helpers, not a production execution engine.

Remaining limitations: generic mapped live Gazebo deployment, orientation outputs
and arbitrary frame conversion are unimplemented; some derivative forms/domains
are rejected explicitly. Preview has fixed declared inputs and excludes external
fields. Large systems use bounded internal scrolling. No fresh Gazebo, physical
safety or tracking-quality acceptance is claimed. Stop at D4.3.
