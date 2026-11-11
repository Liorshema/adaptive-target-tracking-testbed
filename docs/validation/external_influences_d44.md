# D4.4 — External influences

Completed review: 2026-11-10. Overall verdict: **CONDITIONAL**.

Implementation, mathematics, immutable authoring and browser workflows are verified.
The ON Gazebo Run exposed an unresolved causal-context consumption stall; full
live response reliability and tracking acceptance are not claimed.

## Design and ownership

Use the existing D3 equation compiler, PositionOutput, SimulationFactory and RK4.
`robot_simulation` owns a versioned derivative-influence YAML profile and generic
composition into effective equations. Base model data stays unchanged; the
optional `influences` dynamics parameter contains complete detached snapshots.
Profile bindings explicitly name acceleration derivative targets. Support direct
world position mappings with scalar position/velocity states or two 3-vectors;
reject nonlinear/unsupported physical projections rather than inventing meaning.
Compile namespaced parameters/intermediates through D3's restricted AST and SI
checks. No post-step velocity correction or additional integrator.

`robot_experiments` freezes the influence snapshot through its existing parameter
Author/Catalog path, retaining model identity/revision/hash. Re-enabling a saved
influence uses its frozen profile; global YAML edits affect only newly opted-in
Scenarios. Explicit OFF disables only the experiment contribution, never base equations.
`robot_dashboard` adds just Robot Response and unavailable Environmental Disturbance
inside Target Dynamics. Its callback calls the backend owner; no formula or gain
editor. Environment selection and active-field rejection remain independent.

Live enabled response requires TF, causal tracking-frame history at each substep,
freshness through the substep end, world frame and finite samples. The tracking
frame (not base centre) is the configured robot position. Reuse provider/history,
clock-reset clearing, waiting behavior and unchanged Runner readiness. No synthetic
position fallback or estimator/controller modification.

Alternatives rejected: permanently editing a Library model would break ON/OFF
identity; applying acceleration after RK4 would make state and physical output
inconsistent. A dedicated Attractor subclass/second evaluator is unnecessary.

Ruling: no existing saved Attractor model was found in accessible workspaces.
Provide the requested six-scalar equations as a reusable owner template named
3D Attractor Dynamics, with explicitly example-valued base parameters; do not
modify or relabel any existing model. The user can save that template into the
existing Library. User's standing authorization removes per-change approval;
no commit/push or unrelated cleanup.

## Implementation plan

- [x] Capture source/history hashes and process/port inventory.
- [x] RED tests: formula bounds/direction/domains, independent integration,
  exact OFF and alternate scalar/vector bindings; implement generic owner profile
  compiler in `dynamics/derivative_influences.py`, extend EquationDynamics parameter
  transport, add owner YAML and Attractor template.
- [x] RED context/preflight tests; reuse causal TF history in simulation node and
  preserve optional physical-base/environment behavior, resets and waiting.
- [x] RED immutable Scenario tests; backend snapshot API and existing Author
  parameters, no Library writes; test global-profile changes and duplicates.
- [x] RED checkbox/layout/persistence tests; add compact controls to Target
  Dynamics, omit internal profile from generic parameter forms, preserve pins.
- [x] Build affected packages; run targeted D3/D4.1/D4.2 and legacy regressions.
- [x] Real Chromium at 1920×1080, 1600×900, 1650×790; genuine paired bounded
  Gazebo Runs with same Seed, initial conditions, no fields and actuation OFF.
- [x] Fresh review, history hash audit, exact files, actual results and eight
  independent verdicts. Preserve all failed evidence; clean owned processes.

## Configuration contract and mathematics

`robot_simulation/config/derivative_influences.yaml` is the authoring-default owner.
`profiles.robot_response` has schema_version, id, enabled, frame, output,
inputs, context, parameters, intermediates, expression and bindings. A model's
`influences` parameter stores complete detached profile snapshots (maximum four;
currently the dashboard exposes one). Each graph is compiled by the existing D3
restricted AST, shape, unit, cost and parameter validators; local symbols are
namespaced to avoid collisions with base equations. No Python `eval` or numerical
postprocessing is used.

The shipped graph computes `relative = target_position-tracking_position_world`,
`distance = norm(relative)`, then
`A_max*exp(-distance/L)*relative/sqrt(dot(relative,relative)+epsilon**2)`.
The exponential argument and smoothed direction are dimensionless, so the result
has units m/s². With positive finite A_max, L and epsilon, its norm is at most
A_max, decays at large distance and is exactly zero at coincidence. Defaults
are 0.3 m/s², 0.8 m and 0.05 m, each with exclusiveMinimum 0.0.

For the scalar Attractor model, `outputs.position: '[x,y,z]'`, position derivatives
are vx, vy and vz, and `bindings: {vx: 0, vy: 1, vz: 2}` injects acceleration
components into those velocity derivatives. A model with vector states `where`
and `rate`, `outputs.position: where` and `where: rate` uses
`bindings: {rate: [0,1,2]}`. Alternate scalar names are also accepted with explicit
matching bindings. Nonlinear, scaled or otherwise unsupported physical mappings
fail closed; the software never infers meaning from familiar state names.

Disabled profiles are validated but omitted from the effective equation graph.
The original base model is retained separately, and the ordinary D3 integrator
advances the complete internal state. The existing analytic PositionOutput maps
the integrated state and total derivative to physical position and velocity.
Mapped models continue rejecting any active incompatible environmental fields.

## Context and ownership boundaries

Enabled response preflight requires `tracking.source: tf`,
`tracking.pose_source: physical` and `tracking.reference_frame: world`. In the
existing bringup, physical tracking TF is published by `tracking_feedback_tf`
from Gazebo base ground-truth odometry and live arm joint feedback, using the
canonical robot kinematics and configured tracking-frame transform. It is the
tracking-frame origin, not the chassis centre, estimated target or a zero vector.

The node reuses TrackingFrameProvider's direct TF history. At each integration
substep it selects the newest source timestamp no later than that substep and
requires its validity through the substep end. Missing/stale context waits through
the existing bounded lookback machinery. Nonfinite or incompatible context is
rejected. Clock reset clears the history and internal state. Context selection
is zero-order hold within an RK4 step, not extrapolated fabricated robot motion.
The OFF path preserves the original integration graph and legacy context path.

The UI is a thin client of ExperimentBackend's `describe_robot_response` and
`draft_with_robot_response`. Author and Catalog own validation/materialization,
immutable revisions and provenance. The backend freezes the effective inherited
base model beside its profile to avoid validating an influence-only override
against unrelated registry defaults. Duplicate and re-enable preserve the saved
profile even if the authoring-default YAML changes. No Library revision is written
by a checkbox. Unsupported models show a disabled Robot Response with a reason.
Environmental Disturbance is visibly unavailable and cannot activate a field.

Ruling: the acceptance fixture uses a detached copy of the existing nominal
composition with physical tracking TF explicitly selected. The production
nominal defaults, physics, limits and readiness policy are unchanged. Validation
fixture resources are frozen with the existing Author mechanism so they survive
setup-process exit. Failed attempts are preserved under results/external_influences.

## Fresh review and recoverability

Fresh read-only review found no Critical/Important implementation findings.
Final: minor (deferred): if the backend rejects an ON click before returning a
candidate, the browser checkbox can retain the clicked value while the
stored draft remains OFF, until cards are redrawn. Error/preflight/save gates
remain active. Reset or reselect/load restores the visible snapshot; this is
presentation polish, not permission to run an invalid profile.

The review excluded ongoing browser/runtime acceptance and the final integrity
audit; those are verified separately below. An additional independently found
recovery case was reproduced RED then fixed: switching an explicit invalid ON
draft back OFF now disables its detached profile before ordinary OFF preflight.
It does not bypass validation. Owner/UI tests passed 4/4; recovery passed 1/1.
An initial 505/506 simulation run failed only the exact template-inventory set;
the legitimate new Attractor entry was added, with no numerical assertion changed.
Focused simulation checks then passed 80/80.

## Reproduction entry points

From `/home/user/ros2_ws`, build the affected packages using the existing optional
Dash environment, then source the workspace:

```bash
source install/setup.bash
build/dashboard_venv/bin/python /usr/bin/colcon build --packages-select robot_simulation robot_experiments robot_dashboard
source install/setup.bash
build/dashboard_venv/bin/python -c 'from robot_dashboard.cli import main; main()' --port 8060 --data-dir results/external_influences/workspace
```

Use the saved Attractor OFF/ON Scenarios in that isolated acceptance workspace.
The normal dashboard data directory remains unchanged. Select a saved Scenario,
Load configuration, inspect Robot Response, and explicitly opt in only if physical
actuation is desired in a separate experiment. This acceptance uses actuation OFF.
New compatible pinned models receive a detached OFF snapshot; saved profiles are
never replaced with current YAML defaults when reloaded or duplicated.

Focused tests (source modules, existing optional environment):

```bash
source install/setup.bash
export PYTHONPATH="$PWD/src/robot_simulation:$PWD/src/robot_experiments:$PWD/src/robot_dashboard:$PYTHONPATH"
build/dashboard_venv/bin/python -m pytest src/robot_simulation/test/test_derivative_influences.py src/robot_simulation/test/test_context_recovery.py src/robot_simulation/test/test_deployment_configuration.py src/robot_simulation/test/test_declarative_motion_templates.py -q
build/dashboard_venv/bin/python -m pytest src/robot_experiments/test/test_influence_authoring.py src/robot_dashboard/test/test_influence_ui.py -q
```

Acceptance scripts under `results/external_influences/` are generated validation
artifacts, not an alternative execution engine. `prepare_workspace.py` constructs
only an isolated fixture through the existing configuration/Library owners.
`browser_acceptance.py` drives real controls; `live_pair.py` uses Backend/Jobs/Runner
and starts a validation-only TF observer in each Runner-owned ROS domain.
`analyze_pair.py` reads existing recordings and computes independent diagnostics.
Preserve existing output before repeating a validation script; saved Runs themselves
remain immutable and new Runner invocations create new result directories.

## Limits

Only direct world position/velocity state pairs are supported for acceleration
injection. Nonlinear output mappings need an explicit mathematically justified
projection in a future task. The dashboard does not infer derivative bindings for
arbitrary Library models or expose an influence equation/gain editor. YAML is the
owner for such reusable profiles, and saved experiments freeze their exact content.

Environmental Disturbance is unimplemented and disabled. Existing incompatible
inherited environmental fields still prevent deployment; select the existing
No external acceleration resource explicitly. Active response additionally needs
a physical tracking-TF deployment, selected in authoritative configuration rather
than silently changing the robot's source in a dashboard callback.

The Attractor template's base/response parameters are examples, not calibrated
physical parameters. OFF/ON actuation-OFF Runs can validate target dynamics and
execution, but do not establish tracking quality or physical joint-speed safety.
No controller, estimator, predictor, safety limit, physics timestep, PID gain or
Runner readiness predicate was changed. No files were removed by D4.4.

## Completed regression evidence

| Check | Actual result | Artifact |
|---|---|---|
| Simulation functional regression, excluding style/copyright checks | 506 passed, 31.92 s | `results/external_influences/simulation_regression_final.txt` |
| Focused context, schema, template and derivative influence checks | 80 passed, 49.80 s | `simulation_focused_final.txt` |
| Dashboard Builder, unified model selection and influence UI | 25 passed, 735.07 s | `dashboard_regression.txt` |
| Library, mapped deployment and influence authoring/recovery | 15 passed, 156.03 s | `experiment_regression.txt` |
| Additional invalid-ON → OFF recovery reproduction | RED then 1 passed | `disable_recovery_red.txt`, `disable_recovery_green.txt` |
| Build simulation/experiments/dashboard | 3 packages finished, 80 s | `build.txt` |
| Rebuild final backend recovery fix | 1 package finished, 27.8 s | `build_recovery.txt` |
| Existing evidence integrity | 19,771 files; 0 changed, 0 missing | `historical_audit.json` |

The 546 tests in the three final regression commands are distinct; the focused
80 and earlier owner/UI checks overlap those suites and are not added to that
total. Dash reported 54 existing DataTable deprecation warnings. Lint/style checks
were not run or claimed as passing. `git diff --check` passed. No numerical
assertion, safety limit or acceptance tolerance was loosened.

The independent mathematical tests cover signs, three-dimensional separation,
large and near-zero distances, coincidence, continuity, parameter positivity,
nonfinite/context/frame rejection, units and unsafe-expression rejection. Exact
OFF integration is bit-for-bit equal to the base model over 200 RK4 steps with
moving robot context; ON matches a separately written formula/RK4 reference to
absolute tolerance 1e-12. Alternate vector state names and an untouched hidden
state verify generic full-state integration and analytic physical output.

The dashboard tests use the actual owner backend, including Save/Duplicate and a
mutated global-default profile after saving. Frozen 0.3 m/s² remains unchanged
when the authoring default is monkeypatched to 0.9 m/s². No Library revision is
created by ON/OFF changes.

## Installed browser evidence

Real installed Chromium selected **3D Attractor Dynamics**, confirmed the default
OFF checkbox, explicitly selected No external acceleration, validated and saved
`EXP-ATTRACTOR-RESPONSE-OFF` r1. It then used the real Duplicate control, enabled
Robot Response and saved `EXP-ATTRACTOR-RESPONSE-ON` r1. Reloading both preserved
their frozen profile state and identical Library pin. Effective target parameters
differ only at the profile's enabled boolean. The Library file's SHA-256 remained
unchanged, with no new Library revision. Environmental Disturbance remained
noninteractive throughout. Evidence: `browser_report.json`, `off_reloaded.json`,
`on_reloaded.json` and `browser_finish.txt` under the validation artifact directory.

Actual screenshots, inspected after capture:

- [1920×1080](../../results/external_influences/influences_1920x1080.png)
- [1600×900](../../results/external_influences/influences_1600x900.png)
- [1650×790](../../results/external_influences/influences_1650x790.png)

The section has exactly two checkboxes, no mathematical editor, extra model
selector or new accordion. At narrower sizes the disabled option wraps naturally;
both controls remain visible. DOM bounds show no influence-level overflow or
nested scrollbar, and document dimensions equal the requested viewport at all
three sizes. Only the existing main editor scrolls. Screens show a detached
editable copy after reload, so its honest status is Unsaved while the selected
immutable saved revision is r1.

Failed browser harness attempts remain preserved. One expected an internal
backend message that the D4.2 view intentionally replaces with its compact draft
notice; another clicked Load before the asynchronous selected revision resolved.
The resumed harness waits for the actual revision and component controls. No
readiness, physics or scientific criterion was relaxed to obtain these results.

## Actual Gazebo results and remaining runtime limitation

Exactly one controlled OFF/ON pair used the **saved browser-created r1 Scenarios**,
Seed 42, identical robot/world and target mathematical initial state, explicit
No external acceleration, four evaluation seconds and one settling second.
`resolved_configuration.json` confirms `enable_actuation: false` and
`publish_commands: false` in both Runs. Both target initialization logs show
p=[1,0,1] m and v=[0,0.2,0] m/s from the frozen six-state model. Legacy Cartesian
spawn defaults remain separately recorded in the owner snapshot; they do not
replace the mapped model's declared mathematical initial state.

| Recorded item | Response OFF | Response ON |
|---|---:|---:|
| Execution | Completed | Completed |
| Runner readiness origin (sim s) | 24.430 | 1.442 |
| Evaluation window (sim s) | 25.430–29.430 | 2.442–6.442 |
| Stop clock (sim s) | 29.454 | 6.444 |
| Target truth records, including outside scored window | 1,422 | 38 |
| Valid distance/pointing samples in scored metrics | 203 | 11 |
| Mean / max distance error (m) | 0.153585 / 0.211640 | 0.387660 / 0.549857 |
| Mean / max pointing error (rad) | 2.559509 / 2.666459 | 1.603045 / 1.912731 |
| Prediction RMSE (m) | 0.330211 | 0.183900 |
| Target position-estimation RMSE (m) | 0.068213 | 0.058316 |
| Recorded RTF | 0.733873 | 0.667031 |
| Invalid/inactive control telemetry samples | 0 / 80 | 44 / 80 |
| Published motion commands | 0 | 0 |
| Max observed scored joint position / speed violations | 0 rad / 0 rad/s | 0 rad / 0 rad/s |
| Physical evidence coverage | 100% | 100% |
| Recorded overall acceptance | null (unclaimed actuation) | false (invalid control gate) |
| Owned cleanup survivors | 0 | 0 |

These are **descriptive recorded metrics, not a tracking-performance comparison**.
Readiness started at different target phases and ON lost fresh target observations.
No ranking or improvement percentage is justified. Physical joint observations
passed the existing scored gates with actuation OFF; this does not close the
project's broader physical-speed safety question or demonstrate tracking actuation.

Both genuine Runner readiness decisions satisfied the unchanged 0.08-s predicate:
maximum signed source ages were 0.062 s OFF and 0.002 s ON. Existing bounded
future allowances also remained unchanged. The actual four-second evaluation
windows started and ended, results/metrics/time series were saved, and the
existing Session references both immutable Runs without copying their storage.

The complete OFF recording agrees with an **independent matrix-exponential
solution** of the linear Attractor equations to maximum position discrepancy
3.5810e-10 m. The existing TimeStepper uses positive remainder substeps as well as
0.01-s steps; the first offline harness incorrectly assumed a fixed source-time
lattice. Its failed analysis is preserved; production integration/timestamps were
not changed. The final analysis uses the analytic solution at actual source times.

Over overlapping recorded target phase 0.620–2.408 s, OFF versus ON trajectories
have mean separation 0.164777 m and maximum separation 0.405301 m (96 OFF source
samples, interpolation of ON records only within overlap). OFF remains at z=1 m;
ON reaches z=1.368813 m before its last committed sample. Actual observed tracking
TF is near [0.822748, 0, 0.689177] m during the ON observer interval, consistent
with the upward repulsion contribution. This is a diagnostic effect observation,
not a complete independent ON integration replay: the separate TF observer began
at 2.098 s, after target initialization at 1.430 s, and the exact internally
selected context/state trace was not recorded. Missing inputs were not fabricated.

**Unresolved:** ON committed its last target sample at approximately 3.850 s.
The simulation log reported `Target context waiting` with state_time=3.83 at
clock=4.096 s. Predictions subsequently became stale; 44/80 scored control
telemetry samples were invalid/inactive, and the existing invalid-control
acceptance gate correctly failed. The last target truth was about 2.594 s old
at stop. No motion command was published and no stale/future robot pose fallback
was introduced.

The independent observer continued receiving world→tracking_frame TF through
6.694 s. Its maximum source gap was 0.068 s and maximum receipt-clock age was
0.026 s (OFF: 0.118 s and 0.032 s). Therefore an upstream TF publication gap alone
is **not** supported as the cause. The failure is localized to availability of a
causal sample for the target's historical integration time in its own consumption
path. Callback scheduling, queue loss or sample rejection are hypotheses, not
proven causes. The default direct TF subscription uses the existing sensor-data
QoS depth; no receipt/selection trace exists to distinguish them conclusively.

Next focused engineering action: instrument source/receipt times, accepted/rejected
history insertions and the preceding/following samples at the failing target
substep, then fix demonstrated retention/scheduling loss within the existing
provider. Retain world-frame, causality, max-age, lookback and readiness checks.
Do not use future poses, extrapolate a robot, reinitialize target time or relax
freshness to conceal this failure. No speculative queue/timeout change was made.

Run IDs:

- OFF: `15f1e290-484d-4cbf-9f46-5d644b60cf97`
- ON: `10c82c9d-2501-49c5-a88d-6fd0c1f6e9eb`
- Session: `a925a832-c7dd-4a1b-b06e-3901f0ddaf03`

Evidence: `live_pair.json`, `live_analysis.json`, `live_session.json`,
`context_off.jsonl`, `context_on.jsonl`, each Run's immutable metadata,
readiness/status/metrics/resolved configuration, launch log and telemetry under
`results/external_influences/`. Target truth is the existing Gazebo-pose-acknowledged
source, not a second independently sensed target pose.

Cleanup verification found both Job worker identities stopped, no relevant live
ROS/Gazebo/Playwright processes, no Runner cleanup survivors, and port 8060 free
(`cleanup_final.json`). Final preservation audit again found all 19,771 historical
files unchanged and present. No commit or push was performed.

To run a new immutable attempt with the saved configuration through the normal CLI:

```bash
source install/setup.bash
ros2 run robot_experiments run --catalog results/external_influences/workspace/catalog.yaml --experiment EXP-ATTRACTOR-RESPONSE-OFF --revision 1 --output results/external_influences/reproduction
ros2 run robot_experiments run --catalog results/external_influences/workspace/catalog.yaml --experiment EXP-ATTRACTOR-RESPONSE-ON --revision 1 --output results/external_influences/reproduction
```

The ON attempt is reproducible configuration, **not guaranteed runtime acceptance**
until the context-consumption limitation above is resolved. Both frozen revisions
keep actuation and command publication OFF. The normal dashboard launch command
above uses the same Catalog and can inspect these Runs through their Session.

## Independent verdicts

| Area | Verdict | Basis |
|---|---|---|
| Mathematical validity | PASS | Restricted expression/unit/domain checks; independent formula/RK4 and analytic live OFF reference |
| Derivative injection | PASS | Explicit scalar/vector velocity derivatives; full-state and physical output tests; no post-step correction |
| Robot context | CONDITIONAL | Correct physical world TF, freshness/causality/reset gates; live in-process historical availability stall unresolved |
| Immutable Scenario behavior | PASS | Real Save/Duplicate/reload, same base pin, frozen defaults mutation test, unchanged Library hash |
| Disabled environmental checkbox | PASS | Disabled in DOM/browser; explicit no-field configuration; incompatible inherited fields rejected |
| Gazebo runtime | CONDITIONAL | Both Runs genuinely completed and cleaned up; ON target stalled and control acceptance failed |
| UI | PASS | Real two-checkbox workflow and all three desktop screenshots/bounds; minor error-path redraw issue documented |
| Regression safety | PASS (software scope) | 546 final functional tests, affected builds, unchanged old evidence; no physical-actuation safety claim |

**Overall: CONDITIONAL.** The feature, mathematics, authoring and UI are implemented
and verified. Reliable uninterrupted live ON context consumption remains open.
No unrelated reliability refactor, algorithm tuning or next milestone was started.

## Exact D4.4 source/documentation file list

Modified (other existing Git changes predate this task and were preserved):

```text
src/robot_experiments/robot_experiments/backend.py
src/robot_dashboard/robot_dashboard/app.py
src/robot_dashboard/robot_dashboard/dynamics_forms.py
src/robot_dashboard/robot_dashboard/assets/dashboard.css
src/robot_dashboard/test/test_builder.py
src/robot_simulation/config/dynamics_templates.yaml
src/robot_simulation/robot_simulation/dynamics/equation_model.py
src/robot_simulation/robot_simulation/engine/simulation_engine.py
src/robot_simulation/robot_simulation/factory/deployment_configuration.py
src/robot_simulation/robot_simulation/factory/simulation_factory.py
src/robot_simulation/robot_simulation/nodes/simulation_node.py
src/robot_simulation/test/test_context_recovery.py
src/robot_simulation/test/test_deployment_configuration.py
src/robot_simulation/test/test_declarative_motion_templates.py
```

Added:

```text
src/robot_simulation/config/derivative_influences.yaml
src/robot_simulation/robot_simulation/dynamics/derivative_influences.py
src/robot_simulation/test/test_derivative_influences.py
src/robot_experiments/test/test_influence_authoring.py
src/robot_dashboard/test/test_influence_ui.py
docs/validation/external_influences_d44.md
```

Generated task-only scripts, screenshots, traces, hashes, test logs and immutable
acceptance Runs are segregated under `results/external_influences/`, outside the
production package structure. Source baseline/audit lists distinguish this task
from the extensive pre-existing dirty tree. No file was deleted or relocated.
