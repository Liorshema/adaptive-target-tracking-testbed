# D4.1 — Generic live output mapping and Preview Settings

## Design and implementation plan

Preserve D4.3 and all historical evidence. Extend the existing target adapter and
SimulationState with a separate optional full mathematical vector; integrate it
through the existing engine/RK4 and evaluate validated PositionOutput only at the
same committed simulation time. Retain the legacy Cartesian vector conversion
and experiment initial p/v override semantics unchanged. Mapped models initialize
from their complete declared state; deployment p/v remain legacy physical values,
never invented mathematical state variables. The node's existing clock-reset and
pose acknowledgement lifecycle must carry this vector through dataclass replace.

The existing owner already supports an empty acceleration_fields list. Expose a
reusable explicit no-field resource through existing Author resources; do not
silently strip inherited active fields. Generic prescribed mapping rejects any
active/opaque field coupling. Existing Cartesian acceleration injection remains
exactly once. No speculative generic force projection, orientation or new solver.

Preview Settings currently lives inside an overflow-hidden preview panel, so
its upward absolute child is clipped despite z-index. Use the existing Details
control and a viewport-bounded fixed panel below the toolbar, with internal
scrolling for many fixed inputs; retain visible Close/summary and no page overflow.

Execution is authorized by the user's detailed task and standing request to
proceed without per-change questions. Work remains in this existing modified
workspace; no reset, commit, push, D4.2 or unrelated runtime refactor.

- [x] RED/GREEN owner/engine tests: exact circular, nonzero initialization epoch,
  reordered names, nonlinear scaled output, extra/vector states, invalid context,
  frame/domain, no-field versus active-field and legacy Cartesian behavior.
- [x] Implement minimal adapter/state/engine/factory/node changes. Record actual
  integration vector separately from physical TargetState and preserve clock reset.
- [x] Test immutable Author/Catalog pinning and add an explicit reusable no-field
  choice via existing resources; retain one-selection workflow and full preflight.
- [x] Reproduce popup clipping in Chromium; fix bounded existing popup; verify
  duration 5→13, step, open/close/reopen and all three desktop sizes.
- [x] Focused regressions and affected builds. One bounded existing Runner Gazebo
  smoke of saved exact three-state model, actuation OFF, analytical truth analysis,
  readiness/stop/cleanup evidence. No timeout/predicate changes or fake acceptance.
- [x] Independent review, historical hashes, actual verdicts, exact files/commands.

## Before and after: actual runtime boundary

D4.3 already compiled and differentiated `PositionOutput` correctly for Preview.
Live deployment still required `deployment.adapter: cartesian_target`, exactly
two ordered vectors (position in m, velocity in m/s), and six scalar elements.
`SimulationEngine.step` always called `state_to_vector(state.target)` and
`vector_to_state(next_vector)`. Thus removing the validation restriction alone
would have initialized the wrong vector and discarded arbitrary internal state.
The node separately constructed its initial physical `TargetState` from launch
p/v. A nonempty inherited environment was also incompatible with the user's
three-position-state ODE.

The final ownership path is:

```text
Immutable DynamicsLibrary revision → Author/Catalog → frozen inputs/target.yaml
→ SimulationFactory → EquationDynamics → existing RK4/TimeStepper
→ TargetStateAdapter → physical TargetState → Gazebo pose acknowledgement
→ committed position truth / scheduled synthetic measurement → existing Recorder
```

`SimulationState.internal_state` stores a detached, read-only complete vector for
mapped models. The existing adapter selects this vector for RK4 without slicing,
reordering or adding states. Model initials own its initialization and reset.
The optional field is absent for Cartesian models: existing physical p/v launch
overrides, six-element conversion and force response are preserved. Dataclass
replacement during context updates and pose acknowledgement retains the vector.

For mapped models, the same D4.3 owner evaluates `p=h(t,s;theta)` and
`v=(∂h/∂s)f+∂h/∂t` at the committed integration time. No production finite
difference, alternate parser/solver, motion-name branch or adapter registration
was added. Finite values, units, shape and supported derivatives remain checked.
Physical output stays three positions and three velocities even when internal
dimension differs. The existing synthetic observation uses that physical vector;
this does **not** turn the mathematical model into a six-state model.

The ROS truth contract remains `geometry_msgs/PointStamped`: recorded position
is published after a successful Gazebo SetEntityPose acknowledgement. Analytic
velocity is supplied in physical `TargetState`, including to the existing
measurement boundary. There was no velocity-truth ROS stream before this task,
and none is invented here. Gazebo's target remains the existing kinematic entity.

## Explicit environment and deployment policy

The environment owner already supports `acceleration_fields: []` and filters
explicitly disabled fields. One reusable resource, `resources.environment_none`,
now selects that owner setting through existing resource inheritance in
`robot_experiments/config/experiments.yaml`. No new YAML file, default-field
change or duplicate environment schema was introduced.

The backend discovers environment choices by resolving the existing Author's
resources. Experiment Lab exposes them in the Target Dynamics card. The UI is a
thin client: changing the selection edits `experiment.resources.environment`,
invalidates saved validation, and runs full Author preflight. It preserves an
existing inline resource and all other component parameters. The exact library
revision remains pinned by the existing single Model Selection control.

| Live model | External acceleration | Initialization |
| --- | --- | --- |
| Existing Cartesian adapter / registered Fight/Flight | Existing field response, added exactly once | Existing launch p/v |
| Supported generic position mapping | Explicit empty effective field list only | Complete declared model initials |
| Generic ODE without physical output | Rejected for live deployment | Preview only |

Active or opaque acceleration fields are rejected for prescribed mapped motion,
with an actionable error naming `experiment.resources.environment` and the empty
field list. They are never dropped, added to derived velocity, or projected into
an arbitrary ODE. Nonempty stochastic disturbance settings remain unsupported by
the existing factory. Mapping capability describes structural support; complete
experiment preflight still validates real resources, robot, frame and bindings.
Live control inputs and undeclared/unsupported context remain rejected. The
existing declared `tracking_position_world` context is supported in f, not h.
Orientation output and arbitrary frame transformations remain unsupported.

Preflight is not a proof of future numerical domains. A syntactically and
dimensionally valid expression singular at its initial state is rejected by
Preview/runtime initialization, and subsequent singular/nonfinite stages fail
closed. No fabricated context is used to claim a numerical preflight proof.

## Exact user revision and immutable authoring evidence

The actual user-created **Circular Motion**, not the earlier six-state circular
composition, was located in `results/dynamics_studio_redesign/workspace/dynamics_library.yaml`:

- Library ID `DYN-5B5F0D60D9EB4FD597EC5ECEDBA6BA4F`, revision **1**.
- Library content SHA256 `f8019f99e97d6fdac746defd8fe4aed881cd9d491931904cef66c690b0de5509`.
- Canonical model SHA256 `863c9f992a16c09d854f6a7e1b57641e8a8ce0c3df8295fd45dcb2debb153bac`.
- Three scalar states in m: x=1, y=0, z=1; omega=0.5 [1/s];
  f=`[-omega*y, omega*x, 0]`; h=`[x,y,z]`; world frame; no Cartesian adapter.

The whole library was copied to a new isolated D4.1 workspace without editing the
original. `user_model_identity.json` records the exact revision equality. Actual
Chromium selected this model once, observed inherited active-field rejection,
selected **No external acceleration**, validated and saved the complete experiment,
then reloaded it after browser restart with the same frozen pin and full model.

Saved experiment: `EXP-GENERIC-MAPPED-CIRCULAR-ACCEPTANCE`, revision **1**;
definition SHA256 `2e68d30eeac4d6f3ce86aaedc30ac30b7768568dc3bfb0156d62b29fb1ab9843`.
Its materialized target has exactly the original three states, the effective
environment has empty acceleration/stochastic lists, and effective control has
`publish_commands: false`, `enable_actuation: false`. Parameter controls, source
provenance, Validate, immutable Save, registered overrides and historical revision
selection retain the existing workflow.

## Independent mathematics and compatibility tests

The new owner tests first failed on the previous live limitation (`engine_red.txt`),
then passed after implementing the complete state boundary, not just validation.
They exercise the actual Factory, Engine and RK4:

- Exact circle: 500 steps of 0.01 s starting at epoch 12.375 s. Each physical
  position and analytic velocity matches independent sin/cos expressions at
  absolute tolerance 2e-10, relative tolerance zero. Internal dimension stays 3,
  reset restores [1,0,1], and deliberately different legacy p/v cannot override it.
- Reordered a/b/c plus a two-element hidden vector: all five states survive;
  hidden [7,8] evolves to [9,6] while h selects [a,b,c]. Snapshots resist mutation.
- Vector state q and explicitly time-dependent h=q+speed*t: both derivative terms
  and the nonzero absolute simulation epoch are checked.
- Dimensionless Lorenz with an explicit 0.1 m scale: independent Python RK4 f
  agrees with the integrated full state and scaled physical p/v. Removing spatial
  units is rejected. No Lorenz implementation was added to production.
- Active field rejection, explicit disabled-field handling, singular initial
  mapping, unsupported frame/context/control and existing invalid-expression/
  nonfinite/output-derivative tests remain enforced.
- Cartesian CV initial p/v overrides, declarative versus registered Fight/Flight,
  external acceleration once, pipeline and context/clock recovery regressions pass.

Actual Author/Catalog tests pin and materialize a generic three-state model,
reject inherited acceleration and verify the full immutable model and empty owner
environment. Existing library revision/hash, composition and reload tests pass.

## One bounded real Gazebo Run

Executed once through `ExperimentBackend.start_run` → existing Jobs/Runner, with
GUI false and explicit actuation false. No readiness/Recorder/physics/controller
changes or parameter sweeps were made.

| Evidence | Actual value |
| --- | --- |
| Run ID | `821fd60f-628e-4944-a3a3-f3f919433126` |
| Job ID | `0f25ba43-c115-4f8f-9034-3301bc6ab849` |
| Session ID | `6d25e0d3-c586-47a2-aaee-fd236ae24bb0` |
| Study | `nominal_20261104T055039_891869Z_qw5mqss3` |
| Execution | **completed**, no execution error |
| Target initial epoch | **1.6560000000000001 s**, dimension **3**, p=[1,0,1], v=[0,0.5,0] |
| Actual readiness decision | **1.708 s**, max signed required-stream age **0.044 s**, unchanged max_skew **0.08 s** |
| Settling / scored duration | **3 s / 6 s** |
| Scored source window | **[4.708, 10.708] s**, stop clock **10.710 s** |
| Recorded target truth | **780** samples, **1.668–10.882 s**, world frame |
| Maximum position error | **4.9919234768e-10 m** |
| Maximum radius error | **8.4710016779e-14 m** |
| Maximum height error | **0 m** |
| Analytic owner velocity replay error | **2.4959617384e-10 m/s** maximum |
| Recorded-position secant / midpoint velocity difference | **2.0833333642e-6 m/s** maximum |
| Command messages recorded | **0** |
| Cleanup | return code **0**, SIGINT, **no survivors**, **1.948 s** |

The comparison uses `tau = source_time - 1.656`, not the Runner's later readiness
or scoring origin. Reference p=[cos(0.5tau),sin(0.5tau),1] and
v=[-0.5sin(0.5tau),0.5cos(0.5tau),0]. RK4 maximum dt remains 0.01 s. The recorded
analysis asserts position error below 1e-7 m and radius error below 1e-8 m;
these conservative numerical checks accommodate integration roundoff and ROS
nanosecond timestamp encoding, without fitting the origin or trajectory.

Velocity evidence is deliberately separated: the live initialization log and
actual engine tests verify physical velocity; replay evaluates the deployed
owner's analytic velocity using this circle's recorded state (here s=p).
Position secants are **offline diagnostics only**, at recorded spacings
0.010–0.020 s. Their midpoint speed difference agrees with finite chord sampling;
they are not presented as measured velocity telemetry or a production derivative.
Recorded truth is the Gazebo-acknowledged target state, not a second independent
Gazebo pose sensor.

Metrics and scored time series were genuinely saved. For context, recorded mean
distance error was 0.685148 m, mean pointing error 2.601623 rad, target estimation
position RMSE 0.075128 m, prediction RMSE 0.459330 m, and RTF 0.866985. These are
compute-only measurements with a stationary robot, **not tracking-quality or
physical-actuation safety acceptance**. Run `acceptance` is null; no quality
threshold or ranking was invented. A startup future-state warning recovered
before actual readiness; it was not suppressed or used to change timing policy.
The Job worker exited **3** (completed execution without scientific acceptance),
not 0; the authoritative Run execution state is **completed**, acceptance **null**.

The existing Session API attaches the Run by immutable ID/path, without copying
telemetry or metrics. Actual Results & History rendering checks use those saved
artifacts. No velocity stream, fake synchronized data, second Runner or result
store was introduced.

## Preview Settings fix and installed Chromium

The pre-change upward absolute panel began at y=252.65 inside a preview ancestor
starting at y=440.43 with overflow hidden at 1650×790. z-index could not escape
that clipping boundary. The existing Details popup now uses a fixed viewport-
bounded position below the toolbar, bounded internal scrolling and a small Close
button. Native Details close/reopen is handled explicitly in one client callback;
there is no new overlay framework or plot/editor resizing.

At **1920×1080, 1600×900 and 1650×790**, actual Chromium:

1. Opened the settings and checked every duration/step/Close control's viewport
   bounds and actual hit target.
2. Changed duration **5→13 s**, set step **0.02 s**, closed and reopened, retaining
   the entered values.
3. Ran the actual preview: **651 samples**, final time **13 s**, full state size 3.
4. Captured and visually inspected the rendered open-panel screenshots. No
   toolbar obstruction, clipped fields or page overflow was present at these sizes.

Evidence is under `results/generic_live_output_mapping/`:

- [Before, 1650×790](../../results/generic_live_output_mapping/preview_settings_before_1650x790.png).
- [Open, 1920×1080](../../results/generic_live_output_mapping/preview_settings_open_1920x1080.png).
- [Open, 1600×900](../../results/generic_live_output_mapping/preview_settings_open_1600x900.png).
- [Open, 1650×790](../../results/generic_live_output_mapping/preview_settings_open_1650x790.png).
- [13-second preview, 1650×790](../../results/generic_live_output_mapping/preview_13s_1650x790.png).
- [Saved mapped experiment](../../results/generic_live_output_mapping/mapped_experiment_saved_1920x1080.png).
- [Actual completed Results, 1920×1080](../../results/generic_live_output_mapping/results_1920x1080.png).
- [Actual completed Results, 1600×900](../../results/generic_live_output_mapping/results_1600x900.png).
- [Actual completed Results, 1650×790](../../results/generic_live_output_mapping/results_1650x790.png).

Lab and Results also fit all three desktop viewports at device scale 1. Actual
Results renders four KPIs and four equal panels using the new recorded Run; no
ranking is assigned. Final screenshots wait for painted cards/charts and absence
of the loading overlay, not merely hidden DOM content. Browser errors: zero.

## Tests, build and review

Suite sizes overlap; these are separate invocations, not a summed unique-test count.

| Invocation / evidence file | Result |
| --- | --- |
| Owner runtime/legacy/environment/context suite, `owner_regression.txt` | **169 passed**, 46.71 s |
| Builder and unified selection, `lab_regression.txt` | **20 passed**, 333.22 s |
| Final mapped/output/preview/composition/library/Author/Studio/math suite, `final_regression.txt` | **95 passed**, 98.71 s |
| Genuine recorded Results and trajectory regression, `results_regression.txt` | **58 passed**, 18.30 s |
| Final formatted-source Engine/layout smoke, `final_smoke.txt` | **8 passed**, 6.62 s |
| Final owner suite including opaque-field rejection, `opaque_field_regression.txt` | **6 passed**, 0.50 s |
| New test-file flake8, `new_test_lint.txt` | **exit 0** |
| Affected symlink-install build, `build_final.txt` | **3 packages finished**, 1 min 20 s |
| Optional UI rebuilt with its Dash interpreter, `dashboard_venv_build.txt` | **1 package finished**, 5.70 s |
| Actual installed `ros2 run` HTTP/layout and cleanup, `installed_verification.txt` | **HTTP 200**, clean exit **0**, port **8060 free** |

Warnings were the existing Dash DataTable deprecation and sourced-underlay
colcon override notice. No failing functional tests are being relabeled as style
issues. Initial RED and intermediate fixture/assertion failures are retained;
the context-recovery fake node was updated to provide the engine now used for
initialization. Independent read-only review found no blocking mathematical,
safety, provenance or Cartesian compatibility issue and separately ran 26 mapped/
PositionOutput tests. Its numerical-preflight caveat is documented above.

After browser capture the installed console entry point was rebuilt with the
existing Dash virtual environment (rather than depending on system Python to
contain Dash). The documented `ros2 run` command served the actual three-tab layout
and was then closed. Final independent process inspection found no survivors in
the Run's isolated Gazebo partition. Only acceptance-owned dashboard processes
were stopped; no simulation was launched for this installation check.

## Preservation and exact changed files

Task-start hashes cover **19,115 existing result artifacts**: **zero changed,
zero missing**. Source audit identifies **15 modified and 3 added** files relative
to the beginning of D4.1, rather than confusing earlier uncommitted D4 work with
this task. No source or historical evidence was deleted. The previous D3/D4/D4.0/
D4.3 reports, user library and existing Runs retain their original contents.

Modified in this task:

```text
src/robot_simulation/robot_simulation/adapters/target_state.py
src/robot_simulation/robot_simulation/state/simulation_state.py
src/robot_simulation/robot_simulation/engine/simulation_engine.py
src/robot_simulation/robot_simulation/dynamics/equation_model.py
src/robot_simulation/robot_simulation/factory/simulation_factory.py
src/robot_simulation/robot_simulation/nodes/simulation_node.py
src/robot_simulation/test/test_position_output.py
src/robot_simulation/test/test_context_recovery.py
src/robot_experiments/robot_experiments/backend.py
src/robot_experiments/config/experiments.yaml
src/robot_dashboard/robot_dashboard/app.py
src/robot_dashboard/robot_dashboard/dynamics_studio.py
src/robot_dashboard/robot_dashboard/assets/dashboard.css
src/robot_dashboard/test/test_builder.py
docs/dashboard.md
```

Added:

```text
src/robot_simulation/test/test_mapped_deployment.py
src/robot_experiments/test/test_mapped_output_integration.py
docs/validation/generic_live_output_mapping_d41.md
```

Generated scripts, logs, screenshots, isolated Catalog/Session/Jobs/Run and hashes
live only in the ignored `results/generic_live_output_mapping/` evidence directory.
No production wrappers or motion-specific model files were added. Nothing was
committed or pushed.

## Reproduction

From `/home/user/ros2_ws`, build and open the saved acceptance workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_simulation robot_experiments robot_dashboard
# Bind the optional UI console script to the existing Dash Python environment:
build/dashboard_venv/bin/python /usr/bin/colcon build --symlink-install --packages-select robot_dashboard
source install/setup.bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir /home/user/ros2_ws/results/generic_live_output_mapping/workspace
```

At `http://127.0.0.1:8060`, select **Generic mapped circular acceptance**,
revision 1, and the recorded D4.1 Session. Results shows the immutable Run above.
For a new execution, create a new Session, load that saved experiment, leave
actuation opt-in unchecked and Run. The original configured three-second
settling and six-second scored window apply. No simulator is launched merely by
opening the dashboard. For new models, Save in Dynamics Studio, select once in
Lab, explicitly choose the compatible environment, Validate and immutable Save.

Equivalent headless API, creating a new Session/Run without replacing old results:

```python
from pathlib import Path
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog

workspace = Path('/home/user/ros2_ws/results/generic_live_output_mapping/workspace')
backend = ExperimentBackend(workspace, catalog=ExperimentCatalog(workspace / 'catalog.yaml'))
session = backend.create_session({'title': 'Mapped circle reproduction'})
job = backend.start_run('EXP-GENERIC-MAPPED-CIRCULAR-ACCEPTANCE', 1,
                        output=workspace / 'runs', enable_actuation=False,
                        gui=False, session_id=session['session_id'])
# Poll backend.get_job_status(job['job_id']); inspect actual Run execution_status.
# On completion attach each Run using backend.add_run_to_session(session['session_id'], run_id).
# To cancel use backend.cancel_job(job['job_id']); existing Runner cleanup remains authoritative.
```

Focused regression command (use the existing Dash virtual environment):

```bash
source install/setup.bash
build/dashboard_venv/bin/python -m pytest -q \
  src/robot_simulation/test/test_mapped_deployment.py \
  src/robot_simulation/test/test_position_output.py \
  src/robot_simulation/test/test_motion_preview.py \
  src/robot_simulation/test/test_motion_composition.py \
  src/robot_experiments/test/test_mapped_output_integration.py \
  src/robot_experiments/test/test_dynamics_library.py \
  src/robot_experiments/test/test_dynamics_library_integration.py \
  src/robot_dashboard/test/test_dynamics_studio.py \
  src/robot_dashboard/test/test_studio_redesign.py \
  src/robot_dashboard/test/test_mathematics.py
```

## Independent verdicts and limits

| Acceptance dimension | Verdict | Scope of demonstrated evidence |
| --- | --- | --- |
| Generic mathematical-state preservation | **PASS** | Complete 3/5-state scalar/vector/reordered integration and reset |
| Mapped world-position output | **PASS** | Analytic circle, scaled nonlinear and time-dependent owner tests; actual acknowledged live truth |
| Derived world-velocity consistency | **PASS** | Existing analytic chain rule, independent Engine references, initialization log and recorded-state replay; no velocity ROS sensor claim |
| Environmental acceleration correctness | **PASS** | Explicit no-field live run, active/opaque rejection, unchanged Cartesian response |
| Experiment preflight and model selection | **PASS** | Actual single-selection browser save/reload, full Author/Catalog/materialization and exact pin |
| Real Gazebo deployment | **PASS** | One isolated headless, actuation-OFF completed Run with unchanged readiness and successful cleanup |
| Existing Cartesian compatibility | **PASS** | Legacy p/v, CV/Fight/Flight/environment and recovery regressions |
| Preview Settings fix | **PASS** | Actual accessible, visually inspected open/edit/close/reopen/13-second preview at all three sizes |
| Historical scientific integrity | **PASS** | All 19,115 prior result hashes preserved; immutable revision tests; unchanged comparison protections |

These are bounded capability verdicts. They do not validate arbitrary nonlinear
models over untested intervals, GUI rendering under all hardware loads, tracking
quality, real-hardware safety or arbitrary disturbance response. Numerical domains
remain checked at evaluation time. D4.2 ranking changes and other future work are
not started; any later force coupling requires an explicit, unit/frame-correct
mathematical input contract rather than acceleration projection. D4.1 stops here.
