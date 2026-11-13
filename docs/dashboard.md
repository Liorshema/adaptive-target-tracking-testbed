# Experiment Dashboard

`robot_dashboard` is the optional Plotly Dash client for the existing
`ExperimentBackend`. It authors experiments, requests execution through the existing backend, and
presents recorded results and persistent Session comparisons. The backend and
Runner retain all process ownership, recording and scientific calculations.

## Install and launch

From the ROS 2 Jazzy workspace, install the UI dependency in an isolated Python
environment with access to ROS's system Python packages. The experiment packages
remain usable with the system Python without Dash.

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m venv --system-site-packages build/dashboard_venv
build/dashboard_venv/bin/python -m pip install 'dash>=3,<4' 'plotly>=5'
build/dashboard_venv/bin/python /usr/bin/colcon build --packages-select robot_experiments robot_dashboard
source install/setup.bash
ros2 run robot_dashboard dashboard --data-dir results/dashboard_workspace
```

Open **http://127.0.0.1:8050**. The installed console script uses the Python
environment used to build the package, so activating the environment is not
required for this launch. Other machines must first build/source the existing
experiment stack. Dash is a dependency of `robot_dashboard` only.

Options: `--host` (default `127.0.0.1`), `--port` (default `8050`),
`--data-dir` (default `$XDG_DATA_HOME/adaptive_tracking`, or
`~/.local/share/adaptive_tracking`), `--catalog EXISTING_CATALOG.yaml`, and repeatable `--result-root PATH`.
The default workspace Catalog is initialized once through
`ExperimentCatalog.initialize()`. No package defaults are edited. Use `--catalog`
to share the same existing writable Catalog with the headless CLI; there is no
separate UI database or experiment engine. The Dash development server is intended
for a local engineering workspace; authenticated remote hosting is outside this local client.

## Workspaces

**Dynamics Studio** holds the motion library, structured mathematical editor and
bounded model-only numerical preview. **Experiment Lab** holds selection/create, schema-driven component cards, duration,
Seeds, immutable revision actions and a compact Play/Stop panel. **Results & History** holds
the selected recorded Run, four KPIs and four equal panels: radial error, pointing
error, raw target 2D/3D telemetry and current-Session comparison. Computation time is
available in Advanced / Diagnostics. All three workspaces remain mounted when switching,
so an unsaved draft or current Job is not destroyed. Current Session selection and
its ⋯ create/refresh/rename/delete menu are shared in the header.

At 100% browser zoom the dashboard fits 1920×1080, 1600×900 and 1650×790 without page scrolling. The Models & Parameters editor has one main scroll region; open component bodies
grow naturally. Execution evidence and history rows have bounded internal scrolling.
Backend/CLI inspection retains full configuration JSON; Results diagnostics remain available. Smaller
screens use normal responsive document scrolling. The dark theme is based on
`docs/reference image Dashboard.png`; its example values and rankings are not data.

## Dynamics Studio

The mounted Studio uses a compact two-column editor inspired by
`docs/Studio_reference_screenshot.png`. It authors general D3 equations through
`backend.dynamics`; model preview never creates a scientific Run.

1. Click **New Model**, or open **Saved Models**. Technical identities are generated
   automatically. Unsaved drafts survive tab changes; replacing a dirty draft asks
   for explicit discard confirmation.
2. Edit the State / Initial Value / Derivative and Symbol / Value / Unit tables.
   Add or delete scalar/vector rows. Under **Advanced**, expose state units and
   shapes, parameter bounds, intermediates and declared context/control inputs.
   Names carry no physical meaning. Units are required, never inferred.
3. Optionally define **World Position (x, y, z)** as an expression such as
   `[a,b,c]`. The owner validates world frame, metres and vector shape, then
   derives velocity by the total derivative of that expression. The typeset
   mathematical panel reflects the actual draft; an unvalidated draft has no
   mathematical-validity claim. Unused parameters are identified after validation.
4. Click **Preview** for bounded D3/RK4 numerical integration. Spatial mappings use
   true-aspect 3D or XY; other systems show state versus time. **Preview Settings**
   holds duration, integration step and explicit fixed input assumptions. External
   fields are excluded. No context is inferred from its name or supplied by a fake
   moving robot.
5. Enter a display name and click **Validate & Save**. The existing library performs
   validation and an atomic immutable revision append with stale-write protection.
   A label-only change preserves mathematical history and hashes. **Saved Models**
   opens the latest available version; exact historical versions and optional
   templates remain under the compact library Advanced section. Existing composed
   models keep their original source contributions, edited in Advanced; they are
   never silently flattened.
6. In **Experiment Lab → Target Dynamics → Model selection**, choose the saved
   model once. It is available immediately, with no registration, refresh, source
   radio or Apply step. That option contains an actual immutable revision; selecting
   it freezes the full canonical definition, mapping, source-symbol provenance and
   hashes in the detached experiment draft. Validate and Save the complete experiment
   separately. Registered-model overrides survive a switch to a library model and
   back. Advanced historical selection is optional.

The sole library is `DATA_DIR/dynamics_library.yaml`. Saved experiments retain
old pins when newer library versions are added, including after dashboard restart;
loading uses their frozen canonical model rather than mutable library latest.
Installed owner defaults and historical Runs are not edited.

Mapped State-Space models can drive the live target without a six-state layout.
The simulation integrates the complete declared state, initializes it from the
model's initials, and evaluates world position and analytic total-derivative
velocity at the same simulation time. Legacy Cartesian models retain their
deployment position/velocity overrides and external acceleration response.

For prescribed mapped motion, select **Experiment Lab → Target Dynamics →
External acceleration / environment → No external acceleration**. This selects
the existing resource mechanism's `environment_none` resource with an explicit
empty acceleration-field list. Inherited active fields cause full experiment
preflight to reject the draft; no field is silently ignored or projected into an
arbitrary ODE. Selection freezes the full model and pin as before. Save the
validated experiment and use the existing Run workflow; actuation remains an
explicit opt-in. Mathematical validity and mapping capability do not guarantee
all numerical domains: Preview checks the requested interval, and runtime checks
every integration stage and physical output.

Orientation/angular velocity output and arbitrary frame transformations are not
implemented. Supported analytic mapping rules use the existing restricted D3 AST,
unit compiler and resource bounds; unsupported derivatives fail explicitly.

Preview Settings opens in a viewport-bounded panel with a Close button. Duration,
step and fixed input assumptions remain available without resizing the editor.

See [D4.1 live mapping verification](validation/generic_live_output_mapping_d41.md),
[D4.3 verification](validation/dynamics_studio_redesign_d43.md),
[D4 evidence](validation/dynamics_studio_d4.md) and
[D4.0 evidence](validation/dynamics_library_composition_d40.md).

## Author an experiment

1. Select a **Scenario**, optionally choosing a historical immutable revision under
   **Advanced · revision history**, then **Load configuration**. To edit the latest
   revision use **Revise selected**. **New** starts from the selected saved base;
   **Duplicate** proposes a short collision-safe name for a copy. Enter a display
   name, Load, then edit. Technical IDs remain stable and can be inspected in the
   identifier disclosure. A historical revision can be a new Scenario's base.
2. Robot estimator, target estimator, predictor, adaptation, fusion, controller and
   target dynamics selectors are directly visible. Selectors stay outside optional parameter accordions. Expanded parameter
   bodies grow naturally within the main editor scroll. Choices come from the owners' live registries;
   availability alone does not prove compatibility. Full preflight is mandatory.
3. Switch **Override** on to edit an owner-schema parameter. Units, bounds,
   defaults and effective values remain visible. Arrays and nested mathematical
   objects retain typed JSON controls. Changing a model clears only its local
   overrides. Saved Dynamics models retain their exact revision/content pins.
4. Set **Duration (simulation s)** and **Seeds** (a JSON integer array), inherited
   from the resolved saved base. These author `experiment.execution`; Catalog
   freezes the compiled duration/seeds. Settling time is additional and unchanged.
5. **Preview** refreshes resolved model/provenance cards. **Validate** invokes the
   existing Author/M07 checks. Neither launches simulation. Field-specific errors
   appear beside the editor. Full developer inspection remains available through
   `backend.preview_experiment(draft)` and the experiment CLI.
6. **Save** creates the Scenario or appends an immutable revision with optimistic
   revision checking. The newly saved identity is selected for Play automatically.
   Edits invalidate validation; a stale form or a selection still loading cannot
   save a different Scenario. Saving reports **Saving…**, **Saved** or **Save
   failed**; conflicts preserve the draft and require deliberate reload/revalidation.

Legacy multi-variant studies are shown as separate Scenarios. Selecting one lazily
freezes that exact configuration through Catalog preflight with its source
ID/revision/hash/variant. Original studies, Run metadata and historical references
remain readable. No historical evidence is rewritten. Schema-2 sensor-assignment
Scenarios are selectable, inspectable and runnable; editing those assignments
remains supported through the existing CLI, rather than the schema-1 Lab editor.

**Reset** discards experiment-local selections and parameters and
restores the pinned base. When editing a saved experiment, that base includes the
saved experiment's values; this action does not undo history or change owner
YAML defaults. Browser drafts are memory-only and are lost on refresh/closing the
page. Each browser tab has its own draft; saved definitions are shared through
the Catalog's lock and compare-and-swap protections.

## Read provenance correctly

Cards identify the effective model. Selection provenance, defaults and bounds
remain available in each card's details or tooltips. Owner defaults and resolved
values are distinct. All seven primary component selectors remain visible outside their parameter
accordions. Expanded parameter bodies grow naturally; the main Models & Parameters region scrolls as one editor. For example, setting target
*damping* to `0.23` in a nominal-derived draft applies to the inherited
`saturated_fight_flight` model; its owner default remains `0.5 s^-1`.
Direct unsaved overrides are indicated by the switch, and the refreshed preview
identifies them as explicit experiment overrides.

The backend cannot prove full per-parameter ancestry through every inherited or
advanced owner override. Such resolved values are labeled **precise parameter
origin unknown**, even when numerically equal to an owner default. The full
preview retains available resource hashes and provenance. After edits, the
previous resolved snapshot is visibly labeled stale until Preview/Validate.

## Simulation configuration and developer inspection

Experiment Lab has no separate Simulation Setup or Open/Close preview controls.
Opening the dashboard never launches a simulator. Physical description, world,
physics, deployment, gravity holding, readiness and cleanup remain owned by the
existing experiment/simulation backend. `ExperimentBackend` retains preview
lifecycle APIs for developer clients; the removed Dash presentation introduces no
new process owner. For full effective configuration, use `backend.prepare(...)`,
`backend.preview_experiment(draft)` or `ros2 run robot_experiments experiment
explain --help`. Existing comparison declarations can be authored through the CLI;
loading and saving in Lab preserves them without a separate advanced policy form.

Runner readiness uses the unchanged freshness predicate and records passed
readiness decisions. See [Runner readiness verification](validation/runner_readiness.md)
and historical [GUI lifecycle evidence](validation/dashboard_simulation.md).

## Run and monitor

1. Create or select a **Current Session**. Select a saved experiment and immutable
   revision in the builder and use **Inspect saved revision**. Unsaved draft edits
   are not launch inputs. Start revalidates through the backend's existing preflight.
2. Choose the output location. **Enable physical actuation** defaults OFF and must
   be explicitly selected for each Start; a successful Start clears the checkbox.
   **Show Gazebo GUI for this Run** also defaults OFF and resets after Start.
   It is independent of physical actuation. The existing Runner can use the same
   frozen system configuration with GUI enabled; the explicit boolean is frozen
   in runtime provenance and the display environment is recorded in Run metadata.
   Unchecking it explicitly overrides a saved GUI-enabled definition. Seeds,
   variants and duration belong to the saved experiment definition. A preview opened by a separate developer client still uses the existing
   cleanup-gated GUI handoff; it cannot overlap a scored owner. Actuation remains
   a separate explicit opt-in.
3. Click **Play** once. The panel reports genuine execution status;
   **Execution / safety diagnostics** retains Job/Run UUIDs, seed/variant, errors
   and cleanup evidence.
   Repeated clicks are guarded in the local Dash process. Opening or refreshing
   the page never launches a job. Changing the builder selection does not alter
   the frozen revision, actuation setting or Session of an already launched job.
4. Status polls every two seconds while a job is active, stopping at terminal,
   lost-worker or status-read-error states. Running, Completed and Failed are actual
   owner outcomes, separate from recorded safety acceptance. Completion follows the
   saved simulation-time window automatically. **Refresh status** retries observation.
   Unknown state and incomplete cleanup stay visible; no progress percentage is
   estimated. New Runs attach idempotently to the Session selected at Start.
5. **Stop / Cancel** calls the backend's cooperative cancellation API. Selecting
   a Run cancellation scope cancels its entire owning Job and remaining attempts.
   A timeout is not proof of cancellation: inspect the resulting status and cleanup.

Execution references are retained in browser session storage across refresh.
Current Session selection uses browser local storage; Session membership and job
records persist through the existing backend. A new browser tab may need to select
its Session again. The transient Start guard is for this single local server;
it does not promise exactly-once delivery across a server crash between job
creation and acknowledgement. Inspect existing records before retrying an
uncertain launch. There is no additional UI database or process owner.

## Results and current Session

Select a real Run in **Results & History**, from its dropdown or a Session row.
Execution, overall acceptance and recorded safety are displayed separately. Metric tables retain recorded units and
availability; missing values remain unavailable. Exactly four KPI cards show:

- **Mean Position Error**: recorded mean absolute radial distance-to-reference
  error, `tracking_distance.mean`; this is not a general XYZ position error.
- **Mean Orientation Error**: recorded target pointing angle, `pointing_error.mean`;
  this is not full 3D attitude error.
- **Estimator Accuracy**: recorded `target_position_error.rmse`, or explicitly
  labeled recorded mean when RMSE is absent.
- **Predictor Accuracy**: recorded `prediction_rmse.rmse`, never its mean field.

The cards and primary charts retain recorded units, including radians. The
comparison's angular RMSE converts declared radians to degrees for its degree
column. Unknown units or missing RMSE remain unavailable, with no mean substitute.
The **Experiment Comparison · RMSE** table displays actual per-Run RMSE:
Scenario | R (m) | Angle (°) | Estimator (m) | Predictor (m) | Rank.
R is error relative to the configured tracking radius, not the radius itself:
`RMSE(||p_target - p_tracking|| - radius)`.

The metric selector defaults to **Range RMSE**; Angle, Estimator position and
Predictor position RMSE are also supported. The backend ranks completed Runs
within one explicitly selected compatible scope, lower first. Exact ties share
competition rank; Run IDs determine stable tie ordering. At least two distinct
Runs are required. Different dynamics, Seeds, scoring contracts, definitions,
units, protected physical/sensing/safety settings or undeclared parameter changes
are excluded. A declared target-dynamics variable is still insufficient to prove
comparable driving difficulty. Scope differences and excluded-Run reasons remain
inspectable in Comparison details; incompatible Runs remain visible as history.
Reactive motion may differ despite the same dynamics and Seed, so these are
**descriptive ranks**, never significance, controller superiority or safety claims.

Recorded scientific acceptance and safety qualification are separate and never
changed by descriptive ranking. Incomplete physical observation coverage can
leave qualification unknown while actual error metrics remain describable. The
existing stricter scientific ranking API and frozen study declarations are
preserved for CLI/developer clients. See [D4.2 verification](validation/experiment_lab_simplification_d42.md)
and [scientific comparison policy](validation/scientific_comparability.md).

The 2×2 panels show radial error, pointing angle, independent raw trajectories
and Session comparison. Charts use original markers without interpolation,
smoothing or alignment. Controller computation time, recorded evaluation windows,
full metric contracts, ownership and safety records remain under Advanced /
Diagnostics. Failed readiness produces no fake plots.

Advanced paging sets the sample offset: up to 1,000 scored samples per series and
20 raw control records. Trajectories scan each target stream from that raw offset
to the end of its recording, in bounded 1,000-message pages. The default offset
zero covers the whole available recording, including startup/settling samples;
it is not restricted to the scored window. Streams retain independent source
spans. Up to 4,000 genuine points per trace are sent to the browser; longer paths
use disclosed display sampling with the first and last actual samples retained.
Visible counts report plotted/valid samples and invalid omissions. No interpolation,
frame transform, resampling or scientific metric is calculated.

The **Forecast** message number selects one recorded origin from the available
history (latest by default). It displays that message's recorded future states,
origin and forecast epochs, with a recorded/declared horizon count. Partial or
unknown horizons are labeled honestly. Separate forecasts are never connected
into one predicted path. History before/after the actual recording is unknown.
Normal and expanded 2D/3D views share this reader and selector semantics; changing
the expanded forecast also updates the normal view. Only matching frames and
metre units are overlaid. XY uses equal physical scale and 3D uses data aspect.
Hover exposes original epochs, XYZ and units. The advanced performance plot uses
computation time because base/arm allocation is not scored in these artifacts.
The backend's scored-series reader
uses its existing 32 MiB JSON-file bound before slicing; this is bounded whole-file
reading, not streaming JSON. Original artifact references and safety/coverage
records remain inspectable. A completed execution does not imply good tracking
or accepted safety outside the recorded window.

Current Session can also pin saved Experiments before execution. The backend's
`add_experiment_to_session(session_id, experiment_id, revision, expected_token=...)`
stores the explicit immutable Catalog revision and its SHA256; it does not start a
Run. `list_session_experiments` resolves the saved names, and the Dashboard shows
them beside the selected Session. Older Sessions without Experiment pins remain
readable. Archiving an Experiment preserves its pins; explicit Scenario purge
removes matching pins through the existing deletion transaction.

Current Session stores references to original Runs. The RMSE comparison table
hides failed executions; their Results, diagnostics and Session references remain
available. Backend comparison data still includes failed or unavailable Runs, and
incompatible Runs are never ranked together. The launch Session remains fixed even
if another Session is selected during execution. Refresh the
Session list/comparison after edits made by another client.

The Session selector's **⋯** menu opens compact rename or deletion dialogs.
Deletion previews the actual number of exclusively referenced managed Runs to
remove and shared Runs to preserve. Confirmed deletion removes that Session and
its exclusive managed artifacts; other Sessions' references protect shared Runs.
Historical roots registered only for viewing remain protected. Concurrent scope
changes invalidate confirmation; active Jobs or unresolved cleanup block deletion.
New Jobs retain their launch Session before a Run exists. Terminal legacy Jobs
require reliable cleanup evidence, including retained Run cleanup when available.

The Scenario selector provides **New**, **Duplicate**, **Rename** and **Delete**. Rename persists a mutable Catalog display alias; original names,
definitions, revision hashes, Run metadata and identities remain unchanged.
Deletion previews matching immutable Run identities and the number of Sessions
to update. After explicit confirmation, it removes owned artifacts, prunes only
their Session references and permanently removes Catalog revisions. Empty or
mixed Sessions survive. The ID remains retired to prevent reuse. External or
ambiguous evidence, active previews/Jobs and uncertain cleanup block deletion.

Deletion authority defaults to **DATA_DIR/runs**. `--result-root` grants read
access only; repeatable **--managed-result-root** explicitly configures writable
ownership boundaries. Ownership and references are scoped to this backend
workspace; do not assign another workspace's shared historical roots as managed.
The backend rechecks impact tokens, all Session references and Job cleanup under
owner locks. Same-filesystem staging and an undo journal restore precommit
failures; postcommit cleanup is retried for the original confirmed scope.
Incomplete cleanup is an error, never success. After browser restart, a pending
transaction can be resumed through the documented backend recovery call; see
[D2.8a evidence](validation/dashboard_cleanup.md). Completed receipts retain no
deleted definitions or Session preimages.

Standard ROS 2 `<Run>/ros-log/latest` aliases are supported only inside verified,
exclusively selected managed Runs, pointing directly to an existing sibling
ROS timestamp/host/PID directory. Deletion inventories and unlinks the alias entry;
it never follows it. Its raw destination is rechecked during confirmation and
staged recovery. Other links, escapes, chains, dangling aliases and Job artifacts
remain protected. See [D2.8b evidence](validation/dashboard_reliability.md).


When creating a Scenario, enter its display name. The Catalog-facing backend
proposes an `EXP-…` ID; inspect it or enable a custom ID under **Advanced · identifier**.
Names are normalized to ASCII uppercase words with punctuation separated by
dashes. Names without an ASCII suffix use a deterministic SHA-256 prefix;
overlong suffixes are truncated with a hash suffix. Active, archived and retired
IDs remain reserved. A collision blocks loading a new draft and never selects
another Scenario. Revising preserves the saved ID. Preview, Validate and immutable
Save retain their existing behavior.

The trajectory header's **Expand** icon opens a large 2D/3D viewer; Close retains
the selected Run and restores the four-panel layout. The viewer follows browser
resizing. Optional individual panel resizing is deferred to preserve the simple
fixed grid. The trajectory toolbar provides zoom/reset and 3D rotation/camera reset. Runs
have separate view revisions, preventing a previous Run's zoom or camera from
being reused. Missing Z, nonfinite coordinates, incompatible frames or units are
reported rather than invented. The compact header tooltip retains frame, source
time and prediction-horizon provenance.

New output roots are recovered from existing job requests after server restart.
To inspect external historical results, supply the same repeatable
`--result-root` paths on subsequent launches; a Session reference does not copy
its original evidence or make an unregistered root discoverable.

## Verified D2 workspace

The validation workspace contains three references: a completed historical Run,
a historical readiness failure, and the single default-OFF D2 lifecycle Run.
To reopen it (stop an existing server on the same port first):

```bash
cd /home/user/ros2_ws
source install/setup.bash
ros2 run robot_dashboard dashboard --port 8051 \
  --data-dir /home/user/ros2_ws/results/dashboard_execution/workspace \
  --result-root /home/user/ros2_ws/results/robot_description_naming/runtime \
  --result-root /home/user/ros2_ws/results/dashboard_backend/runtime_complete
```

Open **http://127.0.0.1:8051**. Select Session
`f11beca8-dbdf-436a-8342-45d19723d116` if using a fresh browser profile.

## Scope and evidence

Open Simulation uses the existing launch/Jobs ownership with an unscored preview
kind. Runtime/readiness and physical-safety limitations documented by earlier
milestones are unchanged. D3 owns historical
cross-session analytics, advanced filters and research conclusions.

See [D1 verification](validation/dashboard_builder.md),
[D2 verification](validation/dashboard_execution.md), and
[D2.5 redesign verification](validation/dashboard_redesign.md), and
[D2.6 GUI lifecycle verification](validation/dashboard_simulation.md), and
[D2.6b paused preview / Play verification](validation/dashboard_workflow.md), and
[D2.6c readiness/completion verification](validation/runner_readiness.md), and
[D2.7 UI refinement verification](validation/dashboard_refinement.md), and
[D2.8 usability and safe management](validation/dashboard_management.md), and
[D2.8a deletion, rename and expanded viewer](validation/dashboard_cleanup.md) for measured checks,
screenshots, limitations and exact task-relative changed-file lists.

When registering historical result roots for viewing, incomplete ownership records
can block the existing conservative deletion preview, even for a newly copied
Scenario. Keep normal workspace scope for management; resolve incomplete archived
metadata separately. The dashboard never bypasses ownership checks to delete.

## Windows one-click launcher (WSL installation)

Open `tools/dashboard/Start Dashboard.cmd` in Windows Explorer, or create a Windows
shortcut to that file. It detects the WSL distribution from the repository's UNC
path, starts the existing WSL/ROS environment and Dashboard virtual environment,
uses the default Catalog, and opens `http://localhost:8060`. Keep the adjacent
PowerShell and shell files together; no Docker installation is used.

`Stop Dashboard.cmd` closes only the launcher's owned Dashboard process group.
Experiment workers retain the existing Runner ownership. Repeated Start clicks
reuse one server. An unrelated server occupying port 8060 is reported explicitly;
it is not adopted or killed. Startup errors remain visible in the Windows console.
Logs and PID/birth/port metadata are under
`~/.cache/adaptive_tracking/dashboard-8060/` (or `$XDG_CACHE_HOME`).

Prerequisites are the existing built workspace, `install/setup.bash`, and
`build/dashboard_venv/bin/python` with Dashboard dependencies. The equivalent WSL
commands are `bash tools/dashboard/dashboard.sh start` and `... stop`. The default
WSL user must be the user who owns the existing installation/Catalog.

Predictor **None** (`none`) holds the latest estimated position and publishes zero
future velocity at the usual forecast timestamps. Select **None** for Adaptation
and Fusion as well; full validation rejects residual correction of this hold.
Unavailable Predictor, Adaptation and Fusion entries are hidden from normal choices.
A selected unsupported historical model remains explicitly labelled and disabled;
backend validation still rejects invalid deployment. Completed execution with failed
safety acceptance remains visible in the comparison table. Failed executions are
hidden there, while their original Results, logs and diagnostic access remain.

A saved comparison may explicitly restrict its permitted model IDs. The preserved
Predictors Comparison permits its original three models; editing a saved member
does not silently expand that scientific declaration. **New** creates an independent
experiment from the selected Scenario's configuration, without inheriting its
comparison restrictions. Choose None for Predictor, Adaptation and Fusion, then
Validate and Create experiment. Reset retains this independent scope. New copies
(including Duplicate) are independent; revisions preserve their saved policy.
Explicit comparisons authored through the existing Author/YAML API still enforce
their permitted models. No existing revision needs to be rewritten.
See [independent draft regression evidence](validation/dashboard_independent_creation.md).
