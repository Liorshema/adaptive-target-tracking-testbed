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

**Experiment Lab** holds selection/create, schema-driven component cards, duration,
Seeds, revision actions, Simulation Setup and Run/Stop. **Results & History** holds
the selected recorded Run, four KPIs and four equal panels: radial error, pointing
error, raw target 2D/3D telemetry and current-Session comparison. Computation time is
available in Advanced / Diagnostics. Both remain mounted when switching,
so an unsaved draft or current Job is not destroyed. Current Session selection and
its ⋯ create/refresh/rename/delete menu are shared in the header.

At 100% browser zoom both tabs fit 1920×1080, 1600×900 and 1650×790 without page scrolling. Parameter bodies,
long execution evidence and history rows have bounded internal scrolling. Advanced
Configuration and evidence popovers keep full JSON available on demand. Smaller
screens use normal responsive document scrolling. The dark theme is based on
`docs/reference image Dashboard.png`; its example values and rankings are not data.

## Author an experiment

1. Select a **Scenario** and condition. **Advanced · revision history** retains
   saved revision selection and the backend's complete explanation. Human labels
   come from Catalog metadata, with immutable recorded names for external Runs.
2. Choose **Revise selected** or **Create new**. Creating needs a display name;
   a deterministic available ID is proposed automatically and can be overridden
   under **Advanced · identifier**. Click **Load configuration**. This explicitly
   replaces any unsaved form edits. Revision editing starts from the latest saved
   revision; use Create new to branch from history.
3. Expand a component card. Choices come from live registries through the backend.
   Library-only implementations are disabled. Availability alone does not prove
   compatibility; full backend preflight remains mandatory.
4. Switch **Override** on to edit a parameter. Numeric/boolean/string/enum controls
   use owner schemas. Arrays and nested mathematical parameter objects use JSON
   fields with their full schema available underneath. No Catalog YAML or resource
   recipe editing is needed. Switching a model clears only that role's unsaved
   parameter overrides; other edits remain.
5. Set **Duration (simulation s)** and **Seeds** (a JSON integer array).
   Values initially come from the Author-resolved base study, not a UI default.
   For example, enter `60` and `[42, 43]` to author a 60-second scored window for
   two seeds. Settling time remains inherited and is additional to this duration.
   These fields use `experiment.execution` in the detached Author draft; the
   Catalog stores its canonical compiled `experiment.duration` / `seeds`. They
   are part of the immutable revision, never runtime launch overrides.
6. **Preview** displays the complete effective configuration, implementation
   identities and provenance. **Validate** runs authoritative Author/M07 preflight.
   Read any field-specific errors and correct them. No experiment is launched.
   Full effective-configuration JSON is under collapsed **Advanced Configuration**.
7. **Create experiment** or **Save revision** persists explicitly through the
   existing Author/Catalog. Edits invalidate prior validation. Saving revalidates
   and uses optimistic revision checking. A conflict preserves the draft; inspect
   the newer revision and reload deliberately before trying again. After creating
   a scenario, select its saved entry before Play; saving does not automatically
   change the selected launch scenario.

**Reset** discards experiment-local selections and parameters and
restores the pinned base. When editing a saved experiment, that base includes the
saved experiment's values; this action does not undo history or change owner
YAML defaults. Browser drafts are memory-only and are lost on refresh/closing the
page. Each browser tab has its own draft; saved definitions are shared through
the Catalog's lock and compare-and-swap protections.

## Read provenance correctly

Cards identify the effective model. Selection provenance, defaults and bounds
remain available in each card's details or tooltips. Owner defaults and resolved
values are distinct. One parameter editor opens at a time; its secondary fields
scroll internally while the primary controls stay visible. For example, setting target
*damping* to `0.23` in a nominal-derived draft applies to the inherited
`saturated_fight_flight` model; its owner default remains `0.5 s^-1`.
Direct unsaved overrides are indicated by the switch, and the refreshed preview
identifies them as explicit experiment overrides.

The backend cannot prove full per-parameter ancestry through every inherited or
advanced owner override. Such resolved values are labeled **precise parameter
origin unknown**, even when numerically equal to an owner default. The full
preview retains available resource hashes and provenance. After edits, the
previous resolved snapshot is visibly labeled stale until Preview/Validate.

## Simulation Setup

The panel reports the selected physical robot description, world source, physics,
entity, deployment, gravity holding and requested runtime actuation settings.
These are read-only resolved configuration details, not a live simulator status.

Select a saved experiment/revision and variant, then **Open Simulation**. The
backend revalidates and freezes that selection, uses the revision's first saved
seed, and opens the existing ROS/Gazebo composition in a visible X11 window, initially
**PAUSED** (`gazebo.start_running=false` in its frozen deployment).
This is an **unscored preview with physical actuation and command publication OFF**,
regardless of the saved actuation setting. It creates ownership/configuration
artifacts, not a scientific Run or metrics. Unsaved Builder edits are not inputs.

The preview panel reports Opening, Paused, Closing, Closed or Error. Paused means
an owned mapped GUI with configured paused startup; an initial ROS clock sample
may be absent while paused. It does **not** certify scene rendering, tracking
performance or safety acceptance. Preview time must remain fixed; observed time
advancement closes the preview with an error instructing use of dashboard Play.
Do not use Gazebo's native Play to bypass scored execution. Current WSL verification
opened the window but captured a blank viewport with Ogre rendering errors; see
[D2.6 evidence](validation/dashboard_simulation.md) before treating visualization
as verified.

**Close** requests cooperative cleanup through the existing Jobs/launch owner.
Opening the same selection twice reuses the current preview. Changing saved
robot/world/revision/variant requires Close and confirmed cleanup before Open.
**Play / Run** safely hands the preview to the existing Runner: it prevalidates
the same saved revision, closes the exact paused owner, confirms cleanup, then
opens a fresh scored GUI in a new isolated domain/partition. The original GUI is
not reused because Runner readiness, recording and reproducibility require a fresh
owned deployment. This handoff is reported explicitly. It never enables actuation
without the separate checkbox. Failed validation preserves the preview; uncertain
cleanup prevents a new owner. Changing selection still requires Close.

The unscored preview has no duration timer because time is paused. The scored Run
completes automatically after its saved duration, **following readiness and the
saved settling time**. Play executes the immutable study's full seed/variant matrix;
the selected preview is not silently substituted for the saved study. Preview and
scored execution cannot overlap within the same backend workspace. Browser refresh observes the existing owner and never opens a new one. Closing
the browser alone does not stop the preview; Close explicitly or stop the
owning dashboard server. The preview watches the server's PID/create-time and
closes when it exits. Closing the native GUI shuts down the owned composition.

This facility requires a working X11 display (`DISPLAY`, including WSLg XWayland)
and `libX11.so.6`. Native Wayland-only verification is unsupported and produces an
explicit unavailable state. Display access is checked before launch; successful
access cannot guarantee a functioning GPU/renderer. GUI jobs select Qt's `xcb`
platform explicitly. No desktop-wide process search/kill or external simulation
adoption occurs. Lost worker or unknown cleanup blocks restart; inspect ownership
artifacts rather than blindly retrying. Separate backend workspaces remain
independent and should not be used to evade this workspace's conflict guard.

The Recorder's drain now returns immediately when current readiness is satisfied,
so a later callback cannot consume the valid state before the Runner checks it.
The Runner rechecks the unchanged freshness predicate and records the exact passed
decision in `readiness.json`. The first verified GUI completion and recorded
metrics are documented in [Runner readiness verification](validation/runner_readiness.md).
Historical D2.6b failure evidence remains unchanged.

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
   variants and duration belong to the saved experiment definition. Close any
   unscored preview before ordinary headless Start, or use Play for the explicit
   cleanup-gated GUI handoff. A paused preview makes Play request a visible scored
   GUI even when the optional ordinary-Start GUI checkbox is unchecked; the UI
   explains the restart. It does not alter the separate actuation checkbox.
3. Click **Play / Run** once. The panel reports genuine execution status;
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
The table consumes authoritative backend Condition ranks: equal-weight mean across
matched Seeds of each Run's saved radial RMSE in metres, lower is better. Ties
share a rank. Sample count, sample standard deviation (unknown for one Seed),
per-seed values and Run links remain in collapsed Comparison details. Physical
safety/command gates, complete seed groups, frozen evidence and compatible
scientific conditions are required. The browser never computes ranks or substitutes
mean error for RMSE. Historical evidence without a frozen Condition/seed declaration
remains descriptive. See [comparison policies and verification](validation/scientific_comparability.md).

In Experiment Lab, **Advanced Comparison Setup** declares one exact registered
model or mathematical parameter leaf. Load a pinned baseline, declare the allowed
model choices or parameter values, Validate and Save each configuration as an
immutable revision with the same declaration and Seeds. Run the saved revisions
and add their Runs to the same Session. Dynamics studies are labeled separately;
other settings must remain protected and compatible. Existing sensor/multi-variable
policies are preserved; the Author/CLI can author multiple exact variables.

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

Current Session stores references to original Runs. Its comparison shows recorded
metrics alongside the backend's scientific comparability reasons, including
failed or unavailable Runs. Incomparable Runs are never ranked. The launch Session
remains fixed even if another Session is selected during execution. Refresh the
Session list/comparison after edits made by another client.

The Session selector's **⋯** menu opens compact rename or deletion dialogs.
Deletion previews the actual number of exclusively referenced managed Runs to
remove and shared Runs to preserve. Confirmed deletion removes that Session and
its exclusive managed artifacts; other Sessions' references protect shared Runs.
Historical roots registered only for viewing remain protected. Concurrent scope
changes invalidate confirmation; active Jobs or unresolved cleanup block deletion.
New Jobs retain their launch Session before a Run exists. Terminal legacy Jobs
require reliable cleanup evidence, including retained Run cleanup when available.

The Scenario selector's **⋯** menu provides **Rename Scenario** and **Delete
Scenario**. Rename persists a mutable Catalog display alias; original names,
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
