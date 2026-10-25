# M06.1 — Architecture hardening

2026-10-16. **Final verdict: CONDITIONAL.** The structural work, functional tests,
builds and isolated installation checks are complete. The bounded nominal Gazebo
run demonstrated the full active pipeline, but returned exit code 1 because the
existing physical joint-speed acceptance check failed. It would be incorrect to
declare an unconditional PASS. No safety threshold was relaxed or failed check
removed. M02–M05 limitations remain separate from the completed source cleanup.

Work is on `refactor/m06-architecture-hardening`, created from verified baseline
`7cceaf33b92377d7a6f5947b14ee046d0be5c6db` (`main`). No commit or push was made.
Untracked historical experiments and user data were preserved. No reset, clean,
controller/filter/target tuning, physics change, M07 or M08 implementation occurred.

The authoritative current navigation guide is [ARCHITECTURE.md](ARCHITECTURE.md).
The original M06 documents now carry a historical-state banner. They remain useful
as migration records, not current import instructions.

## Before and after

The baseline snapshot contains 450 source/packaging/resource files plus README and
the active installation checker (452 hashed files). The current source contains
445 such files: 47 old paths removed, 42 new paths, and 71 existing files
modified (largely imports). Moves count as an old path and a new path here.
Counts exclude generated caches. The exact tree is [m06_1_final_tree.txt](../results/validation/m06_1_final_tree.txt).

| Concern | Before at 7cceaf3 | After |
|---|---|---|
| Packages | 9 | Same 9; ownership enforced |
| ROS adapters | Canonical `nodes/` plus temporary aliases | Only canonical `nodes/`; same 7 ROS executable names |
| Estimator construction | Embedded in ROS node | Pure `factory/estimator_factory.py`; resource lookup stays in node |
| Target frame conversion | Numerical transform embedded in control node | Deterministic `adapters/prediction_frame_transform.py`; TF lookup stays in node |
| YAML loading | Canonical loader plus simulation/prediction aliases | Shared generic loader and prediction-specific section validation, no aliases |
| Future base types | Empty aerial/omnidirectional namespaces | Removed; real differential-drive model retained |
| Domain grouping | Generic common/types, scattered controllers/evaluation/SDF | Spatial/domain records; flat controllers/evaluation; SDF under gazebo |
| Tree display | Filesystem ordering | Actual tree rendered by one tested semantic-order utility |
| Simulation placeholders | Four empty Python/YAML files | Removed; real Gaussian synthetic measurement behavior tested |
| Structural tests | Old-to-new alias identity expectations | 52 architecture/tree/historical-loader checks |
| Functional suite | 873 functional cases at baseline | 921 passed; obsolete alias cases replaced by current contracts |

The first cleanup phase removed shims and extracted two responsibilities. The
subsequent semantic-grouping request was applied to that working tree without
restarting or discarding its evidence. The final tree has canonical structural
folders and domain groupings, with no compatibility namespaces.

## Intentional internal API removals

Every entry below was a temporary six-line module alias. Active imports, tests,
entry points, launch files and current validation tools were inspected; supported
consumers already use the canonical locations. The installed checker now requires
the old imports to fail, rather than requiring alias identity.

| Removed Python module | Canonical replacement |
|---|---|
| robot_control.ros.control_node | robot_control.nodes.control_node |
| robot_estimation.estimation_node | robot_estimation.nodes.estimation_node |
| robot_prediction.prediction_node | robot_prediction.nodes.prediction_node |
| robot_prediction.system_factory | robot_prediction.factory.system_factory |
| robot_prediction.config_loader | robot_prediction.configuration.prediction_config |
| robot_simulation.actuation.velocity_actuation | robot_simulation.nodes.velocity_actuation |
| robot_simulation.factory.config_loader | robot_configuration.yaml_loader |
| robot_bringup.model_factory | robot_bringup.factory.model_factory |
| robot_bringup.tracking_feedback_tf | robot_bringup.nodes.tracking_feedback_tf |
| robot_bringup.tracking_visualization | robot_bringup.nodes.tracking_visualization |

The now-unused `robot_control/robot_control/ros/` directory, including its
docstring-only initializer, was removed. This deliberately breaks those obsolete
internal Python paths. ROS executable names, topics/messages and active launch
arguments remain unchanged. The historical migration map is retained as a removal
inventory for validation; it does not provide runtime compatibility.

Other audited deletions:

| Source path (under src/) | Reason |
|---|---|
| robot_models/robot_models/base/aerial/ | Only an empty initializer/cache; no aerial implementation |
| robot_models/robot_models/base/omnidirectional/ | Same; no implemented base model |
| robot_simulation/robot_simulation/disturbances/gaussian_noise.py | Empty stub; actual noise generation is in measurements |
| robot_simulation/robot_simulation/gazebo/ground_truth_adapter.py | Empty stub; existing working adapters/validation were preserved |
| robot_simulation/robot_simulation/interfaces/integrator.py | Empty file, no supported contract or caller |
| robot_simulation/config/disturbances.yaml | Empty, unreferenced resource; installation glob adjusts automatically |
| robot_bringup/test/test_m06_migration.py | Obsolete alias-identity expectations; replaced with current architecture tests |

`robot_simulation/test/test_measurements.py` was empty but the capability exists.
It now tests Hx/timestamp/input immutability, seeded Gaussian reproducibility and
empirical covariance, and rejection of nonfinite input. Legitimate package
initializers, ROS resource markers and actual interfaces were retained. The
path-level deletion inventory is [m06_1_deleted.json](../results/validation/m06_1_deleted.json).

## Semantic consolidation and exact moves

The final mandatory grouping work moves these 21 modules without numerical changes:

| Previous module | Final module |
|---|---|
| robot_models.common.rotations | robot_models.spatial.rotations |
| robot_models.common.transforms | robot_models.spatial.transforms |
| robot_models.whole_body.factory | robot_models.factory.whole_body_factory |
| robot_models.reference.geometry.reference_geometry | robot_models.interfaces.reference_geometry |
| robot_estimation.common.estimator_interface | robot_estimation.interfaces.estimator_interface |
| robot_estimation.common.measurement | robot_estimation.measurement |
| robot_estimation.common.state | robot_estimation.state |
| robot_estimation.common.timestamp_buffer | robot_estimation.timestamp_buffer |
| robot_estimation.filters.filter_interface | robot_estimation.interfaces.filter_interface |
| robot_estimation.robot.sensors.sensor_interface | robot_estimation.interfaces.sensor_interface |
| robot_control.baseline.jacobian_controller | robot_control.controllers.jacobian_controller |
| robot_control.optimization.weighted_least_squares | robot_control.controllers.weighted_least_squares |
| robot_prediction.confidence.performance_monitor | robot_prediction.evaluation.performance_monitor |
| robot_prediction.confidence.prediction_confidence | robot_prediction.evaluation.prediction_confidence |
| robot_prediction.confidence.temporal_alignment | robot_prediction.temporal_alignment |
| robot_prediction.residuals.prediction_residual | robot_prediction.evaluation.prediction_residual |
| robot_prediction.residuals.residual_statistics | robot_prediction.evaluation.residual_statistics |
| robot_simulation.geometry.model_exporter | robot_simulation.gazebo.model_exporter |
| robot_simulation.geometry.sdf_generator | robot_simulation.gazebo.sdf_generator |
| robot_simulation.state.target_state | robot_simulation.adapters.target_state |
| robot_description.configuration | robot_description.adapters.configuration |

The old models `types.py` was removed: Pose/Twist move unchanged to
`spatial/spatial_data.py`, RobotState to `whole_body/robot_state.py`, TargetState to
`target/target_state.py`, and ConstraintSet to `constraints/constraint_set.py`.
Estimator state and measurement remain two clear flat modules; timestamp buffering
is a flat named timing component, not several one-file namespaces.

Contracts extracted from mixed implementation modules, preserving their class code:

| Previous location | Canonical contract/data location |
|---|---|
| control/tracking/velocity_task.py | control/interfaces/velocity_task.py: VelocityTask and TaskLinearization |
| control/tracking/reference_selector.py | control/interfaces/geometry_parameter_policy.py: GeometryParameterPolicy |
| estimation/robot/models/measurement_model.py | estimation/interfaces/robot_measurement_model.py: RobotMeasurementModel |

Concrete task, reference-policy and measurement implementations remain in their
domains. Models' reference-geometry and estimation's filter/sensor interfaces also
move to canonical `interfaces/` as listed above. No obsolete import facade remains.
Empty initializers belonging to removed common/baseline/optimization/confidence/
residuals/geometry namespaces were deleted; required markers were added for the
new real packages. Unused empty `robot_interfaces/include/robot_interfaces/`, its
parent `include/` and `src/` scaffolding were removed after checking interface-only
CMake registration. No tracked C++ implementation was present or removed.

Accepted grouping: DLS and WLS both implement Controller, so flat `controllers/`
provides one discovery point; WLS is not an independently reusable solver API.
Prediction confidence, residual statistics and performance monitoring share one
evaluation responsibility, so they form flat `evaluation/`. Both geometry modules
produce/export SDF, so they join existing Gazebo builders. Spatial math belongs
in `spatial/`, not `common/`. Physical representation conversion belongs in
`description/adapters/`, pure injected construction in `models/factory/`, and
simulation state/vector translation in `simulation/adapters/`.

Rejected grouping: temporal interpolation is reusable prediction processing, not
performance evaluation; history is reusable buffering; fusion combines predictions;
baseline versus adaptation, robot versus target estimation, base/manipulator/whole
body, integration versus engine, and safety versus pipeline retain real behavioral
boundaries. OnlineSequenceLearner contains concrete validation/prediction behavior
and remains an algorithm base in online_adaptation, not an empty interface wrapper.

The one centralized ordering policy lives in `tools/show_architecture.py`: existing
nodes → interfaces → factory → adapters → package-specific conceptual domain order
→ root files → initializer. Unknown real domains stay visible; caches are ignored.
It generates the linked full tree from actual files. Tests cover shared/domain/file
ordering, unknown domains, absent optional folders, caches, and the current tree.
No manually maintained competing source tree was introduced.

Exact source/destination/class inventory:
[m06_1_semantic_moves.json](../results/validation/m06_1_semantic_moves.json). Generic `common/`
paths and obsolete semantic namespaces are explicitly forbidden by architecture
tests. Package-level types.py remains optional for compact aliases; no package
needs a generic type dumping ground or broader Any annotations.

## Responsibility review and retained components

All nine packages were reviewed against imports, manifests, setup/resource
registration, launch/configuration references and tests. Their current roles and
actual dependency graph are in the architecture guide. No package dependency had
to change: the new estimator factory uses existing NumPy/domain dependencies; the
new control helper uses an existing local data contract. Lower layers never import
bringup, models remain ROS/Gazebo independent, and no simulation control moved into
the reusable controller.

The two extractions have narrow boundaries:

- Estimator factory accepts the existing parameter mapping and physical joint
  mapping, returns the configured estimator/constraint, and has no ROS or package
  resource lookup. Models, initial distributions, indexing, bounds, noise and
  tolerances are unchanged. The node still owns subscriptions, causal measurement
  timing, publication and description-resource resolution.
- Target frame conversion accepts the trajectory, rotation, offset and destination
  frame. It copies states and applies `p' = Rp+t`, `v' = Rv`, preserving timestamps.
  TF availability/lookup and reference-time policy remain in the ROS adapter. This
  keeps the existing fixed-inertial-frame assumption; it does not add moving-frame
  transport terms.

The other inspected large modules were deliberately retained: IMM's model mixing,
adaptation and prediction form one cohesive algorithm; simulation's clock and
asynchronous Gazebo pose commit form one lifecycle; actuation's session validation,
raw feedback, wall watchdog and publication form one transport adapter. Its pure
position guard/reference integrator already live separately in `actuation/`.
Splitting these by length would add navigation cost without an independent unit.

Intentional alternatives are not dead duplication: DLS versus whole-body WLS,
Kalman versus extended Kalman filtering, reference versus target-motion models,
kinematic models versus stochastic process models, control command projection
versus physical actuator guarding, and generic model composition versus bringup
deployment composition. Tested predictor/adaptation/fusion components remain valid
library alternatives even when the current deployed factory does not select them.
The existing learned predictor interface is a real extension contract, not a claim
that learned prediction is deployed. Domain-local contracts stay in their domains;
ROS message contracts stay in `robot_interfaces`.

## Packaging, documentation and enforcement

`test_architecture.py` uses ordinary pytest, AST imports, package XML and setuptools
discovery rather than a new analysis framework. It checks allowed/declared project
dependencies, cycles, runtime **and launch** imports (including function-local
imports), ROS-free numerical layers, canonical node implementation identity,
packaged source, obsolete/empty paths and existing inherited launch-profile
resources. Checks cover flat root numerical modules as well as domain folders.
The renderer and archived-loader tests bring final architecture coverage to 52
cases. Arbitrary dynamic import expressions cannot be proven by this static
check; installed entry-point checks and functional tests supplement it.

The active installation checker now checks the seven explicit canonical console
contracts and rejects all ten removed APIs. It no longer needs an untracked old
audit JSON to determine expected entry points. Its optional `--output` prevents an
ordinary invocation from overwriting historical M06 results. It checks import/share
origins, resource bytes, inherited YAML resolution and generated Xacro limits.
It additionally verifies 28 semantic module locations and rejects 22 removed
semantic imports against a newly built non-symlink prefix.

Active replay/probe imports were migrated. Immutable historical estimator sources
retain their original bytes: `archived_estimation_imports.py` translates only import
statements in memory at compilation. Tests verify it leaves arithmetic, comments
and literal strings unchanged. This is validation-only loading, not an installed
legacy API. Four current/archived probes still pass.

All nine package manifests were parsed. Existing Python setup versions/licenses
agree with their manifests; descriptions and declared domain dependencies are
consistent. There is no evidence authorizing replacement identities or license
grants. Retained metadata debt: estimation declares MIT without a bundled text;
prediction's LICENSE is empty; description (`user@todo.todo`), interfaces
(`user@example.com`) and estimation (`lior@example.com`) have placeholder maintainer
emails. These require owner-supplied provenance. They are documented exceptions,
not misleading source capabilities. Details: [metadata audit](../results/validation/m06_1_metadata_audit.json).

README lists all nine packages and links the current guide. Local links in README,
the current guide, this report and the historical M06 banner targets were checked.
Historical documents/captures retain their original references and measurements;
the link check is not a claim that every historical local run ships in Git.

`docs/validation/` contains executable verification utilities and selected compact
evidence. Historical evidence there may be tracked or untracked; `git ls-files`
identifies what a clone includes. New raw output is in ignored
`results/m06_1_smoke/`, `results/m06_1_semantic_smoke/` and their instrumentation
directories. The before-source snapshot
is preserved locally at ignored `results/validation/m06_1_before/`, with `COLCON_IGNORE`. A second ignored `m06_1_semantic_before/` snapshot
preserves the completed first phase. All **792** pre-existing untracked evidence
paths still exist ([preservation inventory](../results/validation/m06_1_preserved_local_evidence.json)).
Build/log output remains ignored. No large output or prior evidence was deleted.

## Verification evidence

| Check | Actual result / source |
|---|---|
| Expected failing architecture tests before cleanup | 17 failed / 9 passed; obsolete paths and placeholders identified (`m06_1_architecture_red.log`) |
| Expected failing extraction tests | Two missing-module collection errors before implementation (`m06_1_extraction_red.log`) |
| Focused initial tests | 72 passed (`m06_1_focused_initial.log`) |
| Initial architecture + new deterministic tests | **35 passed**, including 27 architecture cases (`m06_1_focused_final.log`), before semantic expansion |
| Final architecture/tree/loader tests | **52 passed** (`m06_1_semantic_architecture_final.log`) |
| Full functional regression | **921 passed**, 18 existing lint/copyright checks separately deselected, 8.17 s (`m06_1_semantic_regression_final.log`); first-phase 896-pass log preserved; no genuine functional failure deselected |
| Syntax/canonical imports | All source/launch Python parsed; seven canonical nodes loaded from isolated install |
| Syntax/docs links | **389 Python files parsed; 39 local links resolved**, no broken links in audited current/M06 docs ([JSON](../results/validation/m06_1_syntax_and_links.json)) |
| Package discovery and normal build | **9 discovered; 9 built**, 12.1 s (`m06_1_packages.txt`, `m06_1_semantic_build.log`) |
| Non-symlink isolated build | **9 built**, 20.7 s (`m06_1_semantic_copy_build.log`) |
| Copied install, workspace paths removed, cwd=/tmp | **7 console entries, 10 absent M06 aliases, 28 semantic modules, 22 removed semantic paths, 64 resources and Xacro limits verified** ([JSON](../results/validation/m06_1_semantic_installed_checks.json)) |
| Configuration/physical preservation | **64 surviving YAML/Xacro/SDF/message assets byte-identical**; only empty disturbances YAML removed ([JSON](../results/validation/m06_1_preservation.json)) |
| Extracted filter numerical equivalence | Baseline/new robot and target mean, covariance and time match **exactly**, after prediction and observation updates; zero absolute/relative tolerance |
| Instrumentation preservation | All four current/archived estimator/actuator probes select expected implementations and globals ([JSON](../results/validation/m06_1_semantic_instrumentation.json)) |
| Semantic numerical preservation | **155 production modules** match pre-semantic AST ignoring only imports; moved 5 records and 4 contract/data classes have exact class AST equality ([JSON](../results/validation/m06_1_semantic_equivalence.json)) |
| Independent bounded code review | No critical/important structural or behavior-preservation finding ([review](validation/m06_1_review.md)) |
| Bounded nominal Gazebo | Active functional loop; **physical speed check failed**, exit 1; detailed results below |

The repository's existing style/docstring/copyright debt is separate from the
functional suite. These 18 checks were also separated in M03/M06; no repository-wide
lint PASS is claimed. The full 921-case suite includes the final numerical-root import scope and
archived-loader test. The installation checker was rerun against the fresh semantic
copy. Earlier phase-one build/test/installation/smoke evidence remains available;
final conclusions below use the post-semantic-migration run.

### Nominal physical closed-loop smoke

Scenario `reactive_tracking_oscillatory.yaml`, strategy `whole_body.yaml`, explicit
actuation, headless, physical ground truth, unchanged limits/PID. Requested duration
16 simulated seconds from target initialization, with 4 seconds settling. Tracking
evaluation interval was 5.252–17.252 simulated seconds; **239** selected control
samples. Measurements use physical Gazebo base pose plus measured joints and
authoritative FK against target truth, not just the estimator's internal error.
Compact source-linked evidence: [m06_1_semantic_smoke_summary.json](../results/validation/m06_1_semantic_smoke_summary.json).

| Metric | Measured |
|---|---|
| Absolute distance error, mean / max | **0.09037 / 0.26059 m** |
| Pointing error, mean / max | **7.984 / 18.575 degrees** |
| Mean RTF | **0.8415** |
| Controller compute, mean / p95 | **17.82 / 37.25 ms** |
| Physical base excursion / yaw excursion | **0.680 m / 0.167 rad** |
| Four joint excursions | **[0.2148, 0.1729, 0.1236, 0.4108] rad** |
| Joint speed peaks, after settling | **[0.2201, 0.5337, 1.7546, 1.5142] rad/s** |
| Authoritative speed limits | **[1.0, 1.0, 1.0, 1.5] rad/s** |
| Maximum position violation, all 7826 observer joint messages | **0 rad** |
| Shoulder range, all received feedback | **0.000640–0.658307 rad** |
| Minimum measured first-link/base clearance | **0.04000 m** |
| Position recovery events / modes | **0 / normal** |
| Actuation active fraction, selected telemetry | **1.0** |
| Watchdog timeout delta after settling | **0** |
| Invalid control messages in evaluation interval | **0** |

The elbow peak exceeds 1.0 rad/s by about **0.755 rad/s**, and wrist peak exceeds
1.5 rad/s by about **0.0142 rad/s**; both exceed the unchanged 0.001 tolerance. Physical bounds, watchdog,
safe command velocity, shoulder elevation/clearance and live tracking-TF checks
all passed. Physical-speed acceptance failed. No attempt was made to solve that
known actuator/PID issue during structural work.

Meaningful pipeline telemetry: 765 robot estimates, 1300 target observations,
344 control and safe-command messages, 3548 joint-position command samples,
172 actuator telemetry samples and 320 live tracking-TF samples were recorded.
The observer counted 277 prediction messages and 277 target estimates (recorded
as 17.3125 Hz when divided by requested duration); all required stages and the
Gazebo clock remained active. Final estimator trace
reports 7817 accepted joint and 945 wheel observations, zero fusion errors and no
clock resets. The trace/stream windows differ because processes finish asynchronously;
counts are not a synchronized throughput benchmark. Original reported topic rates
divide whole observed counts (including startup) by requested duration; they are
not precise steady-state sensor publication frequencies.

Final cumulative actuator telemetry reports zero timeouts/rejected commands,
25 inactive commands during startup and 318 accepted commands. There were no
timeout increments or invalid control messages in the evaluation interval. The
first phase had startup timeout/rejection counters; its evidence remains unchanged.
Process cleanup was verified after the bounded runs.

The earlier M06 nominal smoke had mean/max distance 0.09097/0.29171 m, mean/max
pointing 7.242/16.785 degrees, RTF 0.7292 and the same physical-speed failure.
The preserved first-phase M06.1 run measured mean distance 0.07289 m, pointing
7.170 degrees and RTF 0.8869, also failing physical speed. These are separate short
runs under different host load, not a controlled
performance or tracking-improvement comparison. Exact configuration and filter
equivalence checks are the behavior-preservation evidence; no improvement is claimed.

## Reproduction

Run from `/home/user/ros2_ws` with the existing Jazzy/Gazebo dependencies installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon list --base-paths src
colcon build --symlink-install --packages-up-to robot_bringup
source install/setup.bash
python3 -m pytest src --import-mode=importlib -p no:launch_testing -p no:launch_ros -q \
  -k 'not flake8 and not pep257 and not copyright'
python3 -m pytest src/robot_bringup/test/test_architecture.py \
  src/robot_estimation/test/test_estimator_factory.py \
  src/robot_control/test/test_prediction_frame_transform.py \
  src/robot_simulation/test/test_measurements.py \
  src/robot_bringup/test/test_architecture_tree.py \
  --import-mode=importlib -p no:launch_testing -p no:launch_ros -q
```

Isolated install verification (use a fresh prefix when repeating):

```bash
colcon --log-base /tmp/m06_1_semantic_copy/log build --base-paths src \
  --build-base /tmp/m06_1_semantic_copy/build --install-base /tmp/m06_1_semantic_copy/install \
  --merge-install --packages-up-to robot_bringup
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  bash --noprofile --norc -c '
    source /opt/ros/jazzy/setup.bash
    source /tmp/m06_1_semantic_copy/install/local_setup.bash
    cd /tmp
    python3 /home/user/ros2_ws/tools/validation/m06_verify_installation.py \
      --prefix /tmp/m06_1_semantic_copy/install'
```

Ordinary interactive launch, with explicit physical actuation:

```bash
ROS_DOMAIN_ID=151 GZ_PARTITION=m06_1_manual \
ros2 launch robot_bringup closed_loop.launch.py gui:=false \
  config:=config/reactive_tracking_oscillatory.yaml \
  strategy_config:=config/whole_body.yaml enable_actuation:=true
```

Reproduce the bounded measurement with a **new** output directory; the observer's
nonzero status for existing physical overspeed is expected and must not be suppressed:

```bash
ROS_DOMAIN_ID=151 python3 tools/validation/v2_tracking_gazebo.py \
  --case reactive --config config/reactive_tracking_oscillatory.yaml \
  --strategy-config config/whole_body.yaml --physical-ground-truth \
  --without-markers --shoulder-posture --estimator-trace --actuation-trace \
  --duration 16 --settling-time 4 --wall-timeout 120 \
  --output results/m06_1_semantic_smoke_repeat
```

Local baseline-preservation check (requires preserved before-source snapshot):
`python3 tools/validation/m06_1_verify_preservation.py`. Snapshot is intentionally
not a required installed runtime resource. Fresh clones can compare source assets
to commit `7cceaf3`; the compact preservation result records this workspace's run.
The active instrumentation checker accepts `--case estimation` or `--case actuation`,
optionally `--archived`, with an explicit fresh `--output` directory. Do not rerun
its historical aggregate default over existing M06 evidence.

## Changed files and remaining debt

Exact edited/added/deleted task paths, excluding pre-existing untracked experiments:
[m06_1_modified_files.txt](../results/validation/m06_1_modified_files.txt). Changes comprise
the ten alias removals and unused directories/stubs, two cohesive extractions,
behavioral/architecture tests, current docs, ignored rollback snapshot declaration,
the semantic moves and tested tree-rendering utility, and validation evidence/checker updates. No package manifest, active YAML, Xacro,
message or launch file changed.

Requested review outputs: [git status --short](../results/validation/m06_1_git_status_short.txt)
and [git diff --stat](../results/validation/m06_1_git_diff_stat.txt). Git's unstaged diff stat
does not include new untracked files; the task inventory above includes them and
excludes the preserved pre-existing experiment outputs. Cleanup evidence confirms
[no robot/Gazebo workloads remain](../results/validation/m06_1_process_cleanup.json).

Completed: canonical paths, ownership enforcement, placeholder cleanup, packaging,
documentation and all structural verification. No architectural investigation is
left pending. Open research limitations: M02 physical velocity enforcement, M03 skid
localization/covariance consistency, aggressive tracking/prediction freshness and
host-dependent performance. Open distribution hygiene: verified maintainer/license
provenance and existing lint debt. These are explicitly scoped future work, not
reasons to invent additional architecture during M06.1.

**CONDITIONAL** is retained because the mandatory live run's physical-speed check
fails, despite passing structural and functional integration checks. The code is
ready for review; physical actuation safety is not certified. Stop here before M07.
