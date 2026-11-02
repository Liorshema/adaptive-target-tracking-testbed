# Canonical component configuration: Task 1 validation

Updated 2026-10-27. Scope is the canonical defaults, extensible owner construction,
strict schemas and discovery foundation requested in Task 1. The implemented
[YAML, ownership, interface and compatibility guide](../component_configuration.md)
is the permanent developer entry point. Task 2 scenario cleanup and the dashboard
are excluded.

## Baseline and evidence

Work resumed from the dirty tree without resetting earlier ownership migration
changes. Comparisons use the task-start capture, not Git HEAD. Evidence lives in
ignored `results/component_architecture/`: `baseline.json`, `before.tar.gz`,
`before_preparations.json`, RED/GREEN logs, normal/copied build logs, installation
verification, runtime studies and the final change manifest. Prior evidence is
preserved separately and is not represented as a fresh runtime result.

The initial functional baseline was **1,621 passed, 18 style tests deselected,
1 expected failure**. Test-only plug-ins cover Dynamics, Estimation, Prediction
and Control; additional cases cover strict inactive-family validation, arbitrary
library dimensions, incompatible deployment contracts, immutable defaults,
custom predictor snapshots, full effective dynamics snapshots and provenance.

Review found and reproduced four extension gaps: invalid estimator results could
escape the factory contract; custom target construction assumed builtin parameter
names; WLS vector metadata lacked constructor bounds; and custom prediction
parameters were lost by flat run snapshots. Review also identified incomplete
provenance for partial dynamics documents. Focused regression tests first failed,
then **79 passed** after minimal fixes. Independent re-review reported no remaining
critical or important findings. No physical equations or acceptance gates changed.

## Configuration equivalence

All **21 catalog variant/seed preparations** match the captured baseline after
normalizing only two run-local description paths and the canonical representation
of `prediction_configuration`. The derived comparison-invariant hash is recomputed
from that normalized representation for this comparison only. Runtime artifacts,
immutable catalog hashes, historical revision contents and saved results are not
rewritten. The check masks no numerical values, component identities, selected
models, seeds, frames, physical constraints or safety settings.

Evidence: `check_equivalence.py`, `equivalence_verified.log`, `equivalence.json`
(`cases: 21`, `differences: {}`), before/after preparations. Prediction tests also
verify legacy/canonical defaults and actual residual-correction numerical equality.

## Functional, architecture and installation verification

The final full functional/architecture run passed **1,689 tests**, with the same
**18 style exclusions and one expected failure** as baseline (243.09 s). The
preceding full run had 1,688 passes and one old flat-snapshot assertion failure;
that fixture was migrated to the canonical key, all 22 catalog tests passed, and
the entire final run above then passed. Evidence: `functional_verified.log`,
`catalog_final.log`, and the retained diagnostic `functional_final.log`.

Normal and copied-install builds each completed **all 10 packages**. The copied
installation is `/tmp/component_architecture_install`, built without symlinks in a
separate build directory. Verification runs from `/tmp` after clearing source-overlay
environment variables. It checks the existing M06–M09 chain, eight console entry
contracts, resources, authoritative Xacro limits, explicit nominal/CV/DLS selection,
invalid-manifest rejection, visualization launch construction, **15 new/affected
module origins**, **10 discovery families / 20 components**, default-schema
validation and rejection of a library predictor in the deployed context. The
installed CLI lists metadata and validates the actual revision-2 prediction study.

The first verifier attempt hit the sandbox's read-only default ROS log directory;
the evidence script now explicitly directs logs into this task's writable evidence
root. A generated launch `.pyc` from that attempt was recorded and removed from
the temporary copied installation, then verification ran with bytecode writing
disabled. No source resource or historical artifact was removed.

The 15 core new/rewritten registry, configuration and owner-factory modules pass
scoped Flake8 and pep257 checks. Whole-repository legacy style gates remain outside
the functional run (the same 18 exclusions as baseline). Auto-loaded ROS launch
pytest plugins must be disabled for cross-package collection because they import
same-named test modules in prepend mode; this is a harness issue, not a project
assertion bypass.

## Reproduction

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 -m pytest -p no:launch_testing -p no:launch_ros --import-mode=importlib \
  -q src tools/validation -k 'not flake8 and not pep257 and not copyright'
ros2 run robot_experiments run components list
ros2 run robot_experiments run components show predictor imm
ros2 run robot_experiments run --experiment EXP-PREDICTION-COMPOSITION \
  --revision 2 --validate-only
```

Copied installation and this capture's verification:

```bash
colcon build --build-base /tmp/component_architecture_build \
  --install-base /tmp/component_architecture_install --executor sequential
bash results/component_architecture/verify_installed.sh
python3 results/component_architecture/check_equivalence.py
python3 results/component_architecture/audit_changes.py
```

Gazebo comparisons use the installed experiment runner and original acceptance
settings, without retries to obtain PASS:

```bash
ros2 run robot_experiments run --experiment EXP-PREDICTION-COMPOSITION \
  --revision 2 --output results/component_architecture/runtime
ros2 run robot_experiments run --experiment EXP-SENSOR-NOISE-COMPOSITION \
  --revision 2 --output results/component_architecture/sensor-runtime
python3 tools/validation/compute_only_gazebo.py \
  --output results/component_architecture/compute-only
```

The actual controlled comparison script `run_installed.sh` clears prior overlays,
sources the copied installation, runs outside the workspace, records ROS logs
under this evidence root and records each command's exit code. Use unique output
parents for further captures; experiment study directories never overwrite runs.
The compute-only probe now accepts `--output` so earlier `/tmp/control-runtime`
evidence is preserved. It observes the normal permanent simulation launch.

Continuous simulation uses the same existing launch:

```bash
ros2 launch robot_experiments tracking.launch.py \
  system_config:=config/systems/nominal_tracking.yaml gui:=false
# Add enable_actuation:=true only when physical commands are wanted.
ros2 launch robot_simulation simulation.launch.py gui:=false
```

## Live ROS/Gazebo results and verdict

All four first-attempt runs completed through simulation → estimation →
prediction → control → physical actuation using the copied installation, explicit
component selection and frozen run inputs. Results below use the declared
post-settling windows (6 s for prediction, 8 s for sensing); they are short
integration observations, not a controlled performance-improvement claim.

| Run (seed 42) | Distance error mean / max (m) | Pointing error mean / max (deg) | RTF | Run acceptance |
|---|---:|---:|---:|---|
| Prediction IMM | 0.1267 / 0.3063 | 8.1696 / 17.1271 | 0.599 | PASS |
| Prediction CV | 0.1199 / 0.2133 | 11.2745 / 18.9903 | 0.747 | FAIL: one feedback rejection |
| Sensor baseline | 0.1118 / 0.2714 | 8.4948 / 19.7881 | 0.591 | FAIL: one invalid control cycle and one feedback rejection |
| Sensor higher noise | 0.1346 / 0.3183 | 8.6422 / 23.6154 | 0.668 | PASS |

Both study commands returned exit **3**, retained in `runtime_exit_codes.txt`.
No run was repeated to obtain PASS. Full metrics, statuses, source snapshots and
raw telemetry are in `runtime/` and `sensor-runtime/`; `all_runtime_results.json`
and `runtime_gate_evidence.json` summarize every attempt and failing sample.

Physical position and speed gates passed in all four evaluation windows, with
**100% covered duration**. Maximum raw position overshoot was **6.884e-13 rad**;
maximum speed overshoot was **0 rad/s**, against the unchanged **0.001 rad/rad/s**
numerical gate tolerances. Every run reported zero watchdog events and zero new
rejected commands in its evaluation window. Those windowed deltas do not imply
zero startup counters. Every launch cleaned up with **no survivors**.

At CV source time **14.038 s**, the feedback-rejection counter rose from 0 to 1,
with reason `Future timestamp` (feedback stamp 14.018 s, recorder clock 13.848 s).
In the sensor baseline, the invalid control cycle at **9.220 s** reported
`Robot state is stale`; at **10.624 s** one feedback rejection was added, also
reporting `Future timestamp` (feedback stamp 10.604 s, recorder clock 10.440 s).
The recorder's clock is an independent observation and is not proof of the
actuator's instantaneous clock value. These are observed timing/freshness failures;
this migration does not establish their full scheduling cause. Validation held
control safely; timestamp checks, limits and zero-event gates were preserved.
They remain actionable runtime-reliability work rather than a reason to weaken
acceptance or alter numerical algorithms in this configuration task.

The permanent `ros2 launch robot_simulation simulation.launch.py gui:=false`
compute-only probe passed **837 cycles**, preserved `publish_commands=false`, and
observed no command publisher. It checked stale-joint rejection and recovery after
world-clock reset. Across 4,391 holding samples from 5.008 to 34.988 simulated
seconds, maximum holding-position error was **5.193e-11 rad**, maximum joint speed
**2.678e-14 rad/s**, and minimum tracking height above base **0.439177 m**. The
configured raised posture also settled after reset. No temporary holding plugin
was added. Evidence: `compute-only/result.json` and `compute_only.log`.

The installed headless backend worker produced active command telemetry, executed
its frozen revision **1** after the catalog advanced to revision **2**, then
cancelled cooperatively. Repeated stop remained idempotent, history retained the
original run identity, remaining work was aborted, and cleanup reported no
survivors. Evidence: `job_verification.json`, `job_stopped.json`, `live_jobs.log`.

**Verdict: CONDITIONAL overall.** The architectural foundation, extension tests,
configuration equivalence, both builds, copied installation and backend lifecycle
are verified. Live integration is functional, but two first-attempt runs failed
existing timing/freshness gates. IMM's historical CONDITIONAL scientific status
is preserved despite its successful short run here. No blocking schema/factory
regression remains demonstrated by the tests; the runtime timing limitations are
explicitly unresolved. Do not present these observations as hardware certification.

## Limitations and next task

IMM retains its existing CONDITIONAL scientific runtime status. A short successful
smoke test cannot resolve historical coverage/timing findings. Existing localization
slip/covariance and physical-safety limitations retain their prior evidence-based
verdicts; metadata `deployable` certifies an implemented adapter contract, not
scientific acceptance or hardware safety. CA/CT, RLS and baseline fusion remain
library-only. Schema/context inspection does not replace coupled domain constructor
and full deployment validation.

Task 2 may review redundant explicit scenario overrides and obsolete composition
resources. Preserve independent physics/sensor/field responsibilities, meaningful
physical variants, immutable revisions and historical reproduction bridges. This
task introduces no source deletion and performs no scenario inventory cleanup.

## Exact changed files

The final task-relative list follows here; it excludes earlier dirty work already
present when Task 1 started. Machine-readable hashes and preservation checks are
in `change_manifest.json` alongside `changed_files.txt`. Generated validation
artifacts remain under `results/component_architecture/`, outside production code.

<!-- TASK_FILE_LIST -->
```text
M README.md
M docs/ARCHITECTURE.md
M docs/experiment_management.md
M src/robot_bringup/robot_bringup/factory/system_configuration.py
M src/robot_bringup/test/test_architecture.py
M src/robot_bringup/test/test_explicit_composition.py
M src/robot_bringup/test/test_system_configuration.py
M src/robot_configuration/robot_configuration/resources.py
M src/robot_control/config/control.yaml
M src/robot_control/robot_control/factory/component_registry.py
M src/robot_control/robot_control/factory/project_components.py
M src/robot_control/test/test_control_factory.py
M src/robot_control/test/test_control_node.py
M src/robot_estimation/config/estimation.yaml
M src/robot_estimation/package.xml
M src/robot_estimation/robot_estimation/factory/estimator_factory.py
M src/robot_estimation/test/test_estimation_node.py
M src/robot_estimation/test/test_estimator_factory.py
M src/robot_estimation/test/test_pose_robustness.py
M src/robot_experiments/package.xml
M src/robot_experiments/robot_experiments/cli.py
M src/robot_experiments/robot_experiments/configuration.py
M src/robot_experiments/test/test_catalog.py
M src/robot_experiments/test/test_configuration.py
M src/robot_prediction/config/prediction.yaml
M src/robot_prediction/robot_prediction/configuration/prediction_config.py
M src/robot_prediction/robot_prediction/factory/system_factory.py
M src/robot_prediction/robot_prediction/trajectory_prediction.py
M src/robot_prediction/test/test_component_selection.py
M src/robot_simulation/robot_simulation/factory/registry.py
M src/robot_simulation/robot_simulation/factory/simulation_factory.py
M src/robot_simulation/test/test_actuation_factory.py
M tools/validation/closed_loop_gazebo.py
M tools/validation/closed_loop_safety.py
M tools/validation/compute_only_gazebo.py
M tools/validation/m03_robustness_validation.py
M tools/validation/shoulder_experiment_metrics.py
M tools/validation/test_aggressive_analysis.py
M tools/validation/test_m03_analysis.py
M tools/validation/v2_estimation_audit_collect.py
M tools/validation/v2_estimation_audit_replay.py
M tools/validation/v2_orientation_probe.py
M tools/validation/v2_target_replay.py
A docs/component_configuration.md
A docs/validation/component_configuration.md
A src/robot_configuration/robot_configuration/components.py
A src/robot_configuration/test/test_components.py
A src/robot_control/robot_control/configuration.py
A src/robot_control/test/test_component_architecture.py
A src/robot_estimation/robot_estimation/configuration.py
A src/robot_estimation/robot_estimation/interfaces/robot_estimator.py
A src/robot_estimation/test/test_component_architecture.py
A src/robot_experiments/robot_experiments/components.py
A src/robot_experiments/test/test_components.py
A src/robot_prediction/robot_prediction/factory/component_registry.py
A src/robot_prediction/robot_prediction/interfaces/correction.py
A src/robot_prediction/test/test_prediction_components.py
A src/robot_simulation/config/dynamics.yaml
A src/robot_simulation/test/test_component_architecture.py
```

Deleted source files in Task 1: **none**. Earlier ownership-migration deletions
remain unchanged in the pre-existing Git diff. No commit or push was made.


## Task B — mathematical APIs and owner-local registration (2026-10-28)

**Scoped API/registration/authoring acceptance: PASS. Runtime deployment: CONDITIONAL.**
Task A's working tree was preserved. The optional APIs and their exact signatures
are documented in [component configuration](../component_configuration.md#optional-mathematical-apis-task-b).
Only four existing owner factory modules changed; no runner, authoring, bringup,
ROS node, estimator algorithm, controller algorithm, physics or safety source was
changed by Task B. No production model IDs or numerical defaults were added.

A compatible extension consists of one mathematical Python file with an explicit
`COMPONENTS` declaration and one authoritative parameter group in its existing
owner YAML. Sorted trusted package-local discovery feeds the existing
`ComponentRegistry`; experiment YAML supplies IDs and parameters, never imports.
The three examples remain test-only under `robot_experiments/test/model_fixtures`.
Their owner defaults are declared once in that fixture's `owner_parameters.yaml`.

| Verification | Actual result / evidence under `results/mathematical_models/` |
| --- | --- |
| Final focused mathematics, validation, registration and public authoring | **131 passed**, `scoped_final.log` |
| Full functional regression | **1,879 passed, 18 deselected, 1 existing xfailed**, 966.24 s, `regression.log`; deselection excludes package style/copyright wrappers |
| Final review regressions | Malformed enum schemas reject; genuine predictor uncertainty can be returned without external timestamp state; `review_red.log`, `adapters_review_final.log`, `review_disposition.md` |
| Direct / scientific configuration preservation | **75 / 21 equivalent**, zero differences, `direct_equivalence.json`, `preparation_equivalence.json` |
| Normal and copied builds | Both **10 packages passed**, `build.log`, `copied_build.log`; four affected packages refreshed successfully in each prefix after final review (`build_final.log`, `copied_build_final.log`) |
| Copied-install verification | **22 module origins, 10 families, 20 production components**, existing M06–M09 checks passed; `installed_verification.json` |
| Copied bytes | **8 changed/added production modules and 20 owner YAML resources** identical to source, `installed_byte_identity.json` |
| Scoped style / whitespace | **17 changed Python files** flake8 clean; **4 new production modules** pep257 clean; `style_review_final.log`, `docstyle_final.log`; `git diff --check` clean |
| Preservation | All **29 original YAML files** byte/value/type/order identical; **21 immutable revisions**, **14,031 previous artifacts**, HEAD and staged logical diff unchanged; `change_manifest.json` |

The full suite was collected during review; the final 131-case scoped rerun also
covers review cases added afterward. These are overlapping suites, not additive
counts. No functional failures were waived or numerical tolerances weakened.

Source and copied-install fixtures traverse discovery → public one-file
`ExperimentAuthor` preview/validation/save → Catalog preparation → M07 preflight
and actual owner construction. They verify revised immutable history, reset,
frozen source provenance, effective overrides and restoration of production
inventory. `installed_mathematical_verification.json` records concrete identities:
`AccelerationDynamics`, `StatelessPredictor`, `_VelocityFeedbackController`, with
existing `robot_ekf` / `kalman`, explicit Cartesian task and independent safety.
The initial copied mathematical probe used the wrong positional simulation
registry API; only the probe was corrected to use the established mapping API
and explicit environment dependency. Its failed log/catalog remain preserved;
`installed_mathematical.log` records the successful corrected probe.

Numerical examples exercise more than metadata. For state `[1,2,3,.4,.5,.6]`,
spring stiffness `.4`, damping `.3` and no environment, copied dynamics produce
`[.4,.5,.6,-.52,-.95,-1.38]`. For `[1,2,3,4,5,6]`, origin `10`, step `.5` and
acceleration `.2`, prediction uses timestamps `[10.5,11]` and final state
`[5.1,7.1,9.1,4.2,5.2,6.2]`, with uncertainty absent. Independent feedback tests
expect `[2.1,1.3,2.1,-3.8]`, then `[.2,.2,.2,-.2]` after the test owner's normal
acceleration projection. Fixtures also test environment-once integration,
malformed/nonfinite/complex outputs, unsafe commands and incompatible contracts.

### Retained live result and limitations

Exactly one headless copied-install **EXP-NOMINAL revision 3**, seed 42, was
attempted after builds/tests exited. It failed the original **45 s readiness
gate** at simulation clock **23.916 s**, without establishing an evaluation
origin. Exit code **1**, acceptance **unknown**; distance, pointing, scored RTF,
controller timing and physical-safety metrics are **unavailable**, not zero.
No retry, threshold change or runtime remediation was performed.

Actual streams included 4,265 joint messages, 924 robot estimates, 373 predictions
and 476 control/command messages (`runtime_stream_counts.json`). Stream presence
does not establish synchronized readiness or successful tracking. All nine launch
children exited cleanly; cleanup reported no survivors, and the final process
scan found none. Raw launch/telemetry/status remain under
`runtime/nominal_20261028T011336_419488Z_6n8b1vrx/`; `runtime_summary.json` preserves
the failure and unavailable metrics. This is consistent with the pre-existing
readiness limitation; this task does not establish its cause.

Test-only equations are verified mathematically, through real factories and
composition, **not through Gazebo deployment**. Controller convenience is limited
to the exact declared Cartesian velocity task/frame/selection and model-sized
bounded generalized velocities. Predictor frames cannot be checked because the
existing input/output contracts do not carry frame IDs; the same fixed frame is
required. No implicit covariance propagation, transforms, history or latent-state
projection is added. Registration metadata is not a scientific or hardware safety
certificate. Existing runtime, localization and IMM limitations remain open.
Estimator role-neutrality and Dashboard work remain deferred.

### Reproduction and exact Task-B-relative file list

From `/home/user/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -p no:launch_testing -p no:launch_ros --import-mode=importlib -q src tools/validation -k 'not flake8 and not pep257 and not copyright'
ros2 run robot_experiments run --experiment EXP-NOMINAL --revision 3 --output /tmp/mathematical_api_smoke
```

The final command is a reproduction attempt, not a promise of readiness success.
Exact targeted, normal/copy build, isolated-install, equivalence and smoke commands
are retained in `results/mathematical_models/verification_commands.txt` and the
corresponding `verify_*.sh` / `run_smoke.sh`. Use a fresh isolated catalog/output
when repeating checks; retain previous attempts. Original baseline hashes and
`before.tar.gz` accompany the final manifest and source snapshot in that directory.
Generated evidence is ignored and not installed as production code.

Changes below are relative to the captured **Task A state**, not Git HEAD:

```text
M docs/component_configuration.md
M docs/validation/component_configuration.md
M src/robot_control/robot_control/factory/project_components.py
M src/robot_prediction/robot_prediction/factory/component_registry.py
M src/robot_prediction/robot_prediction/factory/system_factory.py
M src/robot_simulation/robot_simulation/factory/simulation_factory.py
A src/robot_configuration/robot_configuration/model_registration.py
A src/robot_configuration/test/test_model_registration.py
A src/robot_control/robot_control/controllers/feedback_model.py
A src/robot_control/test/test_feedback_model.py
A src/robot_experiments/test/model_fixtures/context.py
A src/robot_experiments/test/model_fixtures/control/mathematical_example.py
A src/robot_experiments/test/model_fixtures/dynamics/mathematical_example.py
A src/robot_experiments/test/model_fixtures/owner_parameters.yaml
A src/robot_experiments/test/model_fixtures/prediction/mathematical_example.py
A src/robot_experiments/test/test_mathematical_authoring.py
A src/robot_prediction/robot_prediction/baseline/stateless_model.py
A src/robot_prediction/test/test_stateless_model.py
A src/robot_simulation/robot_simulation/dynamics/acceleration_model.py
A src/robot_simulation/test/test_acceleration_model.py
```

**6 modified, 14 added, 0 deleted.** Prior Task A edits remain untouched. No commit
or push was performed. Task B is complete; overall research runtime remains
CONDITIONAL because the representative deployment did not pass readiness.
