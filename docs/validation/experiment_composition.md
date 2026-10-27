# Experiment-centric composition consolidation verification

Updated 2026-10-27. Task 2/2 continues the completed canonical-component foundation.
No commits, pushes, dashboard, algorithm tuning or physical-limit changes.

## Result and scope

The catalog now owns common composition, selection deltas, scenario/resource
recipes, direct-launch cases and controller allocations. Removed 42 redundant
source YAMLs and their obsolete installation entries after resolved equivalence
and consumer migration. Configuration YAMLs: **61 → 20** (all source YAMLs 62 → 21,
including the intentionally invalid test fixture). Physical assets, numerical
implementations, defaults and safety acceptance thresholds are unchanged.

Architecture/configuration migration is verified by the checks below. Overall
research runtime verdict remains **CONDITIONAL**: short migration runs cannot
close prior IMM readiness/coverage, localization/slip, timing or real-hardware
safety limitations. Failed runtime gates are retained, never rerun to obtain PASS.

[Composition/API/CLI guide](../experiment_composition.md) is the current workflow
reference; earlier ownership-migration documentation retains historical paths.

## Ownership and implementation

- Description owns geometry, mount, joint limits, inertial and collision values.
  Catalog wrist recipes explicitly override that authoritative description; M07
  validates all model/estimator/controller/actuator consumers against it.
- Simulation retains dynamics equations/defaults, fields, integration, sensing,
  terrain, PID/holding, physics, transport and actuation. New `deployment.yaml`
  contains reusable deployment defaults with unbound experiment resource slots.
  `target.yaml` contains target metadata and an unselected dynamics slot.
- Experiments owns selections and variations; calibration deltas remain distinct.
  True measurement noise and estimator-assumed covariance remain independent.
- Bringup retains the existing shared preflight and deployment mechanics; algorithm
  packages acquire no experiment dependency. No parallel validator or runner exists.
- The new composition boundary reuses shared resource/group loading, mapping merge,
  strict owner overrides, M07 reference-origin resolution and actual owner factories.
  Recipes support `base`, `overrides`, optional atomic typed-slot `selection`, and
  explicit optional-field `omit`. Lists/null replace; source documents are detached.
- Process-lifetime generated YAMLs retain transitive source/group provenance and
  source fingerprints. Source drift rejects a stale preparation before freezing.
  Relative paths retain declaration origins, including inherited groups and recipe
  bases. Complete owner replacements clear inherited parameter groups.
- Existing M08 materialization freezes plain effective inputs and original source
  fingerprints. Dynamics defaults are expanded through their registry **before**
  strict owner overrides, enabling valid damping variations without duplicated YAML.
  Explain uses stable origin labels, rather than ephemeral cache filenames.
- Catalog revisions 1/2 remain unchanged; revision 3 selects consolidated resources.
  Bounded catalog migration data supports historical revisions and legacy entry
  names. There is no second on-disk YAML tree.

## Equivalence and regression evidence

Evidence root: `results/composition_consolidation/` (ignored generated artifacts).
The task-start capture includes earlier dirty work; comparisons are against that
capture, not HEAD. `before.tar.gz`, `baseline.json`, `source_yaml_before.json`,
`direct_before.json` and `before_preparations.json` preserve the starting evidence.

| Check | Result / source |
|---|---|
| Direct selections / controller allocations | **75 exact** resolved physical/numerical comparisons; `direct_equivalence_verified.log`, `direct_equivalence.json` |
| Seeded scientific preparations | **21 exact** contracts, conditions, identities and effective inputs; `preparation_equivalence_verified.log`, `preparation_equivalence.json` |
| Focused provenance / workflows | **42 passed**, including registered dynamics/estimator/predictor/controller adapters selected through catalog; `recipe_edges_green.log` |
| Final functional regression | `1731 passed, 18 deselected, 1 xfailed in 534.06s (0:08:54)`; `functional_complete.log` |
| Normal build | **10 packages passed**, `build_final.log` |
| Fresh copied build | **10 packages passed**, `isolated_build.log`; final experiment package recopied in `isolated_build_verified.log` |
| Copied-install M06–M09 chain | **PASS**, 17 module origins, 10 families, 20 components; `installed_verification.json/log` |
| Standalone compose / historical import | **PASS**, frozen ordinary inputs and revision-1 preflight; `installed_compose.log` |
| New/changed experiment-core lint | 9 files pass flake8 and pep257; `style_complete_verified.json/log` |
| Preservation | **14 original revisions**, **13,140 historical artifacts**, HEAD/staged contents and protected numerical/physical implementations unchanged; `change_manifest.json` |

The first full post-cleanup diagnostic reported 1,725 passing, 3 failing, 18 style
checks deselected and 1 expected failure. The new workflow exposed override/default
ordering; the other failures were stale invalid-manifest and installation fixtures.
They were fixed without relaxing assertions. `full_regression_failure_closure.log`
records focused closure; final full results are listed above. An intermediate
regression collected old fixtures and was stopped early; its log is retained.

Repository-wide style gates were already excluded by the inherited Task-1
functional invocation (18 checks). They are not claimed fixed. The existing
Simulation launch style violations remain distinct from the clean experiment-core
check. Migrated consumer formatting was applied only where AST identity held;
formatting never justified numerical or acceptance changes.

The final changed-file flake8 comparison is explicit: task-start scope had 1,598
style diagnostics; current scope has 1,566. Of the current diagnostics, 1,380 align
to unchanged baseline lines and **186 are introduced formatting diagnostics in
migrated tests/probes** (61 line-length, 76 import ordering/grouping, 29 import
placement, and 20 spacing/indentation/semicolon diagnostics). There are no new
Pyflakes `F` diagnostics in that comparison. These 186 remain technical debt;
repository-wide lint is **not PASS**. The nine-file experiment-core check is a
separate passing scope. `style_delta.json`, `style_delta_final.log` and the archived
pre-change sources make the distinction inspectable. A broad cosmetic formatting
trial was discarded in favor of the minimal already-tested migration; both trial
states remain in generated evidence. No lint configuration or acceptance policy
was weakened.

Equivalence normalizes only resource origins verified through their actual contents,
run-local paths, canonical prediction representation and derived comparison hashes.
It compares geometry, physical/holding limits, PID values, dynamics, integration,
noise, runtime selection, strategy and scientific policy; it does not normalize
physical quantities or acceptance thresholds.

## Acceptance matrix

| Area | Verdict |
|---|---|
| Catalog composition, strict owner validation and historical revisions | PASS |
| Physical/numerical configuration equivalence | PASS |
| Functional regression and both installation builds | PASS |
| Permanent holding, compute-only, reset and owned-worker cancellation | PASS |
| Scored runtime automation / tracking reliability | CONDITIONAL |
| Repository-wide style closure | CONDITIONAL; remaining consumer formatting debt |
| Long-horizon IMM / real-hardware safety certification | Not established |

## Runtime verification

Copied prefix: `/tmp/composition_consolidation_install`. Tests/builds finish before
measured Gazebo runs. Commands execute outside the source checkout with clean
AMENT/Python paths, bounded processes and isolated runner-owned ROS/Gazebo contexts.

| Run | Distance error mean / max (m) | Pointing error mean / max (°) | RTF mean | Original acceptance |
|---|---:|---:|---:|---|
| runtime/cv_seed42 (98c4a048) | — | — | — | readiness_timeout |
| runtime/imm_seed42 (7f47ebdc) | — | — | — | readiness_timeout |
| sensor-runtime/baseline_seed42 (6bcacfc1) | 0.0990 / 0.1945 | 6.4523 / 18.6696 | 0.651 | rejected |
| sensor-runtime/higher_noise_seed42 (07f167ff) | 0.1409 / 0.3318 | 7.4240 / 20.4964 | 0.761 | accepted |
| workflow-runtime/fight_cv_dls_seed42 (30e1a3a3) | 1.5858 / 1.9753 | 8.4463 / 13.0772 | 0.819 | accepted |
| workflow-runtime/fight_cv_dls_seed42 (b996d9bd) | 89.7013 / 332.6526 | 22.8163 / 35.2738 | 0.439 | unknown |


**Six actual Gazebo trials: four completed, two failed readiness.** Original gates
accepted two completed trials, rejected one, and left one unknown. Acceptance
here follows the existing availability/safety policy; it is not a tracking-quality
threshold or hardware certification. In particular, the custom unsaturated
Fight/Flight + CV + DLS variant is **not a stable tracking baseline**: repeat means
were 1.58585 m and 89.70132 m (maximum 332.65255 m). Its readiness origins were
2.272 s and 13.226 s; evaluation windows were [5.272, 11.272] and [16.226, 22.226] s.
The target was already evolving before readiness. Identical configuration hashes
do not imply identical trajectories or equivalent physical state at window start.
No parameters were tuned or failed trials replaced.

The completed sensor-baseline window rejected one command and one feedback
measurement; other scored invalid-command and watchdog counters were zero. The
higher-noise window passed its original gates. Scored RTF means ranged from 0.439
to 0.819. No scored distance/pointing/RTF is available for the readiness failures;
those cells intentionally remain unavailable.

In the four completed windows, measured physical speed excess was zero. Maximum
position excursion beyond a nominal bound was 3.6521e-5 rad, within the existing
1e-3 rad numerical tolerance. Maximum measured arm speed was 0.66235 rad/s;
authoritative configured bounds are 1.0 rad/s for the first three joints and
1.5 rad/s for the wrist. Three windows had complete original physical coverage;
the second custom repetition had only 99.8%, so its safety gate remains unknown.
`physical_runtime_evidence.json` records each joint's extrema and actual limits.
No physical acceptance is inferred for trials that never opened an evaluation window.

Readiness uses the unchanged latest-observed-clock predicate with an 80 ms bound.
Both IMM/CV attempts published all required streams and active commands but never
opened a readiness window before 45 wall seconds. Arrival-order replay found no
qualifying CV cohort (minimum maximum stream/clock skew 88 ms); IMM had only three
potential callback-level cohorts, which the run loop did not observe as ready.
This is evidence of freshness/timing sensitivity, not proof of a single scheduler
root cause. `readiness_evidence.json` retains counts and the replay method.
`runtime_source_preservation.json` verifies the recorder/execution code is unchanged.
This task does not redesign readiness or loosen its predicate.

Every attempt is retained in `all_runtime_results.json` and raw study status,
metrics, telemetry and snapshots. Repeated custom Fight/Flight + CV + DLS executions
use a writable catalog with damping 0.23; these are explicit experiment variations,
not production tuning. Their Run IDs and snapshots must differ while effective
scientific conditions remain equal. Snapshot noise, weights and damping variations
are also covered by the focused catalog workflow test.

Compute-only evidence is under `compute-only/`; the probe passes an explicitly
materialized standalone deployment to the **normal permanent** simulation launch.
The compute-only probe passed **800 cycles** with `publish_commands=false`. Across
3,816 holding samples, maximum hold error was 5.193e-11 rad and minimum tracking
height above the base was 0.43918 m; stale-input handling and clock-reset recovery
also passed. These stationary holding measurements do not certify dynamic safety.
The live worker proof is `job_verification.json`; it checks frozen revision execution,
active command telemetry, cooperative cancellation, retained history and no owned
survivors. See `runtime_exit_codes.txt`, `compute_only.exit`, `live_jobs.log` for
actual completion; no missing measurement is inferred from a process exit alone.

The prior Task-1 four-run comparison is preserved under
`results/component_architecture/`: two accepted, two rejected by timing gates,
with complete coverage and physical position/speed checks within original gates.
Short-run differences here are not causal performance improvements; source/environment
and wall-clock scheduling changed. No tuning was done to improve the verdict.

## Reproduction

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest \
  -p no:launch_testing -p no:launch_ros --import-mode=importlib -q src tools/validation \
  -k 'not flake8 and not pep257 and not copyright'
python3 results/composition_consolidation/check_direct.py
python3 results/composition_consolidation/check_preparations.py
colcon build --build-base /tmp/composition_consolidation_build \
  --install-base /tmp/composition_consolidation_install --executor sequential
bash results/composition_consolidation/verify_installed.sh
```

The original comparison attempts remain untouched. A syntax error in the temporary
custom-workflow setup prevented its first two invocations from launching; that
script and errors are retained. Only those missing executions were then run via
`run_missing_workflow.sh`; it did not retry any failed or rejected Gazebo trial.

Generated task-local verification scripts are retained with the evidence, rather
than installed as another permanent production interface. Use fresh output
locations if repeating `run_installed.sh`, `run_compute_only.sh`, `run_jobs.sh` or
`verify_compose.sh`; they deliberately do not overwrite existing scientific runs.
Permanent researcher commands for creating/editing/resetting/running/repeating/
deleting experiments and direct/standalone launch migration are in the guide.
A no-argument Simulation launch now fails explicitly: pass `config` from `run compose`.
Removed files must not be passed directly to generic bringup; freeze inputs first.

## Final configuration tree and counts

| Package | Before | After |
|---|---:|---:|
| robot_bringup | 1 | 1 |
| robot_control | 1 | 1 |
| robot_description | 3 | 1 |
| robot_estimation | 1 | 1 |
| robot_experiments | 14 | 4 |
| robot_prediction | 1 | 1 |
| robot_simulation | 40 | 11 |
| **Total configuration YAMLs** | **61** | **20** |

```text
robot_bringup/config/tracking_visualization.yaml
robot_control/config/control.yaml
robot_description/config/robot/robot.yaml
robot_estimation/config/estimation.yaml
robot_experiments/config/calibration/manipulation_pose.yaml
robot_experiments/config/calibration/simulation.yaml
robot_experiments/config/experiments.yaml
robot_experiments/config/manipulation_analysis.yaml
robot_prediction/config/prediction.yaml
robot_simulation/config/actuation.yaml
robot_simulation/config/deployment.yaml
robot_simulation/config/dynamics.yaml
robot_simulation/config/environment.yaml
robot_simulation/config/gazebo/joint_position_hold.yaml
robot_simulation/config/gazebo/robot.yaml
robot_simulation/config/measurements.yaml
robot_simulation/config/sensors/planar_localization.yaml
robot_simulation/config/simulation.yaml
robot_simulation/config/target.yaml
robot_simulation/config/terrain/flat.yaml
```

Two calibration files are retained because their wheel covariance values differ.
Physical assets, terrain, native actuator plugin, holding and sensing defaults stay
separate. No backup/archive is installed under active package resource directories.
The normal-build retry removed only 84 dangling symlinks for the retired resources;
`removed_stale_build_symlinks.json` lists them. The fresh copied build has no such
retired files.

## Removed resources and replacements

All source deletions matched baseline SHA256 before removal; replacements were
resolved and tested first. `removed_resources.json` includes each checksum and
reason; `before.tar.gz` retains the original source evidence. `source_after.tar.gz`
retains the final active source tree with hashes in `source_after.json`.

| Removed file | Canonical replacement |
|---|---|
| `src/robot_description/config/robot/robot_wrist15.yaml` | `experiments.yaml → resources.robot_wrist15` |
| `src/robot_description/config/robot/robot_wrist30.yaml` | `experiments.yaml → resources.robot_wrist30` |
| `src/robot_simulation/config/gazebo/robot_manipulation.yaml` | `experiments.yaml → resources.gazebo_robot_manipulation` |
| `src/robot_simulation/config/gazebo/robot_wrist15.yaml` | `experiments.yaml → resources.gazebo_robot_wrist15` |
| `src/robot_simulation/config/gazebo/robot_wrist30.yaml` | `experiments.yaml → resources.gazebo_robot_wrist30` |
| `src/robot_simulation/config/sensors/planar_localization_noise_study.yaml` | `experiments.yaml → resources.localization_noise` |
| `src/robot_simulation/config/scenarios/environment.yaml` | `experiments.yaml → resources.environment_reactive` |
| `src/robot_simulation/config/scenarios/environment_manipulation.yaml` | `experiments.yaml → resources.environment_manipulation` |
| `src/robot_simulation/config/scenarios/environment_oscillatory.yaml` | `experiments.yaml → resources.environment_oscillatory` |
| `src/robot_simulation/config/scenarios/environment_oscillatory_aggressive.yaml` | `experiments.yaml → resources.environment_oscillatory_aggressive` |
| `src/robot_simulation/config/scenarios/environment_oscillatory_aggressive_reference.yaml` | `experiments.yaml → resources.environment_oscillatory_aggressive_reference` |
| `src/robot_simulation/config/scenarios/target.yaml` | `experiments.yaml → resources.target_independent` |
| `src/robot_simulation/config/scenarios/target_3d.yaml` | `experiments.yaml → resources.target_3d` |
| `src/robot_simulation/config/scenarios/target_3d_aggressive.yaml` | `experiments.yaml → resources.target_3d_aggressive` |
| `src/robot_simulation/config/scenarios/target_fight_flight_low_damping.yaml` | `experiments.yaml → resources.target_fight_flight_low_damping` |
| `src/robot_simulation/config/scenarios/target_geometry.yaml` | `experiments.yaml → resources.target_geometry` |
| `src/robot_simulation/config/scenarios/target_saturated.yaml` | `experiments.yaml → resources.target_saturated` |
| `src/robot_simulation/config/launch.yaml` | `experiments.yaml → resources.deployment_standalone` |
| `src/robot_simulation/config/scenarios/base_relative_manipulation.yaml` | `experiments.yaml → resources.deployment_base_relative_manipulation` |
| `src/robot_simulation/config/scenarios/moving_target.yaml` | `experiments.yaml → resources.deployment_moving_target` |
| `src/robot_simulation/config/scenarios/oscillatory_reactive_tracking.yaml` | `experiments.yaml → resources.deployment_oscillatory_reactive_tracking` |
| `src/robot_simulation/config/scenarios/reactive_manipulation.yaml` | `experiments.yaml → resources.deployment_reactive_manipulation` |
| `src/robot_simulation/config/scenarios/reactive_manipulation_noise_study.yaml` | `experiments.yaml → resources.deployment_reactive_manipulation_noise_study` |
| `src/robot_simulation/config/scenarios/reactive_manipulation_wrist15.yaml` | `experiments.yaml → resources.deployment_reactive_manipulation_wrist15` |
| `src/robot_simulation/config/scenarios/reactive_manipulation_wrist30.yaml` | `experiments.yaml → resources.deployment_reactive_manipulation_wrist30` |
| `src/robot_simulation/config/scenarios/reactive_target.yaml` | `experiments.yaml → resources.deployment_reactive_target` |
| `src/robot_simulation/config/scenarios/reactive_target_3d.yaml` | `experiments.yaml → resources.deployment_reactive_target_3d` |
| `src/robot_simulation/config/scenarios/reactive_target_3d_aggressive.yaml` | `experiments.yaml → resources.deployment_reactive_target_3d_aggressive` |
| `src/robot_simulation/config/scenarios/reactive_target_low_damping.yaml` | `experiments.yaml → resources.deployment_reactive_target_low_damping` |
| `src/robot_simulation/config/scenarios/reactive_target_oscillatory_aggressive.yaml` | `experiments.yaml → resources.deployment_reactive_target_oscillatory_aggressive` |
| `src/robot_simulation/config/scenarios/reactive_target_oscillatory_aggressive_reference.yaml` | `experiments.yaml → resources.deployment_reactive_target_oscillatory_aggressive_reference` |
| `src/robot_simulation/config/scenarios/reactive_target_saturated.yaml` | `experiments.yaml → resources.deployment_reactive_target_saturated` |
| `src/robot_experiments/config/systems/cv_prediction.yaml` | `experiments.yaml → compositions.common + compositions.selections.cv_prediction` |
| `src/robot_experiments/config/systems/dls_control.yaml` | `experiments.yaml → compositions.common + compositions.selections.dls_control` |
| `src/robot_experiments/config/systems/localization_noise_study.yaml` | `experiments.yaml → compositions.common + compositions.selections.localization_noise_study` |
| `src/robot_experiments/config/systems/manipulation_B.yaml` | `experiments.yaml → compositions.common + compositions.selections.manipulation_B` |
| `src/robot_experiments/config/systems/manipulation_C.yaml` | `experiments.yaml → compositions.common + compositions.selections.manipulation_C` |
| `src/robot_experiments/config/systems/manipulation_D.yaml` | `experiments.yaml → compositions.common + compositions.selections.manipulation_D` |
| `src/robot_experiments/config/systems/manipulation_E.yaml` | `experiments.yaml → compositions.common + compositions.selections.manipulation_E` |
| `src/robot_experiments/config/systems/mobile_manipulator_pose_tracking.yaml` | `experiments.yaml → compositions.common + compositions.selections.mobile_manipulator_pose_tracking` |
| `src/robot_experiments/config/systems/nominal_tracking.yaml` | `experiments.yaml → compositions.common + compositions.selections.nominal_tracking` |
| `src/robot_experiments/config/launch_profiles.yaml` | `experiments.yaml:direct_launch` |

## Exact task-relative changed files

This list excludes earlier Task-1 changes, even where Git still marks them dirty.
`change_manifest.json` and `changed_files.txt` retain the complete task-local audit.

```text
M README.md
M docs/ARCHITECTURE.md
M docs/component_configuration.md
M docs/experiment_management.md
M src/robot_bringup/test/test_aggressive_oscillatory.py
M src/robot_bringup/test/test_architecture.py
M src/robot_bringup/test/test_explicit_composition.py
M src/robot_bringup/test/test_mapping_composition.py
M src/robot_bringup/test/test_nominal_composition.py
M src/robot_bringup/test/test_reactive_manipulation_configuration.py
M src/robot_bringup/test/test_reactive_v3_configuration.py
M src/robot_bringup/test/test_sensor_inventory.py
M src/robot_bringup/test/test_shoulder_safety.py
M src/robot_bringup/test/test_system_configuration.py
M src/robot_bringup/test/test_visualization_launch.py
M src/robot_experiments/config/experiments.yaml
M src/robot_experiments/config/manipulation_analysis.yaml
M src/robot_experiments/launch/tracking.launch.py
M src/robot_experiments/robot_experiments/catalog.py
M src/robot_experiments/robot_experiments/cli.py
M src/robot_experiments/robot_experiments/configuration.py
M src/robot_experiments/robot_experiments/explain.py
M src/robot_experiments/robot_experiments/launching.py
M src/robot_experiments/setup.py
M src/robot_experiments/test/test_catalog.py
M src/robot_experiments/test/test_configuration.py
M src/robot_experiments/test/test_explain.py
M src/robot_experiments/test/test_explicit_composition.py
M src/robot_experiments/test/test_nominal_composition.py
M src/robot_experiments/test/test_scientific_comparison.py
M src/robot_experiments/test/test_sensor_assignments.py
M src/robot_simulation/config/target.yaml
M src/robot_simulation/launch/simulation.launch.py
M src/robot_simulation/setup.py
M src/robot_simulation/test/test_configuration.py
M src/robot_simulation/test/test_deployment_configuration.py
M src/robot_simulation/test/test_oscillatory_experiment.py
M src/robot_simulation/test/test_physics_engine.py
M src/robot_simulation/test/test_resource_installation.py
M src/robot_simulation/test/test_simulation_factory.py
M tools/validation/compute_only_gazebo.py
M tools/validation/joint_limit_recovery_authority_probe.py
M tools/validation/joint_limit_recovery_probe.py
M tools/validation/joint_limit_recovery_verify_installation.py
M tools/validation/m02_resolution_reset.py
M tools/validation/m03_localization_run.py
M tools/validation/m03_localization_verify_installation.py
M tools/validation/m07_verify_installation.py
M tools/validation/m09_verify_installation.py
M tools/validation/reactive_challenge_report.py
M tools/validation/saturated_dynamics_probe.py
M tools/validation/target_context_lifecycle_run.py
M tools/validation/v2_performance.py
A docs/experiment_composition.md
A docs/validation/experiment_composition.md
A src/robot_experiments/robot_experiments/composition.py
A src/robot_experiments/test/test_catalog_composition.py
A src/robot_experiments/test/test_composition.py
A src/robot_simulation/config/deployment.yaml
D src/robot_description/config/robot/robot_wrist15.yaml
D src/robot_description/config/robot/robot_wrist30.yaml
D src/robot_experiments/config/launch_profiles.yaml
D src/robot_experiments/config/systems/cv_prediction.yaml
D src/robot_experiments/config/systems/dls_control.yaml
D src/robot_experiments/config/systems/localization_noise_study.yaml
D src/robot_experiments/config/systems/manipulation_B.yaml
D src/robot_experiments/config/systems/manipulation_C.yaml
D src/robot_experiments/config/systems/manipulation_D.yaml
D src/robot_experiments/config/systems/manipulation_E.yaml
D src/robot_experiments/config/systems/mobile_manipulator_pose_tracking.yaml
D src/robot_experiments/config/systems/nominal_tracking.yaml
D src/robot_simulation/config/gazebo/robot_manipulation.yaml
D src/robot_simulation/config/gazebo/robot_wrist15.yaml
D src/robot_simulation/config/gazebo/robot_wrist30.yaml
D src/robot_simulation/config/launch.yaml
D src/robot_simulation/config/scenarios/base_relative_manipulation.yaml
D src/robot_simulation/config/scenarios/environment.yaml
D src/robot_simulation/config/scenarios/environment_manipulation.yaml
D src/robot_simulation/config/scenarios/environment_oscillatory.yaml
D src/robot_simulation/config/scenarios/environment_oscillatory_aggressive.yaml
D src/robot_simulation/config/scenarios/environment_oscillatory_aggressive_reference.yaml
D src/robot_simulation/config/scenarios/moving_target.yaml
D src/robot_simulation/config/scenarios/oscillatory_reactive_tracking.yaml
D src/robot_simulation/config/scenarios/reactive_manipulation.yaml
D src/robot_simulation/config/scenarios/reactive_manipulation_noise_study.yaml
D src/robot_simulation/config/scenarios/reactive_manipulation_wrist15.yaml
D src/robot_simulation/config/scenarios/reactive_manipulation_wrist30.yaml
D src/robot_simulation/config/scenarios/reactive_target.yaml
D src/robot_simulation/config/scenarios/reactive_target_3d.yaml
D src/robot_simulation/config/scenarios/reactive_target_3d_aggressive.yaml
D src/robot_simulation/config/scenarios/reactive_target_low_damping.yaml
D src/robot_simulation/config/scenarios/reactive_target_oscillatory_aggressive.yaml
D src/robot_simulation/config/scenarios/reactive_target_oscillatory_aggressive_reference.yaml
D src/robot_simulation/config/scenarios/reactive_target_saturated.yaml
D src/robot_simulation/config/scenarios/target.yaml
D src/robot_simulation/config/scenarios/target_3d.yaml
D src/robot_simulation/config/scenarios/target_3d_aggressive.yaml
D src/robot_simulation/config/scenarios/target_fight_flight_low_damping.yaml
D src/robot_simulation/config/scenarios/target_geometry.yaml
D src/robot_simulation/config/scenarios/target_saturated.yaml
D src/robot_simulation/config/sensors/planar_localization_noise_study.yaml
```

## Remaining limits

The supported configurable unsaturated variation can run away and is not suitable
as a stable tracking baseline. Startup readiness does not synchronize the physical
state at evaluation origin across repeats. These observed limitations remain open;
configuration reproducibility is established, trajectory repeatability is not.

Short acceptance runs do not establish long-horizon IMM prediction coverage or
hardware safety. Existing localization/slip and timing limitations remain. Historical
software/native dependencies must accompany archived runs for software-level replay;
frozen YAML and provenance are not a full environment distribution. ROS/Gazebo
verification requires this workspace's native dependencies and IPC permissions.
The Git index byte fingerprint changed during verification, consistent with stat-cache
refresh; the staged diff is empty both before and after, and HEAD is unchanged.
No index repair/reset was attempted. `change_manifest.json` distinguishes index
bytes from logical staged content rather than claiming bytewise preservation.

The bounded historical alias data remains necessary for immutable revisions; it
is deliberately not a second active config directory or an arbitrary filename guess.
