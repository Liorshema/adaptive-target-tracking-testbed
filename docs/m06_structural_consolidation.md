# M06 — Structural consolidation

> Historical M06 state. For current canonical paths and ownership, read
> [ARCHITECTURE.md](ARCHITECTURE.md) and the
> [M06.1 verification report](m06_1_architecture_hardening.md).

2026-10-16. **M06 verdict: PASS for the structural migration.** M02–M05 remain
CONDITIONAL; physical actuation safety and aggressive tracking are not upgraded.
No M07/M08 work, algorithm tuning, commit or push was performed.

## Final structure and boundaries

The workspace now has nine ROS packages. `robot_configuration` is the sole new
package; the other eight retain their responsibilities. Actual complete source
tree: [`validation/m06_final_tree.txt`](validation/m06_final_tree.txt).

```text
robot_configuration/robot_configuration/yaml_loader.py
robot_bringup/robot_bringup/{nodes/,factory/}
robot_control/robot_control/{nodes/,factory/,pipeline/,adapters/,tracking/,optimization/,safety/}
robot_estimation/robot_estimation/{nodes/,common/,filters/,robot/,target/}
robot_prediction/robot_prediction/{nodes/,factory/,configuration/,baseline/,fusion/,history/,
                                  residuals/,confidence/,online_adaptation/,interfaces/}
robot_simulation/robot_simulation/{nodes/,actuation/,factory/,gazebo/,dynamics/,disturbances/,
                                  integration/,engine/,state/,measurements/,geometry/,interfaces/}
robot_description/{config/,urdf/,launch/,robot_description/}
robot_models/robot_models/{base/,manipulator/,whole_body/,task_space/,reference/,camera/,constraints/}
robot_interfaces/{msg/,CMakeLists.txt,package.xml}
```

Equivalent ROS adapters now use `nodes/`; assembly uses `factory/`; prediction's
typed configuration wrapper uses `configuration/`. Domain-specific model/filter/
safety/integrator directories remain meaningful and unchanged. The existing
SimulationNode was already canonical. CMake messages remain CMake-based; numerical
models have no artificial nodes/configuration framework. All four control-pipeline
layers remain intact.

`robot_configuration` owns only the extracted YAML mapping loader: relative-parent
inheritance, recursive mapping merge, list/scalar/null replacement and existing
cycle/alias/invalid-document validation. Its implementation body is unchanged.
Runtime dependency is PyYAML; standard ament_python/setuptools packaging and pytest
support are declared. It has no ROS runtime, robot geometry, global registry,
experiment schema, installed-resource policy or algorithm factory. Bringup and
simulation declare it as a direct dependency. Existing YAML paths/schemas and
parameter precedence were not changed.

## Exact implementation moves

Paths below are Python modules, with unchanged ROS executable names. Each old path
is retained as one thin compatibility alias: **10 implementations relocated,
10 compatibility modules**, including six node relocations.

| Old module | Canonical module |
|---|---|
| `robot_control.ros.control_node` | `robot_control.nodes.control_node` |
| `robot_estimation.estimation_node` | `robot_estimation.nodes.estimation_node` |
| `robot_prediction.prediction_node` | `robot_prediction.nodes.prediction_node` |
| `robot_simulation.actuation.velocity_actuation` | `robot_simulation.nodes.velocity_actuation` |
| `robot_bringup.tracking_feedback_tf` | `robot_bringup.nodes.tracking_feedback_tf` |
| `robot_bringup.tracking_visualization` | `robot_bringup.nodes.tracking_visualization` |
| `robot_bringup.model_factory` | `robot_bringup.factory.model_factory` |
| `robot_prediction.system_factory` | `robot_prediction.factory.system_factory` |
| `robot_prediction.config_loader` | `robot_prediction.configuration.prediction_config` |
| `robot_simulation.factory.config_loader` | `robot_configuration.yaml_loader` |

Machine-readable filesystem/module map and active import updates:
[`m06_migration_map.json`](validation/m06_migration_map.json). New internal imports,
tests, console targets and active top-level validation utilities use canonical
paths. No blanket replacement touched historical nested captures or archived
before-source implementations.

Compatibility aliases use `sys.modules` forwarding to the canonical module. Both
paths therefore share module identity, classes and function globals; there are no
duplicate implementations or fallback algorithms. This matters for actual probes
that patch methods or execute archived source in module globals. Active estimator
and actuation probes explicitly import canonical modules. Legacy `python -m`
module execution is not an additional supported interface; ROS console executable
names remain the supported launch API.

Remove shims only after a documented deprecation period, migration of all supported
callers/instrumentation, verification that supported historical replay bootstraps
still work, and an explicit compatibility decision. Old strings in immutable logs
alone do not justify editing historical evidence. No removal date was invented.

## Packaging and cleanup

- Declared missing direct dependencies: prediction `ament_index_python`, `std_msgs`;
  simulation `tf2_ros`, `robot_configuration`; bringup `python3-numpy`,
  `robot_configuration`; models `python3-numpy`. Description display directly uses
  `launch` and `launch_ros`; these are now declared too.
- Added explicit ament_python buildtool declarations where missing. Models XML
  version now matches setup version 0.0.1. Bringup/models/description descriptions
  no longer advertise TODO or agricultural-only metadata. README separates current
  synthetic measurements from planned perception/experiment packages and states
  current robot/predictor support honestly.
- Added populated-package initializers for the new locations. A copied install
  exposed an existing defect: control `factory/` and `pipeline/` lacked
  `__init__.py`, so find_packages omitted their implementations even though a
  symlink/source import succeeded. Docstring-only markers and a RED→GREEN discovery
  regression fix this packaging issue without changing numerical code.
- Recursive simulation resource installation excludes hidden directories/files,
  `__pycache__`, backups and bytecode. The exclusion test creates synthetic debris,
  so merely moving current backups cannot make it pass accidentally.
- Preserved and hash-verified the three backups before removing source originals:
  `control_node.py.bak` and `control_node.py.before_tf.bak` now live under
  `results/legacy_backups/robot_control/`; the aggressive environment YAML backup
  lives under `results/legacy_backups/robot_simulation/`. Full mapping/hashes:
  [`m06_backup_archive.json`](validation/m06_backup_archive.json). The archive is
  gitignored runtime storage; the pre-task rollback snapshot also retains bytes.
- Removed only the identified broken old installed backup symlink left by the
  prior symlink installation. No broad install/source cleanup was performed. The
  independent copied install contains no such backup resources.

Pre-task changed/untracked Git inventory and file snapshots are preserved under
`validation/m06_migration_before/`. Its `COLCON_IGNORE` prevents discovery of the
snapshot's package.xml files as duplicate packages. Initial build preflight caught
that issue before any package built; adding the marker corrected snapshot handling.
No repository reset, clean, stash or destructive revert was used.

## Verification

| Check | Actual result / source |
|---|---|
| Implementation equivalence | All 10 relocated bodies identical to snapshots after canonical import-path normalization; 65 configuration/message/Xacro/SDF assets unchanged (`m06_behavior_preservation.json`) |
| New contracts RED | Loader absent; 10 canonical paths absent; synthetic backup/cache exclusion failed; later 2 setuptools discovery checks failed before package-marker fix |
| Initial affected/dependent symlink build | 9 packages passed, 14.7 s (`m06_build.log`) |
| Isolated non-symlink copied build | 9 packages passed, 32.7 s (`m06_copy_build.log`) |
| Packaging correction rebuild | Control and bringup rebuilt in normal and copied installs (`m06_build_packaging_fix.log`, `m06_copy_packaging_fix.log`) |
| Focused groups | 113 passed initially; **115 passed in 1.60 s** after final package-discovery fix (`m06_focused_final.log`) |
| Full functional suite | **871 passed, 18 existing style/copyright tests deselected**, 15.10 s (`m06_regression.log`) |
| Installed entry points | All 7 executable names load canonical targets from the copied prefix |
| Legacy compatibility | All 10 pairs are the same module object, not independent class re-exports |
| Installed resources | 65 assets including all 52 YAML resources, Xacro/URDF, launch files, model/world and RViz assets match source; mapping inheritance resolves; empty legacy disturbances YAML remains an identified stub |
| Installed Xacro | Generated URDF joint limits agree with authoritative description |
| Real instrumentation | Four isolated current/archived estimator/actuation probes select the intended source and share canonical globals (`m06_instrumentation.json`) |
| Recorded replay | Both before/after M03 counters and numerical errors match saved replay to 1e-12 (`m06_replay_compatibility.json`) |
| Independent bounded review | No critical/important migration finding; reviewer checked snapshot equivalence, backups, alias globals, setup selection and probes (`m06_review.md`) |

The full suite was run once after bulk structural changes. The subsequently found
copied-install defect required only docstring package markers; final focused
discovery/import tests and copied-install checks were rerun after that fix. No
filter/controller implementation changed. Existing lint/style failures were not
reformatted or represented as passing.

The copied-install check ran from `/tmp`, with workspace PYTHONPATH and ament prefix
variables removed, sourcing ROS and the copied install's **local_setup.bash**.
Assertions require project modules and package shares to resolve under the copied
prefix, preventing a source-tree/symlink import from masking missing packages.
Results: [`m06_installed_checks.json`](validation/m06_installed_checks.json).
This is an independent installed-resource check on the current machine, not a
fresh-OS rosdep provisioning claim.

The replay preserves the original recording and saved M03 result; it writes a new
M06 compatibility result. Accepted joint observations remain 24,931 before versus
31,351 after, with 6,420 formerly rejected joints; no timestamp/constraint
mathematics was modified.

## One short headless nominal smoke

Used the existing validation utility with normal production launch composition,
16 simulated seconds, 4 s settling, WLS nominal scenario, physical truth observation
and no markers. Read-only estimator/actuation trace prefixes use the migrated
entry points; `observed.launch.py` is the production launch source with those
prefixes, not a separate controller implementation. No second Gazebo run was made.

Evidence: [`m06_smoke/result.json`](validation/m06_smoke/result.json), raw streams,
generated SDF, timing/actuation traces and logs in the same directory.

- Base excursion 0.720 m; all arm joints moved (excursions
  `[0.206,0.194,0.122,0.405]` rad).
- Settled valid-sample actuator-active fraction 1.0; 318 accepted commands,
  zero rejected commands/feedback rejections and zero post-settling watchdog events.
- Position/command bounds, shoulder posture, first-link clearance and live tracking
  TF gates passed. Physical mean/max distance errors 0.0992/0.2957 m; mean/max
  pointing errors 7.214/17.507 degrees.
- RTF 0.729; controller mean/p95 wall compute 23.87/45.62 ms. This short traced run
  uses a different evaluation window/load from M04's 60 s captures (RTF
  0.882/0.872); it is integration evidence, not a matched performance regression
  experiment or a performance improvement claim.
- Raw measured speeds `[0.2684,0.8425,1.7973,1.5438]` rad/s still exceed elbow/wrist
  limits. The observer exits **1** for the known physical-speed gate. Simulation
  completion and structural PASS do not make that gate pass.

M04 nominal reference measured physical errors 0.0774/0.1837 m and 8.340/18.014 deg
over its longer settled interval. This smoke remains in the same operating regime
with safety recovery/holding infrastructure intact; different windows/reactive
trajectories preclude attributing numerical differences to structure. Body-equivalence,
unchanged asset hashes and replay tests are the behavior-preservation evidence.

## Reproduction commands

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robot_bringup
source install/setup.bash

python3 -m pytest src --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros -q \
  -k 'not flake8 and not pep257 and not copyright'
python3 docs/validation/m06_verify_instrumentation.py

# Fresh copy build; choose an unused /tmp prefix when repeating.
colcon --log-base /tmp/m06_repeat/log build --base-paths src \
  --build-base /tmp/m06_repeat/build --install-base /tmp/m06_repeat/install \
  --merge-install --packages-up-to robot_bringup
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  bash --noprofile --norc -c 'source /opt/ros/jazzy/setup.bash
source /tmp/m06_repeat/install/local_setup.bash
cd /tmp
python3 /home/user/ros2_ws/docs/validation/m06_verify_installation.py \
  --prefix /tmp/m06_repeat/install'

# Normal headless launch; executable and resource contracts are unchanged.
ros2 launch robot_bringup closed_loop.launch.py \
  config:=config/reactive_tracking_oscillatory.yaml \
  strategy_config:=config/whole_body.yaml enable_actuation:=true gui:=false
# Add publish_commands:=false for control compute-only with gravity holding.
```

Exact smoke command used (do not overwrite its output when repeating):

```bash
ROS_DOMAIN_ID=149 python3 docs/validation/v2_tracking_gazebo.py \
  --case reactive --config config/reactive_tracking_oscillatory.yaml \
  --strategy-config config/whole_body.yaml --physical-ground-truth --without-markers \
  --shoulder-posture --estimator-trace --actuation-trace \
  --duration 16 --settling-time 4 --wall-timeout 120 \
  --output docs/validation/m06_smoke
```

## Differences, limitations and closure

Implementation follows all 10 approved moves, includes the approved narrow package,
and adds the two missing control package markers plus confirmed description launch
dependencies discovered during installation verification. The target-design header
records these additions. No approved relocation was deferred. Dormant alternatives,
legacy public contracts and historical experiments were not deleted.

Scope decisions: preserve the existing dirty workspace through file-scoped snapshots
instead of a worktree/commit; update active current utilities while keeping archived
capture/source files unchanged. Snapshot metadata and full path map permit selective
rollback without resetting prior user work. Independent review deferred judging
runtime/copy/full-suite checks to the primary agent; the separate checks above
complete those requirements.

Remaining architecture limitations are deliberately unchanged: predictor/estimator
implementation choices are not yet unified through YAML; the robot family remains
planar mobile-manipulator; implicit state/joint/mount metadata needs future profile
validation; reactive feedback remains odometry-derived; compatibility aliases are
temporary debt. M02 physical overspeed, M03 localization/covariance, aggressive
tracking/freshness and performance limits retain their prior verdicts. M06 does not
establish real-hardware safety.

Deliverables:

- This report: `docs/m06_structural_consolidation.md`.
- Actual source tree: `docs/validation/m06_final_tree.txt`.
- Exact task-local modified/moved/added/deleted path inventory:
  `docs/validation/m06_modified_files.txt`; supporting implementation map and
  preservation/review evidence are linked above.

**M06 structural consolidation is complete. Stop here; M07/M08 were not started.**
