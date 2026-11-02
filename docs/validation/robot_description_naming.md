# Task R — canonical robot description naming

**PASS for the naming/compatibility acceptance gate.** Current model construction,
experiment preparation and the copied-install Gazebo trial use the new resource.
The physical YAML is byte-identical; immutable Catalog revisions are unchanged.
This verdict applies to this migration and its bounded verification, not general
tracking performance or real-hardware safety certification.

## Source move and ownership

Moved the sole physical owner, with no numerical edits:

```text
src/robot_description/config/robot/robot.yaml
  -> src/robot_description/config/robots/mobile_manipulator.yaml
SHA256 fd6877be3fb2c8cc20d13c85ddb776e9aad1657d90afa1c6014dce68a0694c76
```

The internal `robot:` mapping, geometry, axes, joint ordering, joint position,
velocity and effort limits, damping, inertias and tracking-frame transform are
identical. Canonicalized rendered URDF XML is also exactly identical. XML
canonicalization changes attribute ordering/comments only; no values are removed.

`robot_description/setup.py` installs `config/robots/*.yaml`. The description
package owns the public default-path helper `default_robot_description_path()`;
the description loader, display launch and standalone control fallback delegate
to it. Generic framework code gains no robot-filename table or new selector.
Explicit resource/configuration selection remains the normal composition path.

Updated active configuration references:

| Owner | Changed entry |
|---|---|
| `robot_estimation/config/estimation.yaml` | `transport.robot.robot_description_config` |
| `robot_simulation/config/gazebo/robot.yaml` | `robot.description.config` |
| `robot_experiments/config/experiments.yaml` | `compositions.common.robot.description.path` |
| Same Catalog | `resources.robot_wrist15.base.path`, `resources.robot_wrist30.base.path` |

The Gazebo deployment file retains its separate name and responsibility.
Holding PID, physics, actuator safety, estimator, predictor and control equations
are unchanged. Existing tests/validation tools and current architecture/sensor
documentation now select the canonical physical resource. Other changes in those
files are line wrapping required by the longer path or comments/docstrings.

## Historical compatibility

The existing `legacy_resources` table gains exactly one explicit mapping:

```yaml
robot_description:config/robot/robot.yaml:
  package: robot_description
  path: config/robots/mobile_manipulator.yaml
  sha256: fd6877be3fb2c8cc20d13c85ddb776e9aad1657d90afa1c6014dce68a0694c76
```

`resolve_system_resource` reuses that bounded table. It handles package resources
and recognized absolute/relative paths under known package shares. For this
checksum-guarded alias it verifies the canonical target and any still-existing
old file. Modified physical content or malformed guards fail explicitly. Existing
unguarded aliases, including string targets and wrist recipes, remain supported.
Unrelated resources and same-named files outside recognized shares are not guessed.

Legacy schema-2 external manifests migrate in memory at each declaring document's
origin, including inherited relative references. Their actual source files,
parents, canonical physical source and alias declaration enter normal provenance
records with real SHA256 fingerprints. Source changes after preparation still
fail. No immutable revision definition/hash or historical snapshot is rewritten.

Independent review found a leaf-symlink bypass of alias recognition. Four negative
cases reproduced it before the correction. Lexical-path recognition now precedes
leaf resolution, with declared/resolved package roots supported; all four reject
changed content and a positive equivalent-symlink case succeeds. Evidence:
`results/robot_description_naming/review.md` and `symlink_red.log`.

There is no old source file, installed copy, forwarding YAML or compatibility
symlink in either active installation. A stale normal-install symlink was removed
only after verifying that it targeted the moved source. Direct filesystem access
such as `open(old_path)` is not a resource-migration API. Use the existing resolver
for legacy resource references; genuine frozen runs retain their own inputs.

## Verification and preservation

Evidence root: `results/robot_description_naming/`. Counts below overlap.

| Check | Actual result / evidence |
|---|---|
| Full functional regression | **1,989 passed**, 18 style wrappers deselected, 1 expected xfail; 1,082.82 s; `regression.log` |
| Focused description/control/estimation/building checks | **112 passed**, 3 style wrappers deselected; `focused.log` |
| Final migration guards and affected analysis helpers | **24 passed** (20 migration cases + 4 helper tests); `migration_formatting_closed.log` |
| Direct configuration equivalence | **75 cases**, zero differences; `direct_equivalence.json` |
| Scientific preparation equivalence | **21 cases**, zero differences; `preparation_equivalence.json` |
| Physical mapping / joint limits / Xacro / wrist variants | Exact identity, recipes change only expected wrist position bounds; `physical_equivalence.json` |
| Genuine frozen snapshots | **3 reconstructed**, from Tasks B, E and E.1; `historical_snapshots.json` |
| Normal symlink build | **10 packages**, 1 min 55 s; `build_fixed.log` |
| Copied build | **10 packages**, 6 min 2 s; `copied_build_fixed.log` |
| Copied final refresh | **1 package**, 1 min 9 s, late simulation-builder docstring only; `copied_refresh.log` |
| Copied M06–M09 verification chain | Passed, 26 module-origin entries, 14 component families / 30 components; `installed_verification.json` |
| Copied Author/Catalog canonical estimation workflow | Passed; real construction, save/revise, frozen bindings; `installed_estimation_verification.json` |
| Copied naming / resource / Xacro / nominal / wrist / legacy checks | Passed; `installed_naming_verification.json` |
| Final copied byte identity | 4 changed production modules, 20 owner YAML resources, display launch identical; `installed_byte_identity.json` |
| Style | 839 inherited findings before / 837 after in changed existing Python scope; **zero new findings**; new migration test and description loader/display checked clean; `lint_existing_verification.json`, `style_closed.log` |

The full suite collected before the last symlink-guard cases were added; the final
20-case migration subset includes those cases. Functional regression excludes the
existing style/copyright wrappers explicitly, rather than claiming a clean global
lint baseline.

Both initial builds stopped before compilation because retained E.1 generated
fixture owners were discovered as duplicate packages. Added the generated boundary
marker `results/COLCON_IGNORE`, preserving all evidence, then reran the original
build commands successfully. Original failed logs remain `build.log` and
`copied_build.log`. The first byte-identity check caught a late docstring-only
copied-file difference; the focused package refresh and final identity check are
retained. No scientific/Gazebo trial was retried.

Path normalization in the 75/21 comparisons is explicit in `check_direct.py` and
`check_preparations.py`: resolved resource locations are compared by resolved
owner documents; frozen-input directory prefixes are normalized, retaining input
filenames. Both scripts first assert the physical-file SHA equals the captured
original. Prior comparison rules for canonical prediction representation and
empty intrinsic-estimator transport envelopes remain unchanged; those envelopes
are checked for their exact algorithm identity and empty parameters before
normalization. Physical values, limits, scientific conditions and equations are
not dropped to obtain equivalence.

`baseline.json` and `before.tar.gz` capture the preexisting dirty tree (768 source
files). `change_manifest.json`, `task.diff` and `after.tar.gz` describe only Task R.
The audit proves 287 production Python executable ASTs unchanged outside the five
explicit path/compatibility owners. All modified YAML mappings differ only at the
declared resource paths plus the one guarded alias. The Catalog's entire raw prefix
before `compositions:` is byte-identical, and its immutable `experiments` mapping
is identical. All **14,896** captured historical artifacts retain size and nanosecond
mtime. HEAD and staged diff are unchanged; no commit or push was performed.

## One bounded copied-install ROS/Gazebo trial

Exactly one attempt, inherited EXP-NOMINAL physical/scientific conditions with the
existing canonical `ekf`/`kf` bindings, seed 42, 3 s settling and 6 s evaluation.
It completed with `acceptance=true`, no error, and every existing acceptance gate
true. Source: `runtime_summary.json`; raw trial:

```text
results/robot_description_naming/runtime/nominal_20261028T130412_296933Z_u30m7yel/runs/imm_seed42
```

| Measurement | Actual result |
|---|---|
| Distance error mean / maximum | 0.123900 / 0.231099 m, 360 valid samples |
| Pointing error mean / maximum | 0.151520 / 0.347597 rad (8.681° / 19.916°) |
| Physical position / speed overshoot maximum | 0 / 0, existing 0.001 rad / rad/s tolerances retained |
| Physical-limit coverage | 6.0 / 6.0 s, 1,115 samples; max gap 0.068 s < existing max-skew 0.08 s |
| Invalid commands / watchdog increments / rejected commands / rejected feedback | 0 / 0 / 0 / 0 |
| Controller compute mean / maximum | 35.747 / 100.584 ms |
| Mean RTF | 0.579461, simulation delta / monotonic delta |
| Cleanup | Exit 0, no owned survivors; independent global launch/server/container scan empty |

This short integration sample demonstrates the renamed resource through real
ROS/Gazebo execution. It is not a matched performance benchmark, proof of all
postures/scenarios, or a new closure of prior scientific/actuator limitations.
RTF and tracking errors are reported as observed; no thresholds, physics fidelity,
PID gains, noise, control weights or target dynamics were tuned for this task.

## Remaining old-path references

`references_after.txt` records the post-migration search. In production source,
the sole literal old path is the checksum-bound legacy alias key; tests retain an
`OLD` constant to exercise compatibility. Separate existing wrist alias keys
remain meaningful mappings to wrist-variation recipes. Historical milestone and
audit documents (M02–M07, shoulder/holding records, earlier architecture proposals,
and the earlier experiment-composition validation tree) preserve their recorded
state. Current `docs/ARCHITECTURE.md` and `docs/architecture/sensors.md` are updated.
Old generated experiment inputs, previous validation scripts/results and archived
source remain unchanged as scientific evidence, and genuine frozen reconstruction
is tested. Raw old absolute paths outside the recognized current package shares
are not silently substituted.

## Reproduction

From `/home/user/ros2_ws`, normal installation and the working simulation:

```bash
source /opt/ros/jazzy/setup.bash
# Needed when retained generated fixture packages exist under results/.
mkdir -p results
touch results/COLCON_IGNORE
colcon build --symlink-install --executor sequential
source install/setup.bash
ros2 run robot_experiments run --experiment EXP-NOMINAL --revision 1 --output /tmp/task_r_nominal
```

This is headless, physically actuated nominal execution through the installed
experiment pipeline; choose a fresh output directory. Direct launch remains
explicitly configuration-driven. To replay the complete tested frozen system in
Gazebo with a GUI:

```bash
ros2 launch robot_bringup closed_loop.launch.py system_config:=/home/user/ros2_ws/results/robot_description_naming/runtime/nominal_20261028T130412_296933Z_u30m7yel/runs/imm_seed42/inputs/system.yaml gui:=true enable_actuation:=true
```

Functional regression and copied installation:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -p no:launch_testing -p no:launch_ros --import-mode=importlib -q src tools/validation -k 'not flake8 and not pep257 and not copyright'
colcon --log-base results/robot_description_naming/copied-fixed-colcon build --build-base /tmp/robot_description_naming_build --install-base /tmp/robot_description_naming_install --executor sequential
bash results/robot_description_naming/verify_installed.sh
bash results/robot_description_naming/verify_naming_install.sh
bash results/robot_description_naming/verify_bytes.sh
```

The actual bounded trial command was `bash results/robot_description_naming/run_smoke.sh`.
Its immutable local Catalog revision 1 is retained at
`results/robot_description_naming/canonical_catalog.yaml`; revision 2 belongs only
to the offline authoring/history check. Reproduce into a fresh output directory:

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/robot_description_naming_install/setup.bash
ros2 run robot_experiments run --catalog /home/user/ros2_ws/results/robot_description_naming/canonical_catalog.yaml --experiment EXP-DESCRIPTION-CANONICAL --revision 1 --output /tmp/task_r_copied_trial
```

## Exact changed files

Relative to completed Task E.1 / Task R's captured starting tree, not Git HEAD:

```text
M docs/ARCHITECTURE.md
M docs/architecture/sensors.md
M src/robot_bringup/test/fixtures/invalid_system.yaml
M src/robot_bringup/test/test_explicit_composition.py
M src/robot_bringup/test/test_mapping_composition.py
M src/robot_bringup/test/test_nominal_composition.py
M src/robot_bringup/test/test_reactive_manipulation_configuration.py
M src/robot_bringup/test/test_shoulder_safety.py
M src/robot_bringup/test/test_system_configuration.py
M src/robot_bringup/test/test_visualization_adapter.py
M src/robot_control/config/control.yaml
M src/robot_control/robot_control/nodes/control_node.py
M src/robot_control/test/test_robot_model_control.py
M src/robot_description/launch/display.launch.py
M src/robot_description/robot_description/model_parameters.py
M src/robot_description/setup.py
M src/robot_description/test/test_camera_visual.py
M src/robot_estimation/config/estimation.yaml
M src/robot_estimation/test/test_estimator_factory.py
M src/robot_experiments/config/experiments.yaml
M src/robot_experiments/robot_experiments/configuration.py
M src/robot_experiments/test/test_configuration.py
M src/robot_experiments/test/test_manipulation.py
M src/robot_experiments/test/test_metrics.py
M src/robot_simulation/config/gazebo/robot.yaml
M src/robot_simulation/robot_simulation/gazebo/robot_builder.py
M src/robot_simulation/test/test_actuation_factory.py
M src/robot_simulation/test/test_position_guard.py
M src/robot_simulation/test/test_robot_builder.py
M tools/validation/closed_loop_gazebo.py
M tools/validation/closed_loop_safety.py
M tools/validation/compare_shoulder_experiments.py
M tools/validation/compare_v2.py
M tools/validation/compute_only_gazebo.py
M tools/validation/m06_1_verify_preservation.py
M tools/validation/m06_verify_installation.py
M tools/validation/reactive_challenge_allocation_verified.py
M tools/validation/reactive_challenge_feasibility.py
M tools/validation/test_aggressive_analysis.py
M tools/validation/test_shoulder_experiment_metrics.py
M tools/validation/v2_estimation_audit_collect.py
M tools/validation/v2_orientation_probe.py
M tools/validation/v2_performance.py
A docs/validation/robot_description_naming.md
A src/robot_description/config/robots/mobile_manipulator.yaml
A src/robot_experiments/test/test_description_migration.py
D src/robot_description/config/robot/robot.yaml
```

Additional ignored generated infrastructure: `results/COLCON_IGNORE`.
New Task R evidence/scripts: `results/robot_description_naming/`.
Only the old physical path was removed from source; its bytes remain at the new
canonical path. No previous source edit, experiment configuration, historical
result or immutable revision was removed. Stop at Task R; no Task C or Dashboard work.
