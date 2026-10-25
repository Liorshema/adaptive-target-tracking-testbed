# M08 scientific sensor comparisons

## Baseline and frozen design

Before production changes, the current materializer was exercised with the real localized system and seed 42. Identical contract hashes were accepted; a change only to localization white-noise standard deviation was rejected; a changed physical shoulder limit was rejected. Rejected pairs suppress all aggregate variant metrics and prediction-horizon aggregates. Evidence: `/tmp/m08_scientific_6b4f0z4q/baseline.json` and its materialized inputs. No baseline owner was edited.

The original contract contains description, physical robot deployment, target initial position/velocity/dynamics, environment, integration, target measurements, tracking/physical context, prediction sampling, holding, terrain, world-source hash, actuation, tracking reference and optional localization sensor configuration. The runner adds duration, settling, skew, physical tolerance and actuation mode before hashing. Its hash algorithm will remain unchanged.

The implementation design is an optional `experiment.comparison` with integer `schema_version: 1`, `domain: sensor_parameters` and nonempty `variables`. Each variable has a unique `name`, an exact dotted `path` into that original resolved contract, and optional distinct permitted `values`. Arrays are atomic values; wildcard, list-index, container and unapproved paths are forbidden. The initial authorized leaves are current stochastic sensor settings, never geometry, seed, publication/frame/age contracts, safety or scenario fields. Future algorithm comparisons require a separate explicit domain and invariant design.

Keep the complete original contract/hash. Compute a separate invariant by removing only authorized, present leaves and combining the remaining contract with protected resolved algorithm/physics/evaluation/setup conditions. Record actual materialized variable values and both views. Strict experiments retain their existing hash decisions. Policy comparisons additionally require complete executions, compatible metric definitions/units, available required measurements and existing physical coverage evidence. Failure/acceptance and comparison eligibility remain separate; poor tracking is not infrastructure failure.

Acceptance tests are frozen before implementation in `src/robot_experiments/test/test_scientific_comparison.py`. They cover strict and declared differences, forbidden/undeclared changes, materialization/seed provenance, historical inputs, missing observations, metrics definitions and incomplete runs. Implementation and measured closure evidence follow below.

## Implemented policy and ownership

`robot_experiments/comparison.py` is the single comparison-policy owner. Existing configuration validation, materialization, runner and benchmark call it; execution, lifecycle, observation models, estimators, predictors, controllers and metric mathematics are unchanged. No new dependency, registry, launcher, results database or UI was introduced.

The optional declaration uses these supported keys (also present in the installed example):

```yaml
experiment:
  comparison:
    schema_version: 1
    domain: sensor_parameters
    variables:
      - name: pose_white_noise_std
        path: localization_sensor.localization_sensor.white_noise_std
        values:
          - [0.04, 0.04, 0.03490658503988659]
          - [0.08, 0.08, 0.06981317007977318]
```

`values` is optional; when supplied the effective materialized value must be one of its finite JSON values. Names and paths must be unique. Null declarations, unknown keys/domains/versions, duplicate permitted values, wildcards, indices, missing leaves and protected/container paths fail clearly. Variable names describe the experiment and do not select implementation classes.

The initial exact authorization set is:

- `localization_sensor.localization_sensor.white_noise_std`
- `localization_sensor.localization_sensor.calibration_bias_std`
- `localization_sensor.localization_sensor.latency`
- `localization_sensor.localization_sensor.latency_jitter`
- `localization_sensor.localization_sensor.dropout_probability`
- `localization_sensor.localization_sensor.outlier_probability`
- `localization_sensor.localization_sensor.outlier_offset`
- `measurements.measurements.target_position.noise.covariance`

The second producer demonstrates that the comparison operation is independent of localization and scenario selection. Adding another domain requires an explicit authorization and invariant design; this task does not permit estimator/prediction/controller variables. Owner configuration validation still governs legal physical sensor values and matrix dimensions.

Only the declared leaves are removed from a deep copy. Undeclared siblings, sensor identity/rate/frame/timing bounds and effective seeds remain protected. The original contract remains complete. Additional protected conditions contain resolved physics, world plugins, Gazebo transport, selected components, merged estimator/controller parameters, controller strategy, full prediction configuration, tracking feedback, runtime, seed consumers, experiment name/seeds/execution settings, requested/required metric families, physical coverage requirement and metric/source definition hashes. Resource selector filenames are provenance; their effective contents, rather than temporary input filenames, determine invariance. Robot description, deployment, native backend/limits, actuation feedback/safety, ground truth, target/scenario, world source, frames, duration/settling/skew and physical tolerance remain in the original invariant contract.

Original hashes use precisely the previous sorted-JSON SHA-256 encoding. Explicit-policy invariant hashes cover both the reduced contract and the additional conditions. Without a declaration the historical hash equality rule remains authoritative. The original materializer was imported from its preserved before-copy and exercised on the baseline: `materialization_comparison.json` records `legacy_hash_unchanged: true`. Equivalent effective configurations through distinct inheritance chains also produce matching invariants.

Comparisons recompute hashes/views from the original evidence and verify the saved variables and invariant contents. They require matched variant/seed sets, completed execution, available finite requested metric observations, compatible metric definitions/units, full-duration windows and existing complete physical-joint observation coverage. They do not require favorable accuracy or `acceptance: true`. Existing acceptance remains a separate gate, and no accuracy threshold was introduced.

Accepted examples: declared noise changes, declared dropout changes, declared target measurement covariance changes, equivalent inherited settings, and poor tracking with failed acceptance but otherwise complete comparable evidence. Rejected examples: declared noise plus undeclared dropout, changed sensor seed/frame, physical geometry/shoulder limit, physics timestep, scenario/container masking, changed metric definitions/units, missing coverage/required metrics and shortened windows. Rejection reasons name changed invariant paths, for example `localization_sensor.localization_sensor.dropout_probability`; forged or stale saved views fail their original-evidence checks.

## Result contract and provenance

Existing `summary.json` is extended with `comparison.schema_version`, experiment/policy, eligibility, specific rejection reasons and `run_index`. Each index contains variant/seed, execution outcome, relative artifacts directory and a JSON pointer into `runs`. Each run retains its configuration identity, actual component implementations, complete original/invariant views and hashes, named effective variables, existing metrics/units/coverage and status. New per-run `comparison.json` is the same inspectable comparison evidence; `resolved_configuration.json` retains the original contract plus the additive view. Existing JSON/CSV, telemetry and status structures remain usable.

The materialized system's existing M07 `sensor_inventory` is recorded in each resolved configuration. It is visibility/provenance, not authority or live-health evidence. Effective sensor values and independent seeds come from actual input owner files. No top-level filename equivalence or inventory-derived settings are used to authorize differences.

The existing taxonomy is preserved: completed versus failed execution, with `error.code: interrupted` / `KeyboardInterrupt` distinguishing a user stop. A preparing/partial result remains partial. Failed/interrupted runs retain artifacts and block comparison aggregates; missing data never becomes zero. Deterministic tests cover failed, interrupted, incomplete and null-evidence cases. No destructive live interruption was needed for this small demonstration.

## Real ROS/Gazebo demonstration

Study: `results/scientific_sensor_comparison/sensor_noise_comparison_20261023T160221_778806Z_zvrff5ij/`.

Two sequential, headless, actuated runs used the existing M08 engine and M07 closed-loop launch. Both completed with `acceptance: true`; CLI exit code was 0. `summary.json` has `comparable: true`, `comparison.eligible: true`, and no rejection reasons. There is one run per variant, seed 42, eight scored simulation seconds after three seconds of settling from each readiness epoch.

The example inherits existing nominal experiment policy and the localized manipulation system. Five small inherited owner files select the higher-noise source without modifying any baseline. `execution.max_skew: 0.15` is a common existing offline alignment/readiness option, explicitly recorded/protected for both variants. It accommodates the existing 80 ms robot-filter reorder holdback. Default 80 ms runner readiness would conflict with that holdback plus callback processing. Runtime sensor ages, estimator timing, physics, numerical safety tolerance and actuator limits were not changed.

| Effective setting | Baseline | Higher noise |
|---|---|---|
| Position standard deviation X/Y (m) | 0.04 / 0.04 | 0.08 / 0.08 |
| Yaw standard deviation (rad) | 0.03490658503988659 | 0.06981317007977318 |
| Target measurement seed | 42 | 42 |
| Localization seed | 3872720740 | 3872720740 |
| Sensor rate / latency / jitter | 10 Hz / 0.04 s / 0.01 s | identical |
| Dropout probability | 0.05 | 0.05 |
| Bias standard deviation X/Y/yaw | 0.01 m / 0.01 m / 0.008726646259971648 rad | identical |
| Scored source epochs (s) | 4.630–12.630 | 5.518–13.518 |

Original hashes differ as intended:

- Baseline: `eaed6bed93ba15fc3af4804cd83465374d8764378c1f84bd0413ce2aeda378cc`
- Higher noise: `70cd56201966e6118151c80ff4d01849e34e5c817232c864b94dc0fcb5de2f11`
- Shared invariant: `838c220b33d3d9b98344d30d7bfed67e7c18ad64a16757046eafdecb843cdfbe`

Metric definition hash is `61b51a684cec7887099b614a39871b2a38078768056f72bf11de2b1dd28aa79a` in both runs and the copied install. Source-tree hash is `6f0899cfb33bbedc2fc9813cf86ebe20b224db1a294cd37e3cfce07633b28a0c` in both runs. These identify the measured code; documentation edits afterward do not change that source hash.

| Existing M08 metric | Baseline | Higher noise |
|---|---:|---:|
| Physical tracking distance MAE / max (m) | 0.08334 / 0.28744 | 0.12094 / 0.29679 |
| Physical pointing MAE / max (degrees, converted from stored rad) | 8.100 / 18.299 | 7.512 / 24.916 |
| Robot estimate planar-position MAE (m) | 0.03303 | 0.04898 |
| Robot estimate yaw MAE (degrees) | 1.0635 | 1.8868 |
| Robot joint-vector error mean (rad) | 0.001216 | 0.001034 |
| Target estimate position MAE (m) | 0.05037 | 0.06213 |
| Prediction RMSE (m) | 0.40058 | 0.41920 |
| Control computation mean / max (ms) | 25.319 / 57.917 | 23.745 / 70.652 |
| Observed RTF | 0.7505 | 0.7593 |
| Valid control samples / scored samples | 160 / 160 | 160 / 160 |
| Invalid commands / watchdog / command rejections / feedback rejections | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| Reference limiting events | 189 | 189 |
| Sampled position max excess (rad) | 0 | 5.58e-13 |
| Sampled speed max excess (rad/s) | 0 | 0 |
| Existing physical observation coverage | 100% | 100% |

Coverage is reported rather than inferred from successful execution. Tracking had 564 / 555 matched samples, robot error 323 / 332, target error 140 / 139 and localization observations 79 / 79. Raw source-time stream coverage: robot estimate 7.998 / 7.992 s; target truth 7.996 / 7.992 s; localization 7.930 / 7.868 s. Maximum localization source gap was 0.200 s in both runs. Joint safety coverage is the existing bounded-hold calculation including valid pre-start feedback: eight seconds, max gaps 0.078 / 0.080 s. Raw stream coverage and this safety coverage are distinct contracts.

A read-only source audit reuses `tools/validation/m03_localization_metrics.py:aligned_pose` with bounded 0.15 s truth brackets, XY interpolation and unwrapped-yaw interpolation at acquisition epochs. All 79 in-window localization observations matched truth in each run. Published localization-source planar-position MAE was **0.047216 / 0.095397 m**, and absolute yaw MAE **1.306426 / 2.661814 degrees**. These are supplemental source-audit results, not a new M08 metric definition. Existing M08 robot-estimation error above measures the fused estimator. Tracking compliance has no existing M08 threshold/metric; it is unavailable here. NEES/NIS remain explicitly unavailable. Unstamped diagnostics retain null source time and are not given fabricated freshness scores.

The existing `robot_experiments.metrics.actuator_trace.analyze_trace` was also applied to every recorded physics iteration, including startup. Both traces are complete and accepted: 6,497 / 6,903 samples per joint, contiguous 2 ms steps with common joint grids. Largest position excess over the entire trace was **6.47153e-5 rad** at the shoulder during startup; largest speed excess was **5.00958e-10 rad/s**. Both are below the unchanged 0.001 rad/rad/s numerical acceptance tolerance; no violation samples occurred. Peak base-yaw/shoulder/elbow/wrist speeds were 0.629324/1.000000/1.000000/0.712867 rad/s and 0.699419/1.000000/1.000000/0.719903 rad/s, against 1/1/1/1.5 limits. Native motor torque is unavailable in this backend; the trace does not certify measured torque or real-hardware safety.

Each run stopped through owned SIGINT cleanup with return code 0, no survivors and cleanup times 0.773 / 0.676 s. A separate process-environment scan found **no processes in either owned Gazebo partition**. No unrelated simulation was stopped for this demonstration. Unique study/run directories preserve earlier experiments.

Equal seeds and protected setup do not imply identical realized trajectories. Localization noise covariance changes with the selected source model; dropout/source behavior can change draw consumption. The target reacts to live robot TF, so sensor-driven robot motion changes target motion. Readiness epochs also differ, as explicitly recorded above. The existing readiness-relative scoring policy is matched, not absolute trajectory phase. These short, asynchronous runs demonstrate infrastructure; neither statistical superiority nor bitwise replay is claimed. RTF is descriptive, not an isolated performance benchmark.

## Verification and reproduction

Evidence root: `/tmp/m08_scientific_6b4f0z4q/`. The new study also contains `validation_audit.json` with the supplemental source/physics audit, copied-install check and independent lifecycle scan.

- Before change: `baseline.json` records accepted identical contracts and rejected noise/physical differences. `red.log` records missing-policy failures before production implementation.
- Final focused M08 + M07 sensor/preflight + architecture suite: **286 passed** in 10.39 s (`focused_closure.log`), including 34 new policy cases. Commands are recorded below.
- Full functional regression: **1,383 passed, 18 deselected** in 22.53 s (`regression_final.log`). An intermediate new null-metrics case initially failed; it was corrected with a regression test, preserving unavailable evidence rather than crashing aggregation. Final suite has no functional failures.
- New policy module and its test file pass focused flake8 (`--jobs=1 --max-line-length=99`). This is not a claim that existing files are lint-clean. Earlier sensor-inventory verification recorded 12 style failures / 6 skips; that unrelated existing debt was not repaired or rerun wholesale here.
- Workspace build: three affected packages passed in 53.4 s (`build.log`). Copied overlay build: three packages passed in 3 min 59 s, followed by a successful experiments-only refresh (`copied_build.log`, `copied_refresh.log`). Byte-compilation-disabled and copied-underlay dependency warnings were nonfatal.
- Clean-shell, `/tmp`-cwd copied verification passed (`copied_verification.json`): current experiment code and all new owner resources are copied, nonsymlink files; materialized resource provenance has no workspace paths; invariant hashes match; metric definition hash is install-location independent. Unchanged dependencies come from the previous full copied installation, explicitly recorded in that evidence. This is a clean affected-package overlay check, not a claim to have rebuilt every unchanged package again.
- Installed example `--validate-only` passed for both variants (`preflight.json`). The real integration then completed both runs, provenance/required metric checks, sampled safety gates, physics-trace audit and lifecycle checks.

From the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_simulation robot_bringup robot_experiments
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_LOG_DIR=/tmp/m08_scientific_ros
ros2 run robot_experiments run --config config/sensor_noise_comparison.yaml --validate-only
ros2 run robot_experiments run --config config/sensor_noise_comparison.yaml \
  --output results/scientific_sensor_comparison
```

Each invocation creates a unique timestamp/suffix study. Inspect `summary.json`, each run's `comparison.json`, `resolved_configuration.json`, `metrics.json`, `metadata.json`, `status.json` and `physics/actuator.csv`. No alternative simulation launcher is required.

```bash
python3 -m pytest src/robot_experiments/test \
  src/robot_bringup/test/test_sensor_inventory.py \
  src/robot_bringup/test/test_system_configuration.py \
  src/robot_bringup/test/test_architecture.py \
  src/robot_bringup/test/test_architecture_tree.py \
  -q --import-mode=importlib -p no:launch_testing -p no:launch_ros
python3 -m pytest src -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros \
  -k 'not copyright and not flake8 and not pep257'
python3 -m flake8 --jobs=1 --max-line-length=99 \
  src/robot_experiments/robot_experiments/comparison.py \
  src/robot_experiments/test/test_scientific_comparison.py
git diff --check
```

The task's temporary audit commands are retained as `check_copied.py` and `analyze_demo.py` in the evidence root. To repeat those read-only checks against the recorded study:

```bash
source install/setup.bash
python3 /tmp/m08_scientific_6b4f0z4q/analyze_demo.py
# Run copied verification only in a fresh shell, before creating its input directories:
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /tmp/m08_scientific_6b4f0z4q/copied/install/setup.bash
cd /tmp
python3 /tmp/m08_scientific_6b4f0z4q/check_copied.py
```

The copied checker creates fresh `copied_materialization/<variant>` directories and refuses overwrite. For a repeat, change its output root to a new directory; retain the original evidence.

## Acceptance gates

| Gate | Verdict | Evidence / boundary |
|---|---|---|
| Explicit experiment-variable declaration | PASS | Strict versioned schema; actual effective values checked |
| Strict-default backward compatibility | PASS | Historical hash encoding/equality preserved; existing tests and old/new materializer proof |
| Protected invariant enforcement | PASS | Exact authorized leaves; undeclared sibling/physical/scenario/physics/time/metric checks |
| Correct resolved-value provenance | PASS | Materialized owners, independent seeds, M07 inventory, equivalent inheritance and copied-resource checks |
| Valid comparison metadata and metrics | PASS | Recomputed views, matched invariants, required metrics/units/windows/coverage and stable JSON pointers |
| Incomplete/failing run treatment | PASS | Existing failure/interruption taxonomy retained; deterministic missing/partial/failed tests |
| End-to-end sensor comparison | PASS | Two real actuated Gazebo runs completed and compared |
| Functional and architecture regression | PASS | Full functional suite plus focused architecture/install tests; legacy style debt separate |
| Physical safety and lifecycle preservation | PASS | No actuator changes; existing tolerances; complete physics traces and no owned survivors in this bounded demonstration |
| Maintainability and generality | PASS | One policy owner, two sensor producers, no robot/scenario constants or duplicate engine |

**Overall: PASS for the requested comparison infrastructure.** This does not close earlier project-wide research limitations or certify hardware safety. It establishes a bounded reproducible example and rejects unsupported scientific comparisons. One seed/eight seconds cannot establish population statistics. Hashes check recorded evidence consistency; they are not signatures against deliberate artifact tampering. Outside a Git checkout the existing worktree source hash is unavailable; resolved owner contents and the install-independent metric-definition hash remain recorded, without claiming complete installed algorithm-source identity. Future algorithm comparison domains, richer source/consistency/compliance metrics, longer seed studies and UI remain outside scope.

## Exact file inventory

Modified in this task:

1. `src/robot_experiments/robot_experiments/configuration.py`
2. `src/robot_experiments/robot_experiments/runner.py`
3. `src/robot_experiments/robot_experiments/benchmarking.py`
4. `docs/m08_experiments_and_benchmarking.md`
5. `docs/architecture/sensors.md`

Added in this task:

6. `src/robot_experiments/robot_experiments/comparison.py`
7. `src/robot_experiments/test/test_scientific_comparison.py`
8. `src/robot_experiments/config/sensor_noise_comparison.yaml`
9. `src/robot_simulation/config/sensors/planar_localization_noise_study.yaml`
10. `src/robot_simulation/config/scenarios/reactive_manipulation_noise_study.yaml`
11. `src/robot_bringup/config/reactive_manipulation_noise_study.yaml`
12. `src/robot_bringup/config/system_localization_noise_study.yaml`
13. `docs/validation/m08_scientific_sensor_comparison_closure.md`

The initial 1,627-file tracked/nonignored snapshot was preserved except the five intended modifications; 1,622 existing files remain byte-identical and none were deleted. Prior dirty edits in those five files were extended rather than reverted. Existing historical result directories were not written; all new runtime evidence is in the unique study above and this task's temporary evidence directory. Git HEAD and index remain unchanged. No staging, commit or push was performed.
