# M07 Phase 1 — Sensor inventory and discoverability closure

Date: 2026-10-23. **Phase 1 verdict: PASS**, with existing repository lint debt and a pre-existing test race documented below. This verdict concerns discoverability, configuration reporting and behavior preservation. It does not recertify physical actuation, localization accuracy or tracking acceptance.

## Delivered behavior and navigation

Start at [the sensor guide](../architecture/sensors.md), now linked from the repository README. It maps each producer, ROS conversion, pure observation model, configuration owner and consumer; explains source/fusion selection and required safety feedback; and identifies the raw-array sensor wrappers as compatibility APIs outside the production ROS path.

The existing `load_system_configuration` now returns an additive `sensor_inventory` field. Its nine entries cover wheel odometry, joint feedback, optional IMU yaw rate, external planar localization, synthetic target position, physical base truth/context, derived tracking TF, simulation clock and committed target truth. Each entry distinguishes configured producer selection, configured fusion/subscription and external dependence. Every entry's live status remains **`not observed`**: the inventory performs no ROS graph discovery and is not a health monitor.

[Inventory construction and formatting](../../src/robot_bringup/robot_bringup/factory/sensor_inventory.py) are colocated in one pure module beside the existing M07 assembler. Extraction reads validated, final owner values; formatting is a separate function. The normal manifest launch logs three concise lines, with detailed fields retained in the resolved plan. The legacy profile-only launch path is unchanged.

M08 already writes the complete resolved M07 plan to `resolved_configuration.json` under `system`; consequently the same inventory is saved under **`system.sensor_inventory`**, without M08 production-code changes or an independent metadata reconstruction. Its materializer calls M07 again after run-local configuration/seed changes. Tests verify both the derived localization seed in the run-local plan and the inventory in the actual JSON written by the runner, including failed-startup evidence.

No new configuration parameters, manifest schema keys, package, factory, registry, node, adapter API or source directory were introduced. `schema_version: 1` inside the new inventory versions an output object, not an editable configuration layer.

## Effective configuration evidence

These are observations of the two source manifests at closure, not a manually maintained configuration table. Canonical values remain in their owner YAML and the computed inventory. Complete snapshots are in `/tmp/sensor_inventory_phase1_yv7zd1ab/*_inventory_final.json`.

| Effective setting | `system_nominal.yaml` | `system_manipulation_D_localized.yaml` |
|---|---|---|
| Wheel configured producer / fusion | Selected / enabled | Selected / enabled |
| Wheel observation variances [v, ω] | `[1e-6, 1e-6]` | `[0.02, 0.04]` |
| Final wheel-variance resource | `robot_bringup/config/simulation_estimation.yaml` | `robot_bringup/config/calibration/manipulation_pose.yaml` |
| Joint configured producer / fusion | Selected / enabled | Selected / enabled |
| Joint q/dq assumed variance | `1e-6 / 1e-6` | `1e-6 / 1e-6` |
| Joint/wheel mandatory feedback | Required for full robot state and active actuation | Same |
| Actuation / commands by default | false / false | false / false |
| IMU built-in producer / subscription | Absent / configured, external messages required | Same |
| IMU observation | Scalar angular velocity z; configured R variance `.01` | Same |
| External pose configured publisher / fusion | Unselected / disabled | Selected / enabled |
| Localization generation | Not configured | 10 Hz; dropout `.05`; latency `.04 s` + uniform jitter up to `.01 s` |
| Localization white-noise std [x,y,yaw] | Not configured | `[.04, .04, .03490658503988659]` |
| Localization epoch-bias std [x,y,yaw] | Not configured | `[.01, .01, .008726646259971648]` |
| Localization covariance / frames | Optional source unconfigured | Message pose covariance x/y/yaw; source world/base_link; acquisition-time transform into odom |
| Robot reorder holdback | `.03 s` | `.08 s` |
| Synthetic target measurement | Selected; configured generation covariance `.01 I₃`, estimator assumed variance `[.01,.01,.01]` | Same |
| Physical base truth | Unselected by default | Selected; explicit imperfect-source/context/FK routing, not direct robot fusion |
| Tracking TF | Derived from wheel pose + measured joints | Derived from physical base pose + measured joints |
| Joint publication rate | Not separately configured | Not separately configured |

Resource provenance is recorded during the existing parameter merge: selected estimator resources first, then the selected `estimation_overrides` resource for its keys. This does **not** reconstruct each inherited key's original declaring ancestor. That limitation is explicit in `provenance_scope`. Runtime truth selection reports the effective runtime override separately from the producer owner.

Source-generation effects and estimator R assumptions remain distinct. Pose covariance is transported, while PointStamped target messages carry no covariance and the target filter constructs R from its own configuration. Process covariance is not mislabeled as measurement covariance. The target filter has separate chronology/future checks; the inventory correctly reports no configured target input-age admission limit rather than borrowing the robot buffer's age limit. TF transport topic/rate values absent from an owner are not invented; its topic is labeled as not configuration-owned TF2 default transport.

## Verification

All evidence below is task-owned under `/tmp/sensor_inventory_phase1_yv7zd1ab`. Previous results directories were not used as output destinations.

| Check | Actual result / evidence |
|---|---|
| Initial missing-feature tests | 14 failures before inventory implementation, `red.log`. |
| Timing-reporting regression | The added target-timing check reproduced incorrect reporting of robot-buffer age; corrected metadata only, `target_timing_red.log`. No estimator change. |
| Focused inventory/M07/configuration/architecture/resource tests | **146 passed**, `focused.log`. Includes both manifests, independent publisher/fusion states, absent source resource, changed topic/override, static tracking, no calibration overlay, detached output and no subprocess creation. |
| Final inventory/provenance suites | **37 passed**, `provenance_final.log`; includes actual M08 resolved JSON and materialized seed/resource checks. |
| Functional regression | Initial run **1,349 passed / 18 deselected**, `regression.log`; final closure run **1,349 passed / 18 deselected**, `regression_closure.log` (22.86 s). |
| Intermediate regression limitation | One run: 1 failure / 1,348 passes. Unchanged `test_immediate_new_session_orphan_remains_owned[1]` read an existing but still empty PID file: `ValueError: invalid literal for int() with base 10: ''`. Neither that test nor the lifecycle implementation was changed. All five parameterized cases passed on targeted rerun; combined targeted run **39 passed**, `focused_final.log`. |
| Repository style, separately | **12 failed / 6 skipped / 1,349 deselected**, `style.log`. These root-invoked tests scan generated build/install copies as well as existing source. No repository lint-clean claim. |
| New-module/test style | `flake8 --jobs=1 --max-line-length=99` passes for both new Python files. Existing modified modules retain pre-existing style debt; the two test functions extended for provenance received local formatting only. |
| Build | Workspace bringup symlink build passed. Clean, nonsymlink copied build: **10 packages finished**, `copy_build.log`; final bringup refresh passed. Byte-compilation-disabled/build warnings were nonfatal. |
| Existing copied-install gate | Passed: **7 console entrypoints, 100 resources, nominal/CV/DLS implementations, invalid manifest rejection and physical Xacro limits**, `copied_install_verification.json`. |
| New copied inventory resolution | Both requested manifests and the new inventory module resolve within the copied prefix; every known selected owner resource stays there, `copied_sensor_resolution.json`. |
| Original resolved-plan equivalence | Both captured original plans compare exactly equal after removing only the additive inventory field, `plan_equivalence_final.json`. |
| Navigation | All **35** local links in `docs/architecture/sensors.md` resolve. |

### Normal ROS/Gazebo startup and cleanup

Used the permanent `robot_bringup closed_loop.launch.py` and each normal installed manifest, with `gui:=false enable_actuation:=false publish_commands:=false`. No temporary holding plugin, altered sensor model or validation-only world configuration was substituted. Each successful trial observed a **3 s simulation window after stream readiness**. Counts below cover the complete observed startup/readiness/window interval, not a measured frequency or acceptance fraction.

| Install / manifest | Robot estimates | Predictions | Control telemetry | Valid control decisions | Localization messages | Cleanup |
|---|---:|---:|---:|---:|---:|---|
| Workspace / nominal | 226 | 53 | 103 | 60 | Not selected | SIGINT; zero owned survivors |
| Workspace / localized | 127 | 61 | 102 | 60 | 47 | SIGINT; zero owned survivors |
| Copied / nominal | 151 | 52 | 104 | 60 | Not selected | SIGINT; zero owned survivors |
| Copied / localized | 175 | 53 | 91 | 60 | 40 | SIGINT; zero owned survivors |

All four logged the computed startup summary. Live publisher queries found no IMU publisher in any isolated test, no localization or physical-truth publisher in nominal tests, and `localization_sensor` plus bridged physical truth in localized tests. These observations are stored separately in the smoke result JSON; they do not mutate the configuration inventory into a purported live-health record. Successful evidence: `workspace_smoke_network/*/result.json` and `copied_smoke/*/result.json`.

The first two sandboxed attempts failed before clock availability because network-interface discovery returned `Operation not permitted`; their startup summaries were visible and cleanup left zero owned survivors. They remain in `workspace_smoke/`. The successful retries used the authorized local networking needed by DDS/Gazebo. No production workaround was added.

These are Compute Only integration checks, not new physical-speed safety, RTF or tracking-performance benchmarks. No 90 s trials or tuning were performed. Existing research and physical-safety limitations remain unchanged.

## Reproduction

From `/home/user/ros2_ws`:

```bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/sensor_inventory_reproduction_ros

python3 -m pytest src/robot_bringup/test/test_sensor_inventory.py \
  src/robot_bringup/test/test_system_configuration.py \
  src/robot_bringup/test/test_architecture.py \
  src/robot_bringup/test/test_architecture_tree.py src/robot_configuration/test \
  src/robot_experiments/test/test_configuration.py \
  src/robot_experiments/test/test_runner.py -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros \
  -k 'not copyright and not flake8 and not pep257'

python3 -m pytest src -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros \
  -k 'not copyright and not flake8 and not pep257'

python3 -m flake8 --jobs=1 --max-line-length=99 \
  src/robot_bringup/robot_bringup/factory/sensor_inventory.py \
  src/robot_bringup/test/test_sensor_inventory.py

ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_nominal.yaml \
  gui:=false enable_actuation:=false publish_commands:=false
# Stop with Ctrl-C, then select the other normal manifest:
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_manipulation_D_localized.yaml \
  gui:=false enable_actuation:=false publish_commands:=false
```

The [navigation guide](../architecture/sensors.md) gives the programmatic preflight command for detailed JSON, without launching ROS/Gazebo. Ordinary startup shows only the concise summary. The task-owned `/tmp/sensor_inventory_phase1_yv7zd1ab/smoke.py --output <fresh-directory>` reproduces both bounded live checks with existing recorder/domain/process helpers. It requires normal DDS/Gazebo local networking. Each output directory must be fresh.

The copied build used the following form, with no workspace overlay or symlink installation:

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH bash -c '
    source /opt/ros/jazzy/setup.bash
    export PYTHONDONTWRITEBYTECODE=1
    colcon --log-base /tmp/sensor_inventory_phase1_yv7zd1ab/copy/log build \
      --executor sequential \
      --build-base /tmp/sensor_inventory_phase1_yv7zd1ab/copy/build \
      --install-base /tmp/sensor_inventory_phase1_yv7zd1ab/copy/install
  '
```

For independent reproduction substitute a fresh prefix. From a clean shell, source that prefix, change directory to `/tmp`, and run `tools/validation/m07_verify_installation.py --prefix <copied-install> --output <fresh-json>` using its absolute source path. The task's copied smoke and preflight checks used exactly that isolated environment and `PYTHONDONTWRITEBYTECODE=1`.

## Independent acceptance gates

| Gate | Verdict | Evidence / scope |
|---|---|---|
| Sensor architecture discoverability | PASS | One linked guide; all required sources and current call paths identified. |
| Correct effective configuration | PASS | Actual M07 merge supplies all values; alternate override/topic/no-overlay tests; original-plan equality. |
| Producer/fusion/physical-truth classification | PASS | Independent enable cases, external-only IMU, truth/context/TF tests and observed isolated graphs. |
| New-engineer navigation clarity | PASS | Canonical owner links, practical selection instructions, extension walkthrough and compatibility warning. |
| Single authoritative configuration hierarchy | PASS | No new input schema/catalog/parameters; selected owner and override provenance only. |
| Functional/architecture regression | PASS | Full closure regression, focused architecture/resource checks and copied-install verification. Existing lint debt and intermittent PID-file fixture race remain explicit. |
| Behavior preservation | PASS | Additive plan equality; unchanged numerical/source/safety owners; four normal Compute Only launches and cleanup. No claim of retested hardware/physical safety acceptance. |
| Maintainability/minimal code complexity | PASS | One pure metadata module, separate formatter, small assembler/launch integration, existing M08 serialization reused; no packages/moves/registries. |

## Exact task file list and preservation

Added:

- `docs/architecture/sensors.md`
- `docs/validation/sensor_inventory_closure.md`
- `src/robot_bringup/robot_bringup/factory/sensor_inventory.py`
- `src/robot_bringup/test/test_sensor_inventory.py`

Modified:

- `README.md` — link the new engineering entrypoint.
- `src/robot_bringup/robot_bringup/factory/system_configuration.py` — track known parameter-resource provenance during its existing merge and attach computed metadata after existing preflight.
- `src/robot_bringup/launch/closed_loop.launch.py` — display the formatter's summary on the manifest path; preserve existing lifecycle logic.
- `src/robot_experiments/test/test_configuration.py` — assert materialized seed/resource inventory; locally format the extended test function.
- `src/robot_experiments/test/test_runner.py` — assert inventory survives actual resolved JSON serialization on failed startup; locally format the extended test function.

The task began with **1,623 inventoried tracked/nonignored files**, including dirty implementation work and the two architecture-review documents. The final content audit confirms only these five existing-file edits and four additions; the other **1,618 existing files remain byte-for-byte unchanged** (`preservation_final.json`). HEAD and the staged diff remain unchanged. Historical experiment results were never output destinations. No owner YAML, estimation/model, prediction, control, actuation or sensor-generation implementation was edited. In particular, M08 production runner/materializer/benchmark comparison rules remain unchanged.

Focused final diff review found one configuration authority, no simulator side effects in inventory construction, explicit optional-source handling and detached output data. The declarative source entries describe existing topology rather than loading implementations dynamically. No adapter consolidation, callback extraction, package moves, new localization implementation or sensor-comparison policy was started. Nothing was staged, committed or pushed. Stop at Phase 1.
