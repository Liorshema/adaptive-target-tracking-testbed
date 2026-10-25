# S3B1 — Nominal, CV and DLS configuration consolidation

2026-10-24. Scope: additive migration through the existing S3A schema-2 compiler.
**Verdict: CONDITIONAL overall. Configuration migration PASS; paired runtime
scientific acceptance CONDITIONAL because IMM never reached the unchanged
readiness gate. CV completed and passed its existing runtime acceptance gates.**

No runtime Python, factories, algorithms, original YAML, packaging implementation,
scientific policy, physics or safety thresholds changed. No staging, commit,
push, legacy cleanup or S3B2 migration.

## Shared baseline and actual behavior

`robot_bringup/config/systems/nominal_tracking.yaml` is the explicit common
composition. It selects the ordinary robot description (not the wrist-15
manipulation robot), robot EKF, target KF, IMM/residual/weighted correction, and
WLS through the existing `robot_control/config/whole_body.yaml` strategy.
It directly selects `robot_simulation/config/scenarios/oscillatory_reactive_tracking.yaml`
and reuses the existing flat `robot_bringup/config/simulation_estimation.yaml`.
Numerical settings stay in their existing component owners.

The physical scenario remains saturated Fight/Flight, acceleration limit 0.15,
transition 0.15, damping 0.5, tangential acceleration 0.06 about axis
`[0.3, -0.8, 0.5]`, and restoring/sinusoidal environment fields. Initial target
position/velocity remain `[2, 0.25, 1]` / `[0, 0.10, -0.12]`. The native
`dartsim_recovery` engine, 0.002 s timestep, geometry/limits, holding, actuation,
measurements, wheel-odometry feedback, TF context and integration are unchanged.
There is no built-in external planar-localization producer/fusion in this family.

`cv_prediction.yaml` inherits nominal once and changes only:

```yaml
prediction:
  type: constant_velocity
  adaptation: none
  fusion: none
```

The actual factory constructs `ConstantVelocityPredictor` instead of
`IMMPredictor`; adaptation and fusion become `NoneType`. The same prediction
owner retains dt 0.1 s and horizon 20. CV does not change the target dynamics.

`dls_control.yaml` inherits nominal once and changes only:

```yaml
control:
  type: jacobian
  strategy: null
```

Explicit null clears the inherited WLS strategy. Omitting it retains WLS and
fails the controller-type consistency check. The actual factory constructs
`JacobianController` with its existing three-axis Cartesian position task,
damping **1.0**, identity gains and **zero lookahead**. Nominal/CV construct
`WeightedLeastSquaresController` with Distance + Pointing, damping 0.08,
lookahead 0.25, existing weights and velocity feedforward. DLS is not advertised
as a matched Distance + Pointing strategy. All routes retain the final safety
pipeline and default disabled actuation/command publication.

## Before/after graphs and measured complexity

Paths below are package-relative. Arrows show source selections/inheritance,
not ROS data flow. All old files remain available and installed.

```text
OLD nominal: robot_bringup/config/system_nominal.yaml
OLD CV/DLS: system_cv.yaml / system_dls.yaml -> system_nominal.yaml
  profile: reactive_tracking_oscillatory.yaml -> reactive_tracking_3d.yaml
    -> reactive_tracking_saturated.yaml -> reactive_tracking.yaml
    -> optimal_tracking.yaml -> closed_loop.yaml
  deployment: robot_simulation/config/scenarios/oscillatory_reactive_tracking.yaml
    -> reactive_target_3d.yaml -> reactive_target_saturated.yaml
    -> reactive_target.yaml -> moving_target.yaml -> ../launch.yaml
  calibration: robot_bringup/config/simulation_estimation.yaml

NEW: systems/cv_prediction.yaml / systems/dls_control.yaml
    -> systems/nominal_tracking.yaml (also directly launchable)
  direct deployment: robot_simulation/config/scenarios/oscillatory_reactive_tracking.yaml
  calibration: robot_bringup/config/simulation_estimation.yaml (same owner)
  component owners: same description/robot/holding/target/environment/integration/
    measurements/terrain/world/actuation/estimators/prediction/control/strategy
```

| Metric, old → new | Nominal | CV | DLS |
|---|---:|---:|---:|
| System inheritance parent edges | 0 → 0 | 1 → 1 | 1 → 1 |
| Bringup forwarding-profile documents | 6 → 0 | 6 → 0 | 6 → 0 |
| Deployment inheritance parent edges | 5 → 0 | 5 → 0 | 5 → 0 |
| Captured configuration resources | 32 → 21 | 33 → 22 | 32 → 21 |
| Effective component-owner files | 16 → 16 | 16 → 16 | 15 → 15 |
| Newly required composition/deployment YAMLs, cumulative per path | 0 → 2 | 0 → 3 | 0 → 3 |

Counts come from actual M08 resource snapshots and parsed parent chains in
`results/configuration_nominal_composition_s3b1_validation/equivalence.json`.
Owner-file counts mean independent selected numerical/physical role roots,
including world and calibration, excluding source manifests/deployments and
their ancestry. DLS has no strategy owner. The exact role list and full effective
dictionaries are recorded there. Six profile documents and five deployment
ancestor documents are no longer needed on each migrated path: **11 fewer
captured resources**. The existing M08 generated run-local `profile.yaml` remains
part of its launch ABI; this is not a source bringup forwarding chain.

Total additions are **five YAMLs**: three user-facing systems, one direct internal
deployment and one additional experiment entry. The explicit baseline has
36 leaf assignments (arrays count as one leaf). The variants change only three
and two leaves; shallow reuse avoids **67 leaf assignments** compared with three
full manifests (`2 × 36 - 5`, excluding `extends` metadata). No new loader code,
registry, framework or calibration wrapper was introduced.

## Frozen baseline and exact equivalence

Before implementation, `baseline.json` captured SHA256 for 1,654 existing
tracked/nonignored files, HEAD and Git index. `frozen_systems.json` captured
original M07 plans, actual implementation identities, configured control,
deployment and calibration. `before_all_preparations.json` captured the complete
M08 preparation and generated inputs for **six original entries / 19 expanded
runs**, under the original code path. These artifacts remain in the ignored
evidence directory `results/configuration_nominal_composition_s3b1_validation/`.

The post-change audit proves all 19 original preparations and inputs remain
identical, normalizing only each ephemeral run-directory prefix. Its fixed
cross-revision source label is a functional audit device, not runtime eligibility
evidence. Every original source YAML remains byte-identical.

Same-final-revision equivalence checks additionally cover nominal, CV and DLS
with seeds **42 and 917** and the real additive paired entry. They compare:

- Actual factory identities, selections and effective configured controllers.
- Full selected owner dictionaries, physical description, initial conditions,
  dynamics, observations, frames, calibration, physics, holding and actuation.
- Effective ROS estimation/control parameters, prediction owners and timing.
- Actual launch `Node` parameters and simulation include arguments, both default
  and explicit actuation/compute-only/strategy/lookahead/feedforward overrides.
- Complete generated M07 plan/context and all **nine** generated YAML inputs:
  control, estimation, measurements, profile, robot, robot_description, simulation,
  system and target_estimation. No additional generated files are ignored.
- Derived seed consumers, original/invariant contracts, protected conditions,
  hashes, metric definitions, runtime/evaluation settings and strict comparison
  decisions. Real target-initial-condition mismatches still fail compatibility;
  interrupted runs retain their existing rejection behavior.

Only run-local directory prefixes are normalized in generated configuration
comparisons. Scientific contract contents/hashes receive no extra normalization,
masking or field exclusions. Original and invariant hashes match between each
corresponding old/new variant at the same revision.

Raw original M07 plans differ only in `source`, `profile_path`, direct deployment
path, the new `composition` provenance block and sensor-inventory references to
those same resources. Inventory values are otherwise equal. The calibration and
component-owner paths themselves remain unchanged. Exact raw differences and
resource lists are preserved in `equivalence.json`; they are not hidden by
generic recursive exclusions. Historical source fingerprints naturally differ
after adding source resources/tests. Same-revision comparisons and the actual
runtime source identity are recorded separately.

The new paired YAML inherits the original study, replaces only the two system
references, and retains name `paired_prediction`, variants `imm`/`cv`, seed 42,
duration 6 s, settling 3 s, all eight metric families, actuation setting,
timeouts, failure policy and original **0.08 s** observation budget. No new
algorithm authorization domain or comparison policy exists. Historical strict
comparison uses its original contract/status checks; its comparable/eligible
flag alone does not prove complete physical coverage or safety acceptance.

## Deterministic verification and installation

| Check | Actual result |
|---|---|
| Tests before adding new YAML | 31 failed, missing new resources, 3.09 s (`red.log`) |
| Final focused nominal/S3A/S2 tests | **146 passed**, 66.89 s (`focused_final.log`) |
| Full functional/architecture/legacy regression | **1,548 passed**, 18 deselected, 127.81 s (`regression.log`) |
| Original experiment compatibility | **6 entries / 19 full preparations identical** (`legacy_verification.json`) |
| New-test flake8 / pydocstyle | **0 / 0 diagnostics** (`style_final.log`) |
| Normal affected-package build | **3 packages passed**, 1 min 20 s (`build.log`) |
| Symlink installed resources/launch/explain | **5 resources, 3 launch pairs, 10 explain checks passed** (`symlink_installation.json`) |
| Isolated copied affected-package build | **3 packages passed**, 4 min 42 s (`copied_build.log`) |
| Copied checks from `/tmp` | **5 regular resources, 3 launch pairs, 10 explain checks passed** (`copied_installation.json`) |

The 18 excluded checks are the established repository style/license checks,
not functional failures. Existing assembler lint debt documented in S3A is
unchanged: no runtime Python was edited. Scoped checks on both new tests pass;
there are no newly introduced functional failures. The importlib collection mode
and disabled launch-testing pytest plugins are the existing multi-package test
invocation, not changes to acceptance thresholds. The independent read-only
review found no blockers (`review.md`).

Source-tree tests use the actual package source owners. Installed checks run
from `/tmp`, resolve all new nested YAMLs through ament, assemble actual classes,
evaluate the real launch actions, and exercise old/new nominal/CV/DLS explain,
the original nominal/paired entries, S3A and validate-only. Explain never
acquires a runtime and correctly leaves runtime eligibility unknown.

The fresh copied prefix is
`results/configuration_nominal_composition_s3b1_validation/copied/install`.
It uses the unchanged complete S3A copied underlay for dependencies, then builds
the three affected packages from fresh source copies without `--symlink-install`.
Primary workspace `src`, `build` and `install` are absent from the tested module
origins. The five new resources are regular files inside the fresh prefix.
Corresponding installed old/new explain scientific views match exactly; the
four production entry explain checks also pass validate-only in both installs.

```bash
# Reuse the recorded copied build (all paths are absolute; run outside the workspace).
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /home/user/ros2_ws/results/configuration_nominal_composition_s3b1_validation/copied/install/setup.bash
cd /tmp
python3 /home/user/ros2_ws/results/configuration_nominal_composition_s3b1_validation/installed_checks.py copied
```

## One bounded ROS/Gazebo smoke and retained failure

Actual study:
`results/configuration_nominal_composition_s3b1/paired_prediction_20261024T165445_163245Z_t2c8zpgk/`.
One invocation, seed 42, original IMM/CV ordering and all original deadlines and
80 ms coverage/readiness budget. No source edits during execution; both run
metadata fingerprints equal the final equivalence audit:
`799796fec4c9ccb7e17b1751e40d33e956d7fdf1a18e5fa2e5dcf9a55180c3e5`.
Full status, raw telemetry, launch logs, host samples, generated world/configs and
comparison records are retained. CLI exit **1** is the actual failure result.

| Runtime evidence | IMM | CV |
|---|---:|---:|
| Execution | Failed: readiness timeout | Completed |
| Evaluation window, simulation seconds | Unavailable | 12.998–18.998 |
| Existing acceptance result | Unknown; no window | True |
| Distance error mean / max | Unavailable | 0.119374 / 0.211844 m |
| Pointing error mean / max | Unavailable | 0.235459 / 0.376181 rad (13.4908° / 21.5536°) |
| Control compute mean / max | Unavailable | 25.7144 / 48.7798 ms |
| Window RTF | Unavailable | 0.766822 |
| Observed position / speed overshoot | Unavailable | 0 rad / 0 rad/s |
| Physical joint observation coverage | No evaluation | 100%; 708 samples; max gap 60 ms |
| Invalid commands / watchdog / command rejections / feedback rejections | No evaluation | 0 / 0 / 0 / 0 |
| Cleanup | SIGINT, no survivors | SIGINT, no survivors |

The existing physical gate accepts CV's **sampled, bounded-gap** observations.
No physics-step actuator CSV was requested by this historical paired study.
These observations do not prove continuous speed/torque bounds, long-duration
behavior or real-hardware safety. They are also not a before/after tracking
performance comparison: only one new paired smoke was authorized/needed here,
and IMM has no valid evaluation window. No winner or aggregate comparison is
reported. The original policy correctly returns `comparable: false`,
`comparison.eligible: false`, reason `seed 42: interrupted execution`.

IMM received all 11 required streams: clock 5,211 records, target truth 1,288,
target measurement 383, joints 2,388, base truth 484, robot estimate 958, target
estimate/prediction 382 each, control 376, command 298 and actuation 163. This
is not a missing topic/algorithm failure. At wall elapsed 45.0114 s, simulation
clock 24.022 s, no readiness origin existed. Its raw active-control messages
and startup holding/future-stamp warnings remain in the logs.

The read-only arrival-order replay in `readiness_audit.json` uses the existing
`required_streams()` selection and the resolved 0.08 s readiness policy.
Among **all** callback snapshots after all required streams existed, IMM had
**zero** simultaneously fresh snapshots. Its best worst absolute source age
was **100 ms** at clock 1.282 s: target truth/measurement were 100 ms ahead of
the observer clock and joints were 90 ms behind. Thus even checking after every
callback, instead of after executor batches, cannot make the captured IMM run
ready. CV had nine fresh callback snapshots; readiness origin 9.998 s agrees
with its recorded window. The audit cannot reconstruct unrecorded middleware
delivery or attribute clock/source skew to one upstream producer.

This establishes a source-alignment/readiness reliability failure, not an
old/new mathematical or physical configuration difference. All corresponding
materialized owners/inputs match the frozen originals; no new runtime logic
exists to account for a semantic change. It does not prove that the old IMM
entry would succeed under identical scheduling. A separate follow-up should
instrument observer clock/topic latency and executor scheduling under nominal
IMM load before proposing a timing correction. Extending the deadline, widening
the 80 ms budget or using wall-time timestamps would conceal this evidence and
was not done. No repeated smoke was used to select a favorable outcome.

Actual startup-log identities for **both** runs match all ten expected roles:
robot/target estimators, predictor, adaptation, fusion, controller, model, safety,
target dynamics and integrator. IMM failure metadata has an empty actual-types
mapping because the existing runner populates it after successful window
completion. `live_measurements.json` therefore reports independently parsed
startup-log identities separately, without rewriting the original metadata.
CV metadata itself contains the correct actual identities.

`live_measurements.json`, `live_verification.log` and `readiness_audit.json`
record these measurements and their limits. Runner cleanup reported no survivors;
an independent host process audit found **zero** remaining processes in either
owned ROS domain (112/61) or unique Gazebo partition (`final_process_check.json`).
No unrelated simulation was stopped.

## Reproduction and rollback

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select robot_simulation robot_bringup robot_experiments \
  --symlink-install
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
ros2 run robot_experiments run --config config/paired_prediction_composition.yaml --explain
ros2 run robot_experiments run --config config/paired_prediction_composition.yaml \
  --output results/configuration_nominal_composition_s3b1

ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/systems/nominal_tracking.yaml gui:=false enable_actuation:=true
# Same command with cv_prediction.yaml or dls_control.yaml selects those components.
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/systems/nominal_tracking.yaml gui:=false publish_commands:=false

python3 -m pytest src/robot_bringup/test/test_nominal_composition.py \
  src/robot_experiments/test/test_nominal_composition.py \
  src/robot_bringup/test/test_explicit_composition.py \
  src/robot_experiments/test/test_explicit_composition.py \
  src/robot_experiments/test/test_sensor_assignments.py \
  -q --import-mode=importlib -p no:launch_testing -p no:launch_ros -p no:cacheprovider
python3 -m pytest src -q --import-mode=importlib -p no:launch_testing \
  -p no:launch_ros -p no:cacheprovider -k 'not copyright and not flake8 and not pep257'
python3 results/configuration_nominal_composition_s3b1_validation/equivalence.py
python3 results/configuration_nominal_composition_s3b1_validation/baseline_preparation.py after
python3 results/configuration_nominal_composition_s3b1_validation/verify_live.py
python3 results/configuration_nominal_composition_s3b1_validation/readiness_audit.py
```

Rollback requires selecting `config/system_nominal.yaml`, `config/system_cv.yaml`
or `config/system_dls.yaml` in M07, or original `config/nominal.yaml` /
`config/paired_prediction.yaml` in M08. No file deletion, revert, restore or
result overwrite is necessary. Legacy direct profile launches remain supported.

## Exact source/document delta

Added:

- `src/robot_bringup/config/systems/nominal_tracking.yaml`
- `src/robot_bringup/config/systems/cv_prediction.yaml`
- `src/robot_bringup/config/systems/dls_control.yaml`
- `src/robot_simulation/config/scenarios/oscillatory_reactive_tracking.yaml`
- `src/robot_experiments/config/paired_prediction_composition.yaml`
- `src/robot_bringup/test/test_nominal_composition.py`
- `src/robot_experiments/test/test_nominal_composition.py`
- `docs/validation/configuration_nominal_composition_s3b1_plan.md`
- `docs/validation/configuration_nominal_composition_s3b1_closure.md`

Modified:

- `docs/m07_unified_system_configuration.md`
- `docs/m08_experiments_and_benchmarking.md`

Ignored new evidence/output roots are
`results/configuration_nominal_composition_s3b1_validation/` and
`results/configuration_nominal_composition_s3b1/`. They contain scripts, snapshots,
logs, copied build/install and the unique runtime artifacts; they are not new
production components. Historical validation directories are preserved.
The final `preservation.json` audit verifies exactly these nine additions/two
document modifications, no missing baseline files, no changes to any original
source/YAML, and unchanged HEAD/index. Of 1,654 captured existing files, 1,652
remain byte-identical and only the two listed documentation files changed.

## Remaining scope

No tracking tuning, estimator redesign, actuator redesign or long-duration
reliability claim is part of this migration. Existing localization/covariance and
physical-coverage limitations remain independent concerns. The S2 252 ms and
S3A 158 ms joint-observation gaps against their unchanged 150 ms gate remain
historical failures; S3B1 does not change that gate or substitute it for this
paired study's original 80 ms budget.

Recommended S3B2 scope, requiring separate authorization: inventory manipulation
A–E behavior first, reuse shared explicit selectors only where physical/sensor/
controller settings genuinely match, keep different robot/strategy/measurement
owners independent, retain original studies and verify full seeded/materialized
and scientific equivalence. Do not copy the nominal physical scenario into
manipulation entries or expand algorithm overrides. S3B2 is not started.
