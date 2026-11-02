# Task E.1 — frozen estimator algorithm parameters

Task E.1 closes the intrinsic-setting transport boundary recorded by Task E.
Architecture acceptance and the bounded runtime observation are reported separately
below. KF/EKF mathematics, physical models, Q/R ownership, initialization, sensor
routing, timestamp/reset policy, prediction, control and safety limits are preserved.

## Implementation

The missing path was explicit: Task E's role builder rejected every nonempty
algorithm default mapping; role schemas and ROS declarations had no intrinsic
settings transport. Materialization could freeze physical estimation parameters
but could not reconstruct a parameterized algorithm independently of installed
defaults.

Algorithm definitions still live once in `estimators.<id>.parameters` in the
Estimation owner YAML. Their trusted `ModelRegistration` declares schema, types,
ranges and dependencies. Production `kf` and `ekf` retain empty intrinsic mappings.
Process/measurement covariance and physical parameters stay in their existing
separate sections. There are no new production algorithms or YAML profiles.

The existing role registry exposes nested `algorithm_parameters` using the selected
algorithm's schema. ExperimentAuthor accepts
`experiment.parameters.<robot_estimator|target_estimator>.algorithm_parameters`.
It uses the existing recursive mapping merge and registry validation: owner defaults,
applicable inherited settings, then explicit local edits. Lists/scalars replace;
false, zero and empty values retain their types when permitted. Current registration
schemas do not permit null parameters. Changed algorithm identity discards the old
algorithm's settings while retaining compatible physical-model overrides. Aliases
compare canonical identity; `robot_ekf` → `ekf`, `kalman` → `kf` remains unchanged.

Each role resolves independently. No mutable defaults or effective mappings are
shared. Preview displays the effective map; Catalog save/revise captures it through
existing overrides. M07 preflight resolves and validates the complete mapping before
construction and materialization. The public CLI and factory defaults remain
compatible.

Existing `inputs/estimation.yaml` now contains two optional ROS STRING fields:

```yaml
robot_algorithm_parameters: '{"algorithm":"ekf","parameters":{}}'
target_algorithm_parameters: '{"algorithm":"kf","parameters":{}}'
```

These JSON envelopes preserve nested types and canonical identity using the existing
flat typed ROS boundary. They are transport fields, not a second authoring surface:
the public Author rejects them in component parameter edits to prevent competing
representations. Prepared snapshots always contain both complete mappings, including
empty ones. Existing source-resource provenance also records the algorithm owner
YAML; resolved scientific conditions include the frozen values. Snapshot schema and
Catalog format are unchanged; no second snapshot/result store is introduced.

The ROS node declares and forwards both strings through its existing parameter
cache, including clock-reset reconstruction. The role builder checks the envelope's
canonical algorithm identity and validates its complete map. The existing
`ComponentRegistry.validate/create` accepts `use_defaults=False` for resolved values;
both the decoder and numerical algorithm construction use it. This bypasses recursive
default merging, including new nested default keys. Current installation schemas and
trusted implementation code are still required; incompatible schema changes reject
rather than silently filling missing settings. Missing transports on historical
empty-setting deployments remain supported. Nonempty settings without a frozen
transport reject at the actual ROS construction boundary.

Unknown keys, invalid types/ranges, duplicate JSON keys, mismatched identities and
nonfinite values anywhere in a mapping reject. Direct library construction may
still resolve defaults; it is distinct from production frozen reconstruction.

## Parameterized fixture and reproducibility

The test-only declaration `example_fading_kf` lives under
`src/robot_experiments/test/model_fixtures/estimation/fading_filter.py`. Its sole
intrinsic setting `fading_factor` is a finite number in `[1,4]`, with default `1.5`
declared in the existing fixture owner YAML. It uses trusted namespace discovery,
the real KF interface and existing compatible CV/position/target deployment binding.
It never enters production owner defaults, installed packages or the production
inventory.

The fixture implements fading memory:

\[
P^- = \lambda A P A^T + Q,\quad
K=P^-H^T(HP^-H^T+R)^{-1},\quad
x^+=Ax+K(z-HAx).
\]

The original process Q and assumed measurement R are unchanged. Independent test
equations check predicted covariance and posterior mean; changing lambda changes
the observed posterior. This demonstrates a meaningful intrinsic numerical option,
not a renamed physical noise parameter or an additional production estimator.

The full public Author/Catalog/materialization test freezes lambda `2.0`, verifies
child inheritance and local override `3.0`, saves a second immutable revision, then
changes the isolated owner default to `3.75`. Loading the original frozen system
and constructing the actual estimator preserves the original mean/covariance
exactly. A nested actual-factory test also proves a new default key is not filled
into an older frozen map. Role-isolation tests resolve distinct settings for one
algorithm independently without claiming unsupported cross-role deployment.

Independent review found and reproduced two input defects: permissive JSON
nonfinite values in schema-open empty arrays, and competing public mapping/encoded
overrides. Both have retained failing tests followed by passing corrections in
`results/estimation_parameters/`. No acceptance threshold was weakened.

## Verification and evidence

Task-start source/history snapshot and all new evidence are under ignored
`results/estimation_parameters/`. Previous Task A/B/E changes and experimental
artifacts remain separate.

| Check | Actual evidence |
| --- | --- |
| Frozen numerical replay, legacy and canonical IDs | Each **124,305 values exactly equal**, maximum difference **0**; `numerical_legacy.log`, `numerical_canonical.log` |
| Configuration preservation | **75 direct / 21 scientific preparations equivalent**, zero scientific differences; `direct_equivalence.json`, `preparation_equivalence.json` |
| Full functional regression | **1,967 passed, 18 style wrappers deselected, 1 existing xfailed**, 1226.70 s; `regression.log` |
| Scoped functional regression | **237 passed**, `focused.log` |
| Final boundary/Author corrections | **20 transport/factory tests + 9 parameterized Author tests passed**, `transport_final.log`, `author_closed.log`; counts overlap other suites |
| Normal / copied builds | Both **10 packages passed**, 3 min 14 s / 9 min 2 s; `build.log`, `copied_build.log` |
| Final copied refresh | **4 packages passed**, `copied_build_final.log` |
| Copied installation | Production discovery remains **14 families / 30 entries**; M06–M09 checks, canonical Author/Catalog preparation and aliases passed; `installed_verification.json`, `installed_estimation_verification.json` |
| Actual parameterized copied construction | Public test-fixture workflow passed; all seven checked module origins from copied prefix; source-default mutation replay exact; `installed_parameterized_verification.json` |
| Real ROS parameter/reset boundary | Actual filter received lambda **2.25**; isolated owner default changed to **3.5**; reset retained 2.25 and exactly equal covariance; mismatched algorithm identity rejected |
| Copied source identity | **8 production Python modules + 20 owner YAML resources** byte-identical; `installed_byte_identity.json` |
| Real historical frozen inputs | Task B `robot_ekf`/`kalman` and Task E `ekf`/`kf` snapshots reconstruct without edits; canonical/alias predictions equal; `historical_snapshots.json` |
| Style | Six changed/new factory/Author/test files flake8-clean; new codec pep257-clean; **104 inherited findings unchanged, zero new** across six legacy files; `style_closed.log`, `docstyle.log`, `lint_existing_verification.json` |
| Preservation | **21 immutable revisions, 14,494 prior artifacts preserved**; no production config, numerical core, physical constraint, controller/predictor/simulation change; `change_manifest.json` |

The 75/21 comparators explicitly verify each added transport envelope has the
correct canonical identity and an empty mapping, then remove only these new
metadata fields for comparison with historical evidence. All old numerical,
physical, sensor, frame, scenario and safety values remain compared unchanged.
The final guard cases added after full-suite collection passed in the targeted
runs; counts overlap and must not be summed. Full functional assertions and
numerical tolerances were not waived.

The exact Author/Catalog frozen `inputs/estimation.yaml` was also fed directly to
the real copied-install ROS node: lambda **2.0** reached the actual filter and
remained **2.0** after an isolated owner-default change to **3.5** and clock reset
(`frozen_node_verification.json`). This closes the complete saved-input-to-ROS chain,
in addition to the independent 2.25 boundary probe.

`parameter_effect.json` retains the actual numerical demonstration. For the same
frozen CV model and position measurement `[1,-2,3]`, posterior x is
**0.9952155399263194 m** at lambda 2 and **0.99680521389093 m** at lambda 3.
Changing only the later owner default to 3.75 leaves the frozen lambda-2 posterior
mean and covariance **exactly unchanged**, maximum absolute differences **0 / 0**.
Runtime acceptance is reported separately below.

## Verdict and bounded runtime

**PASS for Task E.1 architecture and frozen intrinsic-parameter transport**, within
the existing algorithm/model/adapter contracts. A compatible parameterized algorithm
can register once, resolve owner/inherited/local settings, freeze each role, survive
later default changes and reach an actual estimator through ROS without runner
branches. Current numerical and historical behavior is preserved.

Exactly **one** copied-install headless Gazebo trial ran after all builds/tests
exited: `EXP-ESTIMATION-CANONICAL` revision 1, seed 42, inherited unchanged
`EXP-NOMINAL` revision 3. Production EKF/KF use the new explicit empty frozen maps;
the nonempty test-only filter was verified in real ROS separately and was not
promoted into Gazebo's production inventory. **Completed, accepted, exit 0**, with
all original readiness, safety and scientific gates retained. No retry.

| Scored observation | Actual result |
| --- | --- |
| Evaluation | Simulation **4.500–10.500 s**, 6 s after 3 s settling; readiness origin **1.500 s** |
| Absolute distance error, mean / max | **0.138583 / 0.265727 m**, 503 aligned samples |
| Pointing error, mean / max | **7.99366 / 13.36454 degrees**, 0.139516 / 0.233255 rad |
| RTF | **0.920020** |
| Controller compute time, mean / max | **8.346 / 23.782 ms** |
| Physical position / speed overshoot | **0 rad / 0 rad/s**, complete six-second coverage; 3,002 observations, maximum gap 0.002 s |
| Invalid commands / watchdog / rejected commands / feedback rejections | **0 / 0 / 0 / 0** |
| Cleanup | Launch exited normally, no survivors; final global process scan clear |

Evidence: `runtime_summary.json`, `runtime.log`, `process_cleanup.json` and
`runtime/nominal_20261028T081419_956657Z_g_rq4hwm/` with original gate decisions,
telemetry, metrics, frozen inputs and provenance. Original physical tolerance is
0.001 rad / rad/s; it was not changed.

This is bounded runtime acceptance, not a matched tracking/performance improvement
claim. Task E's earlier run used a different readiness origin and scoring window.
Existing localization/slip, covariance consistency, IMM/readiness variability,
long-duration runtime reliability and hardware-safety limitations remain. Current
schemas, compatible trusted implementations and model/deployment contracts must
still be installed; incompatible schema evolution rejects instead of inventing
missing values. Arbitrary new layouts/sensors and unsupported role pairings are
not advertised. No E.1 capability remains deferred.

## Reproduction

From `/home/user/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -p no:launch_testing -p no:launch_ros --import-mode=importlib -q src/robot_estimation/test/test_algorithm_parameters.py src/robot_experiments/test/test_estimator_parameters.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -p no:launch_testing -p no:launch_ros --import-mode=importlib -q src tools/validation -k 'not flake8 and not pep257 and not copyright'
```

The exact retained production integration condition can be reproduced with a
fresh output directory:

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/estimation_parameters_install/setup.bash
ros2 run robot_experiments run --catalog /home/user/ros2_ws/results/estimation_parameters/canonical_catalog.yaml --experiment EXP-ESTIMATION-CANONICAL --revision 1 --output /tmp/e1_reproduction
```

The copied prefix and isolated catalog are local validation artifacts; preserve
or transfer them with the evidence record. `verification_commands.txt` lists
numerical replay, 75/21 comparisons, copied build/discovery, actual parameterized
construction, ROS reset and preservation commands. Fixture save/revise checks
require fresh output paths; retained attempts must not be overwritten.

The fixture is intentionally test-only. A normal production one-file experiment
continues selecting `robot_estimator: ekf` and `target_estimator: kf` with inherited
bindings and no intrinsic parameter block. A future compatible algorithm declares
its numerical defaults/schema once and uses the nested Author parameter mapping;
the Experiment Runner needs no algorithm-specific branch.

No commit or push. Dashboard development is outside Task E.1.

## Exact Task-E-relative files

Relative to the task-start captured Task E tree, not Git HEAD:

```text
M docs/component_configuration.md
M docs/experiment_composition.md
M docs/validation/estimation_architecture.md
M src/robot_bringup/robot_bringup/factory/system_configuration.py
M src/robot_configuration/robot_configuration/components.py
M src/robot_estimation/robot_estimation/factory/estimator_factory.py
M src/robot_estimation/robot_estimation/nodes/estimation_node.py
M src/robot_estimation/robot_estimation/parameter_schema.py
M src/robot_estimation/test/test_estimation_bindings.py
M src/robot_experiments/robot_experiments/authoring.py
M src/robot_experiments/robot_experiments/configuration.py
M src/robot_experiments/test/model_fixtures/context.py
M src/robot_experiments/test/model_fixtures/owner_parameters.yaml
A docs/validation/estimation_parameters.md
A src/robot_estimation/robot_estimation/factory/algorithm_parameters.py
A src/robot_estimation/test/test_algorithm_parameters.py
A src/robot_experiments/test/model_fixtures/estimation/fading_filter.py
A src/robot_experiments/test/test_estimator_parameters.py
```

**13 modified, 5 added, 0 deleted.** No unused production files or redundant profiles
were introduced or removed. Prior Task A/B/E edits remain preserved.
