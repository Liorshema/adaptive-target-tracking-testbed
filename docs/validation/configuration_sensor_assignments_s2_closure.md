# S2 — Validated Experimental Sensor Assignments

2026-10-24. **CONDITIONAL**. S2 implementation, configuration equivalence and
integration are verified. The one real comparison completed, but its higher-noise
ROS joint-observation coverage gate withheld runtime eligibility. No gate was
relaxed. S3 and legacy-resource archival remain out of scope.

## Implementation and ownership

Experiment schema 2 selects one global `experiment.system_config`. Each variant
contains exactly `name` and `parameters`. Every named comparison variable needs
one explicit assignment and explicit permitted `values`. Comparison schema 1,
its existing eight sensor leaves, masks, hashes and runtime eligibility rules are
unchanged. Schema-1 loading and per-variant system selection remain supported.

`assignments.py` binds names through the existing comparison policy, requires
finite numeric values without coercion, checks exact permitted values, and routes
only into the selected planar-localization sensor or target-position covariance
owner. Existing producer validators check shapes, probabilities, latency budget,
covariance symmetry/PSD and actual enabled producer availability. It does not
change estimator covariance assumptions. The direct producer-validator dependency
is declared in the package manifest and the acyclic architecture test.

All schema-2 variants undergo temporary materialization/M07 preflight during
experiment loading, including `--validate-only`, before any results or runtime
acquisition. A bad later variant therefore cannot launch an earlier valid one.
Runtime/explain reuse the same materializer: load detached owners, apply and
validate assignments, derive independent seeds, write run-local inputs, execute
M07 preflight on those actual files, then form existing comparison views.

Duplicate declaration keys and YAML merge keys at declaration scopes are rejected;
shared `extends` inheritance is unchanged. A system reference is atomic: provide
a complete `{package, path}` mapping or relative filename at its declaring document.
Mixed global/per-variant system selection is rejected. No unrestricted dotted-path
mutation, second resolver, source modification, or new comparison permission exists.

Explain uses the same actual materialized values and reports baseline/requested
values, source owner and declaring document/variant/parameter separately. Complete
and invariant views remain factual. `runtime_eligibility` is null before execution.
Review caught and resolved nested-merge duplicate bypass and late-variant owner
validation; regression tests cover both and the established CLI output channels.

## Equivalence and simplification

New entry: `src/robot_experiments/config/sensor_noise_study.yaml`.
It preserves the internal `sensor_noise_comparison` identity, both variants,
seed 42, 8-second measurement/3-second settling windows, all eight metric families,
actuation, trace recording, readiness/deadlines, 150 ms alignment and policy.

The old and new entries were prepared with the same current implementation.
Both variants match all 11 generated input YAML dictionaries, complete M07 plans
and inventory, identities, context, seed consumers, protected scientific settings
and full comparison views. Only temporary run-directory prefixes were normalized
in generated files/plans; no comparison view was normalized or mask changed.

| Variant | Complete original contract SHA-256, identical old/new |
|---|---|
| baseline | `eaed6bed93ba15fc3af4804cd83465374d8764378c1f84bd0413ce2aeda378cc` |
| higher_noise | `70cd56201966e6118151c80ff4d01849e34e5c817232c864b94dc0fcb5de2f11` |

Both old/new variants have invariant hash
`bb8c245fd7ebb460dd58966bde9be565e51dbb8842658037b9837d825d607d73`
for frozen source fingerprint
`d6607693847339a8cdf02695a1c4f9cfae48276061cd92e41dc925a53f91aa5f`.
Both compatibility decisions accept the configuration with no rejection reasons.
Target seed remains 42; independently derived localization seed is 3872720740.

Experiment-specific forwarding resources: **4 → 0**. Baseline captures 46 source
resources in both entries; higher noise captures 50 old versus 46 new. New higher
noise selects the baseline localization owner plus explicit assignment; old higher
noise retains its four wrapper sources. This provenance difference is recorded,
not hidden. All five existing experiment/system/profile/scenario/sensor YAMLs
remain intact, runnable and unarchived. Scientific contracts protect resolved
owner contents under the existing rules.

## Verification evidence

Evidence root: `results/sensor_assignments_s2_validation/` (generated, ignored).
An interruption removed earlier `/tmp` logs; essential final checks were repeated
into this workspace directory. Existing implementation was preserved.

- `focused_completion.log`: **57 S2 tests passed**, 34.00 seconds.
- `full_completion.log`: **1459 passed, 18 style tests deselected**, 72.88 seconds.
- `equivalence.json`: complete deterministic old/new materialization evidence.
- Installed `explain_old.json` and `explain_new.json`: complete comparison views
  match; repeated new explain JSON is identical; stderr empty; runtime pending.
- `style.json`: flake8/pydocstyle clean for assignments, explain and S2 tests.
  Existing formatting debt remains in configuration.py (365 diagnostics) and
  architecture tests (20); the earlier scoped delta check introduced none.
- Normal dependency build completed through robot_simulation; final bringup and
  experiment build: 2 packages passed in 13.3 seconds (`build_completion.log`).

## Runtime, installation and preservation

The fresh copied installation built all **10 packages** in 3 min 29 seconds
(`copied_build_completion.log`), including native Gazebo plugins. Both old and new
installed explain entries were invoked from `/tmp` with the primary workspace
environment unset: copied modules/resources are regular installed files, no
primary source/build/install paths occur in the explanation, and comparison
views match. `copied_verification.json` records the check. An external install
without a Git checkout correctly has no checkout source fingerprint; that is not
normalized into the frozen checkout fingerprint.

The new entry ran through the existing M08 runner in:

`results/sensor_assignments_s2/sensor_noise_comparison_20261024T113624_303754Z_d62m_2cw`

Both runs completed, all selected source/estimation/prediction/control/command/
actuation streams were recorded, both observed component identity checks passed,
and both launches exited cleanly on SIGINT. Complete and invariant hashes exactly
match the deterministic preparation evidence above; both run metadata source
fingerprints equal the frozen source. Requested/baseline values and assignment
provenance are saved in each `resolved_configuration.json`.

| Per-run observation | baseline | higher_noise |
|---|---:|---:|
| Mean distance error (m) | 0.098709 | 0.097353 |
| Max distance error (m) | 0.189094 | 0.205038 |
| Mean pointing error (deg) | 6.679060 | 6.636176 |
| Max pointing error (deg) | 18.844227 | 18.922775 |
| RTF | 0.783794 | 0.766176 |
| Localization records | 124 | 127 |
| Target measurement records | 200 | 205 |
| Joint-state records | 3298 | 2427 |
| Robot estimate records | 508 | 485 |
| Prediction records | 200 | 204 |
| Control records | 263 | 259 |
| Actuation records | 128 | 119 |

Counts above cover the complete recording, including startup/settling. Tracking
statistics use each eight-second window. All streams report zero invalid or
out-of-order records. Invalid commands, watchdog events, rejected commands and
feedback rejections are zero in both evaluation windows. These are raw per-run
measurements; the runner suppresses aggregate comparison eligibility.

**Conditional result:** baseline acceptance is true; higher-noise acceptance is
null, study CLI exit code is **3**, and `summary.json` preserves
`higher_noise seed 42: required physical joint observation coverage incomplete`.
Higher-noise observed joint coverage spans 7.898/8.0 seconds (98.725%), with a
252 ms maximum gap, exceeding the unchanged 150 ms observation budget. No
infrastructure launch failure occurred, and no configuration mismatch was found.
The cause of the ROS observation gap is not established in S2. It is not treated
as a legitimate scientific comparison or bypassed using a larger skew budget.

The complementary 2 ms physics traces are complete from startup through shutdown:
6474 samples per joint for baseline and 6676 for higher noise. They have zero
position/speed violations under the existing 0.001 rad / 0.001 rad/s numerical
tolerances. Maximum shoulder penetration below its zero boundary is
`6.471531042777723e-5 rad`; maximum speed excess above the description is only
`5.009579417958321e-10 rad/s` on the elbow. Neither tolerance nor physical limit
changed. Peak measured speeds are:

| Joint | Limit (rad/s) | Baseline peak | Higher-noise peak |
|---|---:|---:|---:|
| base_yaw_joint | 1.0 | 0.686244 | 0.642810 |
| shoulder_joint | 1.0 | 1.000000 | 1.000000 |
| elbow_joint | 1.0 | 1.000000 | 1.000000 |
| wrist_joint | 1.5 | 0.692714 | 0.682137 |

Native motor torque is unavailable (`max_abs_written_effort_nm: null`), so these
traces establish position/speed evidence, **not physical effort compliance**.
The existing `analyze_trace` acceptance does not replace the M08 ROS observation
coverage gate. There is no real-hardware safety claim.

`live_measurements.json` and the original study directory retain all measurements,
coverage/status decisions, assignments, traces and cleanup evidence. Both cleanup
survivor lists are empty. Independent process-environment scanning found no
processes in either owned Gazebo partition (`processes_after.json`).

`preservation.json` verifies that all 1643 files present at resumption, including
the five legacy YAML resources and unrelated edits, retain identical SHA-256;
6558 historical result files retain their sizes and modification times. The
checkout source fingerprint, Git HEAD and index are unchanged. The only new
nonignored file after recovery is this requested closure report; S2 implementation
edits had already survived the interruption. Generated copied-build resources are
excluded from workspace discovery by an ignored `COLCON_IGNORE` marker.

| Gate | Verdict |
|---|---|
| Validated assignments, owner boundaries, schema-1 regression | PASS |
| Old/new complete/invariant and materialized configuration equivalence | PASS |
| Explain, functional tests, normal/copied installs | PASS |
| New-schema Gazebo launches, source activity, saved evidence, cleanup | PASS |
| Measured two-variant runtime comparison eligibility | CONDITIONAL |

S2 is finished with an overall **CONDITIONAL** verdict. To obtain an eligible
scientific sample later, repeat the unchanged bounded study and inspect the
recorded joint observation coverage; investigate delivery/recording if gaps recur.
Preserve this result and the existing strict gate. No tuning, refactoring or
additional run was used to manufacture a passing comparison.

## Reproduction

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install --packages-up-to robot_experiments
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
ros2 run robot_experiments run --config config/sensor_noise_study.yaml --validate-only
ros2 run robot_experiments run --config config/sensor_noise_study.yaml --explain
ros2 run robot_experiments run --config config/sensor_noise_study.yaml \
  --output results/sensor_assignments_s2
python3 -m pytest src/robot_experiments/test/test_sensor_assignments.py -q \
  --import-mode=importlib -p no:launch_testing -p no:launch_ros -p no:cacheprovider
python3 -m pytest src -q --import-mode=importlib -p no:launch_testing \
  -p no:launch_ros -p no:cacheprovider \
  -k 'not copyright and not flake8 and not pep257'
```

Output parent is isolated; the runner reserves unique study directories. Do not
compare reactive Gazebo trajectories for bitwise identity. One seed and eight
seconds establish integration, not statistical significance or hardware safety.
Schema 2 deliberately supports only the existing sensor policy. S3 composition,
filter selection, dashboards and eventual legacy deprecation require separate work.

## Exact implementation/documentation changes

Modified:

- `src/robot_experiments/robot_experiments/configuration.py`
- `src/robot_experiments/robot_experiments/explain.py`
- `src/robot_experiments/package.xml`
- `src/robot_bringup/test/test_architecture.py`
- `docs/m08_experiments_and_benchmarking.md`

Added:

- `src/robot_experiments/robot_experiments/assignments.py`
- `src/robot_experiments/config/sensor_noise_study.yaml`
- `src/robot_experiments/test/test_sensor_assignments.py`
- `docs/validation/configuration_sensor_assignments_s2_closure.md`

Generated logs, equivalence, copied build and runtime evidence stay in isolated
ignored result directories. Unrelated dirty edits and historical validation data
are preserved. No staging, commit or push.
