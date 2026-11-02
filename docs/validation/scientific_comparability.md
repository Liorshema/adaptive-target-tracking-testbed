# D2.9 — Scientific comparability and condition ranking

Implemented and verified with 90 engine/comparison tests, 40 Builder/Results tests,
two successful package builds and actual Chromium viewport checks. **Scientific
comparability: CONDITIONAL; dashboard ranking: CONDITIONAL for live acceptance.**
Both real non-actuated Runs completed, but CV physical coverage was incomplete;
the existing gate correctly withheld comparability and Rank. No safety criterion
was relaxed and no eligible live ranking is claimed.

## Implementation plan and scientific contract

Extend the existing comparison engine. Preserve the schema-1 sensor policy and
strict hash comparisons. New schema-2 declarations authorize exact registered
component selections or owner-schema parameter leaves, with a separate dynamics
category. Retain original contracts/hashes; derive mirrored projections from
explicit roles and owner schemas, never by dropping whole protected dictionaries.

1. Add typed owner-validated declarations and exact mirror handling; test real
   materialization for IMM/CV, DLS gains and dynamics parameters. Keep incompatible
   estimator/controller task interfaces fail-closed.
2. Revalidate frozen evidence, group authoritative Experiment/Condition/Variant
   identities, and require complete matched seeds. Rank conditions by equal-weight
   per-seed recorded radial RMSE, with sample standard deviation and per-seed values.
   Tracking outcome, execution, safety and comparability remain separate.
3. Add minimal collapsed Lab declaration controls through Author/Catalog and make
   the existing Rank column consume backend decisions. No independent editor,
   comparison engine, Run store, simulation lifecycle or numerical algorithm change.
4. Run focused regressions/build/browser checks. Use historical Runs read-only and
   at most one isolated bounded paired study if practical, with actuation explicitly
   OFF. If live newly-declared ranking cannot be demonstrated, mark that gate
   CONDITIONAL. Preserve all existing evidence and do not commit/push or start D3.

The dirty-tree baseline is `results/scientific_comparison/before.tar.gz`.

## Policy and ownership

Schema 1 `sensor_parameters` and strict original-hash comparisons retain their
saved representation and existing behavior. Schema 2 adds three typed domains:

| Domain | Exact public paths | Owner |
| --- | --- | --- |
| `component_models` | `components.predictor`, `components.controller`, `components.robot_estimator`, `components.target_estimator` | Registered component selection, schema and deployment preflight |
| `component_parameters` | `parameters.<role>.<registered scalar/array leaf>` for the four roles above | Active component's parameter schema and exact resolved transport |
| `target_dynamics` | `components.target_dynamics`, `parameters.target_dynamics.<registered leaf>` | Dynamics owner; explicitly labeled dynamics study |

The policy has exactly `schema_version`, `domain`, `variables`. Each variable
has `name`, exact `path`, and optional permitted `values`; model selection requires
registered permitted IDs. Parameter values must satisfy owner schemas and the
active resolved owner's schema. Permitted values are an allowlist, not a declaration
of a full factorial design. At least two distinct variable assignments and complete
matched-seed Conditions are required for a model/parameter ranking.

```yaml
comparison:
  schema_version: 2
  domain: component_models
  variables:
    - name: predictor
      path: components.predictor
      values: [imm, constant_velocity]
```

Jacobian gain experiment (use the same compatible DLS base for both revisions):

```yaml
comparison:
  schema_version: 2
  domain: component_parameters
  variables:
    - name: gains
      path: parameters.controller.gains
      values: [[1.0, 1.0, 1.0], [2.0, 2.0, 2.0]]
```

Dynamics-parameter experiment (not evidence of predictor superiority):

```yaml
comparison:
  schema_version: 2
  domain: target_dynamics
  variables:
    - name: acceleration_limit
      path: parameters.target_dynamics.a_max
      values: [0.15, 0.3]
```

These declarations are supported both in canonical experiment definitions and
one-file `experiment.comparison` Author inputs. `null` explicitly selects strict
comparison when branching from a declared base. Model/parameter choices are
exposed by `ExperimentBackend.list_comparison_variables()` from deployable owner
registries. The dashboard's collapsed **Advanced Comparison Setup** selects one
exact variable; existing multi-variable and sensor declarations are preserved
without reduction. Canonical CLI/Author inputs support multiple exact variables.

### What is retained and rejected

Original contracts, original hashes and raw protected conditions are retained.
Only detached derived views are projected. The invariant hash includes the
projected contract and projected scientific conditions. Predictor selection
projects its selection leaf; all prediction parameter blocks stay protected.
Controller identity uses the exact flat/strategy mirrors. Estimator identity
normalizes its selected/canonical encoded algorithm identities, retaining process,
measurement and adapter bindings. Model-specific mathematical leaves are projected
only individually, after verifying they equal registered defaults. Shared leaves
remain protected. Exact declared parameter leaves project only their active
transport, including flat DLS gains and encoded intrinsic estimator parameters.

Rejected examples include `parameters.controller.*`, any dictionary/container
mask, bounds/safety/timing/binding declarations, undeployed model IDs, an undeclared
adaptation/fusion/controller/sensor change, changed robot or physics, different
scoring/settling windows, missing metrics/coverage, incomplete seeds, failed
execution, and modified frozen comparison/hash evidence. Selecting an existing
WLS preset versus the existing DLS preset is **not** a controller-only comparison:
their tasks, lookahead and sampling settings differ. Those differences are retained
and rejected. Current KF/EKF deployments do not expose interchangeable target
interfaces; registering an ID alone cannot bypass owner preflight. A compatible
registered estimator extension is verified with the existing isolated test fixture.

## Conditions, evidence and ranking

The backend identifies a Condition using frozen Experiment ID, named Variant and
exact declared effective variable assignments. Immutable revisions with the same
Condition are grouped across matching seeds; a UUID remains Run provenance. For
strict modern evidence, the revision participates in identity. Duplicate
Condition/seed observations, extra seeds and incomplete declared seed sets block
ranking. Declared paired-study Variant coverage is also checked. Separate saved
revisions should share a pinned baseline, declaration, seed set and protected
conditions; unrelated base scenarios do not become comparable just because their
model IDs differ.

Each new-policy Run's comparison view is recomputed and checked against
`comparison.json` and the independent existing `resolved_configuration.json`
copy. Original contract, hash, invariant fields and effective variables must agree.
Metadata/status Experiment identity, frozen resolved selections and the existing
owner seed-consumer evidence are checked. The primary measurement seed must match
the Run's declared Seed; no timestamps, measurements or historical metadata are
rewritten. These checks detect inconsistent artifacts; this is not a signed or
adversarially tamper-proof storage system.

Scientific comparability retains metric definitions/units, required observations,
complete physical coverage, completed execution and the full configured scoring
window. Poor measured tracking remains comparable. Ranking has separate guards:
physical safety must be accepted, existing recorded command-safety gates must
pass, and every Run must have a finite nonnegative saved
`tracking_distance.rmse` in metres with a positive integer observation count.
Missing RMSE never falls back to mean error. Actuation-disabled execution does not
claim hardware/actuation safety or demonstrate closed-loop tracking performance.

Rank is based on the equal-weight mean of each matched Seed's recorded radial
RMSE, with sample standard deviation across Seeds (unknown/null for one Seed),
count and per-seed Run references retained. Equal aggregate scores share a rank.
No weighted score, significance claim or universal winner is generated. Independent
pointing/estimation/prediction RMSE remain separate dimensions. The existing CLI
benchmark's historical mean-statistic summaries remain unchanged; its additional
ranking object uses the same backend ranking function as Sessions.

Historical undeclared Runs retain strict descriptive comparisons when their
existing evidence supports those comparisons. Missing frozen Condition/seed
policy cannot manufacture a paired rank. Session JSON continues to hold immutable
Run references only, without a new results store.

## Real paired study — actual results, not ranking evidence

Exactly one paired study was executed, through the existing immutable Catalog and
Runner, headless with `enable_actuation: false`, `gui: false`, Seed 42, six scored
simulation seconds and the inherited three-second settling policy. No physics,
readiness/skew/timeout, algorithm or acceptance settings were changed. The CV
variant changes only prediction type and retains adaptation/fusion and all other
protected conditions. The actual loaded classes were `IMMPredictor` and
`ConstantVelocityPredictor`; frozen source hashes match across both Runs.

Study: `results/scientific_comparison/live/nominal_20261102T092746_387134Z_qzhhxo3p`.

| Measurement | IMM | CV |
| --- | ---: | ---: |
| Run ID | `4ce2a7ef-299c-41b9-afb6-bf51e5df9ee9` | `d1477a39-39ca-413e-a4ff-777516a9c41d` |
| Execution | Completed | Completed |
| Scoring window [simulation s] | 4.402–10.402 | 4.842–10.842 |
| Recorded radial RMSE [m] | 0.765429 | 0.762845 |
| Recorded pointing RMSE [deg, converted from rad] | 7.741472 | 8.226913 |
| Prediction RMSE [m] | 0.535568 | 0.519967 |
| Recorded target estimation RMSE [m] | 0.073993 | 0.077085 |
| RTF | 0.836924 | 0.832048 |
| Physical position/speed observation coverage | 100% | 98.9333% |
| Maximum physical observation gap [s] | 0.020 | 0.144 |
| Recorded physical safety acceptance | True | Unknown |
| Cleanup survivors | 0 | 0 |

Session: `3349f30d-c4cf-454d-abf6-38ed20e583aa` references the two original immutable
Runs. Its backend result is `live_session_comparison.json`; compact numerical
provenance is `measurements.json`. Both configured windows completed and metrics
were saved. Runner returned code 3 for incomplete acceptance, **not** failed
execution: actuation was intentionally not demonstrated, and CV's physical
coverage did not satisfy the existing gate. The scientific comparison correctly
rejects the incomplete CV coverage; Rank is **Unavailable**. These descriptive
numbers cannot establish predictor superiority. No retry or acceptance relaxation
was performed. Distinct ROS domains/Gazebo partitions and cleanup evidence are
saved in the original Run metadata/status.

## Verification and limitations

- Real owner materialization/preflight covers predictor-only IMM/CV, original and
  doubled Jacobian gains, WLS gains, model/estimator parameters, dynamics parameter
  and model changes, and a compatible isolated registered estimator extension.
- Test-only paired ranking evidence uses two Conditions and Seeds 42/43. Saved
  RMSE fixtures aggregate to 0.3 m and 0.5 m, each n=2, sample std approximately
  0.141421 m, ranks 1/2. Equal one-Seed scores tie at rank 1 with std null. These
  are explicitly unit-test values, not recorded experimental results or Runs.
- Negative tests cover undeclared neighbors, sensor/controller/robot/physics/task
  differences, incomplete/extra/duplicate Seeds, failed execution, absent or invalid
  RMSE/count/units, metric contracts/windows/coverage, safety gates, seed relabeling,
  and altered derived/original/rehash evidence. Existing sensor and strict tests
  are retained. UI tests verify authority comes from backend Condition ranks and
  aggregated saved RMSE rather than browser sorting or metric-mean substitution.
- Installed Chromium renders only the genuine live study. Lab (closed and expanded
  Comparison Setup) and Results have document dimensions exactly 1920×1080 at
  device scale 1, no page scrolling, four KPIs and four equal 936×368 Results
  panels. No browser page errors; both real ranks remain Unavailable. Screenshots:
  `lab_1920x1080.png`, `lab_comparison_1920x1080.png`, `results_1920x1080.png` under
  `results/scientific_comparison`; assertions are in `browser_report.json`.
- All 748 pre-existing recorded JSON artifacts match the pre-task SHA-256 manifest
  (`preservation.json`). Existing dirty source edits and evidence were preserved.
  No production files were removed, no new results store or execution engine was
  added, and no controllers, predictors, estimators or physical dynamics were edited.
- Single-Seed studies have no estimable across-seed variability or significance.
  Matching Seeds do not imply deterministic or identical reactive trajectories.
  Incompatible deployed estimator/controller interfaces remain rejected; future
  models still require owner registration and compatible deployment preflight.
- **Scientific comparability: CONDITIONAL for live acceptance.** Configuration,
  pairing, integrity and rejection behavior are verified, but the sole live pair
  has incomplete physical coverage. **Dashboard ranking: CONDITIONAL for live
  acceptance.** Backend-driven positive/tie/multi-seed ranking is unit-tested;
  real-browser verification demonstrates correct withholding, not an eligible live
  colored rank. The existing coverage reliability issue remains outside D2.9.

## Reproduction

From `/home/user/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select robot_experiments robot_dashboard --symlink-install

# Read-only actual Session/Run evidence.
build/dashboard_venv/bin/python - <<'PY'
import json
from pathlib import Path
p = Path('results/scientific_comparison/live_session_comparison.json')
r = json.loads(p.read_text())
print(r['session']['session_id'], r['comparability'], r['ranking'])
PY

# A manual rerun creates a new study; frozen configuration explicitly disables actuation.
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  ros2 run robot_experiments run \
  --catalog results/scientific_comparison/live_catalog.yaml \
  --experiment EXP-CONTROLLED-PREDICTION

# Existing dashboard; the validation listener is closed after screenshot capture.
build/dashboard_venv/bin/python -c 'from robot_dashboard.cli import main; main()' \
  --port 8061 --data-dir results/scientific_comparison/workspace \
  --catalog results/scientific_comparison/live_catalog.yaml \
  --result-root results/scientific_comparison/live
```

In Lab, load a pinned saved base, declare the exact variable through **Advanced
Comparison Setup**, select permitted registered models or parameter values, then
Validate and Save. Revise the same experiment to the second configuration using
the same declaration, seed set and other conditions; Validate/Save and run each
saved revision. Add matching Runs to the same Current Session. The backend, not
manual table selection, determines eligibility. Existing multi-variant canonical
studies can also run through the ordinary Catalog/CLI Runner.

## Exact production, test and documentation changes for D2.9

This list is relative to the preserved dirty-tree snapshot, not to Git HEAD:

- `src/robot_experiments/robot_experiments/comparison.py`
- `src/robot_experiments/robot_experiments/comparison_variables.py` (new, exact owner projections/discovery)
- `src/robot_experiments/robot_experiments/benchmarking.py`
- `src/robot_experiments/robot_experiments/preparation.py`
- `src/robot_experiments/robot_experiments/authoring.py`
- `src/robot_experiments/robot_experiments/backend.py`
- `src/robot_experiments/test/test_component_comparison.py` (new)
- `src/robot_experiments/test/test_condition_ranking.py` (new)
- `src/robot_dashboard/robot_dashboard/app.py`
- `src/robot_dashboard/robot_dashboard/forms.py`
- `src/robot_dashboard/robot_dashboard/results.py`
- `src/robot_dashboard/robot_dashboard/assets/dashboard.css`
- `src/robot_dashboard/test/test_builder.py`
- `src/robot_dashboard/test/test_results.py`
- `docs/dashboard.md`
- `docs/validation/scientific_comparability.md` (new)

Generated immutable study, isolated Catalog/Session metadata, logs, test outputs,
scripts, preservation manifest and screenshots are contained separately in
`results/scientific_comparison/`. `changed_files.json` is the machine-readable
source/test/doc list. No commit or push; D3 was not started.

### Final test/build evidence

- `verified_engine.txt`: **90 passed**, 93.30 s. Includes all new component/Condition
  cases, existing sensor/strict comparisons and benchmark/report regressions.
- `final_ui_tests.txt`: **40 passed**, 231.34 s, with both genuine historical
  completed/failed Run fixtures enabled. Includes Builder HTTP save validation,
  duration/revision, Results, Session and authoritative rank presentation tests.
- `final_build.txt`: **2 packages built successfully** (`robot_experiments`,
  `robot_dashboard`), 22.0 s. Final `git diff --check` for changed tracked source
  and tests passed. No claim of a repository-wide style/lint pass is made.
- `browser_verified.txt` / `browser_report.json`: actual installed Chromium check
  exited 0; registered model choice → Preview succeeded, no page errors, required
  viewport/panel assertions passed and genuine unavailable ranks were displayed.
- Earlier red/development runs and their logs were retained. The broad combined
  regression attempt `final_regression.txt` ended with signal/exit 143 before a
  pytest summary and is **not** counted as a pass. The completed final focused
  suites above are the regression evidence; no full-workspace test claim is made.

```bash
source install/setup.bash
source results/dashboard_execution/evidence.env
export PYTHONPATH="src/robot_dashboard:src/robot_experiments:$PYTHONPATH"
build/dashboard_venv/bin/python -m pytest \
  src/robot_experiments/test/test_condition_ranking.py \
  src/robot_experiments/test/test_component_comparison.py \
  src/robot_experiments/test/test_scientific_comparison.py \
  src/robot_experiments/test/test_reporting_benchmark.py -q
build/dashboard_venv/bin/python -m pytest \
  src/robot_dashboard/test/test_builder.py \
  src/robot_dashboard/test/test_results.py -q
# With the isolated dashboard serving on 8061:
build/dashboard_venv/bin/python results/scientific_comparison/browser_check.py
```
