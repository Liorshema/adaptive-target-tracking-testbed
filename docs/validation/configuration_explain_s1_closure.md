# S1 — Experiment explain and effective configuration inspection

Date: 2026-10-24 (client timezone Asia/Jerusalem). **Verdict: PASS. S1 only; S2 is not implemented.**

## Problem and implementation

The existing validate-only command checked configuration/components but did not materialize every variant/seed or show actual scientific variables before execution. `--explain` now produces one versioned JSON document from the real preparation path, then removes its isolated temporary inputs. No study, ROS domain, recorder, ROS node or Gazebo launch is created.

The implementation follows the inspected production path and the architecture proposal's section 7.2. No new YAML interpreter, configuration schema, assignment mechanism, registry, algorithm selection option or execution engine was added.

Both execution and explanation call the existing `configuration.materialize_run`, followed by the extracted `preparation.complete_preparation`. The latter contains the former runner's readiness/skew, scoring-window, metric-definition fingerprint, scientific conditions, `comparison_views` and configuration identity statements. It has no lifecycle or measurement responsibilities. The runner keeps partial original-contract evidence when comparison preparation fails.

Existing comparison checks now reside in `comparison.comparison_reasons`, called by the benchmark with its unchanged runtime default. Explain explicitly selects configuration-only checking: the same strict hash/policy/invariant checks run, while execution completion, metric/coverage evidence and measured acceptance remain unevaluated. No dummy measurements or fabricated completed runs are used. Allowlists, masks, original/invariant hash algorithms and runtime metric gates are unchanged.

Materialization order, owner values, independent seed injection, M07 factories/preflight and sensor inventory remain unchanged. The only metadata additions on the normal path are:

- `source_ancestry` in the resolved experiment/spec: child-to-parent paths collected by its existing loader transform.
- `parents` in captured YAML owner-resource snapshot records, collected by the existing materializer's inheritance walk. World-source snapshots are not YAML inheritance records and appear as leaf provenance in explain.

Original results are not rewritten. Original configuration contracts/hashes retain their previous meaning. Explicit-policy invariant hashes include the existing source-tree fingerprint, so hashes for a changed implementation revision legitimately differ from earlier recordings; no historical hash was reinterpreted.

## CLI contract

```bash
source install/setup.bash
ros2 run robot_experiments run \
  --config config/sensor_noise_comparison.yaml \
  --explain
```

Output is JSON on stdout. Dependency diagnostics and explain errors go to stderr. Preparation must finish successfully for all runs before success JSON is printed. Invalid inputs return 2 with empty stdout; valid explanations return 0 even when the configurations are incompatible for comparison. Such incompatibility is a scientific input finding, not a failed explanation.

`--explain` and `--validate-only` are mutually exclusive. Unknown flags remain argparse errors. `--config`, normal execution, normal error reporting and the original validate-only identity output are preserved. `--output` is permitted in explain mode as a prospective destination shown in the experiment metadata; it is never created. There is no persistent explain-output option or new experiment catalog.

## JSON interface and actual example

Authoritative output has `schema_version: 1`, `mode: explain`, effective `experiment`, seed-major `runs`, and `comparison`. All configured variants/seeds are represented.

| Field | Contents / interpretation |
|---|---|
| `experiment` | Name, description if supplied, source, source ancestry, seeds, duration, settling, execution/runtime policy, metrics requested and prospective output |
| `runs[].system` | Original system/profile identity, selected component identifiers and actual preflight implementation classes |
| `runs[].scenario` | Actual dynamics/environment, initial state, tracking/context, integration, physics, transport, terrain and world hash |
| `runs[].robot` | Validated authoritative description/topology/limits, deployment and native holding configuration |
| `runs[].estimation` | Separate robot/target implementations and merged parameters |
| `runs[].prediction` | Selected predictor/adaptation/fusion implementations and full configuration |
| `runs[].control` | Controller/model/safety implementations, owner parameters, strategy, actuation constraints and runtime flags |
| `runs[].sensors` | Materialized M07 producer/fusion/measurement/routing/covariance/timing/resource inventory; configuration, not live health |
| `runs[].sources` | Original selected resources, captured hashes and parent edges, runtime/seed override summary and provenance scope |
| `runs[].seed_consumers` | Effective target and independently derived optional component seeds |
| `runs[].experimental_variables` | Name/path/value, permitted declared values and matching selected captured owner resources |
| `runs[].configuration_validation` | Materialized M07 owner-preflight result, not measured safety or control validity |
| `runs[].comparison` | Complete original/invariant contracts and hashes, protected conditions, configuration decision and null runtime eligibility |
| `comparison.configuration_compatibility` | Whole-study compatible boolean and precise rejection reasons |
| `comparison.runtime_eligibility` | Null; runtime completion, measurements, metrics and coverage have not been evaluated |

Projection of real installed-command output (other fields omitted from this example, not from actual output):

```json
{
  "schema_version": 1,
  "mode": "explain",
  "runs": [
    {
      "variant": "baseline",
      "seed": 42,
      "experimental_variables": {
        "pose_white_noise_std": {
          "path": "localization_sensor.localization_sensor.white_noise_std",
          "value": [0.04, 0.04, 0.03490658503988659]
        }
      }
    },
    {
      "variant": "higher_noise",
      "seed": 42,
      "experimental_variables": {
        "pose_white_noise_std": {
          "path": "localization_sensor.localization_sensor.white_noise_std",
          "value": [0.08, 0.08, 0.06981317007977318]
        }
      }
    }
  ],
  "comparison": {
    "configuration_compatibility": {"compatible": true, "rejection_reasons": []},
    "runtime_eligibility": null
  }
}
```

Full JSON is retained in `/tmp/configuration_explain_s1_ax_dakwl/installed/sensor_noise_comparison.json`. It shows separate robot EKF / target KF implementation classes, IMM / residual adaptation / weighted-correction fusion, WLS controller, configured actuation true, headless runtime and enabled physical truth. No actual control validity or physical performance is claimed.

The two original hashes are respectively:

- `eaed6bed93ba15fc3af4804cd83465374d8764378c1f84bd0413ce2aeda378cc`
- `70cd56201966e6118151c80ff4d01849e34e5c817232c864b94dc0fcb5de2f11`

These exactly match the original hashes from the prior M08 demonstration. The shared invariant hash for this source checkout was `27c569b827b016d00b5731212a2e9dddaf3e1db36dc5b30a21a84952db53a134`. It includes current source provenance; it is not required to equal a historical different-source invariant. Target seed is 42 and derived localization seed is 3872720740 for both variants.

## Effective values, provenance and limitations

Source identities and selected resources remain stable, original package paths. Transient run-input prefixes are labeled `<materialized>` **only in display metadata**; those paths identify inputs already removed at command completion. Scientific comparison views and hashes are returned unchanged, without display-path rewriting or extra ignored fields. Repeating the real noise explanation under the same checkout and arguments yielded identical parsed JSON.

Experimental variables use existing declaration semantics: `declared_values` is a permitted set, not a unique expected value or per-variant assignment. Actual materialized `value` is shown independently. `owner_resources` identifies selected captured owner documents with that effective leaf; ancestry is available separately. It does not falsely claim that the selected document originally declared every inherited key.

Original estimator/calibration resources are discoverable in `sources.selected_resources`; the actual merged values and final materialized parameter ownership appear in estimation and sensor inventory. Scope is selected-resource/ancestry and final owner metadata, not complete per-key historical tracing. No general provenance-history engine was introduced.

Parameter dictionaries are validated owner/preflight inputs. Actual implementation identities and selected strategy describe the instantiated component. Runtime flags describe launch precedence: for example WLS may supersede the ROS owner's default DLS selection, and runtime command/truth publication flags can override owner defaults. The explanation exposes both rather than silently modifying captured owner dictionaries.

Window `start` and `end` remain null. Duration/settling are available, but readiness source epoch is not known until execution. Static sensor enablement is not runtime health. Configured physical limits/holding/safety are not a certificate of measured physical safety. No performance, tracking-error, coverage or acceptance metrics are fabricated.

The copied install outside Git reports source-tree identity unavailable rather than inventing it. Metric-definition fingerprinting remains install-location independent. Equal inputs/seeds do not imply bitwise deterministic reactive trajectories; S1 adds inspection, not replay guarantees.

## Validation and evidence

Evidence root: `/tmp/configuration_explain_s1_ax_dakwl/`.

| Check | Actual result / evidence |
|---|---|
| Red phase | `red.log`: 14 failed / 2 passed before implementation; failures were missing `--explain`. |
| Initial implementation | Tests caught non-YAML world-resource provenance lacking YAML parent metadata; corrected the projection, leaving contract evidence untouched. |
| New explain tests | 19 cases in `test_explain.py`; included in focused/full results below. |
| Focused M08 + shared loader/resources + M07 inventory/preflight + architecture | **326 passed in 25.30 s**, `focused.log`. |
| Full functional regression | **1,402 passed, 18 deselected in 39.51 s**, `regression.log`. |
| New preparation/explain/test files and modified comparison owner flake8 | Passed with `--jobs=1 --max-line-length=99`; `new_style_final.log` empty. |
| Workspace affected-package build | `robot_experiments` passed in **8.81 s**, `build.log`. |
| Copied package build | `robot_experiments` passed in **3.76 s**, `copied_build.log`; nonfatal byte-compilation-disabled/underlay warnings are retained. |
| Old versus new real normal preparation | **15/15 cases equivalent**, `before_after.json`: original pre-S1 runner plus original materializer compared with new normal path before any runtime acquisition. Generated YAML values, M07 parameters/identities/inventory, complete comparison views and hashes match. Only each run-owned file-path prefix is normalized for file/dictionary comparison; comparison views/hashes are compared exactly. |
| Installed CLI / all entries | **4 entries, 15 expanded runs**, `installed_checks.json`; normal command, one complete JSON per entry, clean scratch and no prospective results directory. |
| Copied install CLI / all entries | Same **15 runs** from clean-shell `/tmp` cwd, `copied_checks.json`; code loaded from copied nonsymlink prefix, no workspace resource paths, same original hashes/seed consumers as workspace. Sensor invariant hashes match within each environment. |
| Existing results preservation during installed CLI checks | **6,566 existing result files** retained identical size/mtime metadata before/after; `installed_checks.json`. No prior results were output destinations. |
| Independent code review | PASS; no material findings in scoped before/current diff. Reviewed preparation/failure evidence, default runtime gates, cleanup and display/hash boundaries; read-only, no runtime launch. |

All supported entries were exercised with actual owner configuration:

| Entry | Variants | Seeds | Expanded runs | Configuration compatibility |
|---|---|---|---:|---|
| `nominal.yaml` | imm | 42 | 1 | true |
| `paired_prediction.yaml` | imm, cv | 42 | 2 | true under existing strict contract |
| `reactive_manipulation.yaml` | A, B, C, D, E | 42, 917 | 10 | false; protected physical/scenario paths reported, as intended |
| `sensor_noise_comparison.yaml` | baseline, higher_noise | 42 | 2 | true under existing explicit sensor policy |

This represents ten configured variant entries across four experiments and two distinct seed values; 15 expanded runs. The manipulation incompatibility is preserved, not hidden to make the command pass.

Tests also cover allowed values versus actual values, missing sources/leaves, protected scenario changes, invalid/incomplete declarations, missing package resources, inheritance cycles, owner-preflight failure, CLI incompatibilities/unknown flags, partial failure after a first successful variant, existing result sentinel preservation, diagnostics on stderr, source YAML hashes, temporary-directory cleanup and unchanged validate-only/normal delegation.

Lifecycle entry points (`DomainLease`, `OwnedLaunch`, `run_window`, `Recorder`, experiment runner) are forbidden by test doubles during explain tests. Normal preparation-equivalence checks deliberately stop before runtime acquisition. Temporary inputs are removed on successful and failing explanations. Read-only Git subprocesses for existing source provenance are not experiment sessions. No ROS/Gazebo experiment was launched by `--explain`, and no Gazebo performance run was necessary: preparation equivalence and the functional suite demonstrated the extraction preserved runtime inputs/behavior.

The 18 deselected checks are style/copyright families. Existing broader lint debt is separate: earlier project validation recorded 12 style failures / 6 skips. That historic debt was not repaired or claimed freshly revalidated here. New scoped style checks pass; this is not a claim that every existing module is lint-clean.

### Reproduction commands

```bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_LOG_DIR=/tmp/configuration_explain_s1_ros
ros2 run robot_experiments run --config config/sensor_noise_comparison.yaml --explain
ros2 run robot_experiments run --config config/paired_prediction.yaml --validate-only

python3 -m pytest src/robot_experiments/test src/robot_configuration/test \
  src/robot_bringup/test/test_system_configuration.py \
  src/robot_bringup/test/test_sensor_inventory.py \
  src/robot_bringup/test/test_architecture.py \
  src/robot_bringup/test/test_architecture_tree.py \
  -q --import-mode=importlib -p no:launch_testing -p no:launch_ros \
  -p no:cacheprovider -k 'not copyright and not flake8 and not pep257'
python3 -m pytest src -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros -p no:cacheprovider \
  -k 'not copyright and not flake8 and not pep257'
python3 -m flake8 --jobs=1 --max-line-length=99 \
  src/robot_experiments/robot_experiments/preparation.py \
  src/robot_experiments/robot_experiments/explain.py \
  src/robot_experiments/robot_experiments/comparison.py \
  src/robot_experiments/test/test_explain.py
colcon build --symlink-install --packages-select robot_experiments
git diff --check
```

The copied build used the previous complete copied-package installation as unchanged underlay and copied only the affected experiment package. Reproduce with fresh build/install paths:

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /tmp/m08_scientific_6b4f0z4q/copied/install/setup.bash
colcon build --packages-select robot_experiments --executor sequential \
  --build-base /tmp/s1_recheck/build --install-base /tmp/s1_recheck/install
source /tmp/s1_recheck/install/setup.bash
cd /tmp
ros2 run robot_experiments run --config config/sensor_noise_comparison.yaml --explain
```

Task-specific scripts `compare_before.py`, `installed_checks.py`, `check_copy.py` remain in the evidence directory. They intentionally create fresh evidence directories; choose new destinations when repeating them, preserving recorded outputs.

## Exact file changes

Modified:

1. `src/robot_experiments/robot_experiments/cli.py` — explain flag, mode exclusivity and stdout/stderr boundary.
2. `src/robot_experiments/robot_experiments/configuration.py` — additive experiment/resource ancestry metadata.
3. `src/robot_experiments/robot_experiments/runner.py` — use shared final preparation and retain partial failure evidence.
4. `src/robot_experiments/robot_experiments/comparison.py` — share existing pair/policy checks with explicit configuration-only mode; runtime default preserved.
5. `src/robot_experiments/robot_experiments/benchmarking.py` — delegate existing eligibility-reason collection to its comparison owner.
6. `docs/m08_experiments_and_benchmarking.md` — practical command, JSON navigation, precedence and acceptance interpretation.

Added:

7. `src/robot_experiments/robot_experiments/preparation.py` — extracted pre-runtime comparison/scoring preparation.
8. `src/robot_experiments/robot_experiments/explain.py` — temporary preparation and detached structured projection.
9. `src/robot_experiments/test/test_explain.py` — focused CLI/effective-value/equivalence/no-side-effect coverage.
10. `docs/validation/configuration_explain_s1_closure.md` — this report.

No source YAML was changed, moved, renamed or added. M07, sensor producers, estimation, prediction, control, native actuation and lifecycle implementations were not changed. The architecture proposal and earlier experiments/evidence remain intact. No staging, commit or push was performed.

Final content audit (`preservation_final.json`): all **156** pre-existing YAML files are byte-identical. Of **1,636** initial tracked/nonignored files, **1,630** remain byte-identical; only the six listed existing files changed and the four listed files were added. None was deleted. Git HEAD and index are unchanged. `git diff --check` and the scoped style check passed after documentation completion.

## Acceptance and next boundary

**PASS**: all four supported entries explain every variant/seed using actual materialized owners; effective selections/noise/seeds/constraints and source ancestry are discoverable; original/invariant contracts and configuration decisions reuse M08; runtime acceptance remains pending; cleanup, backwards compatibility, source/install behavior, no-runtime guards, full functional regression and before/after equivalence pass.

Remaining limitations are resource-level rather than full per-key ancestry, source identity unavailable outside Git, and absence of measured runtime evidence by design. S1 provides no parameter assignments or new algorithm comparison domains. Next step, only after separate authorization: **S2 — validated sensor parameter assignments**, retaining existing paths and requiring old/new materialized-value/contract equivalence. Do not combine it with M07 schema redesign, YAML consolidation, archival or dashboard implementation.
