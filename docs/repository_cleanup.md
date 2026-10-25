# Repository cleanup

2026-10-25. M11.1 implementation; resumed from the same dirty working tree.
**Cleanup verdict: PASS for the demonstrated cleanup scope.** R1C benchmarking remains
**CONDITIONAL**; this task changes no scientific acceptance policy.

## Structure and configuration ownership

- `docs/validation/`: 44 permanent Markdown reports, reduced from 1.398 GB to
  0.000713 GB allocated. Historical reports describe their original milestones.
- `tools/validation/`: all 84 active/reusable/historical Python reproducers and
  seven fixture YAMLs moved together, preserving sibling imports. Their data and
  archived-source defaults now use `results/validation/`. They remain checkout
  tools rather than a new runtime package.
- `results/validation/`: 879 former top-level artifact entries, including exact
  captures, profiling results, figures and before-source snapshots. Captured bytes
  and embedded original paths remain historical evidence. Markdown links and
  current executable/data commands were updated by responsibility.
- `results/archives/`: six lossless archives replace large loose historical data
  and unused copied/build/log trees. All 11,111 members were verified, including
  SHA-256 for regular files, before each original tree was removed. Archives total
  1,593,491,137 bytes versus 10,584,569,961 regular-file bytes. Recent R1A/B/C
  evidence remains directly readable.
- Root `pytest.ini` limits default collection to source and reusable validation
  tests, supplies their sibling import path and uses importlib collection.
  `.gitignore` no longer carries artifact-specific exceptions under documentation;
  outputs belong under `results/`. `tools/show_architecture.py` owns tree rendering.

Configuration YAML count: **100 → 93**. The literal `src/**/*.yaml` count is
**102 → 93**, also removing two generated CMake logs from an inactive source-local
build cache. No new YAML abstraction or profile was introduced.

Four aliases were equal after production inheritance resolution and were deleted:

| Removed resource | Existing canonical owner |
|---|---|
| `robot_bringup/config/system_manipulation_A.yaml` | `robot_bringup/config/system_nominal.yaml` |
| `robot_bringup/config/simulation_estimation_localized.yaml` | `robot_bringup/config/calibration/manipulation_pose.yaml` |
| `robot_simulation/config/scenarios/reactive_target_oscillatory.yaml` | `robot_simulation/config/scenarios/oscillatory_reactive_tracking.yaml` |
| `robot_simulation/config/scenarios/reactive_manipulation_localized.yaml` | `robot_simulation/config/scenarios/base_relative_manipulation.yaml` |

All live consumers, tests and current reproduction references use these owners.
Variant A keeps its scientific label. The distinct bringup profile named
`reactive_manipulation_localized.yaml` remains. The two meaningful rejected
allocation candidates `whole_body_arm_preferred.yaml` and
`whole_body_oscillatory_arm_preferred.yaml` now inherit `whole_body.yaml` and
override only their original motion weights. Both runtime/preflight use the
existing shared loader. All **21 full experiment preparations** match the baseline
exactly: generated inputs, identities, selection, context, comparison contracts
and seed assignments. Only temporary run paths and a fixed functional-test source
label are normalized; runtime fingerprints and comparison eligibility are intact.

Other deletions, checked for static/dynamic consumers and backed up:

- `robot_simulation/config/comparison/baseline.yaml`,
  `config/experiments/flat_baseline.yaml`, `config/experiments/turn_baseline.yaml`:
  unconsumed legacy templates; obsolete install registrations and empty directories
  removed. These old manual resource paths are intentionally retired.
- `robot_prediction/robot_prediction/history/trajectory_buffer.py` and
  `twist_buffer.py`: unused empty subclasses of `VectorBuffer`; substantive history
  implementations and mathematical extension modules remain.
- Root `project_tree.txt` and `project_tree_current.txt`: stale generated exports.
- Seven dangling symlinks in each of the existing `build/` and `install/` caches:
  these referenced the seven deleted YAMLs. Targets are recorded for rollback.
  The first incremental build exposed the stale cache; rebuilding after this
  scoped cache cleanup passed. No unrelated cache links were removed.

Archived loose trees: `results/reactive_challenge_study`, the `copied` subtrees
of `sensor_assignments_s2_validation`, `configuration_composition_s3a_validation`
and `configuration_nominal_composition_s3b1_validation`, old root `log`, and inactive
`src/robot_simulation/build`. Exact source versions, raw physics CSVs, failed attempts
and metadata were compressed, not truncated or discarded. Normal root build and
installation remain usable. The newly verified copied build lives in `/tmp`.

## Verification

| Check | Actual result |
|---|---|
| Functional baseline | 1,586 passed; 18 style checks excluded; one expected failure |
| Full post-cleanup functional regression | 1,586 passed; same exclusions and expected failure |
| Focused relocation/configuration checks | 405 passed; three style checks excluded; one expected failure |
| Final targeted checks | 27 passed; one expected failure |
| Current/archived instrumentation | All four cases pass using actual module globals |
| Configuration | All 93 source configuration YAMLs resolve; all seven installed experiment entries validate; 21 preparations identical |
| Normal symlink build | All ten packages pass after known stale-link removal |
| Fresh non-symlink build | All ten packages pass; byte-compilation-disabled warnings retained |
| Fresh installation from `/tmp` | All ten packages, native actuator, M07/M08/M09 contracts, resource bytes, Xacro limits and launch construction verified; retired resources/modules absent |
| Preservation | All 1,670 initial files and 16,844 artifact records accounted for; no unexpected data changes; numerical/physics/safety source hashes unchanged; HEAD/index unchanged |

The initial recursive test command collected archived source trees and failed with
104 collection errors. The corrected active baseline explicitly supplied sibling
imports and disabled the two ROS launch pytest plugins; the new default collection
scope prevents archive collection. A root-scanning lint attempt was stopped and
excluded from evidence. Package-local checks show existing lint debt:
bringup flake8 506 → 505 diagnostics, simulation 1,213 → 1,209; pep257 remains
7 and 6 respectively. No new diagnostic code per file was introduced. Style is
not claimed clean, and no lint rule was weakened.

The normal installed `config/paired_prediction.yaml` ran once, headless, seed42,
three seconds settling plus six seconds evaluation for each variant. Both reached
readiness and completed simulation → estimation → prediction → control → actuation.

| Variant | Distance mean / max m | Pointing mean / max deg | RTF | Control mean / max ms | Acceptance |
|---|---:|---:|---:|---:|---|
| imm_seed42 | 0.0928 / 0.2046 | 10.07 / 14.39 | 0.838 | 16.73 / 60.74 | UNKNOWN / CONDITIONAL |
| cv_seed42 | 0.1373 / 0.3177 | 10.00 / 16.46 | 0.889 | 17.36 / 42.94 | PASS |

The runner returned **3**. IMM joint evidence covered 5.952/6 s (99.2%), with a
128 ms maximum gap versus the unchanged 80 ms policy. Its observed position/speed
checks pass, but incomplete coverage keeps physical acceptance unknown. CV covered
6/6 s with a 38 ms maximum gap and passed the configured gates. Both passed the
invalid-command, watchdog, rejection and feedback gates. These are short smoke
observations, not a cleanup performance benchmark or new hardware-safety claim.
No retry was used to select a passing dataset. Both process trees shut down with
SIGINT, return code0 and no survivors; the final host check found no ROS/Gazebo
children. [Raw study](../results/repository_cleanup/runtime/paired_prediction_20261025T192709_423001Z_85ij7j2j/summary.json) and
[extracted measurements](../results/repository_cleanup/runtime_summary.json) retain
the unchanged acceptance outcomes. R1C remains CONDITIONAL.

## Storage and reproducibility

| Allocated storage (decimal GB) | Before | After |
|---|---:|---:|
| `.` | 19.781276 | 10.818781 |
| `docs/validation` | 1.398391 | 0.000713 |
| `results` | 17.328660 | 10.110833 |
| `build` | 0.248795 | 0.248766 |
| `install` | 0.003465 | 0.003437 |
| `log` | 0.120189 | 0.009679 |

Workspace reduction: **8.962 GB (45.3%)**, including the preserved
archives and rollback backup. Relocating 1.4 GB from documentation into results
means that results storage includes those captures as well as compressed archives.
[Before sizes](../results/repository_cleanup/storage_before.json) and
[after sizes](../results/repository_cleanup/storage_after.json) contain exact bytes.

Allocated bytes are measured by `du -s -B1`; apparent sizes are also recorded.
Workspace totals include `.git`, retained evidence, cleanup backups and fresh
validation output; `/tmp` copied installation is excluded. Compression preserves
raw observations and does not change experiment metrics.

The reversible initial backup and exact per-file change manifest are local:

- [Backup verification](../results/repository_cleanup/backup.json),
  [initial baseline](../results/repository_cleanup/baseline.json) and
  [change/hash manifest](../results/repository_cleanup/manifest.json).
- [Exact relocation map](../results/repository_cleanup/relocations.json),
  [deletions/canonical owners](../results/repository_cleanup/deletions.json),
  [experiment equivalence](../results/repository_cleanup/equivalence.json),
  [retired build links](../results/repository_cleanup/retired_build_links.json),
  [retired installation links](../results/repository_cleanup/retired_install_links.json).
- [Build log](../results/repository_cleanup/build.log),
  [copied build](../results/repository_cleanup/copied_build.log),
  [copied installation](../results/repository_cleanup/copied_installation.json),
  [full regression](../results/repository_cleanup/functional_tests.log),
  [lint comparison](../results/repository_cleanup/style_comparison.json).

Archive manifests under `results/archives/*.json` give each member's original
path/type/mtime/checksum, whole-archive SHA-256 and exact restore command. Example,
from the workspace root:

```bash
tar -xzf results/archives/reactive_challenge_study.tar.gz -C .
```

The initial backup restores every initially nonignored file plus the ignored tree
export: `tar -xzf results/repository_cleanup/before.tar.gz -C .`. It is a rollback
snapshot, not an instruction to overlay old files onto ongoing work blindly.
Restoring deleted cached symlinks also requires their recorded targets. Source
restoration plus a normal rebuild recreates installation resources. Local ignored
data/archives/backups must accompany scientific evidence during transfer; they
do not ship with a source clone. No commit or push was performed.

Reproduction after sourcing ROS:

```bash
colcon build --symlink-install --executor sequential
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -q \
  -p no:launch_testing -p no:launch_ros \
  -k 'not flake8 and not pep257 and not copyright'
ros2 run robot_experiments run --config config/paired_prediction.yaml --validate-only
ros2 run robot_experiments run --config config/paired_prediction.yaml \
  --output results/cleanup_reproduction
python3 tools/validation/m06_verify_instrumentation.py --output results/instrumentation
```

The runner reserves unique study directories; exit code 3 means completed but
conditional/failed acceptance. Historical hash-preservation utilities remain
milestone-specific reproducers, not current gates; do not rewrite their original
expectations to manufacture success. Substantive alternate numerical modules,
meaningful scenario/controller/sensor variants and distinct engineering reports
were retained because they have actual mathematical or reproducibility roles.
No unrelated numerical refactoring, parameter tuning or safety relaxation occurred.

Independent read-only review found no Critical/Important implementation defect. It
flagged one stale canonical filename in the manipulation variant table; that
reproduction reference was corrected to `system_nominal.yaml`. The reviewer sampled
archive members and deferred full archive/runtime execution to this task: the
executor verified every member before archival removal, rechecked all six whole
archive hashes, and completed installed/live checks above. An initial reviewer
could not run because of a usage limit; the replacement review completed.

**Verdict: cleanup PASS; research benchmarking CONDITIONAL.** No demonstrated
functional regression, lost evidence or changed numerical/safety source was found.
Remaining limits are existing lint debt, local-only evidence distribution, intentional
retirement of the named compatibility resources, short runtime coverage and the
recorded IMM coverage gap. Historical reports and substantial alternate algorithms
remain because they preserve distinct engineering evidence and reusable mathematics.
