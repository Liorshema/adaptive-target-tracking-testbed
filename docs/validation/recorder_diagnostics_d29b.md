# Instrumented Recorder diagnostics — D2.9b

**Loss diagnosis: PARTIAL. Observation reliability: CONDITIONAL. Scientific
comparability: PASS for this instrumented pair. Dashboard ranking: PASS.**

The historical **scored-window coverage failure was NOT REPRODUCED** in the one
authorized instrumented IMM/CV pair. Both scored windows have 100% physical
coverage under the unchanged 80 ms hold rule. This is **not FIXED**: CV still
has real pre-scoring Recorder observation gaps, independently witnessed by the
actuation subscriber. Diagnostics narrow one interruption to the executor/service
envelope rather than a long measured normalization or persistence operation, but
do not distinguish DDS delivery loss, history eviction or the precise cause of
delayed service. No production correction is justified by this evidence.

No production code, configuration, QoS, executor, algorithm, physics, timeout or
safety threshold changed. Exactly one pair ran; no post-analysis Gazebo retry.

## Configuration, provenance and isolation

Evidence root: `results/recorder_diagnostics_d29b/` (generated, ignored).
Study: `live/nominal_20261102T143918_895978Z_ol7_mqis`.
Ordinary managed Session: `5e2889f4-356c-4a77-a12f-a01834f75369`, stored under
`workspace/sessions`; its two references point to the immutable Runs, without
copying results. No historical Session was edited.

`setup_study.py` cloned the original D2.9 definition and overrides through
`ExperimentCatalog` into an isolated writable catalog. Its immutable identity is
still `EXP-CONTROLLED-PREDICTION`, revision 1, definition SHA-256
`596f2aaf088c412fa26db7788dd3ef2fbdfe6727f461c321cfd2dec5d2c12698`.
Preflight verified Seed 42, duration 6 s, settling 3 s, actuation false, GUI false,
`max_skew=0.08 s`, and existing 45/12/150/5 s readiness/stall/wall/shutdown limits.
`--observer-diagnostics` enabled the existing collector in both Runs.

Predictor is the only experimental variable: IMM versus constant velocity.
Both retain residual adaptation and weighted-correction fusion; robot, sensors,
Fight/Flight dynamics, controller and scoring policy match. The authoritative
backend checks frozen protected conditions and reports no inadmissible differences.
Actual component identities were checked by Runner. Both Runs have source SHA-256
`f8aa299f327b7d9e445e4be3596296da0fc0a15fb00735f85f880367fea960ca`.
This is the current source snapshot, not the older D2.9 source hash; production
and test files were not edited during this study.

| Provenance | IMM | CV |
| --- | --- | --- |
| Run ID | `d69bdbd0-49e4-4237-bb1d-81a12d172de4` | `ce42fd90-4caf-4fe2-a557-eec740a7d197` |
| Execution | Completed | Completed |
| ROS domain | 145 | 209 |
| Gazebo partition suffix (`robot-experiments-…`) | `f6aa966fcf784ac49dd4a46a53989538` | `e84a833d98274960b2bce029b2b48280` |
| Scoring source window [s] | 4.708–10.708 | 5.048–11.048 |
| Owned cleanup | SIGINT; zero survivors | SIGINT; zero survivors |

Runtime: ROS 2 Jazzy, `rmw_fastrtps_cpp`, Ubuntu 24.04 WSL2; BLAS/OMP/MKL threads
each 1. The pre-existing dashboard on port 8060 remained running and untouched.
No other simulation was running before or after the pair. Root resource counters
and Runner descendant CPU are different scopes, not summed together here.

## Existing instrumentation audit

Reused owners: `recording/ros.py`, `storage.py`, `normalization.py`,
`observer_timing.py`, `execution.py` and `runner.py` in `robot_experiments`.
No duplicate collector or additional instrumentation was introduced.

| Capability | Actual retained evidence and boundary |
| --- | --- |
| Callback entry/exit and duration | Exact per-stream count/min/mean/max and histograms; detailed marks for callbacks at least 20 ms, bounded to 128. Exit precedes diagnostic aggregation. |
| Normalization, serialization, JSONL write | Separate wall durations around the real operations; synchronous line-buffered writes remain unchanged. Wall duration includes descheduling within an operation. |
| Executor service | Public `spin_once` envelopes, batch/drain timing and budgets; between-batch summaries. No private dispatch/take hook. |
| Callback/batch gaps | Exact aggregate summaries, bounded recent service-gap/batch rings. Lower-rate stream gaps also occupy the shared 128-event ring. |
| Recorder resources | Independent 2 Hz root CPU/RSS/I/O, host load, leader `/proc/self/schedstat`, cgroup counters; no scheduler trace or blocked-I/O attribution. |
| ROS MessageInfo | Two-argument public callback selected correctly. Source/received middleware timestamps and publication sequence numbers available for all joint callbacks. Publisher GID and reception sequence **unavailable for all**. |
| Scientific data | Original raw payload, header-derived simulation source time, host-monotonic arrival and last observed simulation clock remain separate. |

Middleware timestamps are raw middleware nanoseconds, not converted to either
simulation seconds or host monotonic time. Without GIDs, the collector cannot
establish per-publisher sequence continuity: its `publication_sequence_gaps: 0`
is **not evidence of zero transport losses**. Only periodic metadata samples are
retained (46 IMM and 46 CV joint samples in the final shared ring). No middleware
message-lost event callback is attached. These are explicit diagnostic limits,
not fabricated zero-loss measurements.

The final diagnostic documents are complete with no collector/resource failure.
Their constant-size rings do not retain every individual callback/executor call.
Exact all-message summaries and raw scientific arrivals remain available. The
existing stable `TimeoutObject` executor fix is already present and was preserved.

## Source-time observations and coverage

The authoritative `metrics.evaluate()` replay reproduces **every saved metric
field** for each Run. Every joint JSONL `source_time` equals its raw ROS header;
JSONL counts agree with Recorder availability and diagnostic callback counts.
Joint normalization rejection, out-of-order and duplicate-source counts are zero.
Spacing percentiles below are exact linear-interpolated empirical quantiles of
adjacent recorded joint source times inside the scoring window, not publisher
frequency measurements.

| Physical evidence | IMM | CV |
| --- | ---: | ---: |
| Joint observations, entire recording | 4,131 | 3,861 |
| Joint observations, scored window | 1,830 | 2,085 |
| Valid position/speed samples including genuine pre-start hold sample | 1,831 | 2,086 |
| Source spacing min / median [ms] | 2 / 2 | 2 / 2 |
| Source spacing p95 / p99 [ms] | 8 / 14 | 8 / 12 |
| Maximum scored source gap [ms] | 68 | 24 |
| Covered / uncovered scoring duration [s] | 6 / 0 | 6 / 0 |
| Position and speed coverage | 100% | 100% |
| Physical sampled safety acceptance | True | True |
| Maximum observed position / speed excess | 0 rad / 0 rad/s | 0 rad / 0 rad/s |
| Overall acceptance | Unknown | Unknown |

Overall acceptance remains Unknown because the existing
`physical_actuation_demonstrated` gate is null with actuation intentionally OFF.
Sampled safety of a held robot does not demonstrate moving-actuator or hardware
safety. Full coverage means the existing finite-hold contract is satisfied; it
does not mean every nominal 2 ms observation was recorded. No sample, header,
interpolation or replacement joint data was manufactured for safety evaluation.

## Timeline and causal isolation

The following arrival values are the persisted **host-monotonic seconds**,
recorded just after callback entry; they are not simulation epochs. Detailed
callback entry/exit marks, where retained, are labeled separately below.

### IMM: service interruption inside scoring, coverage remains complete

- Joint source **6.812 → 6.880 s**; arrivals
  **76186.284041974 → 76186.378656141**, a 94.614 ms host gap.
- Last all-stream arrival before the main interruption: actuation source
  **7.006 s**, observed clock **6.814 s**, at **76186.285793174**.
  Next: clock source **6.880 s** at **76186.374570943** (88.778 ms).
  The already-future actuation source demonstrates asynchronous backlog; it must
  not be aligned by replacing the recorded clock or header.
- Batch 2414: **76186.286413874–76186.385008339**, 98.594 ms, seven
  callbacks. Its initial public spin envelope lasted about 88.5 ms before drain;
  all measured callback operations in this interval were under 20 ms.
- No Runner monitor tick falls inside this joint-arrival interval. Independent
  actuation feedback supplies no timestamp strictly inside this particular gap.
  Absence of that low-rate witness is not proof the publisher stopped.
- The 68 ms source gap is below 80 ms: there is **no uncovered scored interval**.

### CV: genuine observation loss before scoring

Two largest whole-recording source gaps are outside the scoring window:

| Joint source [s] | Recorded host arrivals [s] | Host gap | Independent real feedback inside source gap |
| --- | --- | ---: | --- |
| 1.382–1.496 | 76199.872420217–76199.880033915 | 7.614 ms | Joint header 1.486, telemetry source 1.506, arrival 76199.882734514 |
| 3.696–3.800 | 76202.668710266–76202.781317830 | 112.608 ms | Joint headers 3.698 and 3.798, telemetry sources 3.702 and 3.800; arrivals 76202.462636432 and 76202.580699494 |

The witnesses come from the existing independent actuation subscriber's accepted
JointState header copied into `feedback_stamp`; actuation remains false. Their
arrivals preceding Recorder arrivals in the second row corroborate branch lag.
They are diagnostic evidence only, never substituted into `joints`.

The second gap overlaps batch 1290:
**76202.663142268–76202.778120531**, 114.978 ms, four callbacks. Public
nonblocking spin-call maximum is **108.003 ms**. The all-stream arrival blackout
is **76202.668710266–76202.777858831**, 109.149 ms, with clock advancing
from last observed 3.694 to 3.798 s. No callback normalization/serialization/write
operation of at least 20 ms was recorded there. No Runner monitor tick is inside
that interval. This supports delayed observation service rather than a single
long measured persistence operation; dispatch/take, diagnostic aggregation,
Python runtime pauses and OS scheduling are not individually resolved.

The 114 ms first-row source gap has only 7.6 ms between its own arrivals. It must
not be attributed to a 114 ms stall occurring between those callbacks. Earlier,
the largest CV all-stream arrival blackout lasts 333.139 ms at startup
(odometry source 1.340/clock 1.342 → clock 1.344); only 2 ms of observed clock
advances across it. IMM likewise has a 334.133 ms startup blackout with no clock
advance across its endpoints. These are counterexamples to equating a wall pause
with a simulation-source gap. The exact startup subcause is not established.

CV's largest **scored** joint gap is **6.156–6.180 s**, arrivals
**76205.629022893–76205.654515330** (25.492 ms). A Runner monitor tick starts
inside it at 76205.632688099, descendant CPU 453.7%, with base truth already
195.999… ms ahead of observed clock. This is correlation with asynchronous lag,
not evidence that this tick caused a loss exceeding the hold allowance.

### Measured processing, persistence and load

Full-recording summaries (including startup; durations use host monotonic time):

| Measurement | IMM | CV |
| --- | ---: | ---: |
| Scientific callbacks | 11,030 | 10,470 |
| Joint normalization mean / max [ms] | 0.157 / 13.080 | 0.182 / 11.398 |
| Joint serialization mean / max [ms] | 0.126 / 7.878 | 0.156 / 19.612 |
| Joint write mean / max [ms] | 0.093 / 17.544 | 0.112 / 28.021 |
| Joint callback total mean / max [ms] | 0.388 / 30.871 | 0.461 / 28.334 |
| Sum normalization / serialization / write [s], all streams | 1.839 / 1.223 / 1.086 | 1.986 / 1.387 / 1.247 |
| Sum measured callback total [s], all streams | 4.276 | 4.753 |
| Executor batch count / max [ms] | 3,044 / 98.594 | 2,630 / 114.978 |
| Between-batch mean / max [ms] | 0.168 / 38.177 | 0.214 / 70.352 |
| Root CPU mean / max [% of one CPU] | 67.476 / 103.1 | 68.525 / 101.1 |
| Root RSS mean / max [MiB] | 135.563 / 156.434 | 165.093 / 165.457 |
| Host load-1m min–max | 0.874–4.806 | 4.806–6.797 |
| Leader runtime / runnable-wait counter deltas [s] | 10.999 / 1.840 | 11.056 / 2.358 |

The resource deltas span retained first-to-last samples (18.281 s IMM, 17.804 s
CV), not just scoring; sample intervals reach 569/539 ms despite nominal 2 Hz.
Cgroup throttle counters remain zero. Schedstat proves runnable waiting occurred,
but is not an off-CPU trace attributing either specific interruption. Concurrent
load and instrumentation can affect scheduling. RSS includes the same Python
process across sequential variants; the higher CV RSS is not a model memory
benchmark. Write-byte deltas are 15,802,368/15,130,624; these counters do not
identify a blocked write syscall.

One concrete long callback: CV joint source 2.020 s entered at
**76200.465130343**, wrote during **76200.465437243–76200.493458235**,
and exited at **76200.493464235**. The 28.021 ms write envelope accounts for
most of this 28.334 ms callback. It is outside scoring and distinct from the
104 ms source gap; it does not prove a disk bottleneck caused that gap. IMM's
longest callback is 30.871 ms at source 9.202 (13.080 ms normalization,
17.544 ms write); CV also has a 23.260 ms clock callback at source 6.700.

### Classification and correction decision

| Candidate | Supported conclusion |
| --- | --- |
| Global publisher absence | Cannot explain CV's witnessed missing epochs: another ROS subscriber received them. Not every nominal publication is independently proved. |
| Recorder normalization / post-recording metric rejection | Not the cause of the retained gaps: zero local joint rejections; counts/headers and full metric replay agree. |
| Single long normalization / serialization / JSONL operation causing the 104 ms gap | Not supported: no such long operation overlaps; 108 ms public executor envelope is much larger than measured callback stages there. |
| Callback/executor service interruption and asynchronous branch backlog | Observed, temporally associated with real CV pre-scoring loss. Exact dispatch versus runtime/OS subcause is unresolved. |
| DDS delivery loss or finite-history eviction | Still possible. Unsupported GID/reception metadata and lack of take/history tracing prevent distinguishing them. Sequence values alone cannot locate loss. |
| OS scheduling as the sole cause | Runnable waiting is measured, but no event-level causal trace proves it is the sole cause. |
| Scientific coverage-calculation defect | Not demonstrated; authoritative replay and independent finite-hold arithmetic agree. |

**No fix applied.** A changed depth, QoS, thread model, persistence queue or hold
allowance would be speculative. Exact root-cause diagnosis is PARTIAL, despite
useful exclusion of long measured callback stages for one real interruption.
If pursued in a later task, the remaining evidence gap is publisher-identified
transport/take continuity and event-level executor/OS scheduling around an actual
loss, not another parameter sweep. No further simulation was launched here.

## Diagnostic overhead and scientific interpretation

A separate bounded offline benchmark replays the same **10,870** historical raw
dictionaries with explicit original arrivals through the actual Recorder drain
loop, normalization, serialization and writes. One warmup pair plus three
alternating OFF/ON pairs: measured wall overhead **+0.810%, +26.874%, −5.502%**,
median **+0.810%**. Scientific JSONL files are byte-identical in every pair.
The broad variation prevents a precise overhead claim. Close/finalization are
measured separately; diagnostics-ON close takes 10.9–14.0 ms and explicit final
serialization 9.5–13.2 ms in measured rounds.

This benchmark uses a scripted FIFO executor and already-decoded dictionaries;
it excludes DDS, ROS payload reflection and real MessageInfo extraction. It
measures pipeline cost only, not live overhead or uninstrumented reliability.
The new measured RTFs are 0.780969 IMM and 0.822111 CV. Historical uninstrumented
D2.9 RTFs were 0.836924/0.832048, but those are different runs/load and do not
establish causal instrumentation overhead or a before/after improvement.

| Recorded descriptive metric | IMM | CV |
| --- | ---: | ---: |
| Radial error mean / RMSE / max [m] | 0.757759 / 0.761102 / 0.832361 | 0.752644 / 0.756207 / 0.824192 |
| Pointing error mean / RMSE / max [degrees] | 7.235319 / 7.837879 / 12.495323 | 7.380992 / 8.063799 / 12.442676 |
| Prediction RMSE [m] | 0.528336 | 0.510837 |
| Backend eligible / displayed Rank | true / 2 | true / 1 |

These are the existing policy's ranks by radial RMSE across matched Seeds,
**n=1**, sample standard deviation unavailable. No IMM/CV superiority,
statistical significance, tracking-quality acceptance or actuation demonstration
is inferred. Protected configuration comparability and execution completion are
distinct from quality and overall physical acceptance.

The actual backend and `comparison_view()` component return eligible ranks for
this complete pair. Independently, the retained incomplete D2.9 audit Session
`0107977c-ac52-47f7-bf3b-4198e88ed3df` still returns `comparable:false`,
`ranking:null`, and two **Unavailable** rank cells. This read-only check neither
rewrites that Session nor alters its original 98.9333% CV coverage. Dashboard
verification here is a component/API check, not a new browser/screenshot check.

## Tests, build and preservation

- Before the study: **57 passed, 27.74 s** — Recorder, ObserverTiming and
  observation-coverage suites. Includes diagnostics ON/OFF scientific-envelope
  equality and tests rejecting the historical 144 ms gap/incomplete windows.
- Essential metrics/execution/Runner/comparability/ranking/dashboard regression:
  **126 passed, 8 skipped, 45.31 s**. Skips required explicit retained Run paths.
- Those eight optional recorded-results tests were then supplied this new genuine
  completed IMM Run and an existing readiness-failed Run: **8 passed, 7.69 s**
  (15 unrelated tests deselected). No skips remain unverified in that selection.
- Total: **191 passing tests**. No red/green defect test or before/after fix
  claim: production behavior was not changed.
- `colcon build --packages-select robot_experiments robot_dashboard
  --symlink-install`: **2 packages succeeded, 11.0 s**.
- Full raw metric replay, independent finite-hold coverage arithmetic, header
  identity, record counts and metadata consistency all pass for both new Runs.
- SHA-256 manifest checks **1,467 pre-existing source, test, validation and
  historical evidence files**, including both prior study roots: **zero changed**.
  Scientific policy and frozen configuration remain intact. No observation was
  repaired, and no historical artifact was overwritten or removed.

Generated evidence includes `setup_study.py`, `preflight.json`, `catalog.yaml`,
the single `live/` study, `analyze.py`, `analysis.json`, `analysis.txt`,
`replay_overhead.py`, `replay_overhead.json`, `replay_overhead.txt`,
`focused_tests.txt`, `regression.txt`, `recorded_results_tests.txt`, `build.txt`,
`run.txt`, `preserved_files.json`, `preservation.json`, and the new ordinary
Session under `workspace/`. Raw telemetry and diagnostics remain inside their
original new Run directories; analysis output is separate.

## Reproduction

Analyze the existing evidence without launching or modifying any Run:

```bash
cd /home/user/ros2_ws
source install/setup.bash
export PYTHONPATH="src/robot_dashboard:src/robot_experiments:$PYTHONPATH"
build/dashboard_venv/bin/python results/recorder_diagnostics_d29b/analyze.py
build/dashboard_venv/bin/python results/recorder_diagnostics_d29b/replay_overhead.py
build/dashboard_venv/bin/python -m pytest \
  src/robot_experiments/test/test_observer_timing.py \
  src/robot_experiments/test/test_recording.py \
  src/robot_experiments/test/test_observation_coverage.py \
  src/robot_experiments/test/test_metrics.py \
  src/robot_experiments/test/test_execution.py \
  src/robot_experiments/test/test_runner.py \
  src/robot_experiments/test/test_scientific_comparison.py \
  src/robot_experiments/test/test_condition_ranking.py -q
export ROBOT_DASHBOARD_RECORDED_RUN="$PWD/results/recorder_diagnostics_d29b/live/nominal_20261102T143918_895978Z_ol7_mqis/runs/imm_seed42"
export ROBOT_DASHBOARD_FAILED_RUN="$PWD/results/scientific_comparison/workspace/runs/nominal_20261102T111309_465057Z_mk0nn8f3/runs/imm_seed42"
build/dashboard_venv/bin/python -m pytest src/robot_dashboard/test/test_results.py -q
colcon build --packages-select robot_experiments robot_dashboard --symlink-install
```

Exact one-time real study command already executed (not a request to rerun):

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
timeout --signal=INT --kill-after=20s 360s \
  ros2 run robot_experiments run \
  --catalog "$PWD/results/recorder_diagnostics_d29b/catalog.yaml" \
  --experiment EXP-CONTROLLED-PREDICTION --revision 1 \
  --output "$PWD/results/recorder_diagnostics_d29b/live" \
  --observer-diagnostics
```

Exit **3** is the authoritative conditional-acceptance code, because actuation
demonstration is intentionally unavailable; both execution statuses are Completed.
The outer bound did not fire. An additional invocation would create a new study
and exceed this task's one-pair allowance.

## Exact repository change and verdicts

Only new permanent file: **`docs/validation/recorder_diagnostics_d29b.md`**.
No production/test/configuration file was changed or removed during D2.9b.
Generated evidence is isolated under `results/recorder_diagnostics_d29b/`.
All pre-existing working-tree changes are preserved. No commit/push; D3, MPC and
Dynamics Editor were not started.

| Acceptance area | Verdict | Scope and remaining limitation |
| --- | --- | --- |
| Recorder loss diagnosis | **PARTIAL** | Independent pre-scoring losses and service interruption retained; exact transport/history/dispatch/scheduling subcause unresolved. Historical scored-window deficit NOT REPRODUCED, not fixed. |
| Observation reliability | **CONDITIONAL** | Both instrumented scored windows pass unchanged coverage; startup loss and uninstrumented historical failure remain. |
| Scientific comparability | **PASS** | This pair meets the existing frozen-condition, matched-Seed, coverage and recorded safety policy. One non-actuated Seed is not a superiority or hardware-safety claim. |
| Dashboard ranking | **PASS** | Genuine eligible ranks rendered for the new pair; genuine incomplete historical pair remains unranked. No policy relaxation or fake values. |
