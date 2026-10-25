# R1B2A — Minimal reliability remediation and diagnostic overhead

2026-10-25. **Verdict: CONDITIONAL.** A concrete P0 observer availability defect
was reproduced and corrected using the installed Jazzy public timeout object.
Avoidable diagnostic allocations were removed. The new matched overhead median
is **4.04%**, but its **−28.22% to +22.02%** variation does **not** establish a
reliable <5% bound. Diagnostics remain default-off; instrumented performance
remains potentially perturbed. No new Gazebo acceptance or historical-causation
claim is made.

**Recommendation: Proceed to R1C for validation of a demonstrated reliability fix.**
No R1C, R1B2B or S3B2 implementation was started. No staging, commit or push.

## Scope and evidence

This is a delta to [R1A](runtime_reliability_r1a_diagnosis.md) and
[R1B1](runtime_reliability_r1b1_closure.md), not a replacement for their reports.
All generated evidence is under **`results/runtime_reliability_r1b2a/`**, denoted
**E** below. No new production module, YAML, ROS node or launch file was added.
One closure document was added. `E/workplan.md` is the inline execution plan;
entry copies, benchmark/transport helpers, test logs and preservation checks
are one-time ignored evidence, not installed package utilities.

## Priority classification

| Finding | Measurement meaning / false acceptance | Availability and cause | Priority / disposition |
|---|---|---|---|
| Primitive timeout changes discard the current ready iterator repeatedly under short sustained batches | Does not restamp data or itself make invalid data eligible | Deterministic test and real ROS fixture demonstrate later ready subscriptions unserved; small public-API correction exists | **P0 availability: corrected** |
| Eager `setdefault(key, Summary())` constructs discarded histograms on every callback/spin | Does not change scientific measurements; explicit diagnostic mode only | Profile shows 84,013 constructions for 12,000 observations | **P1: minimal correction implemented** |
| Remaining diagnostic aggregation, metadata and timestamp cost; noisy benchmark timing | Transparent timing cannot be assumed; default-off is the reliable workaround | Collection cost is measured, but an exact live overhead bound is unavailable | **P1: characterized, deferred** |
| Baseline normalization and synchronous writes consume substantial recorder time | Existing payload conversion is required; no incorrect conversion or silent acceptance demonstrated | Profile establishes cost, not a standalone P0 normalization defect | **P1: no scientific serializer rewrite** |
| R1B1 between-batch monitor/host scheduling stalls | Historical gates rejected invalid windows | Exact offending subcall/off-CPU cause remains unmeasured | **P1 hypothesis: no speculative monitor redesign** |
| Missing GID/reception-sequence/loss attribution and larger-event retention detail | Limits diagnosis, not scientific data semantics | Known capability/retention limitations from R1B1 | **P2 for this remediation: deferred** |
| Existing unrelated formatting debt | No scientific effect | Unchanged by this task | **P2: preserved** |

No remaining **demonstrated, uncorrected P0** was found in this scope. Passing
local servicing checks does not prove the whole Gazebo observation pipeline is
now reliable; that is the purpose of the recommended R1C validation.

## Matched replay and cost attribution

`E/replay.py` reuses the R1B1 workload and method: the same first **12,000**
historical S3B1 IMM raw message dictionaries, sorted in recorded arrival order,
through the actual bounded `Recorder` loop and scientific normalize/serialize/
synchronous-write path. Explicit arrival epochs and payloads are identical.
The executor is a scripted FIFO; no DDS dispatch, ROS-object reflection or
actual MessageInfo cost is measured. Enabled mode includes the independent
resource sampler and diagnostic aggregation. Finalization is timed separately.

Task-entry modules were copied into `E/original/recording/` before edits and
loaded in an isolated Python package namespace. The original enabled implementation
therefore remains directly executable alongside the optimized implementation.
No historical helper was invoked to write into old evidence directories.

First, two modes were measured with an excluded warmup and six retained
alternating pairs (`before_replay.json`). Original off/on median processing
times were **1.36190 / 1.76198 s**. Median paired overhead was **29.69%**,
range **19.12–56.04%**. R1B1's earlier 27.51% remains historical evidence;
different host-time results are not relabeled as the same measurement.

Second, all three modes rotated their order over one warmup and six retained
rounds (`after_replay.json`). No tests, builds or ROS workload ran concurrently
with the matched measurements. The separate reviewer performed small read-only
source inspections. Every scientific-file digest is identical across **all
retained modes and rounds**; raw headers/payloads were not altered.

| Processing mode | Wall median s | Wall min–max s | CPU median s | Paired wall overhead vs off |
|---|---:|---:|---:|---:|
| Current diagnostics disabled | 2.28212 | 1.66134–3.87943 | 2.27752 | reference |
| Task-entry diagnostics enabled | 2.68291 | 2.10604–5.69058 | 2.68960 | median **36.85%**, −8.26–46.88% |
| Optimized diagnostics enabled | 2.25708 | 1.72956–2.78464 | 2.26122 | median **4.04%**, −28.22–22.02% |

The optimized enabled implementation was faster than original enabled in all
six matched rounds: median **17.48%** reduction, range **14.55–51.07%**.
CPU overhead medians were **36.95% original / 4.12% optimized**. The median of
paired ratios is not the ratio of separate absolute medians. Negative overhead
estimates demonstrate measurement variability, not free diagnostic work.
The wide ranges prevent a transparent-observation PASS or exact live-overhead
claim. This task stops rather than pursuing increasingly elaborate profiling.

`profile_replay.json`, `original_{off,on}_profile.txt`, `on_profile.txt` and
`profile_summary.json` separate profiled attribution from unprofiled overhead.
Illustrative original-enabled cProfile observations:

| Cost category | Observation | Interpretation |
|---|---|---|
| Monotonic reads | 121,579 calls, .028 s self time, versus 12,280 calls without diagnostics | Timestamp reads exist, but are not the dominant avoidable cost |
| Callback aggregation/bookkeeping | .221 s self / .641 s inclusive for 12,000 callbacks | Includes nested summary/metadata work; inclusive times must not be added together |
| Summary accumulation | 108,704 calls, .131 s self / .231 s inclusive | Histograms/extrema and per-stream summaries are necessary diagnostic work |
| Histogram construction | **84,013** calls, .053 s self / .067 s inclusive | Repeated default-expression construction is avoidable |
| Middleware extraction | .063 s self / .091 s inclusive | Replay has absent metadata; live cost may differ |
| Bounded event/ring maintenance | Included in callback self time and deque operations | No separate precise event-ring duration is claimed; no per-message diagnostic serialization exists |
| Resource monitoring | Separate 2 Hz thread; three-round ablation was noisy | No precise independent percentage or zero-cost claim; no lock/sampler redesign justified |
| Scientific normalization | 2.331 s inclusive off / 2.447 s inclusive on in original profiles | Baseline conversion cost is substantial, not instrumentation overhead itself |

Three-round cost ablations retained timestamps only, omitted callback aggregation,
omitted metadata or omitted resources in the **evidence script only**. They
support attribution but are noisy and overlapping; for example the no-metadata
variant was sometimes slower. They are not shipped configurations or independent
causal percentages. The profile counts, source inspection and allocation
regression provide the firm justification for the selected changes.

Shutdown was excluded from processing overhead, not hidden. In the final six
rounds, median close/finalize times were **22.04/21.08 ms original enabled** and
**18.15/17.90 ms optimized enabled**; disabled close was .057 ms. Close and final
runner finalization are separate writes in the existing shutdown contract.
No new asynchronous persistence or teardown redesign was introduced.

## Implemented changes

**Collector:** preallocate fixed per-stream stage summaries; lazily allocate
each executor-timeout summary once. Reuse a module-level tuple of stage
boundaries, accumulate measured operation duration directly, and compare extrema
directly instead of calling `min`/`max` on every summary update. The profiled hot
path's `Summary.__init__` count falls from **84,013 to 26** (stage preallocation
happens before processing timing; low-rate resource summaries still allocate).

No stage sampling, metadata sampling reduction, lost sequence accounting,
histogram removal or diagnostic schema change was needed. Empty preallocated
stages are omitted on emission as before. Bounded rings, all-message counters,
source discontinuity/age accounting and failure handling retain their meanings.
The scientific store and normalizer were not modified at all.

**Recorder P0 correction:** own one `rclpy.executors.TimeoutObject` per Recorder.
Before each existing public executor call, set `.timeout` to the original
requested duration and pass that same object. The installed Jazzy
`wait_for_ready_callbacks` caches its iterator by argument equality. Previously,
changing primitive `.02` to `0.0`, and back next batch, restarted iteration from
early subscriptions even when later subscriptions in the ready snapshot had
not been visited. Stable argument identity preserves that progress.

Initial requested wait and nonblocking drain durations are unchanged. The loop
still uses at most **128 public spin calls** and the same **10 ms drain guard**,
with unchanged deadline placement after the first call. No private executor
state, multithreading, new waitable, sleep loop, larger cap or QoS change is used.
The intentional disabled-mode change is callback service order/fairness; exact
timing/order equivalence to the defective implementation is not claimed.

A ready generator snapshots timeout when its wait set is constructed. Updating
the object's field does not retroactively alter its existing timer. In this
synchronous subscription path, resuming remaining ready callbacks does not
perform another blocking wait: `yielded_work` is already true. After exhaustion,
a fresh generator uses the current requested timeout. Exhaustion can cause one
early return; subsequent idle calls block normally. The idle transport check
below verifies the concrete installed behavior.

Compatibility is specifically verified against installed ROS Jazzy rclpy
7.1.11. `TimeoutObject` is a public class and the installed executor explicitly
handles it internally, although `SingleThreadedExecutor.spin_once` retains a
float-only annotation. No universal compatibility claim across rclpy releases
is made; the regression/transport checks are the guard for an upgrade.

## Red/green, transport, regression and build

`E/red.log` records **three expected failures** before production edits:

- Accumulating another observation/spin constructs a discarded histogram.
- Partial batches permanently neglect later ready streams, diagnostics off.
- The same fairness failure, diagnostics on.

The fairness regression invokes the **installed public iterator implementation**
with a deterministic continuously-ready generator and 4 ms logical callback
steps. Four bounded batches formerly visited only clock/target truth/target
measurement; the correction visits all eleven streams in order across batch
boundaries. It does not pretend the synthetic readiness generator is DDS.

`E/green.log`: **62 focused tests passed**, including the three new cases,
existing recorder/runner/readiness tests, exact source/raw-envelope preservation,
writer/collector faults, partial shutdown, timeout values and work/time caps.

A bounded actual ROS check (`E/transport_check.py/json/log`) then tested the
important remaining API assumption: real SingleThreadedExecutor and local DDS
must preserve service progress with this object and still wait when idle.
It uses real configured message types, eleven local topics with prepublished
queues, artificial 4 ms callback tails, and isolated leased domains. It launches
no child ROS process or Gazebo. These are fixtures, not an accepted experiment.

| First ten real ROS batches | Clock / target truth / target measurement | Remaining eight streams |
|---|---|---|
| Original recorder | 20 / 10 / 9 callbacks | **0 each** |
| Corrected, diagnostics off | 4 / 4 / 4 | each **3–4** |
| Corrected, diagnostics on | 4 / 4 / 4 | each **3–4** |

Both corrected modes visit all eleven streams in the first eleven callbacks.
Eight idle executor calls requesting 20 ms took **20.33–20.56 ms each**,
**163.42 ms total**, confirming waiting rather than idle busy-spinning. Real
fixture batch callback counts stayed at 2–4 with the artificial cost. Exact
128-call and 10 ms boundary behavior remains covered by existing unit tests.
The finite ROS fixture establishes the local servicing correction, not that
this mechanism caused every earlier 158–412 ms historical event.

Additional verification:

- **1,569 full functional tests passed**, 18 style tests deselected, **98.35 s**
  (`E/regression.log`). This includes affected-package metrics, comparison,
  readiness/coverage, configuration and architecture integration tests.
- Changed production code and `test_observer_timing.py`: clean scoped flake8
  and pydocstyle (`E/style.log`, `E/docstyle.log`). The extended helper module
  `test_recording.py` has the **same 38 pre-existing findings**, identical by
  code/message after line shifts (`recording_test_style_comparison.json`).
- Affected package symlink build: **1 package, 16.3 s**, success (`E/build.log`).
  Installed CLI help and module paths verified (`cli_help.txt`, `installed.json`).
- No copied build: no resource/entry-point/package/install behavior changed;
  no concrete copied-install risk justified repeating that expensive check.
- Independent read-only review found no material implementation issue; noted
  timeout annotation/version and synthetic-test scope (`E/review.md`).

No new Gazebo run was necessary to resolve this bounded public-API question.
Do not infer improved Gazebo readiness, tracking accuracy, physical safety,
window RTF or scientific eligibility from these tests. Those measurements
remain for the separately authorized R1C task.

## Scientific integrity, exact files and preservation

**Two production files modified; zero added:**

1. `src/robot_experiments/robot_experiments/recording/observer_timing.py`.
2. `src/robot_experiments/robot_experiments/recording/ros.py`.

**Two existing test files modified; zero added:**

3. `src/robot_experiments/test/test_observer_timing.py` — three red/green cases
   and adaptation to inspect the timeout's numeric value at call time.
4. `src/robot_experiments/test/test_recording.py` — transport fake supports the
   public timeout object's value; source-data/readiness expectations unchanged.

**One document added; zero existing documents modified:**

5. `docs/validation/runtime_reliability_r1b2a_closure.md`.

`E/baseline.json` froze **1,668 entry files**, HEAD/index and **5,804 historical
artifact stats**. `E/preserve.py`/`preservation.json` verify the exact five-file
delta, all other original source/configuration byte hashes intact, all historical
size/mtime snapshots intact and Git HEAD/index unchanged. No old results were
rewritten. Historical stat checks are not a new cryptographic payload audit.
Final source fingerprint:
`e28c208ff35fa99b30e73e5a1f19b5a7bbaefcb45bd8cdc82c18ba7726ea1299`.

Original **source timestamps, clock domains, raw payloads, physical joint
measurements, seeds, publication rates, QoS, normalization, metric definitions,
comparison contracts, readiness/coverage thresholds, algorithms, dynamics,
physics and safety limits remain intact**. Invalid or incomplete data are not
interpolated, restamped or made eligible. Default diagnostics remain disabled;
collector failures and incomplete-run status retain the R1B1 fault protections.

All fixture contexts/executors were closed before the helper exited. No child
launches or Gazebo processes were created. Independent environment inspection
found no process in the three fixture scopes (`E/final_process_check.json`);
protected system environments that could not be inspected are listed. No
unrelated process was stopped. Existing experimental/untracked work is preserved.

## Reproduction and stop decision

From `/home/user/ros2_ws`:

```bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_LOG_DIR=/home/user/ros2_ws/results/runtime_reliability_r1b2a/ros
python3 -m pytest src/robot_experiments/test/test_observer_timing.py \
  src/robot_experiments/test/test_recording.py src/robot_experiments/test/test_execution.py \
  src/robot_experiments/test/test_runner.py -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros -p no:cacheprovider
python3 -m pytest src -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros -p no:cacheprovider \
  -k 'not copyright and not flake8 and not pep257'
colcon build --packages-select robot_experiments --symlink-install
ros2 run robot_experiments run --help
python3 results/runtime_reliability_r1b2a/preserve.py
```

The matched replay helper supports `profile`, `before` and `after` arguments;
preserve its recorded outputs before intentionally remeasuring. The real
transport helper is a one-shot fixture using exclusive output files; do not
rerun it over the retained fixture evidence. Ordinary experiments continue to
omit `--observer-diagnostics`; no YAML toggle or rollback is required.

The proven P0 defect is corrected and locally verified. Remaining instrumentation
cost/variability is safely confined to opt-in diagnosis. Further optimization,
profiler expansion or speculative persistence/monitor changes would exceed this
task's value and complexity budget. Stop here; the single next action is **R1C
validation of the demonstrated reliability fix**, with original experiments,
policies and default-off diagnostics.
