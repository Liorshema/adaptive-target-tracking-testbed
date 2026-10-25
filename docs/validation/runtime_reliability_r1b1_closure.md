# R1B1 — Observer instrumentation and causal verification

2026-10-25. **Instrumentation: CONDITIONAL. Root-cause verdict: PROBABLE CAUSE.**
Default-off instrumentation works, preserves scientific envelopes and exposes
distinct callback, executor and between-batch delays. Matched replay processing
overhead is **27.51% median**, above the 5% engineering budget. All instrumented
live evidence below is **potentially perturbed, diagnosis only**. Neither a
transparent performance baseline nor a reliability correction is demonstrated.
R1B2 and S3B2 have not been implemented.

The single authorized IMM/CV study finished with **two readiness timeouts** under
the original 80 ms gate. It is scientifically incomparable; tracking metrics,
physical safety coverage and window RTF are unavailable. Failed results remain
failed. No retry or optional sensor study was run.

## Evidence and provenance

All paths are relative to `/home/user/ros2_ws`.

- Validation: `results/runtime_reliability_r1b1_validation/` (denoted **V** below).
- Live study: `results/runtime_reliability_r1b1/paired_prediction_20261025T113837_204754Z_bjg9267l/`
  (denoted **S**); individual evidence is under `S/runs/{imm,cv}_seed42/`.
- Primary evidence: each run's `status.json`, `metadata.json`,
  `resolved_configuration.json`, original `telemetry/*.jsonl`,
  `diagnostics/observer_timing.json`, `launch.log`, `host_performance.json`;
  study `summary.json` preserves incomparability.
- Derived read-only audit: `V/analyze.py`, `V/analysis.json`, `V/analysis.log`.
  It checks diagnostic/scientific callback counts, reconstructs the literal
  readiness inequality and independently parses actual startup identities.
  It does not replace the scientific evaluator or mutate results.
- Source SHA-256 recorded by both runs and preflight:
  `964dc3635499a5c8fa1f7c2956af94f349ae599326d8a4a9c3de8439ac856c68`.
  Git HEAD remains `3efe3205bdc0857cabd6ced8ac84726d4d560536`.
- Historical comparison: [R1A diagnosis](runtime_reliability_r1a_diagnosis.md),
  S2/S3A/S3B1 closure reports and their preserved artifacts. Historical and new
  trajectories are different runs; differences are not instrumentation effects
  measured on equivalent live work.

## Ownership, opt-in and preservation contract

The existing M08 runner constructs one `ObserverTiming` owned by the existing
recorder, only with CLI `--observer-diagnostics`. No experiment YAML flag,
second runner, scientific observation store, clock or database was introduced.
The option is passed outside experiment materialization/comparison contracts.
`--explain` and `--validate-only` acquire neither ROS runtime nor diagnostics.
Installed explain output is byte-identical with and without the flag.

Disabled mode retains message-only callbacks and the same single scientific
normalize/serialize/line-buffered-write path. Enabled callbacks require exactly
two arguments, selecting Jazzy's supported WithMessageInfo callback binding.
Both paths serialize each envelope once. Explicit identical arrivals produce
byte-identical JSONL, including raw payloads and source headers. Live arrival
times naturally vary with instrumentation; byte equality does not imply live
timing noninterference.

Subscription order, topic selection, BEST_EFFORT/VOLATILE depth 100, executor
ownership and waitables are unchanged. The initial `.02` spin followed by `0.0`
drain calls, original deadline placement, 127 additional calls/128 total calls,
and 10 ms drain guard remain unchanged. A callback already running can exceed
that guard; it is not a preemption deadline. Diagnostics distinguish callback
count, spin-call count, elapsed drain budget and an actual budget-check stop.
No private executor patch or scheduling correction was added.

No normalization, readiness, coverage, metrics, comparison policy, source YAML,
algorithm, physics, publication rate, PID or physical safety limit changed.
The independent resource sampler runs at 2 Hz without ROS callbacks. Scientific
failure wins over secondary collector/flush failures; a diagnostic failure
cannot silently produce eligible scientific success. Shutdown attempts an atomic
summary replacement; failed/interrupted runs remain `complete:false`, with the
scientific error and collector failure separately represented.

## Diagnostic schema version 1

Artifact: `<run>/diagnostics/observer_timing.json`. Deterministic structure;
runtime values are not deterministic. Observed sizes were 415,072 bytes (IMM)
and 402,001 bytes (CV). Output size is bounded with respect to message count for
the fixed configured stream set.

| Section | Contents and units |
|---|---|
| `schema_version`, `enabled`, `complete`, `failure`, `run_error` | Version, activation and explicit partial/failure state |
| `instrumentation` | Host-monotonic duration clock, simulation source clock, histogram bounds and unavailable exact dispatch timing |
| `bounds` | Capacity 128 per ring, callback order up to 128 per batch, at most four tracked publishers per stream |
| `executor` | Exact batch count/duration summaries, between-batch summaries, stop reasons, timeout switches, spin duration by actual timeout, recent/long batch rings, per-batch order/count/service ages; durations in seconds |
| `callbacks.streams` | Exact per-stream count, last entry/source/observer times, nonincreasing source count, entry-gap and stage summaries |
| `callbacks.long_operations` | Ring of operations at least 20 ms or errored; stage marks, stream and original source/observer epochs; no payload copies |
| `service_gaps` | Shared bounded ring of entry gaps at least 80 ms and between-batch gaps at least 20 ms |
| `observer_clock.streams` | Signed age `observer_clock-source_time` in simulation seconds and source-ahead counts |
| `middleware` | Feature availability counts, raw metadata samples, bounded per-publisher sequence summaries/events; missing information explicitly unavailable |
| `resources` | Bounded 2 Hz samples, CPU% of one logical CPU, RSS bytes, process IO counters, host load, leader schedstat and cgroup counters, explicit missing off-CPU trace |
| `overhead` | States measurement is external; does not pretend to measure its own perturbation |

Summary count/min/mean/max cover all accumulated observations; fixed histograms
have upper bounds 10/50/100 microseconds, .5/1/5/10/20/50/80/150/200/300 ms,
1/10 s and an overflow bin. The duration bands are diagnostic classifications,
not acceptance tolerances. Resource summaries use their containing field's units;
duration histogram bands are not useful CPU/RSS percentiles. Signed-age min/mean/
max are exact; negative ages share the first histogram bin, so no signed-age
quantile is inferred.

Stage marks are entry, normalization start/end, serialization start/end, write
start/end and exit. `total` ends **before diagnostic aggregation**; overhead
benchmarking includes aggregation. `other` is total minus measured normalization,
serialization and write durations and includes `pre_normalization`; those two
must not be added again. Synchronous write timing includes the original
line-buffered write/flush behavior, not an fsync guarantee. Public spin timing
includes waiting, dispatch and callback work; it is not a pure dispatch latency.
Generator restarts are null because no private hook was installed.

Rings retain recent events, not necessarily the largest events forever. All-run
extrema/counts survive eviction; exact early event correlation may not. This
matters for the CV gap below. Startup/preflight and shutdown resource samples
can also be present; they are not evaluation-window performance samples.

## ROS/RMW capabilities

Installed ROS Jazzy `rclpy` 7.1.11 and default `rmw_fastrtps_cpp` were verified
locally (`V/api_capabilities.json`, R1A dependency versions and the retained
executor probe dependency hashes). `ros_gz_bridge` is 1.0.22. The supported
two-argument callback supplies middleware metadata without another subscription.

In **both actual runs**, source timestamp, received timestamp and publication
sequence number were usable for every observed callback. Publisher GID and
reception sequence number were unavailable for every observed callback. Metadata
timestamps remain **raw middleware nanoseconds**; they are not converted to
simulation or host-monotonic time. Callback entry is never called reception time.

Consequently, per-publisher gap accounting cannot operate in these runs. Its
numeric zero is **not evidence of zero lost samples**: no publisher identities
were tracked. Publication sequences in sparse samples cannot locate a loss or
prove publisher continuity without identity. Source-header gaps are not DDS
sequence losses. Message-lost callbacks are supported by the installed API but
were deliberately not attached because they add executor event waitables;
`message_lost_events.value` is null with that reason. Exact delivery/drop location
remains unavailable. Producer-side publication/bridge evidence and matching
publisher identity would be needed for that attribution.

## Tests, build and installed checks

| Evidence in V | Actual result |
|---|---|
| `baseline_tests.log` | 103 relevant pre-change tests passed |
| `red.log`, `green_initial.log` | Missing collector first reproduced; initial focused green 30 passed |
| `m08_tests.log` | 296 M08 tests passed before final review cases |
| `review_red.log`, `review_green.log`, `review.md` | Four failing cases reproduced flush masking, environment restore and budget flag issues; fixes verified with 38 passing tests; independent re-review found no remaining implementation blocker |
| `regression.log` | **1,566 functional tests passed**, 18 style tests deselected, 140.65 s on the frozen runtime source |
| `final_focused.log` | Fresh completion check: **38 passed**, 7.03 s; includes all 18 instrumentation cases plus recorder/runner tests |
| `style_final.log`, `docstyle.log` | Clean flake8/pydocstyle for collector, recording ROS/store and new tests |
| `cli_runner_style_summary.json` | Broad CLI/runner style remains nonclean: 558 findings in task-entry copies, 612 currently. Net +54 formatting findings in the existing compact style; not all are historical. No added F401/import/docstring count increase. No claim of repository-wide lint cleanliness |
| `build.log` | Affected package built: 1 package, 9.07 s |
| `copied_build.log`, `copied_checks.json` | Independent nonsymlink package install built, 8.25 s; installed collector resolved to new copied prefix from `/tmp`; no-byte-compilation warnings only |
| `explain_disabled.json`, `explain_enabled.json` | Installed CLI outputs byte-identical; both variants present |
| `legacy_verification.json`, `preparation_compatibility.log` | Six original entries / 19 expanded materializations preserve original full prepared plans and generated inputs at the same fixed functional-audit source label |

Tests cover stage accuracy with controlled slow operations, exact envelopes,
missing metadata, bounded rings, no per-message diagnostic writes, enabled QoS
and callback signature, untouched timeouts/order/deadlines, count versus time
boundary reporting, writer/collector/flush faults, partial shutdown, interruption
and environment restoration. Scientific readiness/coverage/comparison regression
tests pass without policy changes. The copied-install check uses the previous
isolated dependency underlay with a newly copied `robot_experiments` overlay;
it does not claim all packages were rebuilt in a pristine system.

## Overhead: engineering budget not met

`V/overhead.py` replays the same 12,000 historical raw messages/explicit arrival
times through the existing `Recorder.spin_once(.02)` loop with a scripted FIFO
executor and real conversion/normalization/serialization/synchronous write.
Enabled mode includes collection and its independent resource sampler. A warmup
pair is excluded; six matched pairs alternate order with no concurrent ROS,
build or test workloads. Every off/on scientific output digest is identical.

`V/overhead.json` reports wall-processing increments **21.79, 23.45, 27.73,
32.51, 27.30 and 39.15%**, median **27.51%**. CPU median increment is **27.72%**.
This fails the <5% aim. Dictionaries omit real ROS-object reflection and actual
DDS/MessageInfo dispatch; shutdown/final-artifact writing is excluded. Therefore
this is a matched processing benchmark, not an exact live-overhead bound or a
live RTF comparison. The earlier noisy store-only exploratory benchmark is
retained but excluded from acceptance evidence. No threshold was enlarged and
no favorable reactive trajectory was selected as an overhead control.

## Controlled unchanged IMM/CV runtime

The sole study used the original paired composition, seed 42, 3 s settling,
6 s evaluation duration, 80 ms readiness/alignment budget, 45 s readiness,
12 s stall, 150 s wall and 5 s shutdown deadlines. Both variants used the
established physical closed loop. Runtime command publication/actuation were
enabled in the actual materialized system; owner-default `publish_commands:false`
in the context is overridden by the effective system runtime. No actuator trace
or publication-rate change was introduced. Actual startup identities independently
match expected IMM/CV, estimator, WLS controller, safety, model and dynamics in
both logs; failed-run metadata's empty actual-identity field is not substituted
for that explicit post-run audit.

| Measurement | IMM | CV |
|---|---:|---:|
| Execution | readiness_timeout | readiness_timeout |
| Monotonic readiness elapsed s | 45.00045 | 45.01501 |
| Last observed clock s | 28.594 | 26.624 |
| Required-callback snapshots after all streams exist | 13,602 | 11,580 |
| Snapshots meeting all unchanged inequalities | **0** | **0** |
| Best worst absolute source age ms | 104 | 120 |
| Batches | 3,123 | 2,750 |
| Time-budget / no-callback stops | 1,972 / 1,151 | 2,064 / 686 |
| Count-budget stops | 0 | 0 |
| Mean / maximum batch ms | 13.952 / 120.615 | 15.513 / 65.693 |
| Maximum between-batch ms | 87.227 | 402.923 |
| Maximum public `.02` / `0.0` spin ms | 34.124 / 112.964 | 43.270 / 61.097 |
| Longest scientific callback-arrival silence ms | 114.394 | 411.886 |
| Evaluation window / coverage / safety / RTF | unavailable | unavailable |

Readiness replay uses exactly `abs(C-s_i) <= .08` with no epsilon or substitute
epoch. Callback snapshots are an optimistic superset of actual end-of-batch
checking opportunities: zero passing snapshots guarantees none were missed by
checking less frequently. At IMM's best snapshot C=19.054, target truth is
104 ms ahead, target measurement 84 ms ahead and base truth 96 ms ahead.
At CV's best C=16.980, base truth is 120 ms ahead and target truth 96 ms ahead.
All eleven required streams did publish; missing startup configuration is not
the explanation. Tracking error, physical-limit acceptance and performance
comparisons cannot be inferred from these failed windows.

### Servicing, stage timing and resources

All diagnostic callback counts match scientific record counts: **15,975 IMM**
and **13,966 CV**. Clock/joint/command/actuation counts are respectively
5,570/2,746/490/244 (IMM) and 4,996/2,363/355/190 (CV).
Both retained last-128 batch tails have **128/128 `clock, clock` prefixes**.
IMM's tail includes 34 command and 20 actuation callbacks; CV's has zero control,
command or actuation callbacks while serving 256 clocks, 115 joints, 62 target
truth and 14 prediction callbacks. At CV shutdown the last command/actuation
callback was about **10.66 host seconds** old while clock and joints were under
9 ms old. This directly demonstrates unequal stream service/activity; upstream
publication readiness and waitset readiness at each missed turn remain unmeasured.

| Stage, mean / max ms | IMM | CV |
|---|---:|---:|
| Pre-normalization | .0105 / 20.271 | .0112 / 6.319 |
| Normalization | .413 / 111.577 | .444 / 28.567 |
| Serialization | .236 / 12.324 | .261 / 37.462 |
| Synchronous write | .276 / 21.498 | .337 / 28.895 |
| Total before diagnostic aggregation | .951 / 112.379 | 1.073 / 37.849 |

IMM's longest raw arrival gap is monotonic **220.206044–220.320439**.
The preceding command callback ran **220.206012–220.318390**, with **111.577 ms
inside normalization**, .589 ms serialization and .164 ms write. Thus that stage
accounts for almost all this specific silence. These are wall durations, not
CPU times: OS descheduling during normalization is not excluded. Its enclosing
0.505 s resource interval contains 92 ms leader runqueue wait. It does not prove
normalization has a CPU algorithm defect or that IMM prediction caused it.

CV's longest arrival gap is **248.126528–248.538413** (411.886 ms). No measured
callback exceeds 37.849 ms anywhere in CV; no long batch exceeds 80 ms. The
all-run maximum between-batch gap is 402.923 ms. A host-performance tick starts
inside this silence at **248.128709**; a second 174.800 ms silence similarly
contains a tick at 269.429410. The existing runner performs descendant/resource
sampling synchronously between batches. Its individual call duration is not
instrumented. The coarse enclosing first-gap resource interval (1.129 s) has
**412.862 ms leader runqueue wait**. This supports a between-batch monitor/host
scheduling explanation, not a measured disk-write explanation. The shared
recent service-gap ring evicted the exact early between-batch event, so the
402.923 ms extremum cannot be assigned an exact timestamp from retained events.
Do not pretend this is a complete off-CPU trace or a proven psutil subcall fault.

Root CPU averages 69.24% of one CPU in both runs; maxima 101.6/101.8%.
Peak RSS is 187,338,752 / 203,247,616 bytes. Samples span preflight/startup and
runtime, so these are not accepted-window statistics. Between first/last samples
the leader accumulates **12.433 / 13.707 s runqueue wait** and **27.116 / 26.761 s
runtime** (IMM/CV). Cgroup throttling counters stay zero. This demonstrates
scheduling contention over the run, while exact per-stall blocked/descheduled
causes remain unknown. IO write-byte deltas are 25,042,944 / 21,446,656; volume
alone does not prove blocking storage. No measured write accounts for the
158–296 ms historical silences or the new 412 ms CV silence.

### Clock and observation backlog

Mean signed source ages for target truth are **−156.34 / −141.91 ms** (IMM/CV);
base truth **−181.79 / −178.44 ms**. IMM target truth is ahead of the last
observed clock on 1,712/1,726 callbacks; CV on 1,443/1,510. Clock callbacks have
zero signed age by construction, not proof of current global simulation time.
Prediction source ages reach **2.314 / 3.620 s**, control **3.902 / 4.548 s**;
individual source streams are also substantially stale, rather than only the
clock being delayed. No source nonincreasing discontinuities were counted.

Latest-epoch disparities, repeated early subscription service, later-stream
backlogs, the configured recorder clock depth 100 versus ordinary ROS TimeSource
depth 1, and R1A's independent producer witnesses support **observer clock queue
lag plus unequal stream servicing**. An epoch ahead of recorder clock is not
thereby physically future-dated. The exact clock queue residence time is not
measured; middleware timestamps were not improperly subtracted from monotonic
or simulation epochs.

CV's maximum observed joint-source gap is **7.102–7.476 s (374 ms)** with
447.631 ms between that pair's callbacks. Actuation references missing interior
joint epochs **7.196, 7.284, 7.392**, and estimator diagnostics **7.188, 7.388**.
Those observations existed at other ROS consumers. Some witness records are
themselves serviced late (observer clock about 9.46–9.52 s), demonstrating
unequal observer backlogs. IMM's maximum source gap is 94 ms with an interior
.824 s joint witness. Neither startup diagnostic gap is a declared coverage
failure: no evaluation window exists. No native actuator trace was requested,
and no DDS loss location or number is invented from these witnesses.

## Causal verdict and next engineering decision

| Incident class / actual evidence | Confidence | Smallest next action, separately authorized R1B2 only |
|---|---|---|
| Exact readiness rejection due to incompatible latest epochs | **IDENTIFIED** mathematical reason for this study's rejection | Preserve inequality; address servicing rather than widen the gate |
| Repeated early-stream prefix, 63.1/75.1% time-budget stops, stale later streams, installed synthetic reset reproducer | **PROBABLE** executor fairness/backlog contributor | Test one bounded fair-service candidate retaining iterator progress across timeout transitions, with an equivalent idle-wait policy and unchanged 128/10 ms caps; validate supported API feasibility before implementing |
| IMM 114 ms silence contains 112 ms command callback, mostly normalization | **IDENTIFIED stage location**; underlying CPU versus descheduling **INCONCLUSIVE** | Profile that stage on CPU/off CPU; do not rewrite persistence on this evidence |
| CV 412 ms silence, short callbacks/batches, large between-batch gap, host tick and scheduler wait | **PROBABLE** between-batch monitor/host scheduling contribution | Add only tick start/end duration and retain largest between-batch events; use a recorder-leader `perf sched` trace if permitted, before moving monitor work |
| Source epochs ahead of recorder clock, depth disparity and other-consumer witnesses | **PROBABLE** observer clock/backlog contribution | Check clock and source queue/service progression together; no restamping, extrapolation or replacement clock |
| Missing recorded joint epochs witnessed by other ROS consumers, no GID/reception sequence/lost events | Drop location **INCONCLUSIVE** | Obtain supported publisher identity and bridge/subscriber delivery evidence; no QoS change or loss-count inference from source gaps |
| Historical S2 252 ms / S3A 158 ms source gaps and 296/212 ms silences | Observer-path involvement remains **PROBABLE**, exact historical stage **INCONCLUSIVE** | Historical traces lack these boundaries; new events cannot retroactively prove their cause |

The retained fairness control (`V/executor_probe.py/json`) exercises the installed
`wait_for_ready_callbacks` with a synthetic always-ready generator. Switching
argument tuples repeats `clock, clock, target_truth, target_measurement, joints`;
constant arguments traverse all eleven control subscriptions. This verifies an
installed code behavior and reproduces the risk. It is not a live generator
restart counter or proof that a neglected subscription was ready at that moment.

There is **no single proven underlying cause** for all incident classes. The
next correction candidate is bounded observer servicing fairness, preceded by
the small between-batch discriminator above; unconditional persistence, DDS,
predictor or physics changes are not justified. Diagnostic granularity should
be reduced and overhead rebenchmarked before any claim of transparent observation.
R1B1 intentionally stops without implementing those follow-ups.

The optional sensor study was not run: this paired study already distinguishes
slow callbacks from outside-callback delays and reproduces joint observations
missing from this observer but present elsewhere. Another unchanged study
cannot recover evicted boundaries, unavailable publisher identity or historical
off-CPU scheduling. The missing discriminators are specific instrumentation,
not another attempt at a favorable acceptance outcome.

## Exact changed files and preservation

Relative to the task-entry dirty working tree, **five existing files changed**:

1. `src/robot_experiments/robot_experiments/recording/ros.py` — opt-in metadata,
   batch and public-spin timing around the existing loop.
2. `src/robot_experiments/robot_experiments/recording/storage.py` — shared single
   scientific write path with optional stage timing and fault preservation.
3. `src/robot_experiments/robot_experiments/runner.py` — explicit runtime option,
   collector ownership/finalization and environment restoration.
4. `src/robot_experiments/robot_experiments/cli.py` — default-off flag.
5. `docs/m08_experiments_and_benchmarking.md` — invocation and diagnostic contract.

**Four files added**:

6. `src/robot_experiments/robot_experiments/recording/observer_timing.py`.
7. `src/robot_experiments/test/test_observer_timing.py`.
8. `docs/validation/runtime_reliability_r1b1_plan.md`.
9. `docs/validation/runtime_reliability_r1b1_closure.md` (this report).

Ignored new evidence lives only under the two R1B1 result roots. No historical
experiment or configuration was replaced. `V/baseline.json` froze 1,664 initial
tracked/nonignored files and size/mtime for 5,501 historical artifacts in seven
roots. `V/preserve.py` / `V/preservation.json` verify only the files above differ,
all existing YAML unchanged, all historical recorded stats unchanged, runtime
source fingerprint unchanged, HEAD and index unchanged. This is a byte-hash
audit of initial source/nonignored files and a stat audit of ignored history,
not an independent hash of every historical payload.

Preservation incident disclosed: an old preparation helper was inadvertently
invoked once and re-emitted its deterministic 405-byte historical
`configuration_nominal_composition_s3b1_validation/legacy_verification.json`.
Contents were checked against the original helper schema/inputs and its original
recorded mtime restored. `V/helper_preservation_note.json` documents this; later
helper output was redirected into V. No historical experimental measurement
was overwritten. Do not rerun historical helpers directly.

Both owned launches stopped via SIGINT, returned zero from cleanup and reported
no survivors. Independent host environment scan (`V/final_process_check.json`)
found no process in domains **80/100** or partitions
`robot-experiments-0f4eb2b27c5c463b99ec851dfb8ece2e` /
`robot-experiments-20aa4656ca2c4b23abf34b25b836cc04`.
Some protected system process environments were unreadable and are listed;
no unrelated process was signaled. No staging, commit or push occurred.

## Reproduction and rollback

From `/home/user/ros2_ws`, with ROS/Gazebo permissions and no competing simulation:

```bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_LOG_DIR=/home/user/ros2_ws/results/runtime_reliability_r1b1_validation/ros
ros2 run robot_experiments run \
  --config config/paired_prediction_composition.yaml \
  --observer-diagnostics --output results/runtime_reliability_r1b1
```

This creates a unique study directory and may again exit 1 on readiness failure;
it is not a promised passing performance run. For normal recording, **omit
`--observer-diagnostics`**. No YAML rollback or historical data deletion is needed.
Do not reset the dirty working tree to disable diagnostics.

Reproduce relevant checks without relaunching Gazebo:

```bash
python3 -m pytest src/robot_experiments/test/test_observer_timing.py \
  src/robot_experiments/test/test_recording.py src/robot_experiments/test/test_runner.py \
  -q --import-mode=importlib -p no:launch_testing -p no:launch_ros -p no:cacheprovider
python3 -m pytest src -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros -p no:cacheprovider \
  -k 'not copyright and not flake8 and not pep257'
python3 -m flake8 --jobs 1 --max-line-length 99 \
  src/robot_experiments/robot_experiments/recording/observer_timing.py \
  src/robot_experiments/robot_experiments/recording/ros.py \
  src/robot_experiments/robot_experiments/recording/storage.py \
  src/robot_experiments/test/test_observer_timing.py
python3 -m pydocstyle \
  src/robot_experiments/robot_experiments/recording/observer_timing.py \
  src/robot_experiments/robot_experiments/recording/ros.py \
  src/robot_experiments/robot_experiments/recording/storage.py \
  src/robot_experiments/test/test_observer_timing.py
colcon build --packages-select robot_experiments --symlink-install
ros2 run robot_experiments run --help
ros2 run robot_experiments run --config config/paired_prediction_composition.yaml --explain
python3 results/runtime_reliability_r1b1_validation/analyze.py
python3 results/runtime_reliability_r1b1_validation/preserve.py
```

Copied-package verification used V/copied/{src,build,install,log}, sourcing
`results/configuration_nominal_composition_s3b1_validation/copied/install/setup.bash`
as its dependency underlay; from V/copied it ran
`colcon --log-base log build --base-paths src --build-base build --install-base install --packages-select robot_experiments`.
The source package was copied without symlinks, and module/CLI checks ran from
`/tmp` after sourcing V/copied/install/setup.bash. Detailed checks and outputs
are retained in V. The matched overhead replay is separately reproducible with
`python3 results/runtime_reliability_r1b1_validation/overhead.py`; preserve its
recorded evidence before deliberately rerunning that helper.
