# R1C — Runtime validation after the executor fix

2026-10-25. **Status: CONDITIONAL (Case 2).** All four variants reached valid
readiness and completed their requested windows with diagnostics off. CV and
the sensor baseline passed acceptance. IMM and higher noise retain incomplete
ROS joint coverage and are **not accepted benchmarking evidence**. Both study
invocations returned conditional exit code **3**; neither was repeated.

**Next action: proceed to S3B2 — Manipulation A–E configuration migration with
a CONDITIONAL, development-only GO.** The incomplete IMM/CV and sensor-noise
datasets remain blocked for fully accepted comparative benchmarking. There is
no unconditional reliability GO and no new demonstrated P0 measurement/acceptance
defect. Configuration work can proceed without drawing conclusions from these
incomplete datasets. S3B2 was not implemented in this task.

## Frozen runtime and preflight

Git HEAD: `3efe3205bdc0857cabd6ced8ac84726d4d560536` (existing dirty working tree).
Source SHA-256, identical in preflight and all four runtime metadata files:
`e28c208ff35fa99b30e73e5a1f19b5a7bbaefcb45bd8cdc82c18ba7726ea1299`.

Evidence is confined to `results/runtime_reliability_r1c/` (**E** below).
`E/preflight.json` verifies imported recorder/store/collector/runner/execution/CLI
files byte-for-byte against source. Imported Recorder bytecode references the
instance-owned `TimeoutObject` correction. Both installed default arguments
disable diagnostics; no run has a diagnostics artifact or enabled metadata.
The installed package was current, so no build or copied installation was needed.

Both installed `--validate-only` and `--explain` commands passed. Existing M07/M08
preparation declared both families configuration-compatible, with runtime
eligibility correctly unknown before execution. Original seed **42** is used
throughout: target measurement RNG 42, and the sensor family's existing derived
localization seed **3872720740** in both variants. Runtime seed consumers match
preflight. Noise assignments remain `[.04,.04,.03490658503988659]` versus
`[.08,.08,.06981317007977318]`; no covariance/algorithm changes were made.

The unchanged paired study uses 3 s settling, 6 s scoring and **80 ms**
readiness/alignment/joint hold. The sensor study uses 3 s settling, 8 s scoring
and **150 ms**. Both retain 45/12/150/5 s readiness/stall/wall/shutdown deadlines,
seed policy, actuation and original comparison rules. Native physics traces
are enabled only by the sensor study's existing configuration. No conflicting
simulation was found; no unrelated process was stopped. Studies ran sequentially.

## Actual outcomes

Study directories under E:

- `paired_prediction/paired_prediction_20261025T123421_197550Z_9ne0c5ie/`
- `sensor_noise/sensor_noise_comparison_20261025T123557_383807Z_0lbzwj39/`

Primary evidence: each run's `status.json`, `metadata.json`, `metrics.json`,
`resolved_configuration.json`, raw `telemetry/*.jsonl`, launch/host logs and,
where configured, `physics/actuator.csv`. Derived arithmetic checks are in
`E/audit.py`, `measurements.json` and `analysis.log`.

| Run | Readiness origin, sim s | Evaluation window, sim s | Best recorded worst absolute age | Fresh required-callback snapshots* | Acceptance |
|---|---:|---|---:|---:|---|
| IMM | 1.476 | 4.476–10.476 | 52 ms | 36 / 5,950 | unknown: joint coverage incomplete |
| CV | 14.166 | 17.166–23.166 | 38 ms | 23 / 13,695 | **True** |
| Sensor baseline | 3.660 | 6.660–14.660 | 100 ms | 317 / 5,990 | **True** |
| Sensor higher noise | 3.318 | 6.318–14.318 | 106 ms | 371 / 5,846 | unknown: joint coverage incomplete |

*All eleven required streams were present at each declared origin and active in
every evaluation window. Callback snapshots are an optimistic superset of actual
end-of-batch readiness checks; an earlier passing callback is not the actual
readiness declaration. For example, CV has a passing callback at clock 2.728 s
but its recorded origin is 14.166 s. Best ages above use the whole recording;
they are not substituted for the declared origin.

At recorded origins, independently observed fresh snapshots have worst absolute
ages **74 / 44 / 146 / 136 ms**, respectively, within the literal original
`abs(observer_clock-source_stamp) <= budget` inequality without added epsilon.
The actual check instant is not separately timestamped by standard telemetry.
Readiness simulation times are the recorded origins. Exact **host** time to
readiness is unavailable without adding instrumentation: callback-at-origin
offsets from the first recorded host-monitor tick are **6.314 / 21.937 / 9.564 /
8.301 s**, respectively, and are labeled proxies, not exact startup latencies.
Recorded total run-window monotonic elapsed times are **18.873 / 32.763 /
23.585 / 23.621 s**.

Actual startup identities match the materialized selection for all ten checked
roles. IMM uses `robot_prediction.baseline.imm.IMMPredictor`, `ResidualAdapter`
and `WeightedCorrection`; CV uses
`robot_prediction.baseline.constant_velocity.ConstantVelocityPredictor`, with
adaptation/fusion `NoneType`. Both sensor variants retain IMM/residual/weighted
correction. Every run uses the expected `WeightedLeastSquaresController`,
`SafeControlPipeline`, robot/target estimators, whole-body model, Fight/Flight
dynamics and RK4 integrator. Full qualified identities are in measurements.

### Joint observations and physical evidence

| Run | Joint records, whole recording / in window | Valid safety samples** | Max scoring joint gap | Covered / requested s | Missing s |
|---|---:|---:|---:|---:|---:|
| IMM | 2,675 / 1,286 | 1,287 | **86 ms** | **5.994 / 6** | **.006** |
| CV | 4,978 / 1,068 | 1,069 | 44 ms | **6 / 6** | 0 |
| Sensor baseline | 3,168 / 1,363 | 1,364 | 52 ms | **8 / 8** | 0 |
| Sensor higher noise | 2,759 / 1,203 | 1,204 | **238 ms** | **7.912 / 8** | **.088** |

**Safety includes the final pre-window sample only when its bounded right hold
overlaps the window. This accounts for the extra sample, not a duplicated
physical observation. Position and velocity coverage agree.

IMM's source-header gap **5.608–5.694 s** exceeds its 80 ms hold by **6 ms**,
explaining the entire missing coverage (99.9% covered). Higher noise's gap
**10.522–10.760 s** exceeds its 150 ms hold by **88 ms**, explaining the entire
missing coverage (98.9% covered). No interpolation, tolerance relaxation or
native-trace substitution was used. The reported null acceptance is retained.

There are zero observed position/speed violations above the unchanged
**.001 rad / .001 rad/s** tolerances in all scoring windows. Maximum ROS-observed
speed is .638 rad/s in the paired family and .698 rad/s in the sensor family,
below the corresponding description limits. This is sampled evidence; incomplete
coverage is not a continuous-safety PASS. All four windows report zero invalid
control commands, watchdog events, rejected commands and feedback rejections.

The two sensor native traces independently pass the existing `analyze_trace`
check. They contain **7,498 / 7,319 samples per joint**, continuous 2 ms grids
from .002 to **14.996 / 14.638 s**, respectively, covering both entire evaluation
windows. Startup shoulder undershoot is **6.4715e−5 rad** and maximum native
speed excess **5.0096e−10 rad/s** on the elbow, within existing numerical
tolerances. Native motor torque remains unavailable, so no measured effort or
hardware-safety claim is made. These complete physics traces do not make the
missing higher-noise ROS observations complete. Paired native traces were not
requested and are unavailable.

### Comparison policy versus acceptance

The paired study's **historical strict policy** reports `comparable:true` and
`comparison.eligible:true`: both executions completed with matching contracts.
That policy intentionally does not add the sensor policy's metric/coverage
evidence requirements. This distinction already exists in the
[M08 documentation](../m08_experiments_and_benchmarking.md), historical S3B1
report and current comparison code. It is not silently reinterpreted or changed
here. IMM still has `acceptance:null`, `coverage_complete:false`,
`unconditional_winner_eligible:false`, and the CLI returned 3. **Do not treat the
legacy comparison flag alone as full physical/scientific acceptance.** CV's own
acceptance is true; this does not turn IMM into an accepted comparator.

The declared sensor policy explicitly rejects the pair with:
`higher_noise seed 42: required physical joint observation coverage incomplete`.
Its `comparison.eligible:false` and both winner flags are false. Baseline remains
individually accepted. Comparison reasons were recomputed using the unchanged
policy and match the stored summaries exactly. No winner, noise-effect claim
or fully accepted IMM/CV ranking is inferred from these datasets.

### Runtime performance, descriptive only

| Run | Window RTF | Control compute mean / max ms | Distance error mean / max m | Pointing error mean / max deg |
|---|---:|---:|---:|---:|
| IMM | .77952 | 26.546 / 49.517 | .111779 / .313334 | 8.0975 / 16.7880 |
| CV | .84053 | 25.938 / 46.351 | .130998 / .238340 | 11.5486 / 15.8511 |
| Sensor baseline | .77728 | 31.170 / 59.629 | .149842 / .371015 | 8.4917 / 20.5741 |
| Sensor higher noise | .71237 | 34.713 / 84.669 | .137726 / .341296 | 7.5608 / 19.6942 |

These are per-run outputs, including explicitly unaccepted runs. They do not
establish a comparative algorithm effect, instrumentation overhead or statistical
significance. Reactive trajectories, startup and scheduling differ between runs.

## Historical contrast and remaining risks

| Indicator | Historical uninstrumented evidence | R1C |
|---|---|---|
| IMM readiness | S3B1 timeout; 0/11,400 fresh snapshots | Completed; 36/5,950; origin 1.476 s |
| IMM best worst age | 100 ms | 52 ms |
| CV readiness | S3B1 completed; 9/10,765 fresh snapshots | Completed; 23/13,695; origin 14.166 s |
| CV joint coverage / max gap | 6/6 s, 60 ms | 6/6 s, 44 ms |
| Sensor baseline joint coverage / max gap | S3A 8/8 s, 52 ms | 8/8 s, 52 ms |
| Higher-noise joint gap | S2 252 ms; S3A 158 ms | 238 ms |
| Higher-noise joint coverage | S2 7.898/8 s; S3A 7.992/8 s | 7.912/8 s |
| Fully accepted sensor comparison | Withheld | Still withheld |

R1B1's instrumented timeouts are diagnostic history, not unbiased pre-fix
controls. R1B2A's deterministic red/green and real transport fixture establish
the local fairness correction; these different Gazebo trajectories demonstrate
integration outcomes, not exact attribution of every earlier failure.

- **P0:** no new incorrect timestamp, readiness inequality, coverage arithmetic,
  unsafe accepted run or uncontrolled process was demonstrated. The documented
  legacy comparison flag still requires separate acceptance/coverage checks.
- **P1:** two of four post-fix windows lack complete ROS joint coverage; neither
  core comparative dataset establishes full acceptance. Recurring coverage gaps
  are material and remain a benchmarking blocker for these data. One post-fix
  trial per family does not establish a universal/systematic failure frequency
  or isolate bridge, DDS, persistence or host scheduling as the cause.
- **P1:** readiness opportunities remain sparse, especially CV. Mean source
  ages relative to observer clock remain negative (target truth about −145 to
  −164 ms; base truth −167 to −185 ms). An epoch ahead of the recorder's last
  observed clock is not proof of physical future-dating. Raw stamps stay intact.
- **P1/P2 deferred:** diagnostic perturbation uncertainty and unavailable exact
  dispatch/loss attribution. No diagnostic rerun or profiling expansion was
  necessary to identify the actual coverage rejection.

This is successful bounded execution validation with a **conditional acceptance
outcome**, not a claim of universal runtime reliability or unrestricted benchmark
readiness. Structural S3B2 work need not consume these incomplete comparisons.

## Independent checks, preservation and commands

`E/audit.py` verifies every saved source stamp directly against the raw ROS
header/clock, and every saved observer clock against callback-order clock replay:
**zero mismatches** in all four runs. It checks all required streams at the
recorded origins, exact origin+settling+duration windows, stop clocks, source
identity and seeds, and stream counts against metadata. A raw-joint interval
union independently reproduces coverage, missing duration, gap, overshoot and
violation counts. RTF is independently reconstructed from source/monotonic deltas.
Original acceptance gates and comparison reasons agree with saved results.

**265 relevant tests passed in 53.70 s** (`E/focused_tests.log`): existing M07/M08
composition/assignment/preparation, recorder/diagnostic/fairness, execution/readiness,
metrics/coverage and scientific comparison tests. Installed CLI and both
explain/validate commands passed. Source was unchanged; no full 1,500+ regression,
rebuild, copied install or new unit tests were needed.

All four launches stopped via SIGINT, returned zero from cleanup and have empty
survivor lists. Independent host environment scanning found zero processes in
their exact ROS domains **58, 80, 175, 76** and unique Gazebo partitions
(`E/final_process_check.json`). Protected system environments that could not be
read are listed there; no unrelated process was signaled.

`E/baseline.json` / `preservation.json` verify **1,669 initial files unchanged**,
including all source, experiments, timing policies, safety configuration and
unrelated edits; **5,879 historical artifact size/mtime snapshots unchanged**;
HEAD/index unchanged. Ignored history is stat-audited, not independently rehashed
in full. No historical result was overwritten. The **only nonignored file
added** is `docs/validation/runtime_reliability_r1c_closure.md`; zero existing
files modified. All generated study inputs/telemetry/helpers remain inside E.
No staging, commit or push occurred.

Exact study commands, from `/home/user/ros2_ws`:

```bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_LOG_DIR=/home/user/ros2_ws/results/runtime_reliability_r1c/ros
ros2 run robot_experiments run --config config/paired_prediction_composition.yaml \
  --output results/runtime_reliability_r1c/paired_prediction
ros2 run robot_experiments run --config config/sensor_noise_composition.yaml \
  --output results/runtime_reliability_r1c/sensor_noise
```

Both omit `--observer-diagnostics`. The runner reserves new unique study paths;
these commands describe reproduction, not an instruction to retry for PASS.
The actual invocations, exit codes and stdout are preserved in E's
`paired_runtime.log`, `sensor_runtime.log` and corresponding exit-code files.

Checks actually executed:

```bash
ros2 run robot_experiments run --config config/paired_prediction_composition.yaml --validate-only
ros2 run robot_experiments run --config config/sensor_noise_composition.yaml --validate-only
ros2 run robot_experiments run --config config/paired_prediction_composition.yaml --explain
ros2 run robot_experiments run --config config/sensor_noise_composition.yaml --explain
python3 -m pytest src/robot_experiments/test/test_recording.py \
  src/robot_experiments/test/test_observer_timing.py src/robot_experiments/test/test_execution.py \
  src/robot_experiments/test/test_metrics.py src/robot_experiments/test/test_scientific_comparison.py \
  src/robot_experiments/test/test_nominal_composition.py src/robot_experiments/test/test_explicit_composition.py \
  src/robot_experiments/test/test_sensor_assignments.py src/robot_bringup/test/test_nominal_composition.py \
  src/robot_bringup/test/test_explicit_composition.py -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros -p no:cacheprovider
python3 results/runtime_reliability_r1c/audit.py measure
python3 results/runtime_reliability_r1c/audit.py preserve
```
