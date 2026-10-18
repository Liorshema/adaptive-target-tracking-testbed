# M08 — Experiments and benchmarking

2026-10-18. M08 starts from committed M07 `b16b649` on
`feature/m08-experiments`. It introduces the tenth package, robot_experiments.
The existing nine packages retain their production behavior; only their
architecture tests and the shared tree renderer change. No commit or push.

## Start here

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robot_experiments
source install/setup.bash
# Preflight all variants before creating outputs or starting a process:
ros2 run robot_experiments run --config config/paired_prediction.yaml --validate-only
# One nominal physical experiment:
ros2 run robot_experiments run --config config/nominal.yaml
# A matched IMM/CV study, with seed 42 and explicit actuation in the owner YAML:
ros2 run robot_experiments run --config config/paired_prediction.yaml
```

These are installed package resources, resolved without assuming a source checkout.
An existing relative file resolves from the current working directory; otherwise
`--config` resolves inside robot_experiments' installed share. `--output` changes
only the parent output directory. Each invocation reserves a timestamped,
random-suffixed study directory; old results are never reused.

To extend the installed example without editing robotics Python:

```yaml
extends: /path/to/robot_experiments/config/paired_prediction.yaml
experiment:
  name: repeated_prediction
  seeds: [42, 43, 44]
  duration: 60.0
  settling_time: 5.0
```

The parent path can be the installed share file; discover it with
`ros2 pkg prefix --share robot_experiments`. Run the new YAML using `--config`.
For compute-only operation set `experiment.runtime.enable_actuation: false`.
The runner then does not wait for unpublished safe commands. Ground-truth
publication is enabled solely for observation; it is not fed into estimation or
control. Existing PID holding remains in robot_simulation.

## Responsibilities and tree

`configuration.py` validates experiments, calls M07 preflight and materializes
seeded inputs. `execution.py` owns isolation, processes and monotonic deadlines.
`recording/` separates pure normalization/storage from its lazy ROS adapter.
`metrics/` owns ROS-free numerical evaluation/alignment. `benchmarking.py` combines
comparable seeds. `reporting.py` owns atomic JSON and unique directories.
`runner.py` coordinates those responsibilities; `cli.py` is the installed adapter.
Small cohesive modules stay flat rather than gaining one-file directories.
The actual tree is [m08_final_tree.txt](validation/m08_final_tree.txt).

The runner invokes the existing `robot_bringup closed_loop.launch.py` with a
materialized M07 manifest. It neither recreates launch assembly nor imports a
validation script. FK reuses the existing robot description/model factory and
WholeBodyKinematics, following the validated physical-observer geometry. Original
validation scripts and historical raw results remain untouched. Lower-level
packages do not depend on robot_experiments. Numerical metrics import no ROS/Gazebo.

## Experiment schema, version 1

```yaml
schema_version: 1
experiment:
  name: paired_prediction
  description: Matched configuration and sensor seed, with reactive motion
  duration: 6.0                 # measured simulation seconds, after settling
  settling_time: 3.0
  seeds: [42]
  variants:
    - name: imm
      system_config: {package: robot_bringup, path: config/system_nominal.yaml}
    - name: cv
      system_config: {package: robot_bringup, path: config/system_cv.yaml}
  metrics: [tracking_distance, pointing_error, prediction_rmse, estimation_error,
            joint_speed_violation, joint_position_violation, realtime_factor,
            control_compute]
  output: {directory: results/experiments}
  runtime: {enable_actuation: true}
  execution:
    readiness_timeout: 45.0
    stall_timeout: 12.0
    wall_timeout: 150.0
    shutdown_timeout: 5.0
    max_skew: 0.08
    failure_policy: continue
```

Name and variant identifiers are safe, unique alphanumeric/underscore/hyphen
strings. Seeds are unique uint32 values. Duration/deadlines/skew must be positive
finite numbers; settling may be zero. Boolean values do not pass numeric checks.
Unknown keys, metrics, schema versions, missing resources and invalid M07
combinations fail before launch. Runtime defaults to disabled actuation.
`execution` has the defaults shown above; runtime and execution are optional.
`metrics` declares requested metric families; the stable result schema also emits
all available core metrics and mandatory physical safety diagnostics. Safety is
never disabled by excluding a family from this list.

Resources use the existing `{package, path}` or relative-file conventions. The
shared loader supports one relative `extends` parent. Nested mappings merge;
lists replace. Variant resources resolve relative to the document declaring them
before merging. Output-directory relative paths are interpreted from the caller's
working directory. M07 remains responsible for component compatibility and all
owner files retain their numerical parameters and physical limits.

## Time, seeds and reproducibility

Readiness requires all selected required streams with source epochs within
`max_skew` of the current observed clock. Measurement starts at
`readiness_clock + settling_time`, and ends after `duration`. Source metrics use
this simulation-time interval. Host monotonic time controls readiness, stalled
clock, total wall timeout and finite shutdown. UTC timestamps label files/status;
they never substitute for source time. A backward clock invalidates the run;
source observations arriving out of order remain recorded, sorted only for
bounded offline alignment. Paused or absent clocks produce explicit failures.

The deployed stochastic consumer is SyntheticMeasurementGenerator's NumPy RNG.
Each run writes `inputs/measurements.yaml` with the requested
`measurements.target_position.random_seed`, and wires it through an input
simulation/profile/M07 system manifest. The original owner YAML is unchanged.
Resolved owner contents, selected source hashes and inheritance parents, initial
conditions, world/SDF text, model settings, physical configuration, algorithm
identities, package versions, Git revision/status/diff hash and platform metadata
are captured. Merged estimator/control parameters preserve owner calibration.

This provides configuration replay and seed-controlled sensor noise. It does not
provide bitwise Gazebo replay: asynchronous callback order, contact/skid physics,
startup epochs and CPU scheduling differ. Reactive target dynamics use live robot
TF, so changing a controller can change target motion even under matched seeds.
Noise sequences can be seed-matched without being applied at identical physical
times. No new simulator stepping loop or Gazebo RNG guarantee is claimed.

## Telemetry and output contract

A study contains experiment.yaml, resolved_experiment.json, summary.json,
comparison.csv, and `runs/<variant>_seed<seed>/`. Every run has:

- inputs/ with materialized owner configuration and system.yaml;
- resolved_configuration.json, metadata.json and launch.log;
- telemetry/*.jsonl and quarantined telemetry/events.jsonl;
- metrics.json, time_series.json, events.json and status.json;
- host_performance.json and generated_world.sdf when observable.

All JSON contracts have `schema_version: 1`. Artifact links are relative to their
containing study/run. Config/source provenance may retain absolute original paths;
portable consumers use artifact links and captured content, not those paths.
Exact launch arguments are captured. Raw original messages include timestamps,
frame IDs, estimates/covariances, future trajectories, measured joints/base,
commands, actuator counters, computation telemetry and diagnostics.

Each telemetry record has `source_time` (seconds from ROS header),
`arrival_monotonic` (host seconds), observed `clock`, `frame_id`, normalized `data`,
original `raw_message`, and topic/message-type provenance. Unstamped diagnostic
strings explicitly have null source time. Nonfinite/malformed messages are
quarantined with tagged values and reasons; no metric silently replaces them with
zero. Availability lists counts, invalid/out-of-order records and last epochs for
all streams, including missing streams. JSON writes reject NaN and are atomic;
raw telemetry is append-only and partial artifacts survive handled failures.

## Mathematical contracts

For physical tracking pose `(p_c, Q_c)`, target truth `p_t`, configured radius R
and configured local forward axis f, the numerical core computes:

- distance error `abs(norm(p_t-p_c)-R)` in metres;
- pointing error `acos(clip((Q_c f)·(p_t-p_c)/(norm(f)*norm(p_t-p_c)),-1,1))`
  in radians; zero target separation makes pointing unavailable;
- future prediction position error against target truth interpolated at each
  absolute predicted timestamp, grouped by future horizon;
- robot planar position/yaw and joint-vector errors against physical base/joints,
  and target position error against target truth;
- physical position/speed overshoot, violation sample counts and observed duration;
- control compute time, binned/overall RTF, freshness, gaps and event-counter deltas.

Tracking uses physical world base pose plus measured joints and the authoritative
mount, never the controller's estimated pose or commanded motion. Robot state is
compared in its configured initial-pose odometry frame; target/prediction metrics
require matching world-frame IDs. Current wrist-forward convention is the owner
configured local +X. Missing/wrong-frame truth produces unavailable metrics.
Robot joint error is the Euclidean joint-vector norm; planar position error is
metres and yaw uses wrapped radians. NEES/NIS remain explicitly unsupported:
there is no matched full truth/innovation/covariance contract for them.

Tracking/estimation use nearest source-time matching bounded by `max_skew`.
Prediction truth uses bounded bracketing interpolation, without extrapolation or
estimated truth. Issuance/source epochs and exact future epochs remain recorded;
reporting groups horizons at 1e-7 seconds to avoid splitting identical steps due
to ROS nanosecond conversion. Later forecasts whose future samples lie beyond
the evaluation end do not gain fabricated truth.

Scalar sample statistics use equal valid-sample weights: mean, maximum,
`sqrt(mean(error**2))` RMSE and linear p95. They are not duration-weighted tracking
scores. Safety duration uses piecewise forward holds clipped to the window and
bounded by `max_skew`; duplicate raw joint samples cannot hide a peak. Overshoot
amounts remain unmodified; violations use the existing 0.001 rad/rad/s numerical
tolerance. Safe samples do not prove safety over uncovered intervals: acceptance
is unknown if coverage is incomplete, unless an observed violation already makes
it false. Latest pre-start feedback can cover the start only by a bounded forward
hold; stale pre-start samples cannot cover it.

RTF is total advancing simulation delta divided by matching monotonic delta, with
interval statistics binned in host time to avoid DDS-batch artifacts. CPU sums use
percent of one logical core across owned descendants; summed RSS includes shared
pages. Cumulative event counts use an observed baseline and deltas; missing
baselines/reset counters make a metric unavailable, not zero.

## Acceptance, failure and fair comparison

Execution and acceptance are independent. `execution_status: completed` means the
window was acquired/evaluated and owned cleanup succeeded. `acceptance: false`
means a measured gate failed; null means evidence is insufficient. Physical
violations always prevent unconditional success. Required tracking/prediction
availability, invalid-control flags and watchdog/rejection deltas are also gates.
There is no arbitrary tracking score or tuning added to manufacture acceptance.

CLI exit codes are 0 accepted, 1 interrupted/execution failure, 2 invalid
configuration, and 3 completed but failed/unknown acceptance. Default continue
policy continues variants after execution failures; abort stops the study.
Ctrl-C always stops the study independently of this policy and preserves the
current run. Handled failures contain error type/code/details, return code and
cleanup survivors. SIGKILL/power loss cannot guarantee final status serialization;
append-only raw evidence and the preparing status remain inspectable.

Each run leases a ROS domain and uses a unique Gazebo partition, plus a private
POSIX session and inherited ownership marker. Cleanup uses recorded process
identities/session/marker and bounded INT→TERM→KILL escalation. It never invokes
killall/pkill or stops unrelated sessions. Advisory leases coordinate this runner;
a prelaunch source probe rejects an already-active selected domain. External
programs ignoring leases can still collide. Descendants deliberately clearing the
marker and escaping their session before discovery are outside the supported
launch ownership contract; normal ROS/Gazebo descendants inherit it.

Comparability checks scenario/physical/measurement/seed contracts, initial
conditions, tracking reference, actuation, prediction dt/horizon, skew and window
definition. Differences in selected algorithm configurations/identities are
captured per run. Mismatched/incomplete pairs are explicitly incomparable and do
not receive pooled variant statistics. Comparable metrics aggregate equal-weight
run means, except prediction uses per-run RMSE and retains horizon-specific RMSE
aggregates. Dispersion is sample standard deviation across seeds, null for n=1.
No statistical significance or unconditional winning run is inferred. Safety
failures disable winner eligibility regardless of tracking error.

To add a variant, reference another compatible M07 manifest in YAML. To add a
metric, implement a pure function in metrics/, define its units/frame/alignment
and missing-data contract, add analytical tests, then register its family in
configuration.METRICS and evaluation. Do not add an experiment-specific launch or
robotics algorithm branch. Future visualization reads metrics/time_series and
raw provenance; no M09/M10 UI, server or plotting package is included.

## Reproduction and verification

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src -q --import-mode=importlib -p no:launch_testing -p no:launch_ros \
  -k 'not flake8 and not pep257 and not copyright'
python3 -m pytest src/robot_experiments/test \
  src/robot_bringup/test/test_architecture.py src/robot_bringup/test/test_architecture_tree.py \
  -q --import-mode=importlib -p no:launch_testing -p no:launch_ros
colcon build --symlink-install --packages-up-to robot_experiments
colcon --log-base /tmp/m08_copy/log build --base-paths src \
  --build-base /tmp/m08_copy/build --install-base /tmp/m08_copy/install \
  --merge-install --packages-up-to robot_experiments
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  bash --noprofile --norc -c '
    source /opt/ros/jazzy/setup.bash
    source /tmp/m08_copy/install/local_setup.bash
    cd /tmp
    python3 /home/user/ros2_ws/docs/validation/m08_verify_installation.py \
      --prefix /tmp/m08_copy/install --output /tmp/m08_install_repeat.json'
```

The focused suite includes analytical metric fixtures, schema/resource/inheritance,
CLI selection, seed 917 materialization, fault persistence, timestamp normalization,
clock resets/stalls, actual orphan subprocess cleanup while an unrelated process
survives, and the architecture DAG/tree. Real ROS/Gazebo results and final counts
are recorded below. Historical failed attempts are retained rather than counted
as successful metric evidence.

## Final evidence and verdict

**Verdict: CONDITIONAL.** The experiment workflow is implemented and verified from
YAML through real ROS/Gazebo execution, physical observations, metrics and paired
results. Existing physical overspeed remains failed acceptance. The final CV run
also records a feedback rejection, incomplete physical sampling and low RTF.
These are reported limitations, not conditions hidden by an execution-success flag.
No physical limit, PID gain, estimator setting, solver equation or target physics
was changed. This milestone does not certify hardware safety or eliminate M02/M03
research limitations.

| Check | Result | Evidence in docs/validation/ |
|---|---|---|
| Full functional regression | 1,071 passed; 18 existing lint/copyright entry points deselected | m08_final_regression.log |
| Focused M08 and architecture | 137 passed: 84 experiment tests and 53 architecture tests | m08_final_focused.log |
| Normal ten-package build | 10 finished, exit 0 | m08_final_build.log |
| Copied non-symlink build | 10 finished, exit 0 | m08_final_copy_build.log |
| Outside-checkout installation | Ten packages, eight console entries, installed variants and seed 917 materialization verified | m08_final_installed_checks.json / .log |
| Actual paired IMM/CV CLI, seeds 42 and 917 | Both variants completed; failed physical-safety acceptance retained | m08_live_evidence.json |
| Actual compute-only CLI | Completed; zero safe-command publications, acceptance unknown as intended | m08_live_compute.log |
| Invalid experiment | Exit 2; no output study or simulation created | m08_negative_validation.json |
| Actual early wall timeout | Exit 1; partial artifacts/status persist; owned cleanup has no survivors | m08_live_timeout.log |
| Existing work preserved | 815 historical untracked paths and all 347 tracked production files in the nine existing packages unchanged | m08_preservation.json |
| Final process inspection | No ROS/Gazebo test process remained | m08_final_process_check.json |

No repository-wide lint success is claimed; functional tests were not excluded.
The [review closure](validation/m08_review.md) documents the independent review,
RED→GREEN corrections and preserved failed acquisition attempts.

### Measured final paired benchmark

The final seeded study is
`results/experiments/m08_final_seed917/paired_seed917_20261018T061032_929053Z_750v4ffu/`.
Its installed CLI used a child YAML of paired_prediction.yaml with seeds [917].
Both variants used six measured simulation seconds after three settling seconds,
matched physical/scenario/measurement/horizon settings and actual sensor seed 917.
The summary marks them configuration-comparable. Host load and reactive trajectories
are not identical; this single-seed pair establishes integration, not algorithm
superiority or statistical significance.

| Variant | Physical tracking samples | Mean/max distance (m) | Mean/max pointing (degrees) | Pooled future position RMSE (m) | RTF |
|---|---:|---:|---:|---:|---:|
| IMM + residual correction + WLS | 446 | 0.1112 / 0.2925 | 6.323 / 13.273 | 0.5834 | 0.661 |
| CV, no adaptation/fusion + WLS | 362 | 0.1679 / 0.3806 | 9.165 / 21.217 | 0.6029 | 0.465 |

Pooled RMSE includes supported horizons 0.1–2.0 seconds (20 reported horizons),
using 1,671 and 1,665 valid future comparisons respectively. Per-horizon counts,
RMSE, mean, p95 and maxima are in metrics.json. Do not interpret the pooled field
`mean` as RMSE: it is mean Euclidean prediction error; `rmse` contains the formula
above, and benchmark aggregation explicitly uses that `rmse` statistic.

Actual node logs identify IMMPredictor/ResidualAdapter/WeightedCorrection for IMM,
ConstantVelocityPredictor with no adaptation/fusion for CV, and the same
RobotStateEstimator, TargetStateEstimator, WeightedLeastSquaresController,
WholeBodyComposer, SafeControlPipeline, FightFlightDynamics and RK4Integrator.
State/covariance, target truth/noisy measurements, future trajectory arrays,
commands, physical joint/base feedback, actuator telemetry and clock are captured
with original source epochs. No measured motion comes from integrating commands.

Observed physical position overshoot was **0.0 rad** in both runs. Measured speed
peaks in authoritative base_yaw/shoulder/elbow/wrist order were:

- IMM: [0.303, 1.000, 1.574, 1.569] rad/s;
- CV: [0.297, 1.089, 1.269, 3.476] rad/s;
- unchanged limits: [1.0, 1.0, 1.0, 1.5] rad/s, tolerance 0.001.

The maximum speed overshoot is 0.574 and 1.976 rad/s. Bounded observed violation
holds total 0.070 and 0.026 s; these are sampling-based durations, not exact contact
physics traces. Joint coverage was 100% for IMM and 92.2% for CV, whose maximum
source gap was 0.536 s. The latter cannot prove safety in the uncovered time.
All recorded safe commands respected configured bounds. Both measured windows had
zero watchdog increments, zero rejected commands and zero invalid control messages;
CV had one feedback rejection. Actuation failures therefore remain visible even
when tracking and command-space checks look acceptable.

Mean/p95 control computation was 21.57/41.98 ms for IMM and 36.60/73.62 ms for CV.
Mean robot planar-position errors were 0.1099/0.1290 m; target-position errors were
0.0877/0.0854 m. These preserve existing localization limitations. The CV RTF is
below 0.5 under this measured host load. No physics fidelity or safety change was
made to improve the number. An earlier separate seed-42 pair achieved RTF
0.988/0.975 and mean distance 0.0855/0.1237 m; this variability prevents a causal
performance comparison. Those artifacts remain available under
`results/experiments/m08_verified_final/` and are included in the compact evidence.

Example actual final IMM metrics (JSON fields retain radians for pointing):

```json
{
  "tracking_distance": {"available": true, "unit": "m", "sample_count": 446,
                        "mean": 0.1112203472, "max": 0.2924937282},
  "pointing_error": {"available": true, "unit": "rad", "mean": 0.1103565053},
  "joint_position_violation": {"max_overshoot": 0.0},
  "joint_speed_violation": {"max_overshoot": 0.5744248447, "tolerance": 0.001},
  "safety": {"accepted": false, "coverage_complete": true}
}
```

The final paired CLI exited 3, each run's execution_status is completed, and
acceptance is false. Neither variant is eligible as an unconditional winner.
Compute-only completed separately with no command stream and null acceptance.
The actual early-timeout status records wall_timeout, launcher return code -2,
and zero cleanup survivors. All raw directories and earlier failures are retained.
Some existing ROS adapters emit KeyboardInterrupt/context-shutdown tracebacks or
child exit codes -2/1 during the requested SIGINT shutdown. These remain in
launch.log; completed means the measurement window finished and owned cleanup
left no survivors, not that every adapter returned zero. Normal launcher return
codes were zero. Shutdown-log cleanup in existing robotics nodes is outside M08.

### Reproduce the final and fault cases

Create a child YAML using the installed parent; no robotics code changes:

```bash
m08_share=$(ros2 pkg prefix --share robot_experiments)
cat > /tmp/m08_seed917_repeat.yaml <<EOF_RUN
extends: ${m08_share}/config/paired_prediction.yaml
experiment:
  name: paired_seed917_repeat
  seeds: [917]
EOF_RUN
ros2 run robot_experiments run --config /tmp/m08_seed917_repeat.yaml
```

For compute-only, inherit nominal.yaml and set runtime.enable_actuation false,
duration 2.0 and settling_time 1.0. For a bounded real failure, additionally set
execution.wall_timeout 0.01 and execution.readiness_timeout 0.001; the runner should
return 1 and preserve a failed status with cleanup results. Duration 0 is a
configuration error, returning 2 before output creation. Use new output parents
if repeating evidence; invocation-level unique study naming also prevents reuse.

See [m08_live_evidence.json](validation/m08_live_evidence.json) for exact artifact
paths, identities, streams, source windows, gates and numeric values. The exact
changed/new source, resources, tests, docs and generated evidence paths are listed
in [m08_modified_files.txt](validation/m08_modified_files.txt). Captured Git status
is [m08_git_status_short.txt](validation/m08_git_status_short.txt). M08 remains
uncommitted on its dedicated branch; no M09/M10 work was started.
