# D2.6c — Runner readiness and scored completion

2026-10-31. **PASS for this task's readiness, duration, saved-results and cleanup
criteria.** One bounded GUI Run completed with the original saved experiment and
unchanged thresholds. This verdict does not reclassify tracking quality, other
milestones or hardware safety. Ogre2 rendering remains a separate limitation.

## Root cause, established before the fix

Readiness uses the latest received source epoch for every required stream and the
latest received ROS clock. All streams must exist, and every absolute source/clock
difference must be at most the saved `max_skew=0.08 s`. Simulation time must be
positive. The Recorder dispatches an initial callback, then drains at most 128
callbacks / 10 ms. Previously the Runner checked readiness only after that drain.

**The current-state predicate is nonmonotonic across asynchronous callbacks.** A
valid state can occur within a batch and become invalid before the Runner checks
it. This is a decision-sampling defect, not evidence that the threshold should be
relaxed or that the streams should be treated as synchronized.

The preserved D2.6b recording demonstrates the exact sequence:

| Event | Host monotonic time | Observer clock | Relevant freshness |
| --- | ---: | ---: | --- |
| Joint callback establishes all-stream freshness | 1726.410901949 | 13.444 | All 11 streams within 0.08 s; largest absolute age 0.062 s |
| Next base-truth callback, 0.928 ms later | 1726.411829834 | 13.444 | Base source becomes 13.550; signed age −0.106 s, outside bound |
| Actual recorded Runner performance tick | 1726.421170715 | 13.444 | Same −0.106 s base age; readiness false |

`failed_trace.json` reconstructs source state from exact recorded callback arrival
order. It also compares 68 real recorded performance-tick boundaries: all failed.
These ticks occur inside `run_window` after spinning and before its readiness
check; no ROS dispatch occurs during the tick. They are a sampled subset of actual
Runner decisions, not an invented complete batch trace. All source stamps at the
closest tick reproduce its recorded observer clock exactly.

Clock and joint records were delivered at about 172 samples per simulation
second in that attempt, whereas base truth was about 20 Hz and actuation about
10 Hz. Fixed-depth asynchronous subscription queues, differing publication rates
and scheduling can expose unequal source epochs. The old trace did not record
middleware queue residence or all dispatch boundaries, so exact attribution to
CPU scheduling versus transport backlog is unavailable. It does prove the missed
fresh state and the subsequent real failed decision. No globally ordered-source
assumption, clock timestamp rewriting or sensor alignment was introduced.

### GUI and rendering

The old GUI used software `llvmpipe` and logged Ogre2 material/shader errors.
Those errors are retained separately. The same readiness sampling defect is
reproduced by a deterministic Recorder/Runner queue test without Gazebo or a
renderer. After the fix, the real GUI Run passed readiness and completed while
Ogre rendering errors remained in its native log. This does not quantify graphics
load's contribution to scheduling. No rendering fix, graphics tuning or causal
performance claim is made. A headless comparison was unnecessary because the
conditional request to run it applied only if GUI readiness still failed.

## Minimal correction

`Recorder.spin_once(timeout, until=None)` retains its existing default behavior
and count/time budgets. Before origin selection, `run_window` supplies its current
positive-clock/readiness condition. The drain returns as soon as that condition
is true, **without consuming subsequent queued callbacks**. The Runner then
rechecks current readiness after its existing cancellation/process/deadline and
clock-reset checks and establishes the origin. No cached historical “ready” flag
is accepted later. Subsequent iterations use the normal Recorder drain.

The zero-clock condition is included so fresh zero-epoch messages cannot prevent
further dispatch while waiting for positive simulation time. A regression test
caught and fixed that edge case before the real check.

The original `TelemetryStore.ready` predicate is unchanged (verified by AST
comparison). No required stream, QoS policy, timestamp, model, physics parameter,
controller, duration/settling arithmetic, timeout or acceptance threshold changed.
Pending messages remain queued and are recorded on subsequent spins.

For inspectable evidence, the existing Runner records:

- Sampled current source epochs/ages in its existing `host_performance.json` ticks.
- An exact one-time `readiness.json` at the actual passed Runner decision, through
  an optional `run_window` callback. It contains monotonic epoch, observer clock,
  threshold, original source times and signed ages in seconds.

`readiness_snapshot` is diagnostic only and delegates its boolean to the existing
predicate. It does not spin, alter state or supply scientific timestamps. No new
execution engine, configuration profile, package or production module was added.

## Real GUI Run

Installed dashboard on **8060**, verified free on Linux and Windows before launch.
Windows localhost subsequently returned HTTP 200 on this port. Existing port 8053
was not used or stopped. The new workspace is
`results/runner_readiness/workspace`.

Saved selection: **EXP-NOMINAL r3 / imm / seed 42**, definition SHA-256
`40d1845815048bbcf37a1b882a657291ed8237b96d8d6a61a3ac2912e29301b2`.
The authoritative saved definition was unchanged. The browser explicitly selected
both physical actuation and GUI for Start; both defaulted OFF, and actuation reset
OFF after Start. This was ordinary scored Start through the existing backend /
Jobs / Runner, not a preview or alternate launch path.

| Required evidence | Actual result |
| --- | --- |
| Exact passed Runner decision | Clock/origin **14.108 s**, all 11 required streams present |
| Largest absolute source/clock skew | **0.042 s**, below unchanged **0.08 s** |
| Settling interval | **14.108–17.108 s** (3.0 s) |
| Scored evaluation window | **17.108–23.108 s** (6.0 s) |
| Automatic stop clock | **23.108 s** |
| Execution | **Completed**, no execution error |
| Runner loop wall elapsed | **54.504530 s**, including readiness, settling and evaluation |
| Cleanup | Return code 0, SIGINT, no survivors; **0.924719 s** |
| Isolation | No live process in owned Gazebo partition; ROS domain lease reacquired |
| Scientific storage | Existing immutable Run, metrics, time-series and raw telemetry artifacts |
| Session | References that Run's UUID/directory; no duplicated results store |
| GUI count | Exactly one scored Job/Run request; no comparison/sweep or second launch |

The exact passed input epochs were:

| Stream | Source time [s] | Observer clock − source [s] |
| --- | ---: | ---: |
| clock | 14.108 | 0.000 |
| target truth | 14.092 | 0.016 |
| target measurement | 14.092 | 0.016 |
| joints | 14.108 | 0.000 |
| base truth | 14.150 | −0.042 |
| robot estimate | 14.070 | 0.038 |
| target estimate | 14.092 | 0.016 |
| prediction | 14.092 | 0.016 |
| control | 14.100 | 0.008 |
| command | 14.100 | 0.008 |
| actuation | 14.100 | 0.008 |

A separate audit reconstructed the latest received epochs immediately before the
recorded decision from raw callback arrivals; all eleven exactly match
`readiness.json`. Negative ages are real source epochs ahead of the observed
clock, accepted by the existing absolute-skew rule, not altered timestamps.

Identifiers:

- Job: `69643bd3-23ee-4091-b93f-659c3755c26a`
- Run: `1a0712b4-6226-47f3-a2cf-5a11a2cb3a2a`
- Session: `7b933686-93d1-4f0a-bfd5-c553f7c93bb6`
- Session title: **D2.6c readiness completion**
- Run directory:
  `results/runner_readiness/workspace/runs/nominal_20261031T180156_975994Z_7syy67id/runs/imm_seed42`

## Recorded metrics and results access

Values below come from the saved metrics artifact; no algorithms or scoring were
changed. The pre-fix failed attempt has no valid scored window, so it cannot serve
as a before/after tracking-quality comparison.

| Recorded metric | Mean | Maximum | Samples |
| --- | ---: | ---: | ---: |
| Tracking distance error [m] | 0.085688 | 0.199011 | 350 |
| Pointing error [rad] | 0.201350 | 0.319140 | 350 |
| Target position estimation error [m] | 0.087465 | 0.162379 | 98 |
| Real-time factor [1] | 0.638583 | 0.749799 | 19 |

Recorded prediction **RMSE** is **0.543899 m**, from 1,632 samples; the mean
prediction error is a different statistic. All selected tracking, prediction,
robot/target estimation, computation-time, RTF and physical/event metrics are
available. NEES/NIS are unavailable and remain explicitly unavailable.

Existing recorded safety gates accepted this particular six-second observation
window: 984 physical joint samples, full configured coverage, zero measured
position/speed violation samples at unchanged tolerances. This is independent
recorded evidence, not an inference from Completed, and does not establish general
hardware safety or acceptance outside this window. Tracking quality remains a
research result with the actual pointing/prediction errors above.

`get_run_results` and `read_time_series` read these unchanged artifacts; backend
pagination returned real tracking samples. Results & History rendered the recorded
KPIs, distance/pointing plots, telemetry panels and Session row in the installed
browser. No unavailable values, synthetic curves, rankings or improvements were
invented.

Actual screenshots:

- [Running after readiness](../../results/runner_readiness/dashboard_running.png)
- [Completed experiment](../../results/runner_readiness/dashboard_completed.png)
- [Results & History with real metrics](../../results/runner_readiness/dashboard_results.png)

Generated proof is under `results/runner_readiness/`: `failed_trace.json`,
`gui_run.json`, `completed_audit.json`, raw Run artifacts, browser scripts/logs,
port checks and copied native `ogre2.log`. Previous D2.6b failure artifacts and
validation report remain unchanged.

## Tests, build and review

- Two initial regressions failed before the fix: the drain consumed the valid
  intermediate state, and real `run_window` timed out instead of selecting its
  origin. See `red.log`.
- A fresh-zero-clock dispatch test failed before its guard was added; see
  `zero_clock_red.log`.
- **66 focused tests passed** (`verified.log`, 34.63 s): recording, lifecycle,
  ObserverTiming and Runner. They include default drain fairness/budgets, queued
  message retention, clock reset/stall/deadlines, failed startup and cleanup,
  compute-only required streams, real duration/settling and diagnostic evidence.
- **53 architecture tests passed** (`architecture.log`, 40.88 s).
- Installed build: **2 packages succeeded** (`build.log`, 18.8 s).
- Installed changed production modules were byte-compared to source before the
  live Run. Predicate AST comparison and task-relative scope audit confirm no
  physical/configuration/safety change.
- Independent read-only review found no important correctness regressions; no
  tests or Gazebo processes were started by the reviewer.
- Changed production modules compile successfully. `git diff --check` flags a
  pre-existing blank EOF line in `recording/ros.py` relative to HEAD; the task
  baseline archive confirms that whitespace already existed. It was not changed
  or counted as a functional regression.

Focused reproduction:

```bash
cd /home/user/ros2_ws
source install/setup.bash
export PYTHONPATH=src/robot_experiments:$PYTHONPATH
build/dashboard_venv/bin/python -m pytest \
  src/robot_experiments/test/test_recording.py \
  src/robot_experiments/test/test_execution.py \
  src/robot_experiments/test/test_observer_timing.py \
  src/robot_experiments/test/test_runner.py -q
build/dashboard_venv/bin/python -m pytest \
  src/robot_bringup/test/test_architecture.py \
  src/robot_bringup/test/test_architecture_tree.py -q
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_experiments robot_dashboard
```

No full-repository sweep, physics rebuild or parameter sweep was performed.
Graphics readiness and deterministic performance across arbitrary host loads are
not guaranteed. Readiness still legitimately fails if current observations never
satisfy the original bound. The fix prevents consuming a valid observed state
before the Runner is allowed to decide on it.

## Launch and inspect

The dashboard is already listening on **http://127.0.0.1:8060**. After stopping that
server, relaunch with:

```bash
cd /home/user/ros2_ws
source install/setup.bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir /home/user/ros2_ws/results/runner_readiness/workspace
```

Select Session **D2.6c readiness completion**, then its Run in Results & History.
For another experiment, use the normal saved-revision workflow and explicit
actuation opt-in. Open remains paused; Play's cleanup-gated preview handoff is
unchanged. The bounded verification script intentionally refuses an existing Job
workspace, preserving this experiment's evidence rather than overwriting it.

## Exact task-relative changed files

Modified production:

- `src/robot_experiments/robot_experiments/recording/ros.py`
- `src/robot_experiments/robot_experiments/recording/storage.py`
- `src/robot_experiments/robot_experiments/execution.py`
- `src/robot_experiments/robot_experiments/runner.py`

Modified tests/documentation:

- `src/robot_experiments/test/test_recording.py`
- `src/robot_experiments/test/test_execution.py`
- `docs/dashboard.md`

Added: `docs/validation/runner_readiness.md` (this report).

No files removed or new production modules/dependencies/profiles. Historical
scientific evidence is preserved. No commit/push; no D3 or dashboard redesign.
