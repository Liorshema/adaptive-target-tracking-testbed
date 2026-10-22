# Whole-Body v2 performance audit: before production performance changes

The smallest measured performance candidate is a configurable 2 ms physics
step, retaining selectable 1 ms operation. The current PID configuration held
the arm stably at 2 ms with gravity, all nine primitive robot collisions, four
position PIDs, effort limits, and physical geometry retained. Headless operation
already runs slowly; the separate GUI adds further CPU cost.

These short observations were made on ROS Jazzy / Gazebo Sim 8 / WSL, with four
available CPUs, two pre-existing default-partition Gazebo servers left untouched,
and unique benchmark Gazebo partitions / ROS domains 101–104. A later read-only
sample found each pre-existing server near one CPU of usage. Absolute RTF is
therefore a loaded-host result, not a clean-host capacity estimate. Runtime maps
confirm DART physics despite the world's `type="ode"` SDF attribute.

| Variant | RTF | Measured wall / simulated seconds | Notes |
| --- | ---: | ---: | --- |
| Exact production launch, environment default, 1 ms | 0.385 | 24.08 / 9.278 | No callback wrappers; actuation enabled |
| One BLAS/OMP thread, 1 ms | 0.395 | 24.06 / 9.510 | Observational actuator counters |
| Separate GUI, one thread, 1 ms | 0.338 | 24.06 / 8.130 | Same headless launch plus owned `gz sim -g --force-version 8` |
| Hold only, one thread, 1 ms | 0.412 | 24.05 / 9.906 | Estimation cProfile enabled, which adds overhead |
| Hold only, one thread, 2 ms | 0.696 | 24.05 / 16.726 | Temporary world / launch YAML; no profiler |
| Active baseline strategy, one thread, 2 ms | 0.755 | 24.06 / 18.178 | Concurrent source edits present by this run; see limitations |
| In-node joint EKF admission at 100 Hz, 1 ms | 0.364 | 24.13 / 8.786 | Temporary prefix wrapper; no demonstrated RTF benefit |
| Joint/clock external relays, 1 ms | 0.454 | 24.09 / 10.934 | Rejected: feedback timestamp / clock lag blocked every command |
| Joint-only external relay, 1 ms | 0.486 | 24.03 / 11.682 | Rejected: delayed joints fell behind wheel fusion watermark |

Each run used 8–10 wall seconds of warmup and 25 wall seconds of collection.
RTF uses two one-second observation endpoints within the collection window.
Topic `wall_hz` uses the entire 25-second collection; `sim_hz` uses that endpoint
simulation span, so the recorded approximate simulated rates can be a few
percent high. Subscribers may miss messages: observed joint-state rates are
not the source plugin rate. Gazebo JointStatePublisher has no configured cadence
and publishes every physics step: 1,000 Hz at 1 ms and 500 Hz at 2 ms.

The default 1 ms run averaged, as percent of one CPU: Gazebo 50.0, bridge 24.0,
target simulation 36.3, estimation 44.3, prediction 26.3, Control 35.1, Actuation
42.5. The separate GUI averaged 55.6%, peaked at 87.6%, and used up to 30 threads.
Its mapped renderer was OGRE-Next GL3Plus; GPU acceleration was not established.
The single-thread settings improved RTF only 2.6% and left node thread counts
unchanged. NumPy and SciPy load Debian reference BLAS/LAPACK in this environment,
so this is not evidence for an OpenBLAS oversubscription fix.

Control runs at approximately 20 Hz simulation time, Actuation and raw wheel
odometry approximately 50 Hz, target measurement / prediction approximately
18–20 Hz. At 1 ms the observer received about 585 joint/clock messages per
simulated second and about 288 full robot estimates; each accepted joint sample
triggers EKF prediction/update and legacy/full state publication. At 2 ms the
source physics and per-step transport work are halved; controller and actuator
periods are unchanged. Estimation profiling recorded 2,235 NumPy solves taking
0.38 s cumulative, versus about 3.3 s for EKF prediction/update, 4.2 s internal
publisher time, and 3.5 s cumulative warning calls over 30.25 s wall profiling.
Throttled warnings still incur caller inspection for suppressed packets. These
are wall-time cProfile observations with profiling/startup overhead, not precise
CPU attribution.

The unchanged actuator uses simulation-time command freshness 0.25 s, feedback
freshness 0.2 s, and independent monotonic receipt age 0.5 s for both. Its 50 Hz
steady-clock watchdog also checks publisher ownership; simulation-time ticks
and every command scan the five actuator output publishers again. Position
references integrate safe arm velocity at positive simulation dt, cap dt at
0.1 s, enforce physical joint bounds, and cap lead relative to measured position
at 0.08 rad. A stop retains the existing reference; there is one Gazebo position
PID owner per arm joint.

The single-thread 1 ms counter wrapper recorded 181 incoming commands, 177
accepted, three inactive and one rejected. There was no command/feedback
watchdog timeout. One future-stamped feedback event and startup readiness
rejections occurred. The in-estimator cadence trial recorded 154 accepted,
nine inactive and one command/feedback timeout; its longest observed incoming
command gap was 0.522 wall seconds. Independent safety correctly stopped under
that scheduling gap. Its lower estimate publication rate did not improve RTF.
Do not increase timeouts or adopt external relays on the strength of these
experiments. At a 20 Hz simulation control rate, expected wall command spacing
is `0.05 / RTF`; it exceeds the current 0.5 s receipt deadline below RTF 0.1,
even when simulation-time freshness is valid. This structural interaction needs
explicit future failure tests if supporting that regime.

Counters cover the process lifetime, including warmup and shutdown; GUI teardown
can extend server life beyond the collection window. They are not steady-window
event counts. The final 2 ms active counter writer failed during interrupted
shutdown; no exact acceptance total is claimed for it. Its observed active safe
commands and nonzero actuator/base output confirm response, and no watchdog
timeout was logged. The counter writer was subsequently repaired for future
benchmarks by capturing its output destination before interpreter teardown.

Settled hold comparison uses samples at simulation t >= 3 s. At 1 ms / 2 ms,
shoulder mean position was 0.483760 / 0.483728 rad; elbow -0.425168 / -0.423232;
wrist -0.001591 / -0.001292. Maximum settled joint speeds were below 0.00154
rad/s in both cases. These samples establish stable holding for this deployment,
not universal stability for every gain, robot, terrain or larger step.

Actual base motion was present in the exact default run: 0.444 m maximum
excursion from its first settled observation, with arm excursions
[0.102, 0.053, 0.096, 0.023] rad. The 2 ms active candidate showed 0.484 m base
excursion and [0.380, 0.134, 0.105, 0.025] rad arm excursions across a longer
simulated window; these are response checks, not phase-matched improvement
metrics. Sampled peak safe generalized velocity in the default run was
[0.2143, 0.0525, 0.0525, 0.0294, 0.0315, 0.0080] in [v, omega, four arm joints].
Weak apparent wheel motion therefore does not indicate a broken base chain.

Historical v1 telemetry at `../closed_loop_active/samples.jsonl` separately
contains 692 full raw/safe command and measured FK samples. Its peak raw and
safe magnitudes were identical:
[0.4313, 0.1231, 0.1231, 0.0738, 0.0485, 0.0120]; none of these 692 samples was
modified by safety. It confirms small arm commands and real base motion in v1.
It does not contain the four commanded position references, and the short
performance runs did not subscribe to those topics. `v2_performance.py
--references` now supports that additional observation, with its added observer
load explicitly flagged; Control raw/safe telemetry is also retained in future
samples. Reference lead saturation rates require that follow-up trace.

Recommended minimal production sequence: make physics step externally
configurable, retain 1 ms baseline and use measured 2 ms deployment candidate;
keep separate GUI support; verify full v2 motion/safety and GUI after the change.
Do not enable cadence gates or transport relays as performance defaults. Later
measure bounded elimination of duplicate publisher graph queries and logging
caller inspection, retaining publisher competition and independent stop tests.

Production files were not edited by this audit. The root/other agents modified
Control and Prediction concurrently, and the final 2 ms active run therefore
cannot be attributed solely to physics step relative to the initial HEAD.
Baseline strategy selection remained empty. Evidence includes every command,
temporary config/launch, runtime log, one-second state trace and CPU trace in
the sibling `v2_performance_before_*` directories. `summary.json` stores motion,
hold, command and rendered-world details; `host_environment.json` and the GUI
`mapped_engines.json` record runtime contention/backends.
