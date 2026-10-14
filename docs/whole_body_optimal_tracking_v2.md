# Whole-Body Optimal Tracking v2

The independent moving-target deployment is implemented and passes the preset
production-launch acceptance checks. The final run measured mean distance error
**0.059 m**, mean optical pointing error **5.64°**,
**0.821 m** base excursion and mean real-time factor **0.746**.
There were **zero watchdog timeouts after settling**. The experimental reactive
scenario and feedforward-disabled alternative have limitations described below.

Work began from HEAD `0a87773`, the committed Closed-Loop Tracking v1 milestone.
Existing untracked v1/arm-hold evidence was preserved. No commit or push was made.

**Launch the validated deployment**

```bash
source /opt/ros/jazzy/setup.bash
source /home/user/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=128
export GZ_PARTITION=whole_body_v2_demo
export GZ_IP=127.0.0.1
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ros2 launch robot_bringup closed_loop.launch.py \
  config:=config/optimal_tracking.yaml strategy_config:=config/whole_body.yaml \
  enable_actuation:=true gui:=false
```

The domain and Gazebo partition isolate this deployment from other running
simulations. In a second sourced terminal, use the same Gazebo partition:

```bash
GZ_PARTITION=whole_body_v2_demo GZ_IP=127.0.0.1 gz sim -g --force-version 8
```

`ros2 launch robot_simulation simulation.launch.py gui:=false` remains the normal
permanent gravity/arm-hold launch. Bringup defaults retain Compute Only and v1
DLS. In the v2 command, `publish_commands:=false` disables high-level publication
while preserving computation and arm holding. Omit `strategy_config` to select
the existing v1 Cartesian position controller; omit `config` to retain the
original estimation deployment. `lookahead:=0.0` and `feedforward:=false` are
explicit experiment overrides; the latter failed the measured-speed check.

**Measured diagnosis and performance**

The [pre-change audit](validation/v2_performance_before_default/audit.md) measured
the actual production v1 launch before performance changes. A 1 ms physics step
generated 1,000 joint-state/clock updates per simulation second, including costly
Python estimation updates and transport. Headless RTF was 0.385. Setting BLAS/OMP
threads to one yielded 0.395, a 2.6% difference; this environment uses Debian
reference BLAS, so OpenBLAS oversubscription was not established. A separately
launched GUI reduced RTF to 0.338 and consumed about 56% of one CPU. The GUI
comparison is from the original 1 ms configuration; no final-GUI RTF is claimed.

The adopted physics step is configurable **2 ms**. Gravity, nine primitive robot
collisions, robot inertia/effort/velocity limits, DiffDrive and the verified four
Gazebo joint-position PIDs remain enabled. The configurable world also retains
1 ms operation. DART is the actual loaded physics implementation, despite the
original template's `ode` attribute. Permanent-launch holding at 2 ms is verified
below. External clock/joint relays and a trial EKF input-admission gate were
rejected after measuring freshness/ordering problems or no performance benefit.

Full robot-state publication is capped at 50 Hz in the simulation calibration;
all accepted measurements still update the filter and legacy publication remains
compatible. Equal-time fusion is retained without duplicate full messages.
Timestamp rejection counts are retained while avoiding per-packet ROS logging
introspection; warnings report cumulative counts at the configured period.
The estimator's clock-reset callback clears its publication watermark.

Final headless v2 RTF was **0.746**, approximately **1.93×**
the initial headless observation. These are loaded-host observations on WSL with
two pre-existing user Gazebo servers left untouched, not isolated capacity
measurements. Per-process CPU percentages are relative to one CPU:

| Process | Original 1 ms v1 | Final 2 ms v2 |
| --- | ---: | ---: |
| Gazebo server | 50.0 | 42.7 |
| Bridge | 24.0 | 23.2 |
| Target simulation | 36.3 | 40.2 |
| Estimation | 44.3 | 47.9 |
| Prediction | 26.3 | 28.1 |
| Control | 35.1 | 38.3 |
| Actuation | 42.5 | 45.6 |

Throughput improved; every node's wall-time CPU did not decrease. Final observed
rates were about 450 joint messages, 52 odometry messages, 20 Control telemetry
messages, 49 actuator Twist messages and 10 actuator diagnostics per simulation
second. Ground truth was about 78 Hz; target measurements/predictions remain
configured at 20 Hz. Observer rates include startup and can miss packets.
Control computation averaged 34.5 ms wall time, p95
55.5 ms, including scheduling on the loaded host.

The original weak-motion symptoms did not establish a broken wheel chain: the
initial v1 audit already measured base movement. V1's nearest radial reference,
position-only objective, damping=1 and zero target feedforward explain its small
arm/wrist response and absent optical pointing correction. An additional
proprioception audit found substantial base-position/wrist lag under the generic
EKF covariance assumptions; replay showed covariance mismatch even with sorted
input. The simulation-specific low-noise encoder/odometry profile improves that
lag without changing physical limits or raw sensors.

**Task geometry and bounded allocation**

The new objective has one distance error and two pointing errors; camera roll
is free. For tracking position p, target t, distance r and configured radius R,
`e_range = r - R`, `d = (t-p)/r`, and the local target direction is `b = Cᵀd`.
Let a be the configured local optical axis, P its tangent projector, B the
configured orthonormal tangent basis, w=Pb, s=|w|, c=aᵀb and theta=atan2(s,c).
The pointing error is the spherical logarithm `Bᵀ (theta/s) w`.

Using the actual spatial Jacobian [Jv; Jw] and target velocity vt:

```text
range robot derivative = -dᵀ Jv
range target derivative = dᵀ vt
local direction rate = Cᵀ [(I-ddᵀ)/r (vt-Jv ν) + [d]× Jw ν]
pointing derivative = Bᵀ Dlog (local direction rate)
Dlog = (theta/s) P + ((theta*c-s)/s³) w aᵀ
```

The aligned branch uses its finite limit. Coincidence is rejected for safe
upstream stopping. At the antipode, no unique differentiable logarithm exists:
a bounded local escape chart selects an achievable tangent using the available
Jacobian, and the task offers opposite signed charts so bounds can select a
feasible escape direction. Exact antipodal escape, including a one-axis camera
at a joint limit, is covered by regression tests. Finite differences validate
the differentiable geometry, target-velocity rates, actual mounted whole-body
kinematics and changes of world coordinates.

The actual six-column spatial Jacobian has rank 4 in the planar/raised examples
and rank 5 in a yawed bent example. The new three-row tracking Jacobian has rank
3 in all four tested representative configurations. The final live run's smallest
normalized task singular value stayed above 0.534. Six commands are not six
independent pose DOFs. Cartesian position tracking remains selectable through
the task registry and the old DLS mathematics are preserved.

The new controller minimizes a bounded weighted least-squares objective using
existing SciPy `lsq_linear`; no new solver dependency, MPC or duplicate controller
pipeline was introduced. Task rates use configured metre/radian scales and
weights. Motion uses velocity normalized by the shared operating limits; costs
cover base/arm preference, translation/rotation, posture, limit proximity and
command smoothing. The optimizer uses the same SafetyPipeline bounds for
velocity, acceleration and next-step joint position. Independent final safety
projection remains authoritative. In the final run raw and safe commands matched;
seven sampled cycles hit optimizer bounds without violating commanded limits.

The radial sphere/LookAt reference remains a baseline and a visual diagnostic;
it does not define the new optimizer's position objective. The optimizer can
choose motion anywhere on the observation sphere. It is a local velocity
controller, without global reachability or collision-avoidance guarantees.

**Prediction and configuration ownership**

The active factory instantiates IMM(CV/CA/CT), ResidualAdapter and
PerformanceMonitor; RLS/learned classes are not active. IMM likelihoods now score
at the observation timestamp rather than rounding to the first 0.1 s forecast
sample. Observation interval and forecast scoring time are separate, including
multiple updates to one forecast. Residual adaptation learns the baseline bias;
monitoring separately records corrected prediction error. Existing update hooks
remain compatible through the optional timestamp-aware update interface.
Control linearly samples position and velocity at now + configured lookahead,
with bounded initial-interval reconstruction and no future-horizon extrapolation.
Distance and pointing target rates provide physical feedforward.

[Recorded-path replay](validation/v2_target_replay.json) used the actual target
filter and active predictor. Sensor covariance is 0.01 m², hence sigma=0.1 m;
it already matches target-estimator R and was preserved. Sweeping process
acceleration std 0.1/0.3/0.5/1/2 gave mean estimated-position errors
0.137/0.065/0.058/0.063/0.073 m. The simulation profile selects **0.5**.
The 0.1 prior reproduced the observed lag; correcting that prior reduced current
query prediction error from 0.129 to 0.059 m in offline replay. This is synthetic
noise replay of a recorded path, not a new live ground-truth estimator benchmark.

Robot geometry, mount, joint metadata and physical limits come from
robot_description. Control and Bringup share an injected model-construction
helper in robot_models; the tracking mount is applied once. Bringup derives
joint count/names and passes the resolved description to Control, Actuation and
the optional measured-FK broadcaster. The concrete SerialArmModel remains
four-joint; generic interfaces/factory helpers do not assume that topology.
Gazebo physics, PID holding, reference integration, bridges and actuator
ownership remain in robot_simulation. No Gazebo logic was added to Control.

YAML `extends` composes a base configuration with recursive mapping overrides;
arrays/scalars replace their parent values, relative parent paths resolve beside
the child file, and inheritance/alias cycles are rejected. Moving-target YAML
now contains scenario differences only. Static tracking input means target
interaction receives a static camera position; alpha=0 in the independent
scenario still allows its target to move under initial velocity/restoring dynamics.

The optional reactive scenario uses alpha=1 and measured odometry/joints to
publish odom→tracking_frame, alongside the configured world→odom transform.
It checks age/skew, rejects old/future packets before caching, clears on clock
reset, and uses the selected robot description. Simulation now uses ROS simulation
time when checking TF age. The TF path was verified with actuation disabled.
The separate reactive dynamics can drive the target far away and active tracking
reached a joint-bound stop; it is explicitly experimental, not part of the
accepted independent-target milestone. See its preserved runtime logs.

**Configuration introduced**

| Location | Parameters / defaults |
| --- | --- |
| Control ROS parameters | `strategy_config_file`, `robot_description_file`, `feedforward_override`, `lookahead_override`; empty retains baseline/default behavior |
| whole_body.yaml task | `type: distance_pointing`, `minimum_distance: .001`, `antipodal_tolerance: .00001`; radius/axes bound to shared tracking settings |
| whole_body.yaml objective | gains [1,3,3], task weights [4,12,12], task scales [1,1,1], motion weights [1,2,3,3,3,2], damping .08, posture weight .01/gain .3, smoothness .005, limit avoidance .002; optional preferred joint positions |
| whole_body.yaml tracking | `prediction_sampling: linear`, `target_velocity_feedforward: true`, `lookahead_time: .25` |
| Simulation world | `physics.max_step_size: .002`, `real_time_factor: 1.0` |
| Actuation diagnostics | `publish_diagnostics: true`, `diagnostic_rate_hz: 10.0`, `diagnostic_topic: /simulation/actuation/telemetry` |
| optimal_tracking.yaml | `estimation_overrides: config/simulation_estimation.yaml` |
| Simulation estimation | wheel linear/angular and joint position/velocity variance 1e-6; target acceleration std .5; `full_state_publish_rate: 50.0`, `diagnostic_log_period: 5.0` |
| Optional tracking TF | `rate_hz: 20`, `max_age: .2`, `max_skew: .05`, `future_tolerance: .01`; frames/topics/names/description supplied by Bringup |
| Simulation TF input | tracking `max_age: .2`, `future_tolerance: .01`; ROS parameters `tracking_max_age`, `tracking_future_tolerance` |
| Shared YAML | `extends: relative-or-absolute-parent-file` |

Desired radius, physical geometry, limits and joint-holding gains/angles were not
copied into Python. The six motion costs are an external deployment choice for
the current robot; a replacement robot must supply matching-dimensional costs.

**Controlled comparisons and safety**

Runs use the same independent target, initial conditions/noise seed, 2 ms physics,
simulation estimator calibration and 35 simulation seconds. Errors are evaluated
from 10–35 seconds after target initialization, using measured Gazebo wheel
odometry and named joints with actual mounted FK against target ground truth.
Feedback skew is bounded at 0.08 s. Values below are means except base excursion;
the motion integral uses the same normalized velocity cost for every strategy.

| Case | Distance m | Pointing ° | Base m | Motion integral | RTF |
| --- | ---: | ---: | ---: | ---: | ---: |
| Hold | .379 | 84.86 | ~0 | 0 | .793 |
| v1 position DLS | .140 | 68.11 | .328 | .729 | .728 |
| Final v2 | 0.059 | 5.64 | 0.821 | 7.592 | 0.746 |
| v2 feedforward off | .071 | 3.85 | .669 | 3.332 | .747 |
| v2 zero lookahead | .057 | 4.28 | 0.786 | 5.166 | .739 |

The smaller v1 motion cost does not demonstrate equal-task efficiency: it omits
pointing. Allocation preferences are independently verified by tests that favor
either base or arm, and by joint-bound cases that redirect motion to the base.
Feedforward and 0.25 s lookahead are not universally better here: zero lookahead
slightly improved range/pointing, and disabling feedforward reduced cost/pointing
error but briefly exceeded wrist speed (1.5197 versus 1.5 rad/s). That alternative
failed the preset physical-speed check; no limit or tolerance was relaxed.

![Measured tracking and motion comparison](validation/whole_body_v2_comparison.png)

![v2 actuator command chain](validation/whole_body_v2_actuation.png)

Final v2 maximum distance error was 0.146 m and maximum pointing
error 12.15°. Arm excursions were
[.198, .428, .154, .571] rad; all four arm joints and the base responded.
Measured wrist peak was 1.500000004 rad/s, within the preset .001 numerical
tolerance; other joint velocities and all safe commands respected limits.
Minimum measured position-limit margin was 0.874 rad.
The actor accepted 670 commands with zero rejections/timeouts, six bounded-reference
limitation events and two startup inactive commands. Actuation was active in all
500 settled evaluation samples. Telemetry is sampled asynchronously, so actor
and Control traces are not exact packet-by-packet acknowledgements.

Acceptance thresholds were defined in [v2_thresholds.yaml](validation/v2_thresholds.yaml)
before the comparisons: mean/max distance .20/.45 m, mean/max pointing 10/25°,
minimum base/arm motion .1, mean RTF >=.5, zero settled watchdog timeouts, physical
limits with .001 numerical tolerance. Final v2 passed every check. Failed earlier
uncalibrated/latency trials are retained rather than hidden.

The [failure/recovery test](validation/v2_safety/result.json) passed controller
stall, stale prediction, stale robot feedback, physical base stopping, arm hold,
clock reset/new session, retired-session replay rejection, controller restart and
single actuator ownership. Command/feedback simulation age .25/.2 s, independent
monotonic wall timeout .5 s, integration dt cap .1 s and position lead .08 rad
are unchanged. At much lower RTF, legitimate command spacing can exceed the wall
watchdog (20 Hz simulation Control implies .5 s spacing at RTF=.1); supporting
that regime would require a separate design, not relaxing the safety deadline.

The [normal simulation Compute Only fixture](validation/v2_compute_only/result.json)
passed 682 safety-checked cycles with `publish_commands=false` and no actuator
command publisher. Across simulation seconds 5–35, maximum holding error was
.03988 rad, maximum joint speed .00139 rad/s and minimum tracking height above
the base .39276 m. It also passed stale-joint blocking/resumption and an actual
Gazebo world reset with the permanent robot and raised arm preserved.

**Regression evidence, limits and modified files**

Eight ROS packages built successfully. The final functional regression ran
**708 passed, 18 lint tests deselected**. Package-level testing reports
**726 tests, 0 errors, 5 failures, 6 skipped**; the five remaining failures are
pre-existing style checks in robot_description (Flake8), robot_prediction
(Flake8/PEP257), and robot_simulation (Flake8/PEP257). The same failures were
reproduced from an archive of HEAD. New Models, Control and Bringup checks pass;
no new style failure is being accepted. The shared files were checked with
`git diff --check`. Logs are archived under docs/validation/v2_*_final.log and
[v2_head_lint_summary.json](validation/v2_head_lint_summary.json).

Reproduce functional regression and live acceptance:

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src --import-mode=importlib -p no:launch_testing -p no:launch_ros \
  -q -k 'not flake8 and not pep257 and not copyright'
python3 docs/validation/v2_tracking_gazebo.py --case v2 --output /tmp/v2_acceptance
python3 docs/validation/closed_loop_safety.py --case failures \
  --config config/optimal_tracking.yaml --strategy-config config/whole_body.yaml \
  --output /tmp/v2_safety
python3 docs/validation/compute_only_gazebo.py
```

The validation scripts isolate domains/partitions and clean up only their own
launch groups. The final process audit found only the two pre-existing user
Gazebo servers; no validation process remains. Run performance comparisons sequentially; the baseline audit is
already preserved. The final acceptance includes a configuration snapshot.
Plots/comparison JSON are reproducible with `python3 docs/validation/compare_v2.py`.

Remaining limits: local optimization does not guarantee global reachability,
obstacle avoidance or long-horizon stability; pointing is undefined at coincidence
and has a chart discontinuity at the antipode. Target velocity remains noisier
and more delayed than position; residual correction applies a horizon-wide state
offset, and no covariance/confidence weighting is active in Control. The generic
EKF's process covariance depends on sensor cadence; only the simulation profile
was calibrated here. Wheel-odometry slip versus true world base pose was not
independently measured. ROS publisher ownership is checked, but arbitrary direct
Gazebo publishers are outside that arbitration. The experimental reactive case
and feedforward-disabled speed overshoot remain explicit limitations. The
measurements establish this independent-target milestone, not general robot or
scenario performance.

The complete [modified-file manifest](validation/v2_modified_files.txt) lists 71
implementation/document files. Main changes are:

- robot_models: distance/pointing geometry and injected whole-body construction.
- robot_description: shared physical component configuration and joint metadata.
- robot_control: interchangeable velocity tasks, bounded optimizer, strategy
  selection/interpolation/feedforward, shared safety bounds and computation diagnostics.
- robot_prediction: timestamp-aligned IMM evidence and baseline/corrected residual monitoring.
- robot_estimation: full-message publication budget and counted timestamp diagnostics.
- robot_interfaces: extended ControlTelemetry and new ActuationTelemetry.
- robot_simulation: configured physics, YAML inheritance, actuator diagnostics and TF freshness.
- robot_bringup: shared model factory, v2 estimation deployment, resolved description
  propagation and optional measured tracking TF.
- docs: audit, replay/production validation tools, regression logs, raw measurements,
  configuration snapshot, comparison JSON, plots and this report.
