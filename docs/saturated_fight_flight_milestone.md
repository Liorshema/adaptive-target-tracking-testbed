# Saturated Fight/Flight milestone

The saturated model is selectable, its numerical verification passed, and the
selected scenario remained bounded for **120 simulated seconds** with live
camera TF feedback and meaningful full-run 3D motion. The permanent production
launch was also verified for 60 seconds. Full acceptance is **not green**:
measured physical joint-speed overshoot persists, and real-time factors in this
session were below the unchanged .5 performance threshold. These failures are
reported rather than hidden by changing gains, physical limits or thresholds.

## Mathematical design and model selection

The deployed target state remains `[px,py,pz,vx,vy,vz]` in world coordinates.
For displacement `d=p-p_C`, distance `r=norm(d)` and the existing regularized
direction `u=d/sqrt(r²+epsilon²)`, the new model evaluates:

```text
p_dot = v
v_dot = a_max*tanh((r-D)/transition_length)*u - damping*v + environment.acceleration(p,t)
```

`FightFlightDynamics` still owns state handling, direction regularization,
damping and external field composition. Parameter objects supply a
`radial_acceleration(distance)` law. `FightFlightParameters` returns the original
`alpha*(r-D)`; `SaturatedFightFlightParameters` returns the tanh law. The existing
factory registers `saturated_fight_flight`, using the same dynamics class, engine
and RK4 integrator. Parameter variations require YAML edits, not new classes.
The existing `fight_flight` selector and all its parameters retain their meaning.

| Parameter | Units | Selected value |
| --- | --- | ---: |
| a_max | m/s² | .15 |
| transition_length | m | .15 |
| preferred_distance, D | m | 1 |
| damping | s⁻¹ | .5 |
| distance_epsilon | m | 1e-6 |

New coefficients are checked for finiteness; acceleration and lengths must be
positive and damping nonnegative. At `r=D` radial interaction is zero. Below D
it attracts toward the tracking point; above D it repels. At large r its scalar
magnitude approaches a_max; vector norm never exceeds a_max because norm(u)<=1.
At coincidence the direction is zero, so the interaction contributes zero;
damping and environmental acceleration can still act. Near D the radial slope
is `a_max/transition_length`, selected as 1 s⁻², matching baseline alpha=1 s⁻².
There is no required coupling between these independent configurations.

Only the interaction is bounded by a_max. Damping and the external field are
additional accelerations; total acceleration is not universally capped. The
unchanged positive restoring field and positive damping provide confinement
for these tests. No target position or velocity clipping was added. The existing
RK4 step .01 s and simulation-time integration remain unchanged.

## Configuration hierarchy and ownership

```text
robot_bringup/config/reactive_tracking_saturated.yaml
  extends reactive_tracking.yaml
  selects robot_simulation/config/scenarios/reactive_target_saturated.yaml
    extends reactive_target.yaml
    selects config/scenarios/target_saturated.yaml
      extends target_geometry.yaml
        extends ../target.yaml; replaces target.dynamics with null
      selects saturated_fight_flight with its own parameter mapping
```

`target_geometry.yaml` uses the loader's existing null-replacement rule to keep
visuals/physics without inheriting linear alpha into another model's parameters.
The final child fills the dynamics mapping. The template is not a standalone
launch configuration. This avoids an ignored legacy parameter or duplicated
geometry files; future target models can reuse it.

The independent moving target, reactive linear baseline, and linear low-damping
selections are unchanged. The saturated initial conditions are explicitly
configured as position `[2,.25,1] m`, velocity `[0,.10,-.12] m/s`. They break
planar symmetry without a prescribed target trajectory. Dynamics and fields
are deterministic; the inherited measurement configuration retains seed 42.

The actual external field remains `config/scenarios/environment.yaml`:
`linear_restoring`, center `[2.2,0,.85] m`, gain matrix `.5*I s⁻²`, no stochastic
disturbances. It is injected by the existing factory. The generic
`config/environment.yaml` with center `[0,0,1]` and gain `2*I` is not selected.
Neither field was edited. Target D and Control radius R remain independently
owned values; both currently equal 1 m.

Tracking remains `source: tf`, `world -> tracking_frame`, max_age=.2 s,
future_tolerance=.01 s. The existing measured-feedback broadcaster uses raw
joints and wheel odometry, with the static world→odom transform. Target traces
record the actual tracking position passed into integration, rather than an
assumed static point. Settled TF availability was 100% in all three comparison
runs. For the selected scenario, the RMS difference between its actual radial
force and the force at the same target states using the initial frozen reference
was **.1538 m/s²**. This is an instantaneous force comparison, not a claimed
counterfactual closed-loop trajectory. It verifies a material live-reference
effect. Ground truth is used only by diagnostics; it does not steer the robot
or target interaction.

## Verification and limited tuning

Before Gazebo, 24 trajectories completed 80 s each: four parameter candidates,
three initial conditions and static/moving reference inputs. Six additional
80 s trajectories tested the selected smaller force. Cases include positive and
negative initial vertical velocities, different starts, strengths and damping.
The diagnostic moving reference is a configurable test input; it is not a
new target trajectory in the application. All 30 trajectories stayed finite
and within the diagnostic 10 m / 10 m/s envelope. Complete repeated trajectories
were bitwise identical under fixed conditions. Raw position, velocity,
acceleration and reference distance are saved as NPY files. The live asynchronous
ROS/Gazebo loop is not claimed to be bitwise repeatable across launches.

Mathematical tests verify interaction signs, zero at D, bounded norm across 3D
directions, coincidence/near-zero behavior, local slope, damping, field addition,
reference sensitivity, invalid parameters, legacy linear compatibility and
deterministic RK4 integration.

Three target-only live candidates were examined; no robot or control tuning:

| Candidate (a_max, length, damping) | Duration | Observed result |
| --- | ---: | --- |
| Suggested (1.5,.5,.15) | 40 s | Finite but excessive workspace: max center distance 5.42 m, target z up to 3.78 m; mean actual range error 4.63 m and pointing 105.3°; joint-bound stops |
| Smaller (.35,.35,.3) | 60 s | Center distance <=1.40 m but mean actual range error .582 m, max 2.17 m; pointing max 70.7°; bound stops and speed overshoot |
| Selected (.15,.15,.5) | 120 s | Center distance <=.355 m; sustained active tracking, comfortable settled position margins; residual physical speed/performance failures |

The first case demands poses beyond the useful tracking workspace; that is not
evidence of a new controller bug. The smaller case also exposes existing
localization/boundary/actuator limitations. Tuning stopped after the contained
configuration demonstrated useful bounded tracking. Single-thread BLAS/OpenMP
settings were used consistently in the fresh final comparison runs; they did
not restore the .5 real-time-factor margin in this session.

## Fresh experimental comparison

All cases use the same robot, limits, effort PIDs, V2 Distance + Pointing task,
wrist +X optical axis, estimation and prediction configurations. The linear
cases use their existing `[2,0,1] / [0,0,0]` initial conditions and run 40 s;
saturated uses its separate initial conditions and runs 120 s. **This is an
observational comparison, not a strictly controlled comparison of dynamics.**

Speed means/maxima and tracking means use seconds 10–duration. Maximum
displacement is measured from the environmental center over the whole run.
XYZ ranges below include startup. Exact target velocities/accelerations come
from the numerical state, not noisy differentiated perception. Tracking errors
use native Gazebo wrist/model poses plus the authoritative mount and target
truth, including periods when Control telemetry is invalid. Joint-speed peaks
use every received joint measurement after settling, not only 20 Hz telemetry.

| Metric | Linear baseline | Linear low damping | Saturated |
| --- | ---: | ---: | ---: |
| Simulated duration, s | 40 | 40 | 120 |
| Mean target speed, m/s | .1668 | 4.199e6 | .06351 |
| Maximum target speed, m/s | .3014 | 7.968e7 | .1867 |
| Maximum displacement from center, m | .3605 | 1.253e8 | .3548 |
| Full-run X/Y/Z ranges, m | .673/.0193/.234 | 1.191e8/2.880e6/3.880e7 | .441/.364/.182 |
| Mean/max distance error, m | .108/.242 | 6.974e6/1.234e8 | .1046/.2323 |
| Mean/max pointing error, degrees | 6.22/14.87 | 28.94/52.99 | 9.96/19.47 |
| Maximum measured joint speed, rad/s | 1.591 | 6.337 | 1.566 |
| Bounded useful workspace over tested duration | Yes | No | Yes |
| Measured physical speed check | Fail | Fail | Fail |
| Mean real-time factor | .401 | .437 | .448 |

Selected target total acceleration peaked at .2730 m/s²; interaction acceleration
peaked at .1406 m/s², below .15. After settling its XYZ ranges were
`.395/.0298/.0357 m`: much of the Y/Z range is initial transient, and later motion
is predominantly X. No claim of sustained large-amplitude Y/Z oscillation is
made. Position-limit checks passed, shoulder minimum was .0006396 rad, first-link
box/base-top clearance >=.040000 m, and settled minimum joint-position margin
was .4674 rad. All four joints and the base moved. Commands respected physical
velocity bounds; no settled watchdog timeouts or command rejections occurred
in the selected run. Native pose/TF and raw measurement evidence is preserved.

The direct **permanent production launch**, without diagnostic prefixes or
native/raw-stream capture, ran for 60 s: mean/max physical tracking distance
error .06785/.21531 m, pointing 7.9817/16.0670°, real-time factor .4717, zero
settled watchdog timeouts. Its checks also failed physical wrist speed
(1.56586 rad/s) and performance; distance/pointing, motion, position and command
limits passed. The observer returned nonzero as intended. These measurements
do not certify a hard physical velocity ceiling.

[Target trajectories](../results/validation/saturated_trajectories.png) and
[speed/tracking-error plots](../results/validation/saturated_tracking.png) accompany
[machine-readable comparison](../results/validation/saturated_comparison.json).

## Control and estimation diagnostics — read-only

Joint speed overshoot is real. In the stressed run, measured wrist speed peaked
at 3.11290 rad/s and timestamped position differences also peaked at 3.11290.
Gentle-candidate peaks were 4.81501 reported / 4.26442 finite-difference wrist
speed; missing samples reduce the finite-difference peak. Ordinary active PID
reference slopes stayed within `[1,1,1,1.5] rad/s`, whereas measured speeds exceeded
them. Reset/hold transitions can additionally jump a reference to measured pose;
one shoulder reference slope was 1.36895 rad/s across such a transition.
The source timestamps, commands, reference positions, physical measurements
and lag-correlation grids are in `saturated_diagnostics.json` and actuation traces.
The violations also occur after startup, so initialization alone is insufficient.
For the stressed wrist, command/measurement correlation was .632 at zero delay
and .780 at 50 ms. At its 3.11290 rad/s peak (t=13.046 s), the latest recorded
command was .20979 rad/s and reference led position by .02879 rad. This supports
lag/catch-up dynamics rather than an oversized instantaneous velocity command.

The configured Gazebo PID mode outputs effort from position error; it is not
a measured-velocity regulator. This is confirmed by the
[Gazebo 8 JointPositionController implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/joint_position_controller/JointPositionController.cc).
Bounded integrated reference motion therefore does not by itself enforce a hard
physical speed bound under PID lag, gravity, coupling and discontinuous reset
references. This supports a low-level actuator/reference-transition diagnosis,
not a missing description limit or a reason to increase physical limits.

The earlier validation also missed peaks by sampling JointState only when valid
20 Hz Control telemetry arrived. The observer now checks all received raw joint
velocities when raw capture is enabled. In the selected run, telemetry samples
suggested a peak of 1.500000003 rad/s, but raw states exposed 1.565835643. The
original result is retained; comparison/diagnostics explicitly mark raw speed
failure. New reproduction runs use the corrected check. No tolerance was relaxed.
Received joint data still have DDS/processing gaps, so reported peaks are captured
peaks; a still faster unsampled physical peak cannot be ruled out.

For the selected run, aligned wheel/world odometry differed by mean/max
`.1408/.2683 m` and `7.68/14.61°`. The static spawn transform is included in this
comparison; robot dimensions supply radius .15 m and track width .68 m, consistent
with wheel centers/axes and configured drive parameters. Gazebo DiffDrive
[integrates wheel joint positions](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/diff_drive/DiffDrive.cc),
rather than publishing physical ground-truth pose. Four-wheel skid/slip and
effective drive-model mismatch remain consistent explanations, not separately
identified contact parameters. Changing a frame label or ignoring time checks
cannot create an independent physical yaw measurement. No active IMU was observed.

The stressed trace's seconds 5–40 contain 1,751 wheel callbacks (50.03 Hz), no
wheel rejection, and 12,857 joint callbacks (367.34 received Hz), with 7,816
joint rejections (60.8%). The nominal joint publisher runs at the 500 Hz physics
step; arrivals/processing and DDS queue loss reduce observed rates. Each individual
stream is monotonic. Joint arrival age median/p95 was 14/20 ms; lateness behind
the global posterior watermark had p95 16 ms. This is legitimate globally
interleaved data, not four thousand within-source timestamp regressions.

Timestamp-only replay of the full trace gives 10,379 late drops with zero
holdback, 2,610 at 10 ms and **zero at 20/30 ms** (19,643 accepted). This verifies
ordering feasibility for this trace, not improved EKF accuracy. The estimator
has one posterior time and predicts before corrections. Applying older observations
directly to the current posterior, or substituting only per-source checks, would
violate its assumptions. A bounded timestamp heap with ~30 ms holdback, preserved
age/future/frame checks, per-source freshness, queue limits and clock-reset
clearing is the smallest defensible next timing change. Its added latency must
be tested against Control freshness. No live filter mathematics were modified.

## Regression results and remaining limitations

- 55 targeted math/factory/configuration tests passed; two observer tests passed.
- Full functional workspace regression: **745 passed, 18 lint tests deselected**.
- Simulation full package: 144 passed, one skipped, two pre-existing lint failures
  (`test_flake8`, 490 diagnostics, and `test_pep257`). The previous package run
  had the same 490 flake8 diagnostics; the new mathematical test file passes lint.
- Bringup full package: 22 passed, one skipped. Both affected packages built.
- `git diff --check` passed. Existing DLS, control algorithms, PID gains, physical
  limits, estimation, prediction, watchdog/replay and clock handling are preserved.

The saturated numerical model and bounded live tracking behavior are verified.
Full physical-speed and performance acceptance remains red. Future work must
resolve those facts rather than relabeling them as numerical tolerance issues.
Pointing mean at 120 s is close to the original 10° mean threshold and drifts with
dead reckoning; the run has no large safety margin. End-to-end asynchronous loop
repeatability and sustained Y/Z amplitude are limited. Initial trace shutdowns
lost target-state output to duplicate interrupts; the logger and observer cleanup
were corrected, and all final comparison traces are complete. Failed candidate
logs are retained. No model clips position/velocity or hides saturation events.

## Reproduction and delivery

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_simulation robot_bringup
source install/setup.bash
export ROS_DOMAIN_ID=160 GZ_PARTITION=saturated_tracking GZ_IP=127.0.0.1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

ros2 launch robot_bringup closed_loop.launch.py \
  config:=config/reactive_tracking_saturated.yaml strategy_config:=config/whole_body.yaml \
  enable_actuation:=true visualize_tracking:=true gui:=false
```

In another sourced terminal with the same domain and partition:

```bash
gz sim -g --force-version 8
rviz2 -d install/robot_bringup/share/robot_bringup/config/tracking_axes.rviz \
  --ros-args -p use_sim_time:=true
```

Use separate terminals for the viewers. For baseline/low damping, replace
`config` with `config/reactive_tracking.yaml` or
`config/reactive_tracking_low_damping.yaml`; stop the previous run first.

```bash
python3 tools/validation/saturated_dynamics_probe.py --output /tmp/saturated_math
python3 tools/validation/v2_tracking_gazebo.py --case v2 \
  --config config/reactive_tracking_saturated.yaml --physical-ground-truth \
  --shoulder-posture --target-trace --duration 120 --wall-timeout 360 \
  --output /tmp/saturated_live
python3 tools/validation/v2_tracking_gazebo.py --case v2 \
  --config config/reactive_tracking_saturated.yaml --physical-ground-truth \
  --duration 60 --wall-timeout 240 --output /tmp/saturated_production
python3 tools/validation/saturated_diagnostics.py
MPLCONFIGDIR=/tmp/saturated_mpl python3 tools/validation/analyze_saturated_milestone.py
python3 -m pytest src --import-mode=importlib -p no:launch_testing -p no:launch_ros \
  -q -k 'not flake8 and not pep257 and not copyright'
```

Live validators intentionally exit nonzero for the measured speed/performance
failures and preserve results. Diagnostics/plot commands above reproduce analysis
of the saved workspace captures; their paths are explicit in the scripts. Raw
evidence is in `results/validation/saturated_*`, with numerical cases, production
snapshots, native poses, exact target states, TF inputs, all received joint speeds,
commands/references, estimator order and comparison figures.

### Task-specific source/document changes

| File | Purpose |
| --- | --- |
| src/robot_simulation/robot_simulation/dynamics/target/fight_flight.py | Shared radial law selection, finite saturated coefficients; preserve old law |
| src/robot_simulation/robot_simulation/factory/simulation_factory.py | Register the additional dynamics selector |
| src/robot_simulation/config/scenarios/target_geometry.yaml | Inherit visuals/physics without obsolete model parameters |
| src/robot_simulation/config/scenarios/target_saturated.yaml | Selected force, length, damping, distance and epsilon |
| src/robot_simulation/config/scenarios/reactive_target_saturated.yaml | Separate inherited TF experiment and reproducible 3D initial conditions |
| src/robot_bringup/config/reactive_tracking_saturated.yaml | Select that experiment with the existing pipeline |
| src/robot_simulation/test/test_saturated_fight_flight.py | Mathematical/compatibility/integration verification |
| src/robot_simulation/test/test_configuration.py | Verify model selection and shared resource inheritance |
| tools/validation/v2_tracking_gazebo.py | Reuse permanent-launch observer; exact traces, raw feedback and reliable shutdown |
| tools/validation/shoulder_experiment_metrics.py | Extend independent physical metrics to XYZ, center and raw speed peaks |
| tools/validation/test_shoulder_experiment_metrics.py | Verify raw velocity peak retention |
| tools/validation/saturated_dynamics_cases.yaml | External standalone sweep parameters and initial/reference inputs |
| tools/validation/saturated_dynamics_probe.py | Numerical sweeps using the existing factory/RK4 engine |
| tools/validation/target_dynamics_trace.py | Read-only exact target state, acceleration and TF input |
| tools/validation/actuation_dynamics_trace.py | Read-only emitted position-reference source timestamps |
| tools/validation/saturated_diagnostics.py | Joint/PID, odometry and ordering analysis/replay |
| tools/validation/analyze_saturated_milestone.py | Reproducible comparison and scientific figures |
| docs/saturated_fight_flight_milestone.md | Results, scope, limits and reproduction |
| results/validation/saturated_modified_files.txt | Exact change manifest |

The recommended next milestone is **actuator physical-speed enforcement and
state-timing/localization reliability**: distinguish regular integration from
reset jumps, validate a gravity-preserving measured-velocity/braking strategy,
then introduce the bounded ordering buffer and an independent yaw/pose source.
Keep the saturated scenario as the bounded regression fixture. No commit or push
was made; previous uncommitted work and evidence were preserved.
