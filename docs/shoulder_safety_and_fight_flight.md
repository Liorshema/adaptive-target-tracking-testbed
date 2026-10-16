# Shoulder safety and Fight/Flight experiment

The authoritative shoulder minimum is now **0.0 rad**. The new low-damping
Fight/Flight experiment is separate from the unchanged reactive baseline.
Normal V2 passed its original live acceptance checks. The requested low-damping
experiment is available, but its target escapes under the unchanged centering
field; it is not a stable tracking milestone.

## Geometry and limit ownership

`robot_description/urdf/manipulator.xacro` places the shoulder at
`body.height/2 + manipulator.base.height = 0.18 m` above `base_link`.
The shoulder origin has zero RPY, its configured axis is `[0,-1,0]`, and
`arm_link_1` extends along +X. Its visual and collision geometry are identical:
a `[0.4,0.08,0.08] m` box centered at `[0.2,0,0]` in the link frame.
At shoulder zero its centerline is horizontal. Positive shoulder rotation raises
the distal end: its direction is `[cos(q),0,sin(q)]` before base yaw.
The base collision box is `[1.0,0.6,0.2] m`, centered at `base_link`.
At zero, the first-link collision bottom is at z=.14 m, above the base's
top z=.10 m. For the permitted nonnegative shoulder range its lowest corner
stays at least .04 m above that plane in base coordinates.

Only `robot_description/config/robot/robot.yaml` owns the new physical bound.
Shoulder upper limit, axis, velocity, effort and damping are unchanged.

- Xacro emits that bound into URDF; `build_robot_model` converts it into the
  Gazebo joint's SDF `<axis><limit><lower>` (tested as 0).
- `load_robot_model_parameters` and the shared description adapter expose the
  same ordered `joint_limits.lower` to the model/control configuration.
  FK computes geometry without clipping measurements; feasibility is enforced
  by the description-derived bounds, rather than an independent FK constant.
- `project_control_configuration` supplies those bounds to the safety pipeline;
  the whole-body optimizer uses that pipeline's next-step feasible bounds.
  At shoulder zero, downward shoulder velocity is infeasible. Its default
  midpoint posture preference follows the new interval automatically.
- `VelocityActuationNode` and `PositionReference` load the description bounds,
  clamp integrated references, and retain their existing watchdog/identity/age
  checks. No second limit was added to Control or Simulation configuration.
- Gazebo starts at joint zero, which remains valid. The existing effort PID's
  raised shoulder target `.523599 rad` remains unchanged. Negative startup
  targets are rejected by the builder before launch.

This constrains the first link. It does **not** implement full self-collision
avoidance for the elbow, wrist, pedestal, wheels or other parts of the arm.

## Experiment inheritance and actual centering field

The existing `robot_bringup/config/reactive_tracking.yaml` remains the baseline.
The new selection chain is:

```text
robot_bringup/config/reactive_tracking_low_damping.yaml
  extends reactive_tracking.yaml
  selects robot_simulation/config/scenarios/reactive_target_low_damping.yaml
    extends reactive_target.yaml
    selects config/scenarios/target_fight_flight_low_damping.yaml
      extends ../target.yaml
      overrides only target.dynamics.parameters.damping: 0.15
```

The resolved parameters are alpha=1, preferred_distance=1, damping=.15,
distance_epsilon=1e-6. Baseline damping remains .5. Appearance, target physics,
initial position `[2,0,1]`, initial velocity `[0,0,0]`, integration and sensor
configuration are inherited, not duplicated.

Both reactive selections inherit
`simulation.environment_config: config/scenarios/environment.yaml` through
`moving_target.yaml`. This is the **actual external centering field**:

```yaml
environment:
  acceleration_fields:
  - type: linear_restoring
    enabled: true
    parameters:
      center: [2.2, 0.0, 0.85]
      gain_matrix: [[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]]
  stochastic_disturbances: []
```

`disturbances/vector_field.py` evaluates `-K @ (position - center)`.
The generic `config/environment.yaml` instead contains center `[0,0,1]` and
gain `2*I`; that file is **not selected** by these reactive experiments.
Neither field was edited.

Both experiments retain `tracking.source: tf`, reference frame `world`, tracking
frame `tracking_frame`, max_age=.2 s and future_tolerance=.01 s.
`SimulationNode._read_tracking_position` calls the existing
`TrackingFrameProvider.position_world()` and refreshes the integration context
on each step. Bringup's broadcaster uses live measured joints and wheel odometry;
the fixed world→odom transform completes the TF chain. There is no static
fallback in this branch. Live captures recorded 827/775 tracking transforms,
with tracking-position excursions .881/18.950 m, respectively.
As before, this TF is measured-feedback FK and inherits wheel-odometry drift;
Gazebo ground truth is observational and is not fed into the interaction/control.

## Actual comparison

The two reactive captures use the same production launch, optimizer, description,
PID settings, initial conditions and field. Each runs for 40 simulated seconds
after target initialization; metrics use elapsed seconds 10–40. Startup is
included separately in posture checks. This is one run per comparison case;
startup phases and callback scheduling differ slightly.

Tracking metrics below come directly from native Gazebo world/model/link poses,
the authoritative wrist→tracking mount and synchronized published target truth.
They include saturated/stopped-control periods, independently of valid Control
telemetry. Speed is finite-difference speed from raw target positions; displacement
is maximum displacement from the target's initial position during evaluation.

| Measured quantity | Reactive baseline, damping .5 | Experiment, damping .15 |
| --- | ---: | ---: |
| Mean target speed, m/s | .1498 | 1.0729e8 |
| Maximum target speed, m/s | .2731 | 2.0019e9 |
| Maximum target displacement, m | .5116 | 3.1593e9 |
| Target path length, m | 4.4857 | 3.1593e9 |
| Mean absolute distance error, m | .09674 | 1.7587e8 |
| Maximum absolute distance error, m | .22161 | 3.1593e9 |
| Mean pointing error, degrees | 4.5088 | 24.2553 |
| Maximum pointing error, degrees | 13.8269 | 51.8154 |
| Minimum first-link collision clearance above base top, m | .040002 | .040000 |
| Minimum received shoulder position, rad | .0006396 | -.00003314 |

The enormous experiment values are the measured runaway, not a stable-speed
claim. Its target exceeds 1 m displacement at elapsed 2.488 s, 10 m at 8.010 s,
100 m at 12.692 s and 1000 m at 16.434 s. At large separation the outward
Fight/Flight term has gain alpha=1, whereas restoring gain is .5. With a
bounded-speed robot, the leading target acceleration is approximately
`.5*position - damping*velocity`; damping alone does not guarantee containment.
The lower damping lets this run escape the nearby tracked trajectory. Changing
the centering field or introducing speed caps would be a separate experiment.

All captured first-link poses remained horizontal or raised relative to the
base, within the unchanged numerical tolerance. Baseline minimum elevation was
.009818 rad; experiment minimum was -5.34e-13 rad (roundoff at horizontal).
The experiment's reported joint undershoot of 3.31e-5 rad is a tiny physical
hard-stop/measurement numerical deviation, below the existing .001 rad tolerance.
World elevation reached -1.15e-6 rad because the mobile base itself tilted
slightly. There was no first-link/base-box penetration in the captured geometry.
The comparisons cover 3,109/3,118 native physical poses and 19,209/18,494 joint
messages, including startup.

Safety checks must be distinguished from physical speed tracking. Position and
command-velocity bounds passed, with zero settled watchdog timeouts in both.
The reactive baseline had a measured wrist peak **1.51569 rad/s**, above its
1.5 rad/s limit and the unchanged .001 tolerance, so that live validator returned
failure. The escaping experiment had measured peaks
`[1.1057,1.2388,2.4888,7.6230] rad/s` against `[1,1,1,1.5]` limits. Bounded velocity
commands feeding effort PIDs do not guarantee bounded physical velocity in this
stress case. No PID gains or safety thresholds were relaxed to hide this result.

The normal independent-target V2 launch passed **all original acceptance checks**
after the shoulder change: mean/max distance error .09576/.22352 m,
mean/max pointing 5.6459/14.4088 degrees, real-time factor .7506, base excursion
.9987 m, all four joints moving, all measured joint-speed limits passing and zero
settled watchdog timeouts. Independent native-pose metrics were .09493 m and
5.7113 degrees mean. Startup invalid/future-clock inputs were safely inhibited.

## Tests, evidence and reproduction

Three affected packages built successfully. Targeted unit/integration tests:
**85 passed**. Final functional regression:
**722 passed, 18 lint tests deselected**. The native pose/metric decoder tests:
**2 passed**. Bringup's full package suite: **22 passed, one copyright skip**.
Description's full package suite: 12 passed, one skipped, one existing flake8
failure. Simulation's full package suite: 121 passed, one skipped, existing
flake8 and pep257 failures (490 flake8 diagnostics). The earlier root-level full
workspace invocation returned 723 passed, six skipped and ten lint failures:
flake8/pep257 in Bringup, Description, Models, Prediction and Simulation; those
root-invoked linters scan the workspace. The introduced Bringup import-order
issue was corrected and its package suite rerun successfully. `git diff --check`
passed. Control algorithms, gains, estimation, prediction and actuator safety
implementation were preserved.

Raw evidence and results are in `docs/validation/shoulder_reactive_verified`,
`shoulder_reactive_low_damping`, and `shoulder_normal_v2_final`.
`shoulder_fight_flight_comparison.json` recomputes the reactive comparison from
raw streams. Initial observation attempts had native JSON timestamp/serialization
or optional-TF decoding errors; their raw captures and failure records are
preserved. The low-damping capture completed 40 s but its original observer
exited during postprocessing; its complete raw data were reprocessed by the
corrected independent analyzer. The final normal V2 observer exited successfully.

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=147 GZ_PARTITION=shoulder_reactive GZ_IP=127.0.0.1

# Existing reactive baseline, including the new authoritative physical limit:
ros2 launch robot_bringup closed_loop.launch.py \
  config:=config/reactive_tracking.yaml strategy_config:=config/whole_body.yaml \
  enable_actuation:=true visualize_tracking:=true gui:=false

# Separate experiment (stop the baseline first):
ros2 launch robot_bringup closed_loop.launch.py \
  config:=config/reactive_tracking_low_damping.yaml strategy_config:=config/whole_body.yaml \
  enable_actuation:=true visualize_tracking:=true gui:=false
```

In another sourced terminal with the same ROS_DOMAIN_ID and GZ_PARTITION:

```bash
gz sim -g --force-version 8
rviz2 -d install/robot_bringup/share/robot_bringup/config/tracking_axes.rviz \
  --ros-args -p use_sim_time:=true
```

Use separate terminals for the two viewers. Headless reproduction and analysis:

```bash
python3 docs/validation/v2_tracking_gazebo.py --case reactive \
  --config config/reactive_tracking.yaml --physical-ground-truth --shoulder-posture \
  --duration 40 --output /tmp/shoulder_baseline
python3 docs/validation/v2_tracking_gazebo.py --case reactive \
  --config config/reactive_tracking_low_damping.yaml --physical-ground-truth --shoulder-posture \
  --duration 40 --output /tmp/shoulder_experiment
python3 docs/validation/compare_shoulder_experiments.py \
  --baseline /tmp/shoulder_baseline --experiment /tmp/shoulder_experiment \
  --output /tmp/shoulder_comparison.json
python3 docs/validation/v2_tracking_gazebo.py --case v2 \
  --physical-ground-truth --shoulder-posture --output /tmp/shoulder_normal
python3 -m pytest src --import-mode=importlib -p no:launch_testing -p no:launch_ros \
  -q -k 'not flake8 and not pep257 and not copyright'
python3 -m pytest docs/validation/test_shoulder_experiment_metrics.py \
  -p no:launch_testing -p no:launch_ros -q
```

The validator returns nonzero on measured-speed violations and retains its
metrics; the analyzer reports these failures rather than treating the experiment
as passing. The exact task-specific edited-file list is
[shoulder_fight_flight_modified_files.txt](validation/shoulder_fight_flight_modified_files.txt).
Previous uncommitted work and evidence were preserved. No commit or push was made.
