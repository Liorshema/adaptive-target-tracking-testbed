# Control Compute Only milestone — 2026-10-13

Permanent simulation robot assembly and raised-arm holding have since been
implemented. See [gazebo_arm_hold.md](gazebo_arm_hold.md) for current launch,
configuration and sustained physics validation. The 208-cycle results below
remain the recorded evidence from the original Control milestone.

The baseline computes finite six-component raw and constrained velocities from
live Gazebo feedback, with actuator publication disabled. It rejects stale,
future, malformed and frame-incompatible input, and recovers after feedback
returns or the simulation clock resets. Verification is a short integration
run, not long-term physical tracking or actuator-execution certification.

## Architecture and current ownership

```text
Gazebo DiffDrive / JointStatePublisher → wheel odometry / joint states ┐
Gazebo clock → robot_simulation → noisy target position              ├→ Estimation
                                                                    ┘
Estimation robot_state_full ──────────────────────────────────────────→ Control
Estimation target_state_full → Prediction → timestamped trajectory ───→ Control
Control: adapters → validation + TF → kinematics → reference → DLS → safety → log
```

| Package | Actual responsibility and status |
| --- | --- |
| robot_description | Authoritative physical YAML and Xacro; current URDF converted to SDF in verification. |
| robot_models | Reusable transformations, base/arm/whole-body kinematics and Jacobians, reference geometry and limit contracts. Existing numerical and behavior tests pass. |
| robot_interfaces | Full stamped state and trajectory message contracts; built and used in live verification. |
| robot_estimation | Robot EKF and target KF; real sensor transport, named arm-joint selection, clock recovery, and full-state publication verified. |
| robot_prediction | IMM baseline, residual correction and performance monitoring; publishes a live future trajectory. Clock recovery verified. Advanced predictor families are outside this milestone. |
| robot_control | Pure computation/factory/safety layers plus Compute Only ROS node; unit, package and live integration checks pass. |
| robot_simulation | Numerical target dynamics, noisy measurements, world generation and Gazebo integration. Normal launch now includes the configured robot, permanent joint holding and wheel/joint feedback bridges. |
| robot_bringup | Existing model factory and skeleton. Full system launch/spawn/bridge orchestration remains the next milestone. |

README mentions robot_perception and robot_experiments, but neither package is
currently present. Synthetic target measurements belong to robot_simulation.

Robot state is `[x,y,yaw,v,omega,q1..q4,dq1..dq4]` (13 components), with a full
row-major 13×13 covariance and acquisition timestamp. Control adapts this to
seven configuration coordinates and six generalized velocities
`[v,omega,dq1,dq2,dq3,dq4]`. Target state is `[x,y,z,vx,vy,vz]`.
Prediction `header.stamp` is the generating estimate time; its sample timestamps
are absolute future times and strictly increase after that reference time.

ControlFactory builds a bare SerialArmModel, then applies `T_E_T` from robot.yaml
once through TrackingFrameKinematics. Pose is `T_W_T = T_W_E T_E_T`; the linear
Jacobian is shifted by `-skew(R_W_E p_E_T) J_angular`. Bringup's older model
factory instead embeds this mount in FixedMountModel. Neither path currently
calls the other. A future composer must choose one ownership path to avoid
applying the mount twice.

Reference selection chooses the first sample at or after `now + lookahead`.
The radial policy selects a point on the configured sphere, and the pose
generator constructs a target-facing frame. The Jacobian controller uses
damped least squares on the configured task axes; the default task controls
position. Reference feedforward remains zero at this baseline. Safety intersects
whole-body velocity and one-step joint-position bounds before projecting the
command. Joint physical limits come from robot.yaml; base operating limits are
explicit examples in control.yaml, not inferred from wheel geometry.

## Confirmed blockers and repairs

- Shipped YAML disabled computation and lacked required base velocity bounds.
  It now enables computation, retains safety, and supplies explicit example
  bounds of ±0.5 m/s and ±1 rad/s. `publish_commands` stays false; requesting
  true requires explicit joint identities and a body frame. The default remains
  false. Production opt-in actuation is documented in [Closed-Loop Tracking v1](closed_loop_tracking_v1.md).
- Estimation publishes successive sensor corrections at equal timestamps.
  Control now retains the latest same-time robot estimate, rejects older ones,
  and throttles rejection logs. Equal-time input still undergoes normal
  acquisition-time freshness checks; receipt does not refresh its timestamp.
- Older sensor callbacks previously corrected the current filter with an older
  timestamp even though prediction returned early. They now return before
  filter mutation. Future packets cannot poison timestamp caches.
- JointState arrays previously required exactly four entries in assumed order.
  Gazebo feedback in this run contains all eight arm/wheel joints. Estimation
  selects configured arm names, ignores other joints and preserves model order.
  Missing/duplicate names, missing velocities and nonfinite values are rejected
  before prediction or correction. No velocities are fabricated.
- Full robot publication now requires validated wheel and joint observations,
  each within its configured age bound. Wheel or optional IMU callbacks cannot
  advertise never-observed or indefinitely stale arm states as a full estimate.
  The legacy vector topic remains available. Source age limits and downstream
  aggregate-state age limits have separate meanings: a cached full estimate can
  remain usable for the downstream grace period after source publication stops.
- Estimation, Prediction and Control clear their state/history when ROS time
  jumps backward or its source changes. Rejected delayed packets from the former
  epoch do not lock out lower valid timestamps.
- TF uses the prediction reference timestamp, rather than unrelated latest TF.
  Position transforms as `Rp+t`; velocity as `Rv`; sample times are unchanged.
  This milestone assumes fixed inertial world/odom alignment. A moving-frame
  velocity model would need additional frame-motion terms. Missing TF blocks
  computation rather than relabeling coordinates.
- Input validation checks horizon coverage at `now + lookahead`. Safety uses
  at least the nominal control interval and the actual elapsed interval for a
  delayed cycle, and does not recompute at a repeated clock time.
- Runtime dependencies are declared in robot_control/package.xml.

The earlier historical clock observations do not prove a clock mismatch.
Current rejection logs include simultaneous `now`, robot and prediction times.
The isolated live run directly verified synchronized acquisition/freshness.

## Verification and reproduction

Run from the workspace root in a shell with ROS Jazzy installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robot_simulation robot_control robot_estimation robot_prediction
source install/setup.bash
colcon test --packages-select robot_control robot_estimation robot_prediction
colcon test-result --verbose
python3 tools/validation/compute_only_gazebo.py
```

The normal simulation launch builds its initial world and robot from the real description.
The manual verifier invokes that launch with its permanent holding and feedback bridges, and
runs the actual Estimation, Prediction and Control nodes. It uses ROS domain 79
and Gazebo partition `control-compute-verification`; reserve them for this test.
It writes logs and result.json under `/tmp/control-runtime`, checks both output
vectors and all velocity/next-joint-position limits on every successful cycle,
withdraws joint feedback while wheels continue, checks inhibition/recovery,
then resets Gazebo and verifies recovery and raised-arm settling again. It also
measures 30 settled simulated seconds of arm holding. It cleans up its own processes.
This is a manual verification fixture, not the future production Bringup.

The robot fixture starts with base_link at z=0.25, so its fixed TF is
`T_world_odom = translation(0,0,0.25)`. Identity TF is appropriate only when the
actual coordinate origins coincide; use the transform matching the real spawn.
Permanent Gazebo position-controller plugins configured in robot_simulation hold the arm against gravity.
Control never sends a velocity to a position topic or publishes an actuator
command. Plugin behavior follows the official
[DiffDrive](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html),
[JointStatePublisher](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointStatePublisher.html)
and [JointPositionController](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html)
interfaces.

For an already running system with the correct topics, clock and TF:

```bash
ros2 run robot_control control_node --ros-args \
  --params-file src/robot_control/config/control.yaml -p use_sim_time:=true
```

Verification results:

- All seven selected/dependency packages build successfully.
- robot_control: **439 passed, 1 skipped**, including its style/doc checks.
- robot_estimation: **52 passed**.
- Workspace functional tests: **585 passed**, 18 lint/copyright checks excluded.
  The whole functional suite was collected with `--import-mode=importlib` and
  `-p no:launch_testing -p no:launch_ros` to avoid generated test-name collisions.
- Six existing package lint failures remain: Bringup Flake8, Description
  Flake8, Prediction Flake8/PEP257, Simulation Flake8/PEP257. Prediction's two
  failures were also reproduced on an exported copy of committed HEAD. These
  checks must not be reported as a fully green workspace suite.
- Live evidence is saved beside this report in validation/control_compute_only_result.json
  and validation/control_compute_only_runtime.log: **208 safety-checked cycles**,
  including lost-joint feedback and full-reset recovery. Only `/parameter_events` is
  advertised by Control; actuator command publication is disabled.

The original temporary fixture dynamically spawned its robot; a full reset
removed it. The permanent launch now includes its robot in the initial world,
and reset retains it. The verifier checks scene persistence and successful
Control/arm recovery. Without robot feedback, Control correctly waits.

## Next milestone boundaries

Bringup should compose package configuration, robot spawning, bridges, correct
world/odom TF, common simulation time and node lifecycle/respawn. It must keep
the tracking mount owned by one construction path and retain consistent named
joint order. Do not move robot mathematics or Gazebo physics into Bringup.

The permanent `robot_simulation/config/gazebo/joint_position_hold.yaml` now owns
holding targets and PID configuration, including validated shoulder/wrist
overrides. Holding under gravity is verified in [gazebo_arm_hold.md](gazebo_arm_hold.md).
The velocity-to-actuator interface must
be designed before enabling high-level command publication. No complete
Bringup, velocity command bridge, MPC or additional adaptive algorithm was added.
