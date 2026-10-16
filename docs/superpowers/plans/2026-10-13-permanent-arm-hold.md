# Permanent Gazebo arm holding

**Goal:** Normal simulation launch creates the authoritative robot with active
Gazebo JointPositionController systems targeting a raised pose under gravity.
Keep Control Compute Only with publish_commands=false.

**Design:** A reusable simulation robot builder renders Xacro, converts to SDF,
validates holding targets against description limits, and adds configured PID,
feedback and differential-drive plugins. World builder accepts extra models.
Launch composes the configured robot into the initial world, so resets retain
it. Configuration owns joint name mapping, gains, startup targets, pose/topics.
JointPositionController.initial_position is a startup target; physical joints
converge under PID effort rather than changing geometric joint origins.

**User authorization:** Implement directly in the existing working workspace;
preserve prior uncommitted work. No per-change approval gates or commits.

- [x] Add failing builder/world regressions for authoritative effort/geometry,
  nondefault joint names and PID/targets, invalid targets, conflicting plugins,
  disabled holding, and model inclusion/duplicate rejection.
- [x] Implement robot_builder.py and configuration in config/gazebo; wire the
  normal simulation launch, install nested configuration, declare dependencies.
- [x] Replace temporary plugin/spawn duplication in the manual verifier with
  the permanent launch. Verify raised pose, steady joint positions/velocities
  under gravity, Control safety and source-loss/reset recovery through real ROS.
- [x] Run functional/control suites and package build; review changes and
  document modified files, all parameters, observed stability and exact launch.

## Review focus

- Hold setpoints must remain within authoritative limits; effort saturation
  must respect the same description, with one controller per arm joint.
- Unknown/missing configured joints and nonfinite PID/pose parameters fail
  before launching; disabled robot remains compatible with target-only setups.
- Robot is present in normal initial world; full resets do not lose it.
- Gazebo plugins and command interfaces remain in Simulation; no high-level
  actuator publishing or competing arm controller is added.
- Verify measured startup/settling instead of claiming initial_position
  instantaneously sets joint angles. Verify actual physics with gravity enabled.

## Evidence and decisions

- Initial builder/world RED run: 10 failures and 1 existing pass. Final focused
  suite: 21 passed, including ownership and malformed override regressions.
- Eight package builds passed; generated world validated with gz sdf -k.
- Common fixture gains were reused first. Actual sustained physics found wrist
  oscillation at1.5rad/s and shoulder error0.081rad. A standalone named-joint
  probe confirmed it. YAML overrides shoulderP200 and wristD0.5 reduced max
  settled speed to0.001387rad/s and max position error to0.039841rad.
- Normal default GUI launch initialized and delivered data, then Gazebo exited
  cleanly at approximately28simseconds. Long measurement used the normal launch
  with gui:=false, not an alternative robot/actuation fixture.
- Final recorded run:523safety-checked Control cycles,30settled simulated seconds
  under gravity, lost-joint-feedback inhibition/recovery, model persistence on
  full reset, and raised settled pose after reset. Publishing remainedfalse.
- Installed Gazebo8.11 lacks JointStatePublisher update_rate support; removed
  the unsupported setting instead of presenting it as functional configuration.
- Independent review found drive/arm ownership overlap and silent override
  typo acceptance. Both were reproduced and fixed with regression tests.
- Full functional suite:605passed. Control package439passed/1skipped. Simulation
  preexisting Flake8/PEP257 failures remain; new builder/tests pass both directly.
- Source geometry and Control/Estimation/Prediction code were preserved.
