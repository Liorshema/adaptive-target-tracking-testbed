# Permanent Gazebo arm holding

The normal simulation launch now loads the mobile robot from robot_description,
adds one Gazebo effort-based JointPositionController per arm joint, and keeps a
configured raised posture under gravity. Robot geometry, masses, axes, limits
and effort ratings are unchanged. No Gazebo logic was added to robot_control;
its `publish_commands` remains false.

## Launch

From `/home/user/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robot_simulation
source install/setup.bash
ros2 launch robot_simulation simulation.launch.py
```

For a server without a GUI, use the same launch and configuration:

```bash
ros2 launch robot_simulation simulation.launch.py gui:=false
```

The robot is included in the initial world, together with the target, before
physics starts. No manual robot spawning or position-command terminal is
required. The simulation also bridges `/joint_states` and `/wheel/odometry`.
There is no ROS actuator command bridge. The built-in DiffDrive receives no
high-level Control commands; arm holding is an independent simulation actuator.

## Configuration

| File / parameter | Default and meaning |
| --- | --- |
| config/launch.yaml → robot.enabled | true; include the physical robot. False retains target-only simulation. Older configs without the section remain target-only. |
| robot.config | config/gazebo/robot.yaml; deployment configuration path, resolved with the existing paths.base convention. |
| robot.joint_position_hold | config/gazebo/joint_position_hold.yaml; holding configuration path. |
| robot.yaml → description.package / xacro / config | robot_description; urdf/robot.urdf.xacro; config/robot/robot.yaml. Physical parameters come from these existing sources. |
| entity_name / pose | robot; xyz/rpy `[0,0,0.25,0,0,0]`, matching the existing flat-ground wheel geometry. |
| joint_name_template | `{key}_joint`; resolves each authoritative manipulator.joints key to a rendered joint. All names are checked against actual SDF. |
| joint_states.topic | /joint_states. Gazebo 8.11 publishes at physics-update frequency; no unsupported rate setting is exposed. |
| differential_drive.enabled | true; enable the existing base feedback mechanism. |
| left_joints / right_joints | External lists of front/rear wheel joint names. Rendered joints must be unique, exist, and be separate from the arm. |
| differential_drive.rate_hz | 50; wheel odometry frequency. Wheel radius and track width derive from robot_description. |
| differential_drive.topic / odom_topic | /simulation/robot/cmd_vel; /wheel/odometry. The velocity input is not bridged from ROS. |
| frame_id / child_frame_id | odom; base_link. |
| joint_position_hold.enabled | true; disable before introducing another arm actuator owner. |
| pid | Common P=100, I=5, D=5, i_min=-1, i_max=1, cmd_offset=0. |
| positions | Startup targets in description order: base_yaw=0, shoulder=0.523599, elbow=-0.4, wrist=0 rad. |
| joint_overrides.shoulder.pid.p_gain | 200; added stiffness for the shoulder's gravity load. |
| joint_overrides.wrist.pid.d_gain | 0.5; prevents the light wrist's velocity-limit oscillation with common D=5 at the 1 ms physics step. |
| launch argument gui | Optional true/false override; default follows gazebo.gui in launch.yaml. |

`cmd_min` and `cmd_max` are generated as ±each joint's authoritative effort
limit: ±20 Nm for base_yaw/shoulder/elbow and ±10 Nm for wrist. They cannot be
overridden by the PID configuration. Targets must cover exactly the description
arm joints and remain within its position limits. Invalid/nonfinite gains,
unknown override keys, missing joints, duplicate systems and competing arm/drive
actuators fail before launching.

The startup target is a physical PID command, not an instantaneous teleport to
a joint angle. The joints lift into the raised posture during startup and then
hold it; the measured settled interval begins at five simulated seconds. This
uses the same effort-based mechanism as the earlier verified fixture. Gazebo's
[JointPositionController source](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/src/systems/joint_position_controller/JointPositionController.cc)
confirms that initial_position initializes the target and PID commands effort.

## Verification

The saved verifier now invokes the installed **normal simulation launch**. It
does not generate alternative holding plugins, spawn a substitute robot, or
provide an extra feedback bridge. It adds only observation/Estimation/Prediction/
Compute Only nodes and the appropriate world/odom TF for integration checks.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 docs/validation/compute_only_gazebo.py
```

`--gui` exercises the default GUI launch without a gui override. GUI startup
was exercised and delivered real feedback, but that session exited cleanly at
about 28 simulated seconds. Quantitative sustained verification completed with
the same permanent launch using `gui:=false` and the final tuned configuration.

Recorded results in [validation/arm_hold_result.json](validation/arm_hold_result.json)
and [validation/arm_hold_runtime.log](validation/arm_hold_runtime.log):

- 35 simulated seconds of running physics; 1,776 observed samples during the
  settled interval from t=5.0 through t=34.957 seconds.
- Maximum position errors `[~0, 0.03984, 0.02761, 0.00175]` rad and maximum
  speeds `[~0, 0.000143, 0.001387, 0.000111]` rad/s across the four joints.
- Tracking-frame height inferred from measured joints and existing whole-body
  kinematics stayed at least 0.39277 m above the base origin, above its top.
- 523 successful six-component raw/safe control calculations, each checked for
  finite values, velocity bounds and one-step joint-position bounds.
- Removing joint feedback while wheel feedback continued blocked computation;
  restoration recovered it. A full world reset retained the robot, recovered
  Control, and returned the arm to a settled raised posture. At t=5.957 after
  reset, joint speeds were at most 0.001322 rad/s and inferred tracking height
  was 0.39340 m above the base origin.
- Control advertised only /parameter_events and publish_commands=false.
- Eight packages built; generated world passed `gz sdf -k` validation.
- 21 focused builder/world tests passed; all 605 workspace functional tests
  passed. Control's complete package suite remains 439 passed / 1 skipped.
- Simulation's existing Flake8/PEP257 checks still fail on older files; the new
  robot builder and its tests pass both checks independently. Other previously
  documented workspace lint failures remain outside this change.

The initial common fixture gains were insufficient for a sustained raised
posture: the wrist oscillated at 1.5 rad/s and shoulder error reached 0.081 rad.
Per-joint measurements isolated those effects. The YAML-only overrides above
removed the oscillation and reduced shoulder error; no constraints, gravity,
robot geometry or masses were weakened to pass verification. These results
cover the current robot and 1 ms physics step, not every alternative robot or
terrain configuration.

## Files changed in this milestone

- Added robot_simulation/gazebo/robot_builder.py and config/gazebo/robot.yaml.
- Populated config/gazebo/joint_position_hold.yaml.
- Updated config/launch.yaml and launch/simulation.launch.py for robot assembly,
  feedback bridges and GUI override.
- Extended robot_simulation/gazebo/world_builder.py to accept additional initial-world models.
- Updated setup.py to install nested Gazebo YAML and package.xml dependencies.
- Added test/test_robot_builder.py and extended test/test_world_builder.py.
- Updated docs/validation/compute_only_gazebo.py to verify permanent holding.
- Added this report and arm_hold_result.json / arm_hold_runtime.log; updated
  Control milestone documentation and the implementation plan.

Paths above are relative to src/robot_simulation unless prefixed with docs.
robot_control, robot_estimation, robot_prediction and the physical description
were not changed by this milestone.

Future Bringup should compose the simulation launch once and avoid spawning a
second copy of its robot. It can override or disable the robot/holding through
configuration while establishing the correct TF and sensor/actuator ownership.
Replacing position holding with generalized-velocity execution still requires
an explicit actuator interface; do not send velocity values to cmd_pos.
Closed-Loop Tracking v1 now provides that bounded adapter and a production
Bringup launch; see [the milestone report](closed_loop_tracking_v1.md).
