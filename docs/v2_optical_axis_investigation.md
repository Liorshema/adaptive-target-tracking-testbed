# V2 optical-axis and tracking-frame investigation

The primary cause was an **optical-axis mismatch**. You confirmed that the camera
must look along wrist +X; Control was configured for tracking-frame +Z. The
tracking frame has identity rotation relative to the wrist, so those directions
are perpendicular. V2 was reducing the error of the wrong physical camera axis.
The downward arm was commanded posture, not an uncontrolled gravity collapse.

A second issue is **wheel-odometry/world-pose disagreement**. The prior 5.64°
metric used wheel-odometry FK, not Gazebo's physical world base pose. The rigid
kinematic model and single tracking-frame mount are correct when supplied with
the actual base pose. Message-order rejection is a separate, confirmed timing
issue; it does not explain the 90° camera-axis mismatch.

**Evidence and verdict**

| Candidate | Finding |
| --- | --- |
| Wrong optical axis | Confirmed: +Z was controlled; intended camera is wrist +X |
| Incorrect mount/FK | Not reproduced: actual-base FK matches Gazebo wrist-plus-fixed-mount pose |
| Unreachable pose | Not indicated: +X task Jacobian rank 3 in tested configurations; physical tracking passes |
| Position saturation | Not indicated: supplied joints have margins >=1.274 rad; corrected run minimum margin 1.340 rad |
| Gravity/PID failure | Not indicated: the controller deliberately requested the downward +Z-looking posture; PIDs remain active |
| Estimator timing | Confirmed: ordered individual streams are globally interleaved; older joint packets are discarded |
| Base localization | Confirmed: wheel odometry differs from physical Gazebo base pose; gyro input is inactive |

The authoritative description and tracking_frame.xacro specify wrist_link,
translation [0.12,0,0], and RPY [0,0,0]. Wrist visuals extend along local +X.
tracking_frame is an empty virtual link, with no camera mesh/sensor/FOV. It is a
wrist-aligned tracking frame, not a separate ROS camera_optical_frame. Therefore,
local +Z was a possible sideways virtual camera convention, but it contradicts
the intended mounting you confirmed. No mount transform was changed.

The shared model ends at the wrist joint, and FixedMountModel applies the tracking
offset once. DistancePointingTask obtains its optical vector from the shared
Control configuration; the reference builder, optimizer, metric and visualization
now all consume tracking_forward_axis=[1,0,0]. Roll stays free; gains, task/motion
weights, PID gains, angles, effort/velocity/position limits and watchdogs are unchanged.

For your supplied joint values, the base-relative tracking transform is:

```text
[ 0.744895 -0.132707  0.653851  0.804003 ]
[ 0.099735  0.991155  0.087545  0.107649 ]
[-0.659685  0.000000  0.751542 -0.122368 ]
[ 0         0         0         1        ]
```

Its +X shaft direction has vertical component -0.660; its old +Z optical axis
has vertical component +0.752. Their angle is 90°. Position-limit margins for
base_yaw/shoulder/elbow/wrist are [3.0069,1.3813,1.3345,1.2738] rad.
Absolute historical world pose and target direction cannot be recovered from
those joint values alone: the corresponding base pose and target position were
not supplied. With a level base at z=.25, the inferred tracking height would be
.1276 m, explaining the low visible tip without a joint-limit event.

**Actual live snapshot and independent ground truth**

The new probe records native SceneBroadcaster poses. Gazebo model poses are
world-relative and link poses are model-relative. It composes:
`world→robot × robot→wrist_link × wrist→tracking_frame` and compares against FK
using synchronized measured joints. The generic Pose_V→TF bridge loses names and
stamps for this SceneBroadcaster message, so the probe retains native JSON.

At t=21.174 s in the corrected live run:

```text
World tracking position: [1.482355, -0.038408, 0.607707] m
World tracking rotation:
[ 0.952564 -0.159167 -0.259398 ]
[ 0.153574  0.987252 -0.041825 ]
[ 0.262748  0.000004  0.964864 ]
Optical +X:       [0.952564, 0.153574, 0.262748]
Target direction:[0.880442, 0.385088, 0.276637]
Target position: [2.253490, 0.298871, 0.850000] m
Angular error: 13.9508 degrees
Distance: .875850 m; radius R=1 m; signed error=-.124150 m
Joints: [.109782, .170367, .049190, .046348] rad
Margins: [3.030218, 1.399633, 1.520810, 1.523652] rad
```

This is a snapshot, not the mean. It shows a raised camera and residual yaw error.
The complete world transforms, target vectors and joint margins are in
orientation_x_corrected/summary.json and poses.json. The before snapshot at
21.150 s has intended +X error 92.29° despite controlled +Z error 13.36°.

With the actual Gazebo base pose, mean FK/physical-frame disagreement was 0.013 mm
before and numerical roundoff after; the before maximum was 1.985 mm / 0.429°
from allowed sampling skew. This confirms the mount/geometry, not an estimated
base pose. With wheel odometry instead, mean tracking-position disagreement was
.162 m before and .172 m after; mean full-frame angular disagreement was
16.63° / 14.15°. At the before 21.150 s snapshot, wheel-inferred base yaw was
.4275 rad while actual base yaw was .0378 rad.

The estimator itself remains close to wheel odometry: mean base-position
disagreement .0041/.0053 m, yaw disagreement .180/.103°, and corrected mean wrist
joint-state disagreement .0050 rad. Thus wheel dead reckoning is not physical
world localization. Skid/slip or effective drive-model mismatch is consistent
with the observed four-wheel behavior, but contact mechanics were not separately
identified or tuned. A functioning independent yaw/pose measurement is needed
for accurate world localization; accepting old joint measurements cannot remove
wheel-odometry drift.

**Measured before/after**

Two 35-second observational runs used the same deployment, target, noise seed,
physics, PID and costs; statistics use simulation t=10–35. Startup timing differs,
so these are controlled configuration comparisons rather than identical samples.

| Physical Gazebo metric | Before (+Z configured) | After (+X configured) |
| --- | ---: | ---: |
| Intended wrist +X mean error | 87.68° | 6.37° |
| Configured axis mean error | 8.01° (+Z) | 6.37° (+X) |
| Mean absolute distance error | .0856 m | .0920 m |

A separate final check through the **normal production launch**, without timing
instrumentation, evaluated physical base pose plus measured joints. It measured
mean distance **.0976 m**, mean pointing **6.95°**, maximum pointing **16.27°**,
physical base excursion **1.015 m**, and RTF **.761**. All preset checks passed,
including physical/safe velocity limits, arm/base response and zero settled
watchdog timeouts. No threshold was relaxed. The old 5.64° result remains
historical evidence for +Z and wheel-odometry inference; it is not a validation
of the intended +X camera or true world pose.

**Visualization and ownership**

![Actual Gazebo axes before and after](validation/orientation_axes.png)

`visualize_tracking:=true` enables a small ROS MarkerArray publisher in the
existing robot_bringup package and an observational Gazebo OdometryPublisher in
robot_simulation. No new visualization package or physical camera representation
was added. The [Gazebo API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html)
and [implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc)
define the 3D world-pose publisher. Its odometry is used only by visualization
and validation, never substituted into Estimation, Control or Actuation.

The live view shows a white physical tracking origin, red/green/blue XYZ axes,
gold selected optical axis, and magenta line to actual target. Faded copies show
the wheel-odometry inference, exposing localization error. Arrows are direction
vectors, not physical link geometry. The marker topic is /tracking/markers;
physical diagnostic pose is /simulation/robot/ground_truth. There is no additional
actuator or ROS TF owner. Markers expire/clear on stale/skewed inputs, reject
future/older packets and recover after a clock reset. Recovery was verified live
with 40 complete 12-marker messages in the failure test.

Configuration ownership is explicit:

- robot_description retains physical geometry and wrist→tracking transform.
- robot_control/config/control.yaml selects the intended local forward axis +X;
  the task factory and visualization receive that same setting.
- robot_simulation/config/gazebo/robot.yaml owns optional ground_truth_odometry
  (disabled normally, dimensions=3, rate=20 Hz, world/base_link, diagnostic topics).
  Simulation launch bridges it when requested; PID/DiffDrive remain unchanged.
- robot_bringup/config/tracking_visualization.yaml owns marker topic, target topic,
  5 Hz publication, .2 s age, .08 s skew, .01 s future tolerance, .3 s lifetime,
  .3/.8 m axis/optical lengths, sizes and colors. Target defaults to actual Gazebo
  target; changing target_topic to perception draws noisy measured observations.
- Tracking axes RViz configuration is installed with Bringup. Control weights,
  limits, safety timeouts, replay protection and prediction settings are unchanged.

**Estimation timing diagnosis and minimal recommendation**

The observational prefix logs inside the real EstimationNode callbacks, before
and after their existing decisions. Sensor publication is 50 Hz wheel odometry,
500 Hz joint states (2 ms physics), and no active IMU publisher. Between t=5–35,
actual callbacks were 1,501 wheel (50.0 Hz) and 13,206 joints (440.2 Hz); neither
stream regressed internally. All wheel callbacks were accepted. **5,030 joint
callbacks, about 38%, were rejected** after another source advanced the shared
filter watermark. Median/p95/max lateness behind that watermark was 6/14/20 ms;
median/p95 joint arrival age was 8/18 ms. IMU callback count was zero.

The node assumes globally nondecreasing robot measurement times. Joint and wheel
subscriptions share _last_robot_time; a newer wheel update can precede older
queued joints. DDS ordering within one publisher does not impose global order
across subscriptions. The EKF has one posterior at one time, predicts to each
accepted stamp, and its correction assigns the measurement timestamp. Applying
an old observation to the current posterior would be mathematically wrong and
could regress its timestamp. The existing rejection is therefore protective;
the scheduling/fusion contract needs improvement, not removal of validation.

The smallest correct next step is a bounded timestamp-order buffer at the
Estimation input boundary. A configured ~30 ms simulation-time holdback would
sort observations before the single EKF prediction/correction. Validate data,
frame and future/age bounds before insertion; process only up to now-holdback;
keep rejecting observations older than the committed watermark or beyond the
bounded delay; limit queue size; clear queue and source/filter watermarks on
clock reset. Preserve per-source freshness independently. The extra estimator
latency must be tested against control freshness and joint safety before enabling
it. Per-source timestamp checks alone are not a correct replacement for global
EKF ordering, and decimating joints does not guarantee order.

Timestamp-only replay of the full recorded callback trace gives 5,610 late drops
at zero holdback, 1,152 at 10 ms, 19 at 20 ms and 16 at 30 ms (16,057 accepted).
This supports a bounded reorder design; it is not a live implementation or an
accuracy-improvement claim. The runtime estimator was deliberately left unchanged
in this correction. Many joint updates remain available and measured state
agreement is small; discarded samples reduce information, but the axis mismatch
and wheel/world drift dominate the observed discrepancy. Ground-truth steering,
blindly accepting stale data, PID changes and weight/limit tuning were avoided.

**Verification and reproduction**

715 functional tests passed; 18 lint tests were deselected in the broad run.
Bringup's full package tests passed (17 passed, one copyright skip). Three affected
ROS packages built. Existing broader style failures remain as documented by the
prior v2 milestone. Regression tests cover +X/+Z marker directions, actual-model
finite-difference Jacobians for both axes and optional observational plugin
ownership. git diff --check passed. The corrected live failure/recovery test
passed controller/prediction/estimation stalls, stop/arm hold, clock reset and
marker recovery, old-session replay, controller restart and single actuator owner.

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=138 GZ_PARTITION=v2_camera_axes GZ_IP=127.0.0.1
ros2 launch robot_bringup closed_loop.launch.py \
  config:=config/optimal_tracking.yaml strategy_config:=config/whole_body.yaml \
  enable_actuation:=true visualize_tracking:=true gui:=false
```

In another sourced terminal with the same ROS_DOMAIN_ID and GZ_PARTITION:

```bash
rviz2 -d /home/user/ros2_ws/install/robot_bringup/share/robot_bringup/config/tracking_axes.rviz \
  --ros-args -p use_sim_time:=true
gz sim -g --force-version 8
```

Run these GUI commands in separate terminals if both views are desired. For
headless acceptance, raw-frame capture and safety regression:

```bash
python3 docs/validation/v2_tracking_gazebo.py --case v2 --physical-ground-truth \
  --output /tmp/camera_physical_acceptance
python3 docs/validation/v2_orientation_probe.py --visualization --output /tmp/camera_probe
python3 docs/validation/closed_loop_safety.py --case failures \
  --config config/optimal_tracking.yaml --strategy-config config/whole_body.yaml \
  --visualization --output /tmp/camera_safety
python3 -m pytest src --import-mode=importlib -p no:launch_testing -p no:launch_ros \
  -q -k 'not flake8 and not pep257 and not copyright'
python3 docs/validation/analyze_orientation.py
MPLCONFIGDIR=/tmp/orientation_plot python3 docs/validation/plot_orientation.py
```

The capture writes native Gazebo poses, raw streams, processing order and a full
snapshot. Its before/after source data are preserved in orientation_z_baseline
and orientation_x_corrected; final acceptance and safety results are in
orientation_final_physical and orientation_safety. Failed diagnostic attempts
remain separate. The complete exact [changed-file list](validation/orientation_modified_files.txt)
contains 21 implementation/document files; logs, JSON and figure artifacts are
under docs/validation/orientation*. Existing untracked evidence was preserved.
No commit or push was made. Wheel/world localization mismatch and the proposed
reorder buffer remain explicit follow-up work; this change corrects the optical
axis and makes physical versus inferred tracking visible.
