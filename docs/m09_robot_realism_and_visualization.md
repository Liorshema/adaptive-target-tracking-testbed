# M09 — Robot realism and trajectory visualization

2026-10-18. Baseline `bf71fdb` (committed M08), branch
`feature/m09-visualization`. M09 extends robot_description and robot_bringup;
the subsequent refinement adds offline numerical diagnostics to robot_experiments;
there is no new package or parallel controller/simulation stack. No commit/push.

## Novice workflow

```bash
cd /home/user/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robot_experiments
source install/setup.bash
# Physical simulation, live spatial data, Gazebo and RViz; computation only:
ros2 launch robot_bringup visualization.launch.py \
  system_config:=config/system_nominal.yaml gui:=true rviz:=true
# Explicitly opt into closed-loop physical commands when wanted:
ros2 launch robot_bringup visualization.launch.py \
  system_config:=config/system_nominal.yaml gui:=true rviz:=true enable_actuation:=true
# The same spatial outputs without any GUI or display requirement:
ros2 launch robot_bringup visualization.launch.py \
  system_config:=config/system_nominal.yaml gui:=false rviz:=false
```

Do not run these simultaneously in the same ROS domain/Gazebo partition. M08's
runner remains the supported owner for isolated benchmark execution. Existing
`closed_loop.launch.py` commands remain valid; visualization is still opt-in:

```bash
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_nominal.yaml visualize_tracking:=true gui:=false
```

The latter publishes markers and paths without starting RobotModel/RViz. The
visualization wrapper adds the display RobotModel and optional clients while
including that existing deployment exactly once. It always keeps the server
headless and starts `gz sim -g` as an independent GUI-only client. GUI/RViz exits
do not request shutdown of physics/control. If display variables are missing,
requested clients are skipped with a clear message; no DISPLAY/WSL value is
invented. RViz availability is checked separately. GUI remains optional.

With the wrapper already running headless, the default installed RViz scene can
be opened independently:

```bash
rviz2 -d "$(ros2 pkg prefix --share robot_bringup)/config/tracking_scene.rviz" \
  --ros-args -p use_sim_time:=true
```

For custom visualization topics/prefixes use the wrapper's `rviz:=true`: it binds
an installed RViz template to the effective YAML, including colors, layers,
fixed frame and TF prefix. The standalone template uses the default contract.

## Ownership and actual structure

The existing `nodes/tracking_visualization.py` executable remains the sole ROS
visualization adapter. Pure `visualization/configuration.py`, `geometry.py`,
`scene.py` and `rviz.py` handle strict styling/layer validation, finite rigid
geometry, bounded semantic state, and RViz-template binding. ROS message handling,
source-time TF lookup and standard MarkerArray/Path publication stay in the node.
`launch/visualization.launch.py` composes existing bringup plus read-only clients.
The actual tree is [m09_bringup_tree.txt](validation/m09_bringup_tree.txt).

robot_description owns the schematic camera visuals attached to its existing
tracking frame. robot_simulation retains world generation, existing ground,
lighting, target and physics. Estimation, prediction, control, M08 recording and
metrics are unchanged. There are still ten packages/eight console entry points.
robot_experiments has no dependency on the visualization node, and normal M08
runs do not launch it. No dashboard/server, custom message or external mesh was
introduced.

## Physically faithful representation

The differential-drive base, four wheels and four-joint arm retain their original
geometry, collision, mass, inertia, joint axes, friction/damping, limits and PID
configuration. The target remains the existing orange simulation sphere with its
owner-selected dynamics. Ground and lighting remain the existing world resources;
RViz adds a world grid and TF reference display rather than decorative physics.

A black schematic camera housing and cyan lens are visual-only elements at the
existing authoritative tracking mount. Their sizes derive from wrist dimensions;
there is no duplicate robot dimension table. They face tracking-frame local +X,
the currently intended wrist-forward camera convention. The camera is an icon,
not a claim of measured camera mass, collision envelope, intrinsics or FOV. Those
physical specifications are unavailable, so no sensor/FOV/collision is invented.

Tests compare the complete generated M08 and M09 URDF/SDF trees after removing
only visual/material elements. Physical signatures match exactly: nine collision
and inertial entries, total mass 17.2 kg and wrist mass 0.4 kg are unchanged.
Gazebo's fixed-joint reduction preserves both camera visuals on wrist_link without
adding a physical link/inertia. See m09_camera_urdf_signature.json and
m09_camera_sdf_signature.json under docs/validation.

## Frames, sources and optical geometry

The default fixed frame is world. A configured visualization frame may be the
owner reference frame or the configured odometry frame; moving-base histories are
rejected. Every nonjoint source uses its header frame and a TF lookup at its
source epoch. Missing/invalid transforms suppress that observation instead of
silently assuming identity. Robot state [x,y,yaw,v,w,q...,dq...] is converted from
its published frame; the target estimate uses the published six-state layout.
Future position and velocity samples are transformed using the fixed-frame TF at
forecast issuance, preserving each absolute future timestamp.

Physical camera pose uses actual Gazebo base pose plus measured joints and the
existing whole-body FK with authoritative tracking mount. Wheel odometry and
estimated robot pose never substitute for physical truth. The actual optical ray
is `Q_camera * tracking_forward_axis`; the axis and radius come from the selected
control owner configuration, and the mount comes from robot_description. Local
+X/+Y/+Z are displayed separately, so a configured optical direction is visible
independently of the arm shaft. The currently selected forward axis is local +X.

RobotModel consumes the real joint-state topic, and uses only a dedicated display
TF namespace: RSP `frame_prefix: visual/`, RViz `TF Prefix: visual`. The visualizer
publishes physical pose to `visual/base_link`; RSP publishes the prefixed arm and
wheel transforms. Control-facing `odom -> tracking_frame` and world/odom ownership
remain unchanged. Nonempty display prefixes are validated; no controller command
or state estimate is published by visualization.

The tracking-radius wire sphere is centered on current target truth with the
owner's R. It represents all candidate camera positions at distance R, not an
optimized theta/phi, selected desired pose, reachable set or collision guarantee.
Actual separation and desired R are labelled. FOV/reachability overlays are not
implemented because the necessary authoritative contracts do not exist.

## Layers, topics and semantics

| Output | Source/provenance | Semantics |
|---|---|---|
| /tracking/paths/target_truth | /simulation/target_position, actual numerical target | Green recent truth history |
| /tracking/paths/target_estimate | /estimation/target_state_full | Orange estimated target history |
| /tracking/paths/target_prediction | /prediction/trajectory | Blue latest future forecast, replaced rather than accumulated |
| /tracking/paths/robot_truth | /simulation/robot/ground_truth | Gray physically observed base history |
| /tracking/paths/robot_estimate | /estimation/robot_state_full | Orange estimated base history |
| /tracking/markers | Physical FK, target truth, actual joint feedback | Axes, camera/LOS, radius and measured utilization/warnings |
| /visualization/robot_description | Selected authoritative Xacro | RobotModel with visual/ display TF |

Default Path/Marker frame IDs are world. Historical PoseStamped epochs are original
source stamps. Forecast PoseStamped epochs are absolute future sample times;
Path header is current publication time. Marker geometry uses its source epoch.
Truth and estimate have separate named RViz displays; future forecast is a
separate replaceable display, not another history. Camera optical ray is cyan,
local axes red/green/blue, target LOS magenta, and R is a translucent cyan wire
sphere. Actual measured limit violations turn the labelled joint text red.
Colors are supplemented by namespaces, named displays, labels, geometry types
and line widths.

Warnings show actual q/dq, authoritative bounds and utilization for each configured
joint. The existing 0.001 numerical tolerance is observation-only; warnings do not
stop control, certify safety or fix physical overspeed. They are not full
self-collision checks. Missing physical pose suppresses the camera/LOS rather than
showing an inferred pose; a measured joint text warning can remain at the fixed
frame origin when no camera pose is available.

## Configuration and time policy

`config/tracking_visualization.yaml` uses the shared YAML loader and supports
`extends`. Schema version 1 is strict: unknown keys/layers/colors, nonfinite or
invalid geometry/rates/counts, malformed frames/topics and invalid RGBA fail early.
It stores styling/visibility only, not duplicated radius, limits or dimensions.
A child file can select visibility without editing robotics Python:

```yaml
extends: /installed/share/robot_bringup/config/tracking_visualization.yaml
history_seconds: 10.0
max_points: 200
layers:
  robot_estimate: false
  joint_limits: true
```

Pass it as `visualization_config:=/path/to/child.yaml`. Replace the example parent
path with the actual installed resource from `ros2 pkg prefix --share robot_bringup`.
Defaults are world, 5 Hz output, 8 seconds history, at most 500 points per layer,
max_age 0.2 s, max_skew 0.08 s and prediction_max_age 0.3 s. Marker width, camera
axis lengths, marker lifetime, colors, topics and the display frame prefix are
configurable. RViz-template binding applies custom topics, layer visibility,
frame, prefix, Path RGBA/metric widths and orbit view; marker styles are applied by the node.
Path displays use Billboards because RViz's Lines style ignores metric line width.
Truth is green (0.025 m), estimate orange (0.015 m), and prediction blue (0.035 m).
Current truth and estimate have independent colored spheres (0.065/0.085 m).
Robot histories are hidden by default and remain independently selectable.
The default TF display is disabled; only physical camera axes/optical ray are drawn
by markers. RobotModel and the complete TF publication are unchanged. Enable
the RViz `Display and tracking TF` checkbox for debugging, or use `show_all_tf: true`.

Additional YAML settings: `path_widths` (one per path), `point_sizes.truth/estimate`,
`overlay_widths.line_of_sight/tracking_radius`, `view.distance/pitch/yaw/focal_point`,
`layers.target_truth_current/target_estimate_current`, `history_max_gap` (0.5 s),
and `history_max_step` (1 m). These thresholds clear the displayed trail on a gap
or jump; they do not reject measurements or change estimation. Adjust them for
faster or deliberately discontinuous scenarios. They are visualization settings.

Each layer has independent availability. Source order/finite shape checks reject
bad observations. Source-future tolerance is bounded; clock rollback clears all
histories/forecasts. History pruning uses duration and a fixed-capacity deque.
A forward invalid sample breaks the next accepted trail; out-of-order samples
cannot overwrite state. Only the current contiguous segment is shown after a
configured source gap or position jump. Displayed points preserve their exact
published values and source epochs: there is no visual smoothing or resampling.
Stale histories disappear, and old forecasts are never drawn as current; only
unexpired forecast points at or after current simulation time are shown. Forecast
age uses issuance/source epoch. Skew gates the physical camera/base/joint/target
combination. Every MarkerArray begins with DELETEALL and uses stable namespace/ID
pairs, followed by the current snapshot. Empty Paths explicitly clear missing or
disabled layers; publishers use bounded transient-local history. Memory cannot
grow with experiment duration. No state estimation, prediction or M08 evaluation
metric is computed in visualization.

To add a layer, extend the strict layer config and pure Scene snapshot with a
source/time/provenance contract, then add the corresponding standard-message
adapter/rendering and analytical tests. Keep physical inputs and algorithm
implementations in their existing owners. Do not create another launch stack.

## Verification and measured results

**Verdict: CONDITIONAL.** Functional headless and GUI visualization checks passed,
including the prefixed RobotModel TF, all five histories/forecast paths, physical
FK/optical geometry, configured radius, bounded IDs/history and owned cleanup.
Actual GUI clients ran and RViz reported OpenGL 4.5. However, the available X11
capture API returned uniformly black windows; no usable screenshot or independent
pixel-level appearance confirmation is claimed. Those files are retained as
unusable captures, not presented as visual proof. Camera physical specifications
and FOV remain unavailable; its housing is explicitly schematic.

| Verification | Actual result | Evidence under docs/validation/ |
|---|---|---|
| Full functional regression | 1,142 passed; 18 existing style/copyright entry points excluded | m09_final_regression.log |
| M09/legacy-marker/architecture focused suite | 126 passed | m09_final_focused.log |
| Normal all-package build | 10 finished, exit 0 | m09_final_build.log |
| Copied non-symlink build | 10 finished, exit 0 | m09_final_copy_build.log |
| Outside-checkout installed verification | Ten packages/eight console entries, M07/M08 examples, M09 module origins/resources/wrapper passed | m09_installed_checks.json / .log |
| Headless real Gazebo with visualization | All 27 visualization gates passed | m09_live_evidence.json |
| Real GUI + RViz with independent probe | All 27 visualization gates passed | m09_live_evidence.json |
| Invalid moving-frame configuration | Rejected before any process started; exit 1 | m09_negative_launch.json |
| Ordinary headless compute-only, display disabled | Completed, zero visualization processes and zero commands | m09_disabled_evidence.json |
| Physical URDF/SDF | Exact M08/M09 nonvisual physical signatures equal | m09_camera_*_signature.json |
| Preservation | 815 original untracked paths retained; 421 protected files and physical YAML unchanged | m09_preservation.json |

The 18 deselections are test entry points for style/copyright, not functional
exclusions. Repository-wide lint success is not claimed. Camera tests passed
focused lint; robot_description/setup.py's preexisting import-order/missing-newline
lint findings were reproduced from the M08 baseline and left unchanged.

The successful headless run measured eight simulation seconds after three seconds
settling. It received nonempty paths 56/44/39/46/46 times for target truth,
target estimate, future prediction, robot truth and robot estimate respectively.
Maximum history lengths were 500/153/20/183/388 points. All output frames were
world; every received path had consistent increasing source epochs, and forecasts
contained only future points. Nine prefixed dynamic display frames were observed,
including visual/base_link and the arm/wheels. Marker snapshots contained at most
10 entries (DELETEALL plus nine unique namespace/ID entries).

The final GUI run measured 12 seconds after three settling seconds. The independent
probe received nonempty paths 77/72/72/72/70 times in that same order, with lengths
bounded at 500/262/20/307/436. It observed both physical display-base and joint TF.
GUI processes exited with the owned session; no survivor was left.

| Physical geometry comparison | Headless | GUI |
|---|---:|---:|
| Compared camera snapshots | 62 | 66 |
| Maximum origin difference against independently matched FK | 0.02379 m | 0.01973 m |
| Maximum optical direction difference | 0.04800 rad | 0.05625 rad |
| Intrinsic marker sphere radius error | < 1.3e-14 m | < 1.5e-14 m |

These runtime FK comparisons allow up to 0.08 seconds source matching, so they
include physical movement between samples; analytical rigid-transform/optical-axis
tests use exact fixtures. R was 1.0 m from control configuration. The GUI sphere
center comparison against a separately recorded target differed by up to
0.02854 m under that time matching, while its own radius stayed at R to numerical
precision. The pure wire geometry never uses an optimized desired pose.

Existing research limits remain visible. M08 evaluation of the headless run gave
mean distance error 0.0933 m, mean pointing error 0.1138 rad (6.52 degrees), and
RTF 0.751. It observed zero position overshoot but physical speed overshoot up to
0.1256 rad/s; joint coverage was 94.9%, so safety is failed. The GUI run measured
0.1284 m, 0.1207 rad (6.92 degrees) and RTF 0.382. No speed violation above tolerance
was sampled there, but joint coverage was only 74.9%; its physical safety is
unknown, not PASS. These are separate short runs under differing host/graphics/
recording loads, not controlled evidence of algorithm performance differences.
The preexisting M08 overspeed, localization and performance limitations remain.
No numerical or physical tuning was made to improve the scene or test result.

The first headless measurement run completed but its verification tool then failed
on dict-versus-message timestamp parsing. Two GUI probes had incomplete TF/Path
collection when sharing an executor with high-rate physical recording. An
independent display-probe executor, using the existing M08 recorder separately,
resolved that acquisition problem; no publisher freshness tolerance was relaxed.
All earlier failed/partial directories and black captures remain preserved.

Independent review identified a missing RViz TF prefix and missing binding of
custom topics/prefix/frame/colors. A pure tested template-binding helper and the
correct `visual` RViz prefix fixed both. A separate regression caught the
namespaced RSP joint-state subscription; it now explicitly consumes the selected
physical joint topic. Review and RED→GREEN closure are documented in
[m09_review.md](validation/m09_review.md).

### Reproduce verification

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/m09-testlogs python3 -m pytest src -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros \
  -k 'not flake8 and not pep257 and not copyright'
colcon build --symlink-install --packages-up-to robot_experiments
colcon --log-base /tmp/m09_copy/log build --base-paths src \
  --build-base /tmp/m09_copy/build --install-base /tmp/m09_copy/install \
  --merge-install --packages-up-to robot_experiments
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  ROS_LOG_DIR=/tmp/m09-copy-logs bash --noprofile --norc -c '
    source /opt/ros/jazzy/setup.bash
    source /tmp/m09_copy/install/local_setup.bash
    cd /tmp
    python3 /home/user/ros2_ws/docs/validation/m09_verify_installation.py \
      --prefix /tmp/m09_copy/install --output /tmp/m09_install_repeat.json'
# These bounded checks explicitly enable actuation inside their owned session:
python3 docs/validation/m09_live.py --output results/m09_headless_repeat
python3 docs/validation/m09_live.py --gui --output results/m09_gui_repeat
```

New study directories are collision-safe. Live validation reuses M08 session
ownership, recording and evaluation; production visualization does not depend on
M08. The runtime probe is read-only apart from explicitly requested launch actuation.
Exact commands/source windows, actual geometry, safety metrics and artifact paths
are in [m09_live_evidence.json](validation/m09_live_evidence.json). Successful raw
runs are under results/m09_headless_verified and results/m09_gui_observer_verified.
No black capture is embedded as a screenshot.

Shutdown left no owned processes. The ordinary display-disabled compute smoke
needed bounded signal escalation (launcher return -15) and still left no survivors;
both visualized runs had launcher return 0. Some existing ROS adapters/bridge
emit shutdown tracebacks/nonzero child codes when the validation owner and ROS
launch both propagate SIGINT. Completed visualization means a verified window and
no surviving owned process, not a claim that every child returns zero. No unrelated
ROS/Gazebo session was stopped. These teardown/graphics limitations remain explicit.

The exact source/resource/test/doc and generated-evidence inventory is
[m09_modified_files.txt](validation/m09_modified_files.txt), with captured
[m09_git_status_short.txt](validation/m09_git_status_short.txt). The workspace
remains on feature/m09-visualization at bf71fdb with reviewable, uncommitted edits.
No M10 dashboard or unrelated milestone was started.

## Visualization refinement and engineering diagnostics — final closure

This section records the subsequent manual-inspection refinement; the original
M09 runs and 1,142-test closure above remain historical evidence. Existing work
was continued in place, with no commit, push, estimator/controller tuning, PID
change, limit relaxation, physics change or new package.

The default installed scene now displays only camera-frame axes plus its selected
optical ray. Full TF remains published and RobotModel remains enabled. Eight-second
green/orange histories, a wider blue forecast, current-position spheres, hidden
secondary robot trails and configurable framing reduce clutter. YAML owns widths,
sizes, visibility and view settings. Published estimate values are unchanged.
Source gaps/jumps and invalid input break trails; clock/frame/order checks remain.
Independent review caught invalid ROS payloads rejected before reaching Scene;
the adapter now notifies Scene to break the next segment. Three real-message
regressions failed before that fix and pass afterward.

### Measured diagnostic findings

- [Target estimation diagnostics](validation/target_estimation_diagnostics.md):
  five preserved M08/M09 recordings, source-time/world-frame alignment, accuracy,
  roughness, turns, covariance, genuine replayed innovations/gains, offline
  sensitivity and prediction/control associations, with ten time-series plots.
  Measurement norm RMSE is 0.169–0.180 m versus estimate 0.077–0.090 m. The actual
  sensor covariance matches R; complete filter replay reproduces every posterior
  mean/covariance exactly. Approximately 3 cm discrete corrections at ~17 Hz
  produce real roughness. Smoother offline settings worsen measured accuracy;
  no deployed setting changed.
- [Joint motion and actuation diagnostics](validation/joint_motion_and_actuation_diagnostics.md):
  four recorded runs, all arm/wheel joints resolved by name, gap-aware travel/RMS,
  commands versus feedback, startup/evaluation violations, wrist events and base
  allocation, with twelve plots. All four arm joints participate. In the fully
  covered IMM42 evaluation, wrist range/travel/RMS are 0.3584 rad / 1.0314 rad /
  0.3524 rad/s. Limited net angle change conceals repeated motion.
  CV917's -3.47610875 rad/s wrist event at 6.802 s agrees with its preceding
  2 ms position increment. It exceeds the 1.5 rad/s description limit by
  1.97610875 rad/s; the unchanged 0.001 tolerance is not a remedy. Raw and safe
  evaluation commands coincide, and the physical response exceeds bounded
  high-level requests. No recorded arm position violation was found.

The phenomena can be temporally related: estimation corrections alter forecasts
and sometimes precede command changes. They are not a single demonstrated cause.
Physical startup overspeed also occurs before tracking measurements/commands.
PID torque and exact consumed-input lineage are unobserved, and sparse seed917
feedback limits duration/RMS conclusions. The reports distinguish observed-duration
integrals from complete-window RMS and never fill missing motion with zeros.
An [independent raw-data cross-check](validation/m09_refinement_independent_crosscheck.json)
recomputed the largest spike from q differences and CV917 target RMSE.

Next engineering ownership: `robot_simulation` for physics-step actuator/torque
observability and M02 physical-speed enforcement; `robot_estimation` for a future
held-out maneuver/model evaluation retaining the known sensor R; `robot_control`
and `robot_experiments` for minimal consumed-prediction/accepted-command lineage.
These are recommendations, not implementations in this task.

### Final verification and limits

| Check | Actual result | Evidence |
|---|---|---|
| Full functional regression | 1,191 passed; 18 existing style/copyright checks excluded | m09_refinement_closure_regression.log |
| Focused visualization/description/diagnostic tests | 252 passed; six style checks excluded | m09_refinement_closure_focused.log |
| New numerical fixtures | 23 target + 11 joint tests included above | target/joint diagnostic reports |
| Focused numerical/visualization-core lint | Passed at 99-column project limit | m09_refinement_closure_lint.log |
| Normal and copied builds | All ten packages passed in each | m09_refinement_closure_build.log; m09_refinement_closure_copy_build.log |
| Outside-checkout copied-install check | Ten packages, eight entry points, diagnostic modules and RViz resources verified | m09_refinement_installed_checks.json |
| Headless compute-only | 29/29 display gates; zero commands | results/m09_refinement_compute_only_verified/headless_20261018T151718_747435Z_t3s3x1a5/result.json |
| Motion-enabled Gazebo + RViz | 29/29 display gates; 56 command messages | results/m09_refinement_gui/gui_20261018T151755_960447Z_qa2z30p7/result.json |
| Actually opened RViz file | TF display disabled; RobotModel enabled; configured widths/view | m09_refinement_runtime_rviz.json |

The compute-only window was [2.716,6.716] simulated seconds; the GUI motion window
was [3.6,9.6]. Both retained camera axes, current target markers, prediction/source
epochs and full display-joint TF, while secondary histories stayed empty. GUI
clients really ran; screenshots remain uniformly black, so pixel-level visual
quality remains unverified. The motion run exited on SIGINT; the compute-only
run required bounded signal escalation. Both left zero owned survivors. These
short checks establish integration, not a matched performance/tuning comparison.
An initial compute-only fixture preflight correctly rejected an inconsistent
publish_commands=true manifest before starting ROS nodes; the fixture now sets
both runtime flags false. Its failed directory remains preserved.

**Verdict: CONDITIONAL.** Visualization and diagnostic functionality passed the
listed checks. Physical speed acceptance remains failed; incomplete telemetry,
unobserved torque/input lineage, previous localization limitations and unavailable
pixel verification remain explicit. No hardware-safety claim is made. Existing
style checks remain outside functional acceptance; focused lint does not claim
that the entire historical workspace is lint-clean.

### Reproduce the refinement

The safe compute-only and explicit `enable_actuation:=true` GUI commands in
“Novice workflow” remain the supported launch commands. After building/sourcing:

```bash
ROS_LOG_DIR=/tmp/m09-refinement-tests python3 -m pytest src -q \
  --import-mode=importlib -p no:launch_testing -p no:launch_ros \
  -k 'not flake8 and not pep257 and not copyright'
python3 docs/validation/m09_live.py \
  --output results/m09_compute_repeat --compute-only --duration 4 --settling-time 1
python3 docs/validation/m09_live.py \
  --output results/m09_gui_repeat --gui --duration 6 --settling-time 1
MPLCONFIGDIR=/tmp/m09-target-plots python3 docs/validation/m09_target_diagnostics.py \
  --output results/m09_target_diagnostics_repeat
MPLCONFIGDIR=/tmp/m09-joint-plots python3 docs/validation/m09_joint_diagnostics.py \
  --output-prefix results/m09_joint_diagnostics_repeat/diagnostics
```

Choose fresh diagnostic output names on repeats; existing evidence is protected
from overwriting. The exact refinement file inventory and preservation checks are
[m09_refinement_modified_files.txt](validation/m09_refinement_modified_files.txt)
and [m09_refinement_preservation.json](validation/m09_refinement_preservation.json).
The machine-readable closure is
[m09_refinement_final_verification.json](validation/m09_refinement_final_verification.json).
