# Control Compute Only implementation plan

**Goal:** Continuous, frame-consistent, safety-constrained computation from
live robot estimates and target predictions, with command publishing disabled.

**Spec:** User-provided “Adaptive Target Tracking Testbed — Project Onboarding,
Architecture Review, and Control Package Completion”, supplied 2026-10-13.

**Architecture:** Retain the current model, factory, reference, controller and
safety interfaces. Correct ROS transport boundaries and startup configuration.
The user authorized direct workspace edits and continuous execution; preserve
the existing uncommitted control package and estimation changes. Do not create
a separate checkout that omits those authoritative files or commit user work.

**Constraints:** ROS 2 Jazzy; compute_enabled=true; publish_commands=false;
safety_enabled=true. No actuator execution, advanced algorithms, full Bringup,
or geometry changes. Static world/odom transform must describe the actual spawn.

## Tasks

- [x] 1. Regression tests for same-time fused robot estimates, older estimates,
  future timestamp cache poisoning, clock-reset recovery, timestamped TF with
  rotation/translation, missing TF, stale inputs and lookahead horizon expiry.
  Files: control ROS node, prediction adapter, input validator and their tests.
  Run each regression against current code before implementing the fix.
- [x] 2. Estimation regressions for shuffled arm joints alongside wheel joints,
  malformed joint data without filter mutation, older sensor data, and reset.
  Configure joint_names in robot_estimator.yaml; reject missing measurements;
  fuse same-time measurements without propagating backward. Reset estimator
  and predictor state on a backward ROS clock jump or clock-source change.
- [x] 3. Enable the real-model Compute Only configuration with explicit example
  base velocity limits; declare runtime dependencies. Build affected packages
  and run the whole source test suite, distinguishing unrelated failures.
- [x] 4. Verify live ROS delivery and, if supported, a headless Gazebo target
  simulation with actual robot feedback. Retain logs of raw/safe commands and
  failure recovery. Document package ownership, verified status, reproduction
  commands and remaining Bringup/position-hold responsibilities.

## Review focus

- Equal timestamps represent successive sensor fusion, not necessarily replay.
- Reject future messages before they poison monotonic timestamp caches.
- Reset clears cached inputs; delayed messages from the former epoch stay invalid.
- Position transforms include translation; velocity transforms include rotation.
  Only fixed inertial frame relationships are supported at this milestone.
- Reject missing joints, stale inputs, unavailable transforms, insufficient
  lookahead and infeasible safety constraints without producing commands.

## Evidence and decisions

- Baseline: 427 passed, 1 skipped in robot_control.
- No live ROS nodes discovered after DDS sandbox restriction was resolved.
- Existing joint_position_hold.yaml is empty; durable arm hold is future work.
- Control uses bare SerialArmModel plus TrackingFrameKinematics; Bringup uses
  FixedMountModel. No double offset occurs in the current ControlFactory path.
- Regression failures were reproduced before fixes: 7 Control cases, 12 initial
  Estimation/Prediction cases, 3 nonfinite sensor cases, 3 source-readiness cases.
- Final package checks: Control 439 passed/1 skipped; Estimation 52 passed.
  Workspace functional suite 585 passed; seven selected/dependency packages built.
  Six unrelated package lint checks still fail; Prediction's failures were also
  reproduced against committed HEAD. See docs/control_compute_only.md.
- Final saved manual Gazebo verifier passed 208 cycles, each checking finite
  six-dimensional raw/safe output, configured velocity limits and next-step
  joint positions. Lost joint feedback with continuing wheel feedback inhibited
  computation; restoration and an actual world reset recovered successfully.
- Actual full Gazebo reset removed the dynamically spawned robot. The fixture
  queried the scene and respawned only when absent. Production lifecycle remains
  a future Bringup responsibility.
- Independent review confirmed finite-input and source-freshness fixes and
  found no remaining Critical/Important issue. Delayed timer/repeated-clock
  coverage was added. Source-age plus downstream cached-state grace is explicitly
  documented; no freshness rules were disabled.
