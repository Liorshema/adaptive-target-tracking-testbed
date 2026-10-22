# Package-local DART recovery backend

This directory contains the non-test DART backend sources from the official
Gazebo Physics 7.6.0 release, Apache-2.0 licensed. `UPSTREAM.json` identifies the
immutable upstream commit, downloaded archive digest and each retained upstream
source digest. Builds use the installed public Gazebo/DART SDK and require no
network access or installed private backend headers.

Local changes rename the backend namespace and collision-detector class symbols
to keep this library distinct from the stock plugin. The only physical motor
integration change is independent `ServoMotorConstraint` ownership in `Base.hh`
and velocity/force/detach handling in `JointFeatures.cc`. DART's authoritative
position, speed and effort limits, gravity, collisions and integration remain.

For a velocity command the backend stores the velocity-clamped command while in
SERVO mode, then selects PASSIVE ordinary dynamics. This prevents interpreting a
velocity command as torque and prevents the automatic combined JointConstraint
from generating a duplicate motor row. A manually owned ServoMotorConstraint
supplies motor impulses bounded by the unchanged effort limits times the actual
step, independently of physical limit rows. A minimal derived constraint changes
only `isActive()` to use the inherited active row dimension: the upstream method
otherwise excludes every PASSIVE joint, even after computing its motor rows.
No new torque gain or physical-state
reset is used for recovery. The upstream defensive nonfinite-state handling is
retained unchanged and is not a joint-limit recovery mechanism.

Motor constraints are removed on force-mode switches, joint detachment, model
removal and backend destruction. Registration checks existing manual constraint
membership on each fresh velocity command, so a reset that clears manual
constraints re-registers when the next command arrives. The native arm actuator
writes fresh velocity demand every physics step; this does not claim registration
refresh without a new velocity command.
Normal world rebuild/reset also destroys or removes the old model ownership.

The plugin is packaged as `_dartsim_recovery` beside the existing `_actuator`.
Resolve its binary with `importlib.util.find_spec`; it is a Gazebo plugin, not an
importable Python extension. License, provenance and this README also install to
`share/robot_simulation/third_party/dartsim_recovery`. Its registered plugin class is
`gz::physics::robot_simulation_dartsim::Plugin`. Explicit configuration must select
this library. Existing native actuator code and target generation are unchanged.

Real-physics RED/GREEN, recovery release, repeated independent starts, full-step
physical safety and copied-install loading are acceptance gates. A build alone
does not establish physical safety or successful recovery.

The first physical candidate retained the upstream `isActive()` enum check and
failed: its registered motors were excluded while joints drifted under gravity.
That rejected candidate is preserved in the investigation results. The derived
activity predicate corrects that integration omission; inherited motor dynamics
and authoritative effort bounds are unchanged.
