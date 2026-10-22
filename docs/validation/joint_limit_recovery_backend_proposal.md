# Proposed local DART physics integration correction

This proposal precedes implementation. The parent investigation established a
real-physics RED in `results/joint_limit_recovery_red/native_boundary_20261021T034132_558095Z_rxzw_s1u/C_wrist_lower`:
the wrist remained at -0.26179938780017414 through four seconds of inward demand.
The matching upper-bound control moves. See `joint_limit_recovery_physics_audit.md`
for backend source and binary evidence explaining limit-row motor starvation.

## Method

Build a distinct physics plugin inside the existing robot_simulation package from
the exact official gz-physics7_7.6.0 DART backend source. Vendor its license,
source commit, archive digest, retained-file digests and local changes. Builds use
installed Gazebo/DART vendor SDKs and perform no network operations. Keep the
existing `_actuator` extension unchanged; package the new plugin as
`robot_simulation.gazebo._dartsim_recovery` for find_spec-based binary discovery.

The custom backend keeps authoritative physical position, velocity and effort
limits. For velocity commands, register DART's existing independent
ServoMotorConstraint once per joint and use PASSIVE ordinary dynamics. Store the
velocity command through SERVO/setCommand/PASSIVE so velocity clamping is retained
without interpreting the command as ordinary torque or changing physical state.
The automatic JointConstraint continues enforcing physical limits; the manually
registered motor row retains its impulse bound of effort times actual step.

Use a distinct internal namespace/plugin class to avoid collisions with the stock
backend, and select its explicit library path only through the main agent's
configuration integration. No joint-name exceptions, limit widening, q/v resets,
new torque-control law or PID gain changes are part of this proposal.

## Ownership and validation

Remove managed motor constraints before switching a joint to force control and
before a managed joint/model disappears. Refresh lifecycle membership before
physics stepping and clear owned constraints during plugin destruction. World
reset must not leave duplicate motor rows or stale raw joint pointers. The main
agent owns matching RED/GREEN fixtures, repeated boundary recovery, recovery
release, C/D/E runs, and full-step physical safety checks.

Risks requiring measured validation are redundant constraint conditioning near
hard stops, effort-limited infeasibility, coupled/contact speed excursions,
reset/removal lifecycle, copied-install library loading and added source/build
maintenance. The patch changes motor/limit integration, not a universal physics
safety proof. Existing failure evidence and historical M02 work remain intact.

Implementation is authorized by the parent task, contingent on successful source
and build feasibility checks. An inability to validate physical safety must be
reported as CONDITIONAL or FAIL, not PASS.

## Rejected first candidate and focused correction

The first native physical candidate failed: registered motor rows were excluded
and joints drifted under gravity. The audit inspected the upstream update method
but missed the separate `ServoMotorConstraint::isActive()` gate (DART 6.13.2,
lines 265–273), which requires SERVO mode. PASSIVE dynamics therefore prevented
activation. The focused correction derives from the installed public constraint
class and overrides only `isActive()` to return `getDimension() > 0`; inherited
update, effort bounds and impulse application remain unchanged. This correction
requires a new build identity and a fresh physical GREEN before acceptance.
