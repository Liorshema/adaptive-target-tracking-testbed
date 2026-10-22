# Joint-limit recovery: read-only physics audit

Date: 2026-10-21. No simulator runs, builds, production edits, staging or commits
were performed by this audit. This file is the audit's only added artifact.

## Evidence and scope

The historical loaded-library capture `docs/validation/m02_physics_engine.json:5–9`
identifies gz-physics 7.6.0's DART plugin and DART 6.13.2. This proves that captured
run's backend; a new reproduction must capture its own process mappings.

The actual retained challenge world is
`results/reactive_challenge_final/reactive_manipulation_20261019T155152_099000Z_nmxd3n7j/runs/D_seed42/generated_world.sdf`.
Its lines 4–5 specify **0.002 s**, despite the task narrative's 0.001 s example.
Lines 168–171 specify shoulder [0, 1.57], effort 20, speed 1; lines 278–281
specify wrist ±0.26179938779914941, effort 10, speed 1.5.
Lines 570–573 select the existing native `velocity_servo`, T=0.1, a=4.
The XML `type="ode"` does not establish the loaded engine.

Native source references:

- `src/robot_simulation/robot_simulation/gazebo/native/actuator_system.cc:219–227`:
  legal incoming references replace the internal target; invalid references are counted.
- Same file, lines 249–257: governed demand is written to `JointVelocityCmd`.
- Same file, lines 304–311 and 387–390: trace records the internal target,
  demand and physics feedback; velocity backend effort fields are unavailable.
- Same file, lines 353–369: authoritative limits and existing PID effort clamp.
- `src/robot_simulation/robot_simulation/gazebo/native/servo_math.hh:43–51`:
  position error, demand slew and one-step interval clipping.

## Exact backend mechanism

[Gazebo Sim Physics.cc, lines 2164–2218](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/systems/physics/Physics.cc#L2164)
maps velocity commands to `SetVelocityCommand`; force components take precedence,
and velocity reset components suppress velocity commands.
[gz-physics JointFeatures.cc, lines 133–168](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.6.0/dartsim/src/JointFeatures.cc#L133)
selects DART SERVO and sets its command. This is a motor demand, not state reset.

Installed `GenericJoint.hpp:341–345` clips SERVO command to velocity limits;
`:2112–2116` sets its ordinary dynamics force to zero. Its motor effect comes
through constraint impulses. Installed `ConstraintSolver.hpp:296` stores combined
`JointConstraint` objects. Binary inspection independently confirms their use:

```sh
objdump -d -C /opt/ros/jazzy/opt/gz_dartsim_vendor/lib/libdart.so.6.13.2 \
  --disassemble='dart::constraint::ConstraintSolver::updateConstraints()'
```

Address `0x3586c3` calls `JointConstraint::JointConstraint(Joint*)`.
[ConstraintSolver.cpp, lines 574–596](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ConstraintSolver.cpp#L574)
creates these for enforced limits or SERVO. The separately installed
`ServoMotorConstraint` class does not establish that it is selected.

[JointConstraint.cpp, lines 181–362](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/JointConstraint.cpp#L181):
position and velocity branches each `continue`, omitting the SERVO branch.
Outside-position rows drive toward the boundary. Velocity rows compare against
`max(vmin,(lower-q)/h)` and `min(vmax,(upper-q)/h)`. Only the SERVO branch uses
motor impulse bounds `±effort*h`. Consequently legal inward demand can be omitted
while a limit row is active. Hard-stop impulses are distinct from motor effort.

[World.cpp, lines 156–182](https://github.com/dartsim/dart/blob/v6.13.2/dart/simulation/World.cpp#L156)
integrates unconstrained velocity before selecting constraints, then applies
impulses before integrating positions. Outward gravity can therefore activate
the velocity-limit branch even at an exact boundary.

## Historical physical observation

Read-only CSV analysis used each joint's consecutive source timestamps and
required inward `|target_velocity|>0.05` with `|v_post|<0.001`. Longest wrist
windows in the preserved challenge directory above:

| Run | First time | Last time | Elapsed source time | Consecutive samples |
|---|---:|---:|---:|---:|
| C_seed42 | 22.874 | 41.610 | 18.736 s | 9369 |
| D_seed42 | 23.568 | 42.062 | 18.494 s | 9248 |
| E_seed42 | 22.588 | 41.748 | 19.160 s | 9581 |

Each trace is `runs/<run>/physics/actuator.csv`; all steps in these windows use
h=0.002. At the final C sample, q=-0.26179938779993822,
reference=-0.20179938779914941, native demand=0.60000000000788811,
v_post=1.4055240446316539e-14, valid_step=1, invalid_commands=0,
saturation=0. This is persistent physical immobility with a current native target,
not just transient lag or a ROS-only observation. Torque was not measured.

## Falsifiable hypothesis and discriminating real experiments

**Hypothesis:** limit-row selection repeatedly removes the bounded SERVO motor
demand. Tiny residual position error and/or gravity's outward unconstrained
velocity sustain that selection. Increasing the position reference then cannot
restore motor authority. This predicts a legal inward demand independent of the
nearly zero resulting physical velocity.

The trace is consistent with this mechanism; it does not alone prove the active
internal branch. A fresh pre-fix real native reproduction remains necessary.

1. At both physical stops, sustain a legal interior reference with the current
   native backend, actual joint limits, actual step, load, native trace and fresh
   ROS authority. Use existing model construction and physical motion to reach
   the boundary; no position/velocity resets. Compare an otherwise identical
   start slightly inside. Persistent interior success with boundary failure
   rejects general command-delivery failure.
2. In isolated experimental worlds only, compare loaded and zero-gravity cases
   at the same stops and targets. Gravity dependence discriminates the velocity
   branch from pure position residual effects; include a horizontal-axis wrist
   and gravitationally loaded shoulder. Record actual q, not requested initial q.
3. Repeat the stuck posture with the **existing** `instrumented_pid` backend,
   unchanged effort/position/speed limits and a conservative test reference.
   Physical inward motion under recorded bounded effort rejects insufficient
   available effort in that posture. This is a causal experiment, not approval
   to restore the historical PID production default.
4. If feasible in a separately built diagnostic backend, record selected
   constraint branch, unconstrained velocity, clamped DART command, and motor
   impulse. A persistently active SERVO branch with sufficient unsaturated effort
   would falsify the proposed starvation mechanism. Do not label transmitted
   wrench or ordinary force-state access as measured motor torque.

## Fix venues and rejected shortcuts

No correction is implemented or certified by this audit. Promising venues after
the failing real experiment are (a) backend integration that preserves a bounded
motor row while retaining unilateral physical limit rows, or (b) a bounded
effort-driven native actuator whose motor force participates before limit
selection. Both require real safety regressions; the latter needs properly owned
control parameters and cannot inherit the old PID's safety claims.

Changing only ROS recovery reference, native target inset, or servo slew cannot
restore a motor row that the backend omits. Preventing all boundary contact with
an interior target policy would also leave recovery from an already reached
physical stop unresolved.

`JointConstraint::setErrorAllowance` is static/global (installed
`JointConstraint.hpp:72–94,164`). In the cited update source, positive allowance
can skip the position branch for a tiny excursion but does not change the
velocity-branch bounds or its `continue`; an outward-loaded stop still suppresses
the motor. `setErrorReductionParameter` is not read by that update implementation
(its C1/C2 use `mErrorAllowance`). These are not supported recovery fixes.

The installed public gz-physics interfaces provide no independent SERVO/limit
row selection or per-joint allowance switch. [WorldFeatures.cc:146–178](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.6.0/dartsim/src/WorldFeatures.cc#L146)
selects Dantzig or PGS boxed solvers, not row construction. A native System calling
a DART global setter would affect all worlds/joints and leave the velocity branch
problem intact. Enlarging limits, resetting state, or disabling physical limits
is unnecessary and violates the task's constraints.

## Follow-up: velocity limits and independent motor integration

The [versioned SDF construction](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.6.0/dartsim/src/SDFFeatures.cc#L123)
copies authoritative position, effort and velocity limits into DART and enables
limit enforcement. [JointFeatures.cc:218–261](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.6.0/dartsim/src/JointFeatures.cc#L218)
supports velocity-limit changes. GzSim's `JointVelocityLimitsCmd` reaches these
setters (`Physics.cc:2059–2084`); it updates existing limits rather than adding a
new physical constraint policy. FORCE mode retains limit rows. Their presence
does not prove a bound against every later coupled/contact impulse; observed
historical overspeed is not proof that SDF velocity limits were absent.

An independent motor constraint is a plausible backend integration experiment:
register DART's existing `ServoMotorConstraint` manually while retaining automatic
physical limit rows. The independent constraint reads its desired velocity from
`joint->getCommand` and bounds its impulse by the authoritative force limits times
the actual step ([ServoMotorConstraint.cpp:107–127](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ServoMotorConstraint.cpp#L107)).
**FORCE mode plus that same command is incorrect:** it also interprets the value
as ordinary torque (installed `GenericJoint.hpp:2108–2110`). PASSIVE mode instead
supplies zero ordinary motor force (`:2112–2116`), while automatic limit rows remain.
For a backend experiment, setting the command while SERVO and subsequently
selecting PASSIVE avoids the PASSIVE nonzero-command warning. The actuator-type
setter changes only the enum ([Joint.cpp:181–184](https://github.com/dartsim/dart/blob/v6.13.2/dart/dynamics/Joint.cpp#L181));
this is not a position/velocity reset. This candidate still needs real validation
of effort bounds, hard stops, independent starts, all-joint coupling and reset.

Within the backend, exact-version [Base.hh:536–556,612–615](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.6.0/dartsim/src/Base.hh#L536)
provides joint/model/world lookup, and lines 395–397 demonstrate manual constraint
registration for an existing weld implementation. Manual constraints are processed
by `ConstraintSolver.cpp:439–443` independently of automatic limit-row creation.
Ownership must remove motor constraints before joint/model destruction, support
force-mode transitions, and recreate cleanly on world reset; stale raw joint
pointers must never reach a solve.

The installed public SDK does not include private DART backend implementation
headers. A small extension linked to installed backend symbols would need exact
7.6.0 private headers and private ABI/layout coupling. A separately named source
build of the DART backend avoids that binary-layout shortcut but adds materially
more maintained source/build scope. Neither is a tested fix here. Explicit engine
selection and run-local loaded-library capture are required for either experiment.

**Correction from the first physical experiment:** inspecting `update()` alone
missed `ServoMotorConstraint::isActive()` at lines 265–273, which rejects PASSIVE
joints. That first candidate failed physically. The subsequent integration uses
a minimal subclass overriding activity with inherited `getDimension() > 0`, while
retaining the upstream effort/impulse computation. The original inference that
an unmodified independent constraint sufficed with PASSIVE dynamics was wrong.
