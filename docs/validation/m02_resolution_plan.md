# M02 actuator resolution design, scope and gate

Preserve uncommitted M09 and historical recordings; no commit/push.
The 50 Hz bounded position reference drives a 500 Hz position-error derivative
PID. Startup physical zero differs from configured target; target steps excite
P/D saturation and coupled motion. Source audit confirms DART limits are active,
but its unilateral velocity rows do not enforce every joint against all coupled
constraint impulses. Do not describe that as a full-timestep delayed check.

Smallest evaluated architecture: one compiled Gazebo System in robot_simulation,
loaded through the existing ament_python package, with selectable instrumented
legacy effort PID and effort-limited native DART velocity SERVO. The latter requests
v*=clip((r-q)/h, max(-vmax,(lower-q)/h), min(vmax,(upper-q)/h)) each physics step.
JointVelocityCmd maps to DART SERVO, whose motor impulses are bounded by retained
SDF effort limits times h. It is not a state reset. All arm motor rows participate
in the coupled constraint solve. No second force writer. High-level u, safe command
transport, reference integration/PositionGuard, controller/estimation/prediction,
physics timestep, geometry and authoritative limits remain unchanged.

Native demand has finite physical acceleration through effort-limited dynamics;
no new acceleration bound or tuned PID gains are invented. Startup moves toward
the configured posture under this same physical motor. The zero-error demand is
zero velocity, maintaining gravity hold through bounded motor constraint impulse.
No PID integral accumulates in SERVO mode. Legacy PID remains selectable, with
unchanged gains and full nominal/requested/saturated-command instrumentation.

Risks: native motor feasibility and DART/contact solver accuracy; backend-specific
semantics; inaccessible actual motor torque; scalar position targets lack source
stamps (existing authenticated ROS adapter remains the command freshness authority).
Reset restores configured posture target; old transport lineage is invalidated.
Neither generic contact disturbances nor hard safety are certified from a few runs.

Trace every physics iteration locally, including startup, with pre/post q,dq,
reference, target velocity/PID effort and effort availability, names/time/iteration.
Use these full traces for continuity/gap-aware speed acceptance. Add accepted
command identity and dq/reference-error fields to existing diagnostic message.
Native actual actuator torque is unavailable; never substitute joint reaction
wrench, zero JointState effort, or commanded force as measured motor torque.

Gate: first instrumented PID and native smoke, then matched CV/IMM x42/917 full
startup + direction-change trials. Require unchanged q/v/effort limits, .001 speed
tolerance, consecutive every-step feedback, no disabled/frozen success, tracking
and base/arm movement, RTF, cleanup. Full/focused/architecture tests, ten-package
build and copied installation. If any physical violation remains, CONDITIONAL.

## Full trial finding and final correction

The direct r-error/h SERVO prototype passed CV42 but full matched trials found
shoulder peaks 1.058209/1.240765/1.316543 rad/s in CV917/IMM42/IMM917. Full
contiguous traces at t=7.174 IMM917 show elbow demand 0->-1 and wrist demand
0->-1.5 rad/s in one 2 ms step while shoulder demand stays +1. Motor impulse
limits make that coupled request infeasible; the shoulder physically overspeeds.
All failed traces are retained, not relabeled as passes.

Final coherent actuator correction: velocity goal (r-q)/T with configured
T=0.1 s, capped to authoritative speed; demand slew capped to a=4 rad/s^2 each
physics step. Existing position interval takes priority if demand constraints
conflict. These are demand-response settings, not new physical limits/guarantees.
This removes the artificial 500/s error response and full-speed per-tick reversals
without changing upstream WLS or PID gains. Validate final four predictor/seed
cases; do not keep tuning if violations remain. Zero native demand is explicit
on missing physics feedback/reset, with unknown torque retained as unavailable.
