# M02 — Current actuator resolution

2026-10-19. **CONDITIONAL overall; measured nominal simulation compliance demonstrated.**
The permanent simulation now defaults to the governed effort-limited DART motor
in robot_simulation. This replaces the explicit position PID for normal holding
and tracking; original PID gains/configuration remain selectable for baselines.
No limits, physics timestep, geometry, high-level algorithms or target dynamics changed.

The original mechanism was bounded position-reference motion feeding a faster
position-error derivative PID, with startup target mismatch, saturation and
coupled dynamics. A direct native motor prototype still produced shoulder
overspeed when other motor demands changed abruptly. The accepted implementation
adds a 0.1 s position-response time and a 4 rad/s² demand slew limit before the
native, effort-limited velocity motor. Neither setting claims a bound on physical
acceleration. All physical limits still come from robot_description/SDF axes.

Four final CV/IMM × seeds42/917 runs recorded every physics step through startup
and tracking: zero velocity or position-policy violations at the existing .001
tolerances, 100% sampled-interval coverage, maximum gap .002 s. Across those runs,
maximum speeds were yaw .625272, shoulder 1.000000000, elbow 1.000000001, wrist
.689062 rad/s against limits 1/1/1/1.5. New instrumented PID baselines reached
wrist 2.392–3.054 rad/s; the historical 3.47610875 rad/s record is preserved.
The small shoulder excursion -6.4716e-5 rad is inside the existing tolerance,
not claimed to be exact geometric nonpenetration.

Mean distance error before→after: CV42 .09587→.10158, CV917 .10697→.11094,
IMM42 .10762→.10017, IMM917 .09147→.09507 m. Largest mean pointing increase
was .54°. Base and every arm joint moved; no frozen/disabled actuation success.
A real world-reset probe recovered control with both physics epochs safe/complete.
Compute-only retained zero commands and passed 29 visualization gates.

Full regression: 1,224 passed (18 existing style/copyright checks excluded).
Normal and copied ten-package builds and independent installed-resource checks passed.
Actual native motor torque, severe contacts/payloads, sustained hard-stop cases
and universal hard velocity enforcement remain unverified. These limits keep the
overall verdict CONDITIONAL; the implemented correction is tested-scenario
simulation compliance, not hardware certification or a general hard bound.

See [the full resolution report](validation/m02_actuation_safety_resolution.md)
for mechanisms, instrumentation, per-joint metrics, plots, exact artifacts and
commands. [M02-only changed files](validation/m02_resolution_modified_files.txt)
separate this work from the preserved, uncommitted M09 changes. No commit/push.

Everything below preserves the earlier closure as historical evidence. Its
“not implemented” actuator proposal refers to that earlier state; the current
implementation and verdict above and in the resolution report take precedence.

## Earlier M02 closure — historical record

**Historical verdict: CONDITIONAL; physical-speed acceptance remained open.**

The numerical joint-boundary deadlock has a simulation-local recovery mechanism, strict high-level bounds remain intact, and position-reference discontinuities are blocked. This does **not** turn the effort PID into a physical velocity servo. M02 must not be marked PASS while measured joint speeds exceed description limits plus the unchanged acceptance tolerance.

## Failure mechanism and evidence

The original user launch was inspected in its actual routing context: `ROS_DOMAIN_ID=172`, `GZ_PARTITION=aggressive_tracking`. The read-only capture is `validation/m02_live_original/events.json`. At its last captured sample:

- Physical shoulder: `-3.884685468268122e-13 rad`; authoritative interval `[0, 1.57] rad`.
- Held shoulder reference: `0.0036350999382840313 rad`, inside that interval.
- Controller: inactive, `Current joint positions violate limits`.
- Actuator: enabled, inactive, `Control invalid or disabled`; 1,118 accepted commands, 13,238 inactive commands, 214 reference-limit events, zero timeout events.

The stop is a boundary deadlock, not a Gazebo freeze. `JointLimits.contains` is an exact test. Ordinary inactive commands retain the preceding legal PID target; they do **not** reinitialize it to the out-of-bounds measured position. The actual cycle is therefore: numerical physical excursion or an out-of-range estimated position → strict controller rejection → retained target too small to lift the gravity-loaded shoulder off its stop → repeated rejection. The initially suspected “hold the current illegal position” is not what the code does.

The rejection is based on **estimated**, not raw, joint position. In the controlled pre-fix reproduction (`validation/m02_B_before`), at simulation time `11.300 s`, the rejected shoulder estimate was `-0.0140453535927 rad`; the nearest raw physical shoulder was `+0.0001933121402144317 rad` at `11.294 s`. Thus the large reported estimate excursion must not be presented as a physical penetration of that magnitude. At `12.552 s`, the rejected estimate was `-0.00291450761427 rad`, while the raw shoulder at the same timestamp was `-1.754901267705943e-13 rad`.

The EKF's constant-velocity joint process does not model hard-stop contact. Its existing shared measurement watermark also rejects out-of-order robot updates. Those estimator limitations remain observable and unchanged; M03 ordering/model work was not started. Timestamp validation and freshness guards were preserved.

### Physical configuration and ownership

`src/robot_description/config/robot/robot.yaml` remains authoritative. Joint order is `base_yaw, shoulder, elbow, wrist`; all pitches use axis `[0,-1,0]`. Shoulder zero is horizontal and positive rotation raises link 1. Its lower bound remains **0.0 rad**. URDF collision primitives, physical PID effort saturation and SDF limits were preserved. There is no independent replacement joint-limit table.

The chain is description YAML → Xacro joint limits → generated SDF and model bounds → optimization/final safe velocity bounds → simulation-local bounded position references → Gazebo effort PID → raw joint feedback. Naming/reordering follows the description insertion order and configured joint-name template. The physical speed limits remain `[1,1,1,1.5] rad/s`; effort limits remain `[20,20,20,10] Nm`.

The first-link collision box extends along local +X. At zero shoulder its bottom is 0.04 m above the top of the base. Measured first-link box clearance and elevation are recorded independently from native Gazebo poses. A shoulder bound alone does not provide general self-collision avoidance. Tracking still uses wrist +X and the live TF tracking-frame position.

## Implemented correction

### Strict diagnostics, unchanged high-level safety

`JointPositionViolation` retains joint index, measured **model** position, lower/upper bounds and exact excess. The ROS adapter supplies externally configured names. Messages identify the joint and angular units; strict rejection is unchanged. For example:

```
Current joint positions violate limits: shoulder_joint q=... rad bounds=[0,1.57] rad excess=... rad
```

### Pure low-level recovery policy

`robot_simulation.actuation.position_guard.PositionGuard` is independent of ROS and Gazebo. It classifies physical feedback, supervises recovery and latches serious position faults. It owns no robot-specific joint names or limits.

All new policy parameters belong to `src/robot_simulation/config/actuation.yaml`, under `actuation.position_recovery`:

| Parameter | Value | Meaning |
|---|---:|---|
| `numerical_tolerance` | `1e-9 rad` | Numerical excursion band; also the inside boundary band eligible for release during a fresh inactive heartbeat. Does not change controller validity. |
| `recoverable_violation` | `0.001 rad` | Maximum recoverable physical excess; matches the existing physical acceptance envelope. Greater excess latches a fault. |
| `recovery_speed` | `0.1 rad/s` | **PID-reference** slew limit, additionally capped by each description velocity. It is not a claimed physical motor-speed bound. |
| `recovery_inset` | `0.06 rad` | Legal inward target offset at an affected boundary; checked against the existing `max_position_lead=0.08 rad`. |
| `release_margin` | `0.005 rad` | Measured interior clearance required before returning control authority. |

For a lower-side recovery, only the affected target may increase toward at least `lower + inset`; an upper-side recovery may only decrease toward at most `upper - inset`. References already farther inside are retained. Other joint references hold and base commands are zero. Each reference increment is bounded by `min(recovery_speed, description_velocity) * dt`. Physical references remain inside the original limits.

Recovery requires explicit actuation enablement, fresh complete feedback, a fresh authenticated source heartbeat, sole command/output ownership, a valid simulation-clock interval, and existing wall/ROS freshness rules. Missing, stale, malformed, replayed, competing, or invalid transport cannot authorize recovery. A validated inactive command is eligible because strict controller rejection must not permanently prevent low-level boundary release. Consequently, **inactive is not an emergency-stop interface**: with actuation enabled, a fresh inactive heartbeat can authorize this narrow inward release. Disabling actuation suppresses all output.

Hysteresis retains recovery until the measured position clears the release margin. Serious faults remain latched across good feedback, new command sessions and clock reset. There is no fault-reset service or generic clearing operation. Fault handling commands zero base motion and retains the last legal effort-limited arm target; it is not an instantaneous mechanical brake. Latching is process-local, so restarting the actuator is an operator action requiring diagnosis, not a fault acknowledgement implemented here.

### Why the inset is 0.06 rad

With authoritative masses and link dimensions, the maximum horizontal shoulder gravity moment is

```
g * [m1*l1/2 + m2*(l1+l2/2) + mw*(l1+l2+lw/2)]
= 9.81 * [1*.2 + .8*.58 + .4*.82]
= 9.73152 Nm.
```

The unchanged shoulder proportional gain is `200 Nm/rad`, with integral saturation up to `1 Nm`. Allowing an adverse integral contribution and requiring measured clearance `0.005 rad`, the sufficient static reference offset is greater than

```
0.005 + (9.73152 + 1)/200 = 0.0586576 rad.
```

The configured 0.06 rad inset provides that static margin and remains below the existing 0.08 rad reference lead. This is a deployment policy derived from the current physical load and existing PID, not a new gain. A focused configuration test recomputes the load from description geometry and world gravity. It must be reassessed for another payload/robot/PID.

The first diagnostic recovery probe used 0.04 rad and correctly exposed insufficient lifting torque: it remained at the hard stop with a 0.04 rad target. That probe is retained as `validation/m02_B_after`; it is **not** the accepted final result. The load-supported recovery probe `validation/m02_B_final` resumed tracking through 90 simulated seconds, with three recovery entries and no final permanent rejection. Final comparisons below use the complete implementation, including startup handoff continuity.

### Position-reference continuity

Ordinary hold/resume retains the last legal reference instead of replacing it with measured q. If measured-lead clipping would introduce a reference step faster than the physical reference-rate bound, it rejects the update and holds with an explicit diagnostic. It does not silently jump to the clipping envelope.

Bringup passes a new `holding_configuration_file` parameter to the actuator, resolved from the same scenario holding configuration used by Gazebo. At first startup the actuator continues those configured PID targets, avoiding a measured-sag handoff step. There are no joint angles or PID constants in Python. Clock reset discards the prior-epoch target and still requires fresh feedback and a new source session; it cannot clear a latched physical fault.

## Physical overspeed: cause, correction boundary and required work

The reference fixes remove identifiable unsafe input steps. For example, the loaded pre-fix baseline had a shoulder reference slope of **2.0303 rad/s** despite a 1 rad/s limit, and its first measured-initialized handoff changed the shoulder target by **-0.04048 rad**. The pre-fix stress reproduction had a shoulder reference slope of **1.6102 rad/s**. The corrected recovery probe respected reference slope limits `[1,1,1,1.5] rad/s`.

However, the physical plant obeys

```
M(q) q_ddot + C(q,q_dot) q_dot + g(q) = tau_PID.
```

Bounding `dq_reference/dt` does not bound `q_dot`. The Gazebo plugin computes PID on position error and writes `JointForceCmd`, with effort saturation. References arrive at 50 Hz, while physics advances every 0.002 s. A reference increment changes the derivative term over a physics step: for a wrist ramp of 1.5 rad/s, a 0.02 s target increment and the unchanged D=0.5 give a possible derivative contribution of approximately `0.5 * (1.5*.02)/.002 = 7.5 Nm`, before plant coupling and saturation. Position error, gravity and arm coupling can also accelerate a joint beyond its reference slope.

The pre-fix stress trace measured wrist speed **5.9353 rad/s** at `t=4.850 s`; the reference command then was bounded. Finite differences independently confirm substantial physical overspeed, although missed samples can understate its instantaneous peak. This is not explained solely by bad joint-name ordering or a speed-reporting artifact.

Primary-source checks:

- [Gazebo Sim 8 JointPositionController](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/joint_position_controller/JointPositionController.cc): the effort branch updates PID and writes joint force; `initial_position` sets a target.
- [Gazebo Physics 7 DART construction](https://github.com/gazebosim/gz-physics/blob/gz-physics7/dartsim/src/SDFFeatures.cc): SDF position, force and velocity limits are copied into DART.
- [DART JointLimitConstraint](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/JointLimitConstraint.cpp): limits react to the current violated position/velocity; position-violation handling takes precedence on that degree of freedom. This is not a next-step velocity guarantee.

**Required actuator change for a hard speed claim:** a physics-step actuator safety layer that filters the PID torque against the coupled dynamics, next-step velocity/position constraints and existing effort bounds, with explicit infeasibility/braking handling and startup/contact tests. The existing gravity-holding PID can remain upstream of that filter. A verified effort-limited inner velocity servo is another actuator design, but merely selecting the position plugin's velocity-command mode would replace the current effort PID path and has not been adopted or certified here. Changing a 50 Hz Python position-reference adapter cannot supply the missing physics-step guarantee.

### Actionable actuator proposal — documented, not implemented

**Location and ownership.** Implement a single Gazebo System under `robot_simulation` (proposed source directory `robot_simulation/gazebo/plugins/` with compiled plugin build support). It must own the arm's effort command once per physics step. Reuse the existing externally configured position PID as the nominal gravity-holding controller, then filter its proposed torque before writing `JointForceCmd`. Do not attach a second independently scheduled force writer alongside the stock position plugin: that would introduce an ownership/order race. `robot_control`, WLS/DLS, command transport and the existing pure recovery policy remain simulator-independent/unchanged. Description geometry, inertias and limits remain authoritative; PID parameters and any validated uncertainty policy remain simulation configuration.

**Candidate mathematics.** Let `v=q_dot`, `h` be the actual physics step, `b=Cv+g`, and `M_eff` the positive-definite effective joint inertia for the modeled base/contact conditions. A candidate torque QP minimizes deviation from the existing PID torque, subject to effort bounds and predicted next-step constraints:

```
minimize_tau  (tau - tau_PID)^T W (tau - tau_PID)
v_next = v + h * M_eff^-1 * (tau - b + d)
q_next = q + h * v_next
-effort_limit <= tau <= effort_limit
-velocity_limit <= v_next <= velocity_limit
lower <= q_next <= upper
```

`W` is a positive metric, not new whole-body tracking weights. Disturbance `d` must include a justified bound for modeling error and base coupling; constraints must hold robustly over that bound, rather than assuming it is zero. A physical excursion already within the approved recovery band needs a feasible inward recovery constraint consistent with that policy, rather than an impossible instantaneous position correction. Infeasible constraints must enter an explicit latched braking/fault mode with bounded effort and reported cause; silently returning zero torque would remove gravity holding. This is a candidate one-step method, not a proved guarantee for DART as currently integrated.

**Assumptions that require validation.** The plugin needs current physics-state q/v, synchronized actual step duration, a validated coupled inertia/bias calculation, bounded external disturbances, and an accurate account of DART's discrete integration and constraint impulses. A fixed-base arm inertia approximation is insufficient while the mobile base accelerates. Contact impulses are not generally bounded by the motor effort limit and can defeat a motor-only velocity guarantee; either incorporate the applicable contact dynamics/bounds or explicitly delimit the certified operating conditions. Current ECM position/velocity feedback and description YAML do not themselves provide this validated dynamics/impulse model. The existing generic `ArmDynamics` utility accepts M/C/g as inputs; it does not construct or certify that effective model.

**Risks and decision.** Implementing this correctly changes compiled Gazebo actuator ownership and build integration, exposes solver/PID-state reset and saturation issues, and needs physics-step observability. The stock effort PID does not expose a ready-made, ordered safety-filter hook. A ROS-rate governor, a smaller reference lead, arbitrary gain changes, or simply lowering a solver speed setting would not prove the needed bound. A reliable implementation is therefore **not a focused patch within the current Python adapter architecture**. No speculative plugin or PID replacement was added during M02 closure. The next engineering action is an isolated physics-step actuator prototype with measured torque/state access, followed by the validation below; retain this milestone as CONDITIONAL until it passes.

**Validation gate before adoption.** First verify the dynamics predictor against DART one-step measurements under loaded gravity and moving-base conditions. Then record **every physics-step** q/v/torque through startup, both limits, reversals, near-limit velocities, retained-reference resume, recoverable excursions, stale transport, clock reset, paused time, and deliberately infeasible disturbances. Verify unchanged effort bounds, physical speed <= description speed + 0.001 rad/s, physical position policy <=0.001 rad, stable gravity hold, monotonic recovery authority, and latched serious-fault behavior. Exercise the existing baseline and unchanged wide scenario for the same 60/90 s horizons, comparing nominal tracking against the existing thresholds. Keep contact/impulse assumptions and prediction residual bounds in the evidence. Sampled ROS telemetry alone is insufficient for the acceptance claim.

No PID gains, physical bounds, speed acceptance tolerances, target dynamics, control weights, sensor rates, estimator behavior or physics timestep were changed to conceal this limitation.

## Final validation

The preserved four-run sequential comparison completed after the approved competing launch was stopped. A uses the unchanged oscillatory baseline for 60 simulated seconds; B uses the unchanged wide aggressive scenario for 90 seconds. Tracking and RTF use the interval after 10 s settling; physical safety includes every delivered startup sample. No further Gazebo run was needed on resume.

| Case | Mean / max distance error (m) | Mean / max pointing (deg) | Mean RTF | Recovery entries | Final controller valid |
|---|---:|---:|---:|---:|---|
| A_before | 0.090904 / 0.244885 | 8.4905 / 18.8733 | 0.7381 | 0 | True |
| A_after | 0.116723 / 0.317227 | 8.3383 / 18.1246 | 0.8292 | 0 | True |
| B_before | 0.127988 / 0.543798 | 24.0243 / 47.4457 | 0.7753 | 0 | True |
| B_after | 0.143271 / 0.596762 | 23.8431 / 47.2519 | 0.8660 | 0 | True |

All four quiet runs ended with active control and no latched physical position fault. **Neither quiet stress run reproduced the permanent boundary stop or entered recovery**; their zero recovery counts must not be interpreted as a live recovery test. The original live capture and `m02_B_before` reproduce the persistent stop, and the separately retained load-supported `m02_B_final` recovery probe exercised three recoveries, returned to valid control after its final position rejection at t=4.650 s, and remained valid through t=90.600 s. That probe predates the startup-handoff seeding correction; its recovery policy is the same load-supported policy. Pure guard/adapter tests deterministically exercise the boundary states, stale transport and latched faults.

The nominal mean distance error increased by 0.025819 m (0.090904 → 0.116723 m); mean pointing improved by 0.1522 deg. All four nominal tracking thresholds remain satisfied. Wide mean distance error increased by 0.015283 m; pointing changed little and remains about 24 deg. Its pointing and maximum-distance thresholds fail both before and after. These results do not establish geometric unreachability and are not a claim that wide tracking is solved. RTF exceeds 0.5 in all quiet runs; the single-run differences are not a certified performance improvement.

#### Physical extrema and speed violations

The following table uses every received raw joint sample, including startup. Configured speed limits are base_yaw/shoulder/elbow = 1 rad/s, wrist = 1.5 rad/s; violations use the unchanged additional 0.001 rad/s numerical acceptance. Durations are sampled estimates.

| Case | Joint | Min q (rad) | Max q (rad) | Max excess (rad) | Peak measured speed (rad/s) | Speed violation time estimate (s) |
|---|---|---:|---:|---:|---:|---:|
| A_before | base_yaw | -0.116622 | 0.110550 | 0 | 0.344161 | 0.000000 |
| A_before | shoulder | 0.000640 | 0.711297 | 0 | 1.041790 | 0.002000 |
| A_before | elbow | -0.457232 | -0.001831 | 0 | 2.066353 | 0.014000 |
| A_before | wrist | -0.409803 | 0.111020 | 0 | 3.077389 | 0.062000 |
| A_after | base_yaw | -0.113996 | 0.119501 | 0 | 0.677144 | 0.000000 |
| A_after | shoulder | 0.000640 | 0.674419 | 0 | 1.142974 | 0.018000 |
| A_after | elbow | -0.442053 | -0.001831 | 0 | 1.728889 | 0.024000 |
| A_after | wrist | -0.453144 | 0.077663 | 0 | 4.704151 | 0.058000 |
| B_before | base_yaw | -0.265577 | 0.245916 | 0 | 1.000000 | 0.000000 |
| B_before | shoulder | 0.000640 | 1.087195 | 0 | 1.347966 | 0.096000 |
| B_before | elbow | -0.500029 | 0.245366 | 0 | 2.492623 | 0.406000 |
| B_before | wrist | -1.241422 | 0.262196 | 0 | 5.847914 | 0.454000 |
| B_after | base_yaw | -0.235944 | 0.246032 | 0 | 1.000000 | 0.000000 |
| B_after | shoulder | 0.000640 | 1.067594 | 0 | 1.331196 | 0.058000 |
| B_after | elbow | -0.456931 | 0.119689 | 0 | 2.653499 | 0.434000 |
| B_after | wrist | -1.047283 | 0.217630 | 0 | 4.093137 | 0.272000 |

**Independent native-pose exception:** B_before includes a first-link elevation of -0.0001266334835 rad, even though that short startup excursion was absent from delivered raw joint states. It is inside the existing 0.001 rad position policy; it is not hidden or reclassified as zero. Native minima for A_before/A_after/B_after were +0.0006396310 / +0.0006396310 / +0.0098184422 rad. Minimum native link-box/base clearance was 0.0400000 / 0.0400000 / 0.0399493 / 0.0400019 m for A_before/A_after/B_before/B_after. No observed first-link base penetration occurred. This is sampled evidence, not a full self-collision guarantee.

Overspeed is not confined to startup. After settling, peak [base_yaw, shoulder, elbow, wrist] speeds were:

| Case | Settled peak speeds (rad/s) |
|---|---|
| A_before | 0.306022, 1.041790, 1.513280, 1.568890 |
| A_after | 0.326761, 1.142974, 1.728889, 1.572870 |
| B_before | 0.595324, 1.347966, 2.492623, 3.121394 |
| B_after | 0.522486, 1.331196, 2.653499, 4.093137 |

The nominal wrist peak increased from 3.0774 to 4.7042 rad/s over the full capture; in wide tracking it decreased from 5.8479 to 4.0931 rad/s. These are mixed physical results, **not** proof of speed enforcement. All four observer processes exited 1 because the measured-speed safety gate failed. The failure artifacts are retained; completed capture does not mean passed acceptance.

#### References, freshness, actuation and configuration audit

| Case | Peak reference slopes [yaw,shoulder,elbow,wrist] (rad/s) | Maximum active feedback / command age (s) | Final timeout count |
|---|---|---:|---:|
| A_before | 0.263130, 0.655102, 0.545270, 1.196241 | 0.020000 / 0.132000 | 0 |
| A_after | 0.646816, 1.000000, 1.000000, 1.500000 | 0.020000 / 0.108000 | 1 |
| B_before | 1.000000, 2.948153, 1.581918, 1.500000 | 0.020000 / 0.122000 | 0 |
| B_after | 1.000000, 1.000000, 1.000000, 1.500000 | 0.020000 / 0.106000 | 1 |

Timeout deltas after settling were zero in every run; the final-code counts of one occurred before the evaluation interval. The watchdogs remained enabled and rejected/future messages still cause diagnosed holds. Initial PID-target handoff steps were exactly zero in both final-code runs, versus approximately -0.040214 / -0.037709 rad at the shoulder before M02. All final-code reference slopes and targets stayed within their configured physical bounds; no zero-dt target jump was recorded. The quiet pre-fix stress run still showed shoulder/elbow reference slopes of 2.948153 / 1.581918 rad/s. Full counter/reason traces, reference/measured arrays and sample gaps are in each run directory.

Actual base excursions were 0.411170 m (A_after) and 0.813149 m (B_after); all four arm joints moved, with settled excursions [0.196141,0.241816,0.121624,0.434473] rad and [0.481675,0.693510,0.526874,1.209860] rad respectively. Live tracking TF excursions were 0.780020 and 1.101781 m. Target speeds averaged 0.277455 / 0.673509 m/s (A_after/B_after), with maxima 0.418550 / 1.019152 m/s. The wide target dynamics/configuration was preserved.

Generated SDF audits passed for all four cases: joint names/order/axes and position/velocity/effort limits match the description, PID gains and effort saturation match the holding configuration, and `use_velocity_commands=false`. The step is 0.002 s. The actual loaded libraries were Gazebo Physics DART 7.6.0 and DART 6.13.2 (`m02_physics_engine.json`). No new physics configuration was substituted during comparison.

| M02 acceptance item | Evidence / status |
|---|---|
| Boundary-stop mechanism understood | Demonstrated in original live capture and pre-fix reproduction; legal held target insufficient against gravity, strict estimated-position rejection and physical numerical boundary both identified. |
| Recoverable deadlock addressed | Pure tests and three-event load-supported live probe; quiet final run did not trigger it. Additional phase-locked fault injection would strengthen coverage. |
| Physical position policy | All observed excursions inside existing 0.001 rad envelope; native startup excursion disclosed; no full physics-step proof. |
| Physical speed acceptance | **FAIL** in baseline and stress, including after settling. |
| Serious fault behavior | Guard/adapter tests show latched stop across fresh feedback/session/reset. No destructive live serious-fault injection was performed. |
| Nominal tracking / RTF | Existing nominal tracking thresholds and RTF threshold satisfied; distance increased by 0.025819 m. Wide tracking thresholds fail. |
| Functional regression | Passing; see test results below. |
| Overall | **CONDITIONAL; M02 is not physically safety-accepted.** |


### Tests and audit scope

- Focused product tests: **67 passed** before the final comparisons, covering exact limits, both overshoot sides, tiny inside/outside perturbations, recovery direction, hysteresis, serious latching, reference discontinuities, reversal, startup handoff, reset, stale/missing/invalid commands, competing publishers and wall-watchdog recovery revocation.
- Relevant workspace regression: **819 passed, 18 lint tests deselected** before interruption (`validation/m02_final_regression.log`) and again after resume (`validation/m02_resume_final_regression.log`). No functional regression was found.
- Independent physical-metric tests: **3 passed** before interruption and on resume; `validation/m02_analysis_tests.log`, `validation/m02_resume_analysis.log`. Targeted product tests were also rerun on resume: **67 passed** (`validation/m02_resume_targeted.log`).
- Changed actuation/launch/test files pass flake8 at line length 99 (`validation/m02_changed_lint.log`). Existing double-quote style in the bounds module is excluded with `--extend-ignore=Q000`; no quote-only cleanup was made. A separate check of the existing Control ROS adapter still reports 11 pre-existing import-group/order findings (`validation/m02_resume_control_node_lint.log`); the newly introduced violation-class import was moved into its correct group. These style findings are separate from the passing functional regression; the workspace-wide style/copyright checks were not claimed as passing.
- Build: `robot_control`, `robot_simulation`, `robot_bringup` built successfully; final rebuild of simulation/bringup installs the archived reference preset. Its installed inheritance resolution was checked.
- Initial regression found two stale assumptions about the manually widened target scenario. The previous modest aggressive field is preserved under `*_aggressive_reference.yaml`; its original bounded experiment test remains, and a separate test pins the exact requested wide field. The user's wide field file was not modified.

Safety audits use **all delivered raw joint-state samples**, including startup and inactive intervals. Native Gazebo camera/first-link poses independently supply tracking and physical clearance. Distance/pointing errors therefore include a permanently stopped robot; they are not restricted to valid controller telemetry. Speed violation durations are left-held sample estimates, not exact physics-step durations; sample gaps and sample counts are retained in each `m02_metrics.json`. Short unobserved violations remain possible. Effort saturation is audited from generated configuration/source, not measured torque: the existing JointState stream does not provide a useful motor-effort trace.

The established acceptance tolerance remains `0.001 rad` for physical position and `0.001 rad/s` for speed. It is not a tolerance in the controller. Nominal tracking is assessed against the existing thresholds: mean radial error <=0.20 m, max <=0.45 m, mean pointing <=10 degrees, max <=25 degrees; RTF threshold 0.5. Before/after differences are descriptive single-run comparisons. Target initialization positions, velocities and seed are unchanged, but TF-readiness timing changes absolute sinusoidal forcing phase; reactive targets also respond to the changed live robot motion. This is not a statistically phase-locked controller benchmark.

## Reproduction

From `/home/user/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_control robot_simulation robot_bringup
source install/setup.bash
export GZ_PARTITION=m02_tracking GZ_IP=127.0.0.1 ROS_DOMAIN_ID=172
ros2 launch robot_bringup closed_loop.launch.py \
  config:=config/reactive_tracking_oscillatory_aggressive.yaml \
  strategy_config:=config/whole_body.yaml enable_actuation:=true \
  publish_ground_truth:=true visualize_tracking:=false gui:=false
```

For the original oscillatory baseline, replace config with `config/reactive_tracking_oscillatory.yaml`. For the previous modest aggressive experiment, use `config/reactive_tracking_oscillatory_aggressive_reference.yaml`. Compute-only mode remains available with `enable_actuation:=false publish_commands:=false`.

The exact final capture commands and return codes are in `validation/m02_comparison_runs.json`. The sequential runner is:

```bash
python3 docs/validation/run_m02_comparison.py
python3 docs/validation/m02_metrics.py docs/validation/m02_quiet_{A_before,A_after,B_before,B_after}
```

The runner intentionally refuses to overwrite existing evidence directories. Use new output paths when repeating. For an individual case, use the observer directly; e.g. final wide test:

```bash
ROS_DOMAIN_ID=133 python3 docs/validation/v2_tracking_gazebo.py \
  --case reactive --config config/reactive_tracking_oscillatory_aggressive.yaml \
  --strategy-config config/whole_body.yaml --physical-ground-truth --without-markers \
  --shoulder-posture --actuation-trace --duration 90 --settling-time 10 \
  --wall-timeout 300 --output /tmp/m02_wide_repeat
python3 docs/validation/m02_metrics.py /tmp/m02_wide_repeat
```

Add `--before-m02-actuation` for the archived pre-fix actuator reproduction. That option loads the archived reference/adapter source only in the observational validation wrapper; normal launch always uses production code. Safety-check failures return a nonzero exit code and are retained as failures even when capture completes.

Product regression command:

```bash
python3 -m pytest src --import-mode=importlib -p no:launch_testing -p no:launch_ros -q \
  -k 'not flake8 and not pep257 and not copyright'
PYTHONPATH="$PWD/docs/validation:$PYTHONPATH" python3 -m pytest \
  docs/validation/test_m02_metrics.py docs/validation/test_shoulder_experiment_metrics.py \
  --import-mode=importlib -p no:launch_testing -p no:launch_ros -q
```

## Changed files and remaining limitations

The exact M02 source/test/document list is [m02_modified_files.txt](validation/m02_modified_files.txt); prior dirty workspace work is retained and is not claimed as M02 work. Raw outputs and logs live under `validation/m02_*`. The approved user launch and its surviving Gazebo child were stopped for the final controlled tests; no unrelated processes were terminated. No commit or push was made.

Open limitations: physical speed acceptance; unconstrained estimator behavior near a hard stop and existing out-of-order rejection; no general self-collision avoidance; process-local fault persistence; sampled rather than physics-step safety observations; unmeasured actual motor effort; and reference-envelope discontinuities deliberately produce a diagnosed safe hold rather than an automatic resynchronization outside this narrow boundary-recovery policy. Successful tracking of every wide target pose is not asserted.

## Resume completion record

On resume, the original four-run comparison runner had already completed and no ROS/Gazebo process remained. Existing captures were retained; no duplicate simulation run was started. Completed the four derived physical audits, filled the final measurements/acceptance table, audited actual generated limits/PIDs/efforts, documented the actuator proposal and its assumptions/risks/adoption tests, created the exact M02 changed-file manifest, and reran targeted/analysis/functional tests. The only product-code edit on resume was placement of the newly added diagnostic import; control and actuation behavior was preserved. Physical speed acceptance remains unresolved and the milestone remains CONDITIONAL.
