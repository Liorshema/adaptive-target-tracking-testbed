# Whole-body v2 audit and implementation sequence

Baseline: HEAD `0a87773` (Closed-loop v1). Existing untracked validation evidence
is preserved. No commit/reset/clean/push is authorized.

Verified findings:

- Control selects Cartesian axes [0,1,2], damping=1, zero reference twist and
  the nearest radial sphere point. Camera pointing is not an active task.
- Target selection rounds up to a future sample instead of interpolating;
  target velocity is not consumed by Control.
- Full wrist Jacobian rank is 4 in the raised planar posture and 5 in a sampled
  yawed/bent posture, never 6 in these representative configurations. Six
  actuators do not imply arbitrary six-axis pose controllability.
- The permanent Gazebo PIDs, DiffDrive, bridges and safe-command boundary are
  present. Prior measured v1 evidence shows physical base/joint response.
- World physics uses 1 ms steps. Joint-state callbacks occur at physics rate;
  timeout semantics mix simulation freshness with independent wall arrival.
  Actual RTF/CPU/GUI/threading and rejection measurements are underway before
  selecting performance changes.
- Prediction's factory instantiates IMM(CV/CA/CT), ResidualAdapter and
  PerformanceMonitor. IMM likelihoods compare the first .1 s forecast to .05 s
  measurements: a verified temporal-alignment defect.
- Model construction is duplicated between Bringup and Control. The camera
  mount is applied through two alternative paths; it is currently applied once
  in each. Scenario YAML repeats the entire base launch configuration.

Implementation sequence:

1. Record existing production RTF, CPU, topic/callback rates and watchdog events.
   Optimize only measured bottlenecks; retain gravity, collisions, PID and
   independent failure detection. Validate holding before proceeding.
2. Add reusable range and S² logarithmic pointing geometry to robot_models.
   Its three-row task has one distance condition and two pointing conditions;
   optical roll is free. Verify world/body signs and target-velocity derivatives
   by independent finite differences. Define an explicit antipodal escape chart.
3. Register interchangeable Control tasks and a bounded weighted least-squares
   controller using the already installed SciPy optimizer. Normalize task units
   and velocities, add configurable posture/motion/smoothing costs, and reuse
   SafetyPipeline's feasible bounds with independent final projection.
4. Keep v1 unchanged as a selectable baseline. Add timestamp interpolation and
   optional target feedforward/lookahead. Fix the proven Prediction alignment
   defect with regression tests.
5. Consolidate physical model construction and joint metadata; compose scenarios
   using explicit base-plus-mapping overrides. Add measured tracking TF only for
   the optional reactive scenario. Do not copy geometry into deployment YAML.
6. Build and run functional regressions, existing safety checks and matched
   hold/v1/v2/lookahead/feedforward Gazebo experiments. Archive raw measurements,
   plot distance/pointing/base/joints/RTF, and report actual failures/limitations.

Compatibility risks: clock epoch/replay behavior, bounded references under
slow simulation, antipodal pointing charts, model mount ownership and old factory
configuration contracts. Defaults retain Compute Only and existing DLS; v2 is
selected by deployment configuration. No new ROS package or solver dependency.
