# M02 focused implementation plan

Preserve the manually widened stress scenario and all existing algorithms/limits.

1. Capture the existing stop and trace per-joint physical state, legal reference, and rejection.
2. Keep strict model/controller bounds. Add structured violation details and named ROS diagnostics.
3. Add a pure simulation-local recovery policy: classify numerical excursions separately from recoverable excursions within the existing 0.001 rad acceptance envelope; latch serious faults; use fresh, authenticated command heartbeats and fresh feedback to slew affected references inward with hysteresis. Missing/stale/invalid transport must hold, not recover. Preserve reference continuity on ordinary resume.
4. Test boundaries, directed recovery, latching, timing, reversal, reference continuity, watchdog and reset before controlled Gazebo runs.
5. Compare the unchanged oscillatory baseline and unchanged wide stress before/after. Record all received physical measurements including startup. Do not claim physical-speed acceptance based on safe command slopes. If effort PID + delayed discrete solver limits cannot provide the required bound, report CONDITIONAL and specify the actuator change required.
6. Run relevant regression once; document exact changes, measurements, commands and verdict in docs/m02_physical_actuation_safety.md. No commit/push, target retuning, PID tuning, or estimator restructuring.

Evidence captured so far: live shoulder q=-3.8847e-13 rad against lower=0; actuator holding shoulder reference 0.0036351 rad; 13,238 inactive commands, timeout_events=0. This is a numerical boundary deadlock, not a simulator freeze. DART JointLimitConstraint checks current velocity violations, not next-step predictive velocity; a reference slew bound cannot prove physical velocity safety.
