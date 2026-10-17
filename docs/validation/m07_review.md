# M07 review closure

Reviewed against the original M07 request and m07_plan.md, preserving baseline
421b773, the existing implementation, and historical artifacts. This record is
limited to M07; it does not close M02 physical overspeed or M03 localization risks.

The existing independent reviewer reported startup validation gaps and then hit
its usage limit before a final follow-up. The findings were addressed and checked
locally. There is no clean final independent-review verdict.

| Finding / integration defect | Focused correction | Evidence |
|---|---|---|
| Estimator publication/reordering startup settings escaped preflight | Share the existing owner runtime validation with preflight and node | m07_resume_red.log; focused/full green logs |
| Inherited owner ROS YAML worked in preflight but not ROS parameter-file parsing | Deliver merged typed dictionaries and use inherited strategy loader at runtime | configuration launch-action tests; control node inherited-strategy test; normal unified live run |
| Primitive ROS DOUBLE values and empty joint-name metadata could fail launch typing | Normalize validated numeric values; derive joint metadata from physical description | parameter/launch tests and actual Gazebo startup |
| Simulation resource typo/dimension settings lacked early deployment validation | Owner factory validates existing dynamics/environment/integration/measurement schemas | test_deployment_configuration.py; full green log |
| Actuator startup checks were incomplete in preflight | Reuse owner create_actuation_components in runtime and preflight; PositionGuard/PositionReference unchanged | m07_actuation_factory_red.log; factory tests; all three physical runs |
| Predictor horizon could be shorter than control lookahead | Validate the selected owner's horizon against selected control lookahead | m07_horizon_red.log and focused green |
| Observer snapshot could not serialize resolved paths | Serialize resolved paths as strings in the existing observer | failed first attempt preserved; subsequent snapshots and complete runs |
| Package containment rejected legitimate symlink installation | Validate lexical share containment before following installed resource symlinks | m07_symlink_red.log; test_resources.py; actual symlink-install Gazebo startup; copied-install checker |

Final checks on the final production source:

- Full functional regression: 986 passed; 18 existing style/copyright checks
  deselected. No functional exclusions were introduced.
- Focused suite: 128 passed, including 52 architecture boundary/tree tests.
- Normal and copied all-nine-package builds: both exit 0.
- Copied installation from /tmp with workspace Python/ament paths cleared:
  7 console entries, 68 resources, real nominal/CV/DLS identities, negative example.
- Actual ROS negative launch: informative CA rejection, no process startup.
- Actual nominal/legacy/CV node identity logs and complete closed-loop streams.
  Physical-speed gate remains failed; position/command/watchdog/posture gates pass.
- git diff --check passes. No physical limits, PID configuration, controller
  equations, PositionGuard/PositionReference algorithms or target dynamics changed.

See m07_runtime_evidence.json and ../m07_unified_system_configuration.md for
numerical results and limitations. Historical untracked data remain intact.
