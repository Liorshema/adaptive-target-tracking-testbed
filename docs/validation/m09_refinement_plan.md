# M09 refinement scope and evidence plan

Continue uncommitted M09 on bf71fdb. Current edits and historical files inventoried
in m09_refinement_baseline.json; do not commit or push.

- [x] Read existing implementation, evidence and new requirements.
- [x] Test-first: configurable camera-only default TF display, 8 s trails, current
  target markers, independent path widths, framing and secondary history visibility.
- [x] Break trails on configured source gaps/jumps or rejected invalid states;
  retain exact sample values, frame/order/reset protections and complete TF.
- [x] Reuse recorded M08/M09 target diagnostics: source-time errors, roughness,
  covariance, valid innovation reconstruction, timing, offline sensitivity if justified.
- [x] Reuse recorded physical joint/command diagnostics: named mapping, integrals,
  violations/coverage, spike events, allocation and coupling uncertainty.
- [x] Focused and full functional regression; ten-package normal/copy installation.
- [x] Bounded compute-only + GUI compatibility verification of installed resources.
- [x] Reports, plots, precise changed files, preservation and conditional verdict.

No changes to estimator/controller/predictor/PID/physics parameters or algorithms.
Analytical diagnostic kernels live only in robot_experiments, ROS adapters remain
in bringup. Recorded data are immutable. Existing M09 verification remains history.
