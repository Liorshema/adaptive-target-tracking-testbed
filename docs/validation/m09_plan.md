# M09 implementation plan and design

Baseline bf71fdb, feature/m09-visualization, clean tracked tree. Preserve all local
historical evidence/M08 outputs. User authorizes autonomous completion/no commits;
skill approval/commit gates do not add permission requirements.

Extend existing tracking_visualization executable. New pure visualization domain
owns strict config, bounded history, geometry and semantic scene state; node owns
ROS/TF/messages. Standard Paths for true/estimated target and robot histories and
latest future forecast, MarkerArray for camera/axes/LOS/radius/physical warnings.
Stable IDs with full-snapshot DELETEALL prevent accumulation. Stale/missing layers
clear independently; clock reset clears all histories; source epochs remain explicit.
RobotModel uses RSP visual/ frame prefix and physical pose TF so no control-facing
tracking_frame authority changes. No command publishers. No robot_experiments dep.

Visual-only camera body/lens attached at authoritative tracking mount, dimensions
derived from wrist dimensions; explicitly schematic, not invented physical camera
intrinsics. Compare generated URDF/SDF physical elements before/after. Existing
world/base/target materials retained or changed only visually if needed. GUI wrapper
includes existing closed_loop launch exactly once, separately controls Gazebo/RViz,
optional GUI failures do not kill control; headless markers need no display.

- [x] 1. Pure configuration/history/geometry/scene state RED→GREEN.
- [x] 2. Read-only existing ROS adapter, five paths, TF/model integration.
- [x] 3. Visual-only camera representation and physical invariance tests.
- [x] 4. Installed visualization launch + RViz config, strict preflight/GUI checks.
- [x] 5. Focused/full functional/architecture tests and all-package/copied builds.
- [x] 6. Real headless markers/paths/TF/math, GUI smoke/capture if display works.
- [x] 7. Independent review, required fixes, preservation, docs/inventory/verdict.

Review focus: source-vs-future timestamps and stale forecasts, transform directions,
physical-vs-estimated provenance, prefixed display TF isolation, bounded history and
marker deletion, layer failures independent of control, GUI unavailable behavior.

## Final closure — 2026-10-18

Completed the existing plan without commits or pushes. Baseline remains bf71fdb.
1142 functional tests passed (18 existing style/copyright tests excluded), 126
focused/architecture tests passed, both ten-package builds and the copied install
checker passed. Actual headless and GUI instrumentation passed all 27 display
gates. No usable GUI screenshot could be captured (uniformly black X11 images);
this is explicitly not presented as visual proof. Independent review findings
were corrected with RED→GREEN tests. Physical properties remain equal to M08;
physical overspeed, sampling/RTF and teardown limitations remain documented.
Verdict CONDITIONAL. Stop before M10. See main report and final verification JSON.
