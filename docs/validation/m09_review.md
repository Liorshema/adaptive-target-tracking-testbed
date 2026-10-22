# M09 review and closure

Baseline bf71fdb; dedicated feature/m09-visualization branch. Independent read-only
review inspected pure scene/config/geometry, ROS source-time transforms, stable
replacement outputs, display TF, launch isolation and physical visual changes.
The reviewer ran 65 scene/adapter/launch tests and found two concrete RViz issues:

1. RobotModel lacked its TF prefix. Installed Jazzy plugin inspection confirmed
   RViz inserts '/' itself: its property is visual, while RSP uses visual/.
2. Custom YAML topics/prefix/frame/path colors were not bound into the static
   RViz template. The new pure configured_rviz helper binds all of them, with
   source immutability/layer tests; the wrapper supplies that generated config.

Both findings were corrected and the reviewer rechecked the helper, reporting no
remaining critical issue in the bounded reviewed scope. Root regression tests
also corrected namespaced RSP's subscription to the actual joint-state topic.
M09 does not change control-facing TF or any actuator command publisher.

Recorded RED→GREEN evidence includes m09_scene_*, m09_adapter_*, m09_launch_*,
m09_camera_*, m09_rsp_remapping_red.log and m09_rviz_prefix_{red,green}.log.
The camera tests compare complete URDF/SDF nonvisual trees to baseline, including
all collision/inertia/joint properties; both signatures match exactly.

Initial live evidence is preserved rather than overwritten. The first runtime
verification had a dict timestamp parsing bug after a completed/cleaned run.
Two GUI probe variants shared an executor with the high-rate physical recorder;
late diagnostic TF/Path collection was starved. A separate read-only display
probe executor resolved acquisition and all 27 GUI gates then passed. No runtime
age threshold or controller/physics parameter was relaxed.

Actual RViz/Gazebo clients ran (RViz OpenGL4.5) but every automated X11 window
capture was black. Captures are labelled unusable and are not shown as proof of
appearance. The final report distinguishes GUI process/topic evidence from
unverified pixel-level appearance. No further WSL graphics investigation or UI
work was attempted. Physical overspeed and incomplete GUI feedback coverage remain
failed/unknown safety respectively. No visual motion is claimed as safety proof.

Final verification: 1142 functional tests, 126 focused/architecture tests, both
ten-package builds, copied-install checks, real headless/GUI publication,
negative preflight and display-disabled compute-only smoke. Existing style tests
remain excluded from the functional count. Runtime ownership cleanup has no
survivors; some baseline adapters log shutdown interrupts and the ordinary disabled
smoke required signal escalation. See main report and m09_live_evidence.json.

No commit/push; historical data and protected algorithm/physics files preserved.
