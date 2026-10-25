# Reactive Tracking v3 — experimental results, 2026-10-15

Optional spatial circulation and one measured performance optimization are implemented. Existing scenario and controller defaults remain selected unless explicitly overridden. Headless performance and physical tracking pass the existing thresholds. Sustained substantial motion in all three dimensions is **not demonstrated**. Both tested arm preferences increased physical wrist overspeed and are **rejected for promotion**. The controller is **not finalized**.

**Dynamics and configuration ownership**

The existing engine provides target position/velocity and live tracking-frame position from TF; it provides no tracking-frame velocity. The experiment therefore uses a configured world-frame circulation axis, without differentiating TF or introducing state/history:

\[
s=p-p_C(t),\quad r=\|s\|,\quad n=n_{cfg}/\|n_{cfg}\|,\quad u=n\times s,
\]
\[
\ddot p=a_{max}\tanh((r-D)/\ell)\frac{s}{\sqrt{r^2+\epsilon^2}}
+a_t\frac{u}{\sqrt{u^Tu+\epsilon^2}}-\gamma v+a_{environment}.
\]

Here `a_t(r)` is constant: **0.06 m/s²**. Positive circulation follows the right-hand rule about `axis_world: [0.3, -0.8, 0.5]`, normalized internally. The axis is an explicit preferred direction; rotating it together with positions, velocities and the environment preserves rotation covariance. This is not an isotropic model with the axis held fixed. There is no XY restriction, world-Z fallback, random perturbation or prescribed target path.

The tangent is orthogonal to `s`, has norm at most one after regularization, and smoothly vanishes at coincidence or on the circulation axis. Zero target velocity needs no special treatment. Invalid/nonfinite or zero axes and negative/nonfinite strength fail at construction. The combined interaction is bounded by `hypot(0.15, 0.06) = 0.161555 m/s²`; damping and the unchanged centering field are separate terms. The measured short 3D run reached 0.159379 m/s² interaction acceleration. The positive damping/restoring field keeps the tested trajectories contained.

`TangentialInteractionParameters` composes with the existing `FightFlightDynamics` through both factory selectors. Omitted circulation preserves the old equation; explicitly zero strength is tested bitwise against the unchanged derivative. Production integration, measurement noise and TF freshness checks are unchanged.

| Parameter | Selected value | Owner |
| --- | --- | --- |
| `a_max`, `transition_length` | 0.15 m/s², 0.15 m | inherited `robot_simulation/config/scenarios/target_saturated.yaml` |
| `preferred_distance`, `damping`, `distance_epsilon` | 1 m, 0.5 s⁻¹, 1e-6 m | same unchanged parent |
| `parameters.tangential.acceleration`, `axis_world` | 0.06 m/s²; `[0.3, -0.8, 0.5]` | new `robot_simulation/config/scenarios/target_3d.yaml` |
| Initial target position / velocity | `[2, .25, 1]` m / `[0, .10, -.12]` m/s | inherited saturated scenario |
| External restoring field | center `[2.2, 0, .85]` m; gain `0.5 I` s⁻²; no stochastic fields | unchanged **`robot_simulation/config/scenarios/environment.yaml`** |

`reactive_target_3d.yaml` inherits `reactive_target_saturated.yaml` and changes only the target config path. `robot_bringup/config/reactive_tracking_3d.yaml` inherits the saturated Bringup selection. Target physics/dynamics stay in `robot_simulation`.

**Allocation experiment**

The existing WLS already penalizes each normalized generalized velocity separately, in order `[base v, base omega, arm yaw, shoulder, elbow, wrist]`. Baseline `whole_body.yaml` remains **`[1, 2, 3, 3, 3, 2]`**. The retained, explicitly experimental `whole_body_arm_preferred.yaml` uses **`[2, 4, 3, 3, 3, 2]`**; an initial `[4, 8, 3, 3, 3, 2]` trial was also rejected. All task costs, posture costs, damping, lookahead, physical limits and PID gains remain identical.

Control loads plain strategy YAML and does not support scenario `extends`. The first trial exposed this and produced no active control. Its evidence is preserved in `validation/v3_3d_arm_launch_failure/`. The experimental strategy now uses the supported complete YAML format, with a regression requiring that only base weights differ from `whole_body.yaml`. This avoids adding a simulation dependency or changing Control's loader/algorithm. Scenario inheritance still uses the existing loader.

**Controlled measurements**

Sequential runs used the same configured robot posture, target state, sensor seed, 30 simulated seconds, and a 10–30 s measurement window. The baseline before/after optimization used the same observer and startup profiler; profiling covers only the first 40 solves, before settling. Actual ROS/Gazebo scheduling and message order differ between runs: these are single-trial observations, not bitwise replay or statistical guarantees.

Errors below use native Gazebo base/wrist poses composed with the authoritative tracking mount and its selected **+X optical axis**, matched to measured target positions. Joint travel uses all received joint positions; base travel uses physical ground-truth XY poses. Travel includes actual PID motion. Speed peaks use all received joint velocities, rather than 20 Hz telemetry, and remain lower bounds if messages were missed.

| Run, 10–30 s | Saturated before optimization | Saturated after | 3D, original weights | 3D, base weights ×2 | 3D, base weights ×4 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Mean / max absolute distance error [m] | .0625 / .1673 | .0767 / .1678 | .0613 / .1371 | .0510 / .1572 | .0507 / .1382 |
| Mean / max pointing error [deg] | 4.58 / 11.13 | 2.92 / 9.04 | 3.45 / 10.16 | 6.69 / 11.37 | 3.73 / 9.89 |
| Target XYZ ranges [m] | .331 / .028 / .060 | .288 / .030 / .037 | .350 / .028 / .058 | .284 / .030 / .046 | .342 / .036 / .054 |
| Target mean / peak speed [m/s] | .0719 / .1690 | .0596 / .1271 | .0792 / .1527 | .0511 / .1384 | .0683 / .1408 |
| Base travel [m] | 2.624 | 2.255 | 2.758 | 2.290 | 2.426 |
| Cumulative absolute base yaw [rad] | .414 | .314 | .355 | .282 | .204 |
| Arm travel: yaw / shoulder / elbow / wrist [rad] | .888 / 1.739 / 1.406 / 2.208 | .733 / 1.502 / 1.253 / 1.992 | .792 / 1.704 / 1.393 / 2.177 | 1.202 / 1.766 / 1.389 / 2.255 | 1.545 / 2.119 / 1.652 / 3.086 |
| Settled wrist speed peak [rad/s], limit 1.5 | 1.558 | 1.500 | 1.524 | **1.550** | **1.574** |
| RTF | .5640 | .7296 | .6906 | .6790 | .6819 |

Compared with 3D/original, ×2 base penalties reduce base travel **17.0%**, increase arm-yaw travel **51.7%**, and change shoulder/elbow/wrist travel by **+3.6% / −0.3% / +3.6%**. Net base displacement is .245 m versus .244 m; reduced path length does not imply reduced net displacement. More elbow/wrist articulation is not established by this modest reallocation. The ×4 trial increased all arm travel substantially but worsened wrist overspeed. Neither setting meets the no-increased-physical-speed-violation requirement, so the longer run retains original weights.

Maximum target displacement from configured start was .506 m saturated-after, .518 m 3D/original, and .410 m ×2. Maximum displacement from the centering field stayed below .356 m in these short runs. Full precision metrics, per-joint excess-speed duration estimates, net displacement, and ten-second XYZ windows are in [reactive_v3_comparison.json](../results/validation/reactive_v3_comparison.json).

**Long run through the permanent launch**

`v3_selected_long/` ran the actual `robot_bringup closed_loop.launch.py` for **90 simulated seconds**, using 3D dynamics and original allocation. There were **no profiling, target, estimator or actuator prefix wrappers**. Independent ROS subscriptions and native Gazebo pose capture remained observational.

| Measurement, 10–90 s unless stated | Actual result |
| --- | --- |
| Absolute distance error, mean / maximum | .1019 / .2667 m |
| Physical +X pointing error, mean / maximum | 6.50 / 16.80° |
| Target mean / maximum speed | .0678 / .1688 m/s, from public-position finite differences |
| Target XYZ ranges | .3911 / .0302 / .0580 m |
| **Late XYZ ranges, 80–90 s** | **.2645 / .0040 / .0376 m** |
| Target path / maximum displacement from start | 5.431 / .451 m |
| Maximum distance from centering field, full run | .3554 m |
| Base path / net displacement / cumulative yaw | 9.978 m / .162 m / 1.648 rad |
| Arm travel: yaw / shoulder / elbow / wrist | 3.437 / 6.326 / 5.261 / 8.354 rad |
| Settled measured speed peaks, same joint order | .316 / .485 / .830 / **1.559** rad/s |
| **Full-run** measured speed peaks, including startup | 1.000 / 1.000 / **1.487 / 3.051** rad/s |
| Minimum shoulder / first-link box-to-base clearance, full run | .000640 rad / .040003 m |
| RTF / mean and p95 control computation | .7277 / 21.64 and 37.38 ms |
| Accepted / rejected commands / timeout events | 1797 / 0 / 0 |
| Reference-limit / feedback-rejection / inactive events | 4 / 1 / 2; existing guards retained |

The added force changes vertical motion and the equilibrium location, but **persistent substantial 3D excitation fails**: Y largely settles to an offset, while X dominates. A constant circulation force relative to a nearly fixed camera-target direction can balance the centering force. Merely adding this tangent does not guarantee an orbit. [XYZ traces](../results/validation/reactive_v3_xyz.png) show that limitation rather than hiding it with scaled animation.

**Performance diagnosis and the single optimization**

The four-CPU WSL2 host initially had the user's separate simulation running. It was stopped with explicit approval before all controlled runs. Historical RTF .45–.47 is therefore not a matched code-only baseline. Fresh comparison is **.5640 → .7296**, a 29.4% improvement.

Startup CPU profiling identified repeated kinematic rotation validation as the dominant measured Control hotspot: 920 `is_rotation_matrix` calls took .682 of 1.789 pipeline CPU seconds across 40 solves; WLS itself took .225 s. Generic `allclose/isclose` calls dominated that validation. The one production optimization replaces that fixed-3×3 check with equivalent elementwise tolerance comparisons, retaining `atol`, original `rtol=1e-5`, determinant validation, and rejection of nonfinite geometry. No physics/sensor/safety settings were reduced. Boundary/rotation/reflection/nonfinite tests verify the old acceptance contract.

A same-process microbenchmark recorded median CPU cost **143.2 → 36.3 µs/check**. Matched unprofiled pipeline wall/CPU means were **44.87/16.38 → 22.59/8.83 ms**. Settled telemetry computation fell **47.82 → 23.23 ms**. CPU scheduling remains material: wall time exceeds pipeline CPU time, host load was about 7–11 with four available CPUs, and CPU-pressure observations showed substantial runnable-task waiting. No cgroup quota throttling was demonstrated.

At 2 ms, physics and per-step joint/clock transport remain nominally 500 Hz. The direct long run observed approximately 404 joint messages/sim-second, 20 Hz physical poses, 20 Hz Control, 50 Hz robot full-state output, and 17.5 Hz target estimate/prediction. Sensor configuration remains 20 Hz noisy target measurements and 50 Hz wheel odometry. Approximate topic-count rates include startup; subscriber counts are not source publication guarantees. Post-optimization headless CPU usage (% of one CPU) was approximately Gazebo 44, bridge 23, simulation 42, estimation 48, prediction 25, Control 35, actuation 46, tracking TF 38, visualization 42. Shared scheduling plus physics/ROS transport remains the system-level constraint; physics-only timing was not isolated.

Matched 30 s **GUI RTF was .4991: FAIL**, versus .7296 headless. Aggregated Gazebo/GUI CPU rose from 44.5% to 117.3% of one CPU. Renderer hardware acceleration was not established. Marker/logging settings stayed identical: visualization is a measurable CPU consumer; no separate logging-off ablation was run. The direct 90 s log was only 37 kB, but that is not proof logging has zero cost. No further optimization is claimed.

**Acceptance and open issues**

- **PASS:** existing distance/pointing thresholds in every completed controlled run, all received joint position bounds including startup, shoulder horizontal/base clearance, bounded/finite target motion, live moving tracking TF, bounded safe commands, zero settled command timeouts, headless RTF ≥ .5, regression tests. The GUI run recorded one startup timeout; none occurred after settling. First-link clearance is not complete self-collision avoidance.
- **FAIL:** measured physical joint-speed limits. Short saturated-after passes the existing *settled* gate, but startup still reaches wrist 2.392 rad/s. 3D/original reaches 3.414 at startup; ×2 reaches **6.614**. Long-run full peaks above expose startup violations that the existing ten-second settling window excludes. Settled violations also persist. Bounds/gates were not loosened, and no velocity limiter was silently added to the effort PID.
- **FAIL:** no-increased-speed-violations for either arm allocation candidate; neither is selected by default or for the long run.
- **FAIL / limited:** sustained substantial three-axis target excitation; GUI performance threshold.
- **OPEN, unchanged:** estimator's shared robot timestamp watermark rejects legitimate out-of-sequence joint/odometry messages. The long log reports at least **17,684 timestamp rejections**. The prior [PID/timing investigation](saturated_fight_flight_milestone.md) remains applicable; stale updates have not been accepted and no estimator/predictor changes were made.

Build succeeded for `robot_simulation`, `robot_control`, `robot_bringup` with existing symlink-installed models. Focused pre-Gazebo tests: **57 passed**; rotation/model checks: **28 passed**. Functional regression: **769 passed, 18 lint checks deselected**; strengthened spatial rank/determinism tests: **11 passed**; native-pose observer regression: **2 passed**. New tests and the changed rotation utility pass flake8. Existing unrelated lint failures in legacy simulation files were not repaired; the earlier changed-files lint log records them. `git diff --check` passes. No commit or push. Benchmark cleanup found no remaining owned ROS/Gazebo processes; the user's approved-stopped launch remains stopped.

**Reproduction commands**

From `/home/user/ros2_ws`, rebuild changed packages and choose an unused ROS domain / Gazebo partition:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_models robot_control robot_simulation robot_bringup
source install/setup.bash
export ROS_DOMAIN_ID=175 GZ_PARTITION=reactive_v3_demo GZ_IP=127.0.0.1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
```

Headless selected experiment (original allocation; safety limitations above remain):

```bash
ros2 launch robot_bringup closed_loop.launch.py config:=config/reactive_tracking_3d.yaml strategy_config:=config/whole_body.yaml enable_actuation:=true visualize_tracking:=true gui:=false
```

GUI, after stopping the headless launch:

```bash
ros2 launch robot_bringup closed_loop.launch.py config:=config/reactive_tracking_3d.yaml strategy_config:=config/whole_body.yaml enable_actuation:=true visualize_tracking:=true gui:=true
```

For saturated baseline use `config:=config/reactive_tracking_saturated.yaml`. For the **rejected allocation experiment only**, select `strategy_config:=config/whole_body_arm_preferred.yaml` (×2). Default DLS selection and Compute Only remain available: omit `strategy_config` for DLS; set `enable_actuation:=false publish_commands:=false` for Compute Only.

Reproduce the direct long observation (a nonzero exit is expected for physical speed failures):

```bash
python3 tools/validation/v2_tracking_gazebo.py --case v2 --config config/reactive_tracking_3d.yaml --strategy-config config/whole_body.yaml --physical-ground-truth --shoulder-posture --duration 90 --wall-timeout 260 --output /tmp/reactive_v3_check
python3 -m pytest src --import-mode=importlib -p no:launch_testing -p no:launch_ros -q -k 'not flake8 and not pep257 and not copyright'
MPLCONFIGDIR=/tmp/reactive-v3-matplotlib python3 tools/validation/analyze_reactive_v3.py
```

**Exact source/config/test/document changes for this task**

1. `src/robot_simulation/robot_simulation/dynamics/target/fight_flight.py`
2. `src/robot_simulation/robot_simulation/factory/simulation_factory.py`
3. `src/robot_simulation/config/scenarios/target_3d.yaml`
4. `src/robot_simulation/config/scenarios/reactive_target_3d.yaml`
5. `src/robot_simulation/test/test_tangential_fight_flight.py`
6. `src/robot_bringup/config/reactive_tracking_3d.yaml`
7. `src/robot_bringup/test/test_reactive_v3_configuration.py`
8. `src/robot_control/config/whole_body_arm_preferred.yaml`
9. `src/robot_models/robot_models/common/rotations.py`
10. `src/robot_models/test/test_rotation_validation.py`
11. `tools/validation/v2_tracking_gazebo.py` — reusable strategy/GUI/profile selection, correct selected-weight snapshot/cost, and estimate/prediction rate observations.
12. `tools/validation/control_compute_profile.py`
13. `tools/validation/analyze_reactive_v3.py`
14. `results/validation/reactive_v3_modified_files.txt`
15. `docs/reactive_tracking_v3.md`

Generated evidence is under `results/validation/v3_*`, plus `reactive_v3_comparison.json`, `reactive_v3_analysis_console.log` and `reactive_v3_xyz.png`. Resolved configuration snapshots preserve both rejected weight sets; archived launch paths describe their original recording locations. Earlier uncommitted work and validation artifacts were preserved.
