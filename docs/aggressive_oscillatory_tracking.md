# Aggressive Oscillatory Tracking — 2026-10-15

Selected experiment: **B, aggressive target with original Whole-Body allocation**. Sustained Y/Z evasion and tracking gates pass in headless and GUI runs. **Overall safety acceptance remains FAIL:** physical joint overspeed persists. C is rejected; defaults and previous experiments remain unchanged. No commits or pushes.

**Configuration and allocation**

The existing sinusoidal field evaluates `a_i(t) = A_i sin(2π f_i t + phase_i)`: acceleration in m/s², frequency in Hz, phase in radians.

| Field | A, unchanged | B/C |
|---|---|---|
| Amplitude | `[0, .35, .30]` | `[0, .55, .45]` |
| Frequency | `[0, .20, .30]` | `[0, .25, .35]` |
| Phase | `[0, 0, 1.57079632679]` | unchanged |

New environment/scenario/Bringup YAMLs inherit the oscillatory selections. The loader replaces lists, so the child environment repeats the identical centering entry; a test checks equality. The actual field remains `linear_restoring`, center `[2.2, 0, .85]`, gain `0.5 I`, with no stochastic fields. Existing saturated radial dynamics, damping .5, tangential circulation .06, RK4, geometry and live tracking-position TF feedback are retained. No target trajectory or scenario branch was added to production dynamics code.

WLS uses the motion term `λ² Σ w_i (u_i / velocity_scale_i)²`, alongside its existing task, posture, smoothness and limit costs. The order is `[base linear, base angular, arm yaw, shoulder, elbow, wrist]`. Original weights are **`[1, 2, 3, 3, 3, 2]`**; C uses **`[3, 4, 3, 3, 3, 2]`**. These triple the normalized base-linear penalty and double base-yaw penalty; they do not change actuator limits. Control loads plain strategy YAML, so the separate candidate uses a complete configuration, with a regression ensuring that only these weights differ. The production `whole_body.yaml`, prior candidates, PID gains, estimation and prediction are untouched.

**A/B/C comparison**

Each run lasted 30 simulated seconds with the same configured target initial position `[2, .25, 1]`, velocity `[0, .10, -.12]`, robot posture and sensor seed. Unless marked “full”, metrics cover **10–30 s**. All three retain physical ground truth and omit the unused RViz worker. Errors use native Gazebo poses, authoritative camera geometry and the actual **+X optical axis**, at radius R=1 m. Speed is sampled position finite differences; joint peaks use every received joint-state message. Missed messages make peaks lower bounds.

Configured initial conditions match, but ROS readiness starts the oscillator at global simulation times **1.502 / 1.848 / 1.410 s** in A/B/C. Effective forcing phases and timestamps are recorded in JSON. These single trials are not phase-locked or bitwise replay, so motion/accuracy differences cannot be attributed solely to weights.

| Metric | A: current oscillatory | B: aggressive, original allocation | C: aggressive, arm preference |
|---|---:|---:|---:|
| Target Y range [m] | 0.589 | 0.542 | 0.546 |
| Target Z range [m] | 0.221 | 0.252 | 0.248 |
| Target maximum sampled speed, full / settled [m/s] | 0.482 / 0.412 | 0.559 / 0.486 | 0.585 / 0.486 |
| Y / Z reversals | 8 / 12 | 10 / 14 | 10 / 14 |
| Base traveled distance [m] | 2.610 | 2.860 | 2.695 |
| Base net displacement [m] | 0.218 | 0.317 | 0.280 |
| Cumulative absolute base yaw [rad] | 0.801 | 0.988 | 0.771 |
| Arm-yaw joint travel [rad] | 1.530 | 1.805 | 2.709 |
| Shoulder joint travel [rad] | 2.085 | 2.034 | 2.294 |
| Elbow joint travel [rad] | 1.647 | 1.663 | 1.818 |
| Wrist joint travel [rad] | 2.842 | 2.885 | 3.203 |
| Mean absolute distance error [m] | 0.152 | 0.056 | 0.049 |
| Mean physical pointing error [deg] | 9.335 | 7.642 | 10.997 |
| Maximum measured wrist speed, full / settled [rad/s] | 3.524 / 1.567 | 3.146 / 1.545 | 2.392 / 1.565 |
| RTF | 0.7156 | 0.7686 | 0.7221 |
| Watchdog events, total / settled | 0 / 0 | 0 / 0 | 0 / 0 |
| Observed geometrically unreachable events | 0 | 0 | 0 |

B's last ten seconds still span **.525 m in Y / .234 m in Z**. Its peak sampled speed increases, while its Y range is slightly smaller than A: higher forcing frequency does not guarantee larger displacement. B stays within **.516 m of the centering field**, with full-run Z **.767–1.182 m**. No runaway or parameter sweep occurred; the initial candidate was retained. [Measured XYZ and pointing traces](validation/aggressive_oscillatory_tracking.png).

All observed target positions have a static geometric certificate: authoritative FK constructs `p_target = p_camera + R * optical_axis` using a valid straight-arm posture plus free base XY placement on flat ground. The certified height interval is **.430–2.310 m**; representative bounded IK residuals are below 1e-12 m. This does not certify dynamic/nonholonomic feasibility or complete collision avoidance. No IK logic was added to Control.

C reduces base path **5.8%** and raises yaw/shoulder/elbow/wrist travel **50.0% / 12.8% / 9.3% / 11.0%** relative to B. Its pointing mean exceeds the unchanged **10°** gate and its speed comparison fails. Increased motion is insufficient grounds for promotion; **original allocation remains selected**.

**Physical safety**

Peak vectors below are ordered `[arm yaw, shoulder, elbow, wrist]`; authoritative limits are **`[1, 1, 1, 1.5] rad/s`**. Existing numerical tolerance .001 is unchanged. Startup is included in the full audit.

| Case | Full-run peaks [rad/s] | Settled peaks [rad/s] |
|---|---|---|
| A | 1.003 / 1.170 / 1.673 / 3.524 | 0.277 / 1.170 / 1.673 / 1.567 |
| B | 0.281 / 1.020 / 2.039 / 3.146 | 0.281 / 0.792 / 2.039 / 1.545 |
| C | 0.525 / 1.086 / 1.902 / 2.392 | 0.416 / 0.927 / 1.902 / 1.565 |
| B GUI | 0.373 / 1.097 / 1.790 / 2.592 | 0.310 / 0.531 / 1.390 / 1.552 |

C increases settled wrist peak **1.545 → 1.565 rad/s** and observed wrist overspeed interval duration **.010 → .042 s**, despite reducing its full-run peak. Full shoulder peak also increases. B itself has significant elbow/wrist violations; it is an opt-in simulation experiment, not a safe actuator milestone. Bounds on commanded velocity pass, while actual effort-PID joint speeds fail. Per-joint samples, episodes and interval-duration estimates for every measured violation are in [the full comparison](validation/aggressive_oscillatory_comparison.json); durations across missing samples are estimates, not continuous-time proofs.

Position/command bounds, first-link clearance and live TF gates pass within existing tolerances. Shoulder minimum remains configured at 0; A/C briefly measure −.000127 rad, within the existing tolerance. First-link box clearance remains approximately 4 cm. A/B/C command rejections are **0/1/15**, feedback rejections **0/1/1**, and watchdog totals **0/0/0**. Guards were retained. B's log still reports **5,855 timestamp rejections**; shared estimator timestamp ordering remains open. No stale-update acceptance, PID tuning, limit widening or actuation redesign was introduced.

**Performance**

One localized launch improvement separates physical ground-truth publication from the optional RViz marker worker. New `publish_ground_truth:=true` works with `visualize_tracking:=false`; the default empty value preserves old behavior. Visualization still enables ground truth automatically; contradictory explicit settings fail early. The validation helper adds `--without-markers` and captures robot deployment metadata. All physical sensors, physics, estimation, prediction and safety remain active.

| Same oscillatory A deployment, 30 s | Markers enabled | Ground truth retained, markers omitted |
|---|---:|---:|
| Headless RTF | .6526 | .7156 |
| GUI RTF | .4697 | .5138 |

This yields **9.7% / 9.4%** measured headless/GUI throughput improvements. The unused worker consumed about **39% of one CPU**. The four-CPU WSL host shows scheduling/rendering pressure: baseline pipeline wall/CPU means were 26.66/10.22 ms, and GUI Gazebo processes consumed about **119% of one CPU**, versus 45% headless. Startup profiling spent .198 of 1.096 pipeline CPU seconds in WLS across 40 solves; optimizing that solver was not justified as the principal remedy. Mean full-pipeline telemetry times for A/B/C were **23.58/20.72/22.62 ms**, with B GUI **34.87 ms**. Physics remains **2 ms / 500 steps per simulated second**. Approximate observed robot-estimate rate is 49–50 Hz, Control 20 Hz, prediction/target estimate 17 Hz; source sensor rates are unchanged. CPU/topic traces are preserved per run. Logging remains roughly 20–23 kB per run and was not disabled.

The selected B ran at **RTF .7686 headless / .5097 GUI**, with physical pointing means **7.64° / 7.83°** and mean distance errors **.056 / .059 m**. GUI margin above .5 is small and hardware acceleration was not established. A's marker-free GUI run passed RTF but failed mean pointing at **11.27°**; performance gains do not guarantee tracking quality. An earlier GUI capture overlapped tests, was retained as `aggressive_A_gui_before`, and was excluded from the matched performance table in favor of a quiet repeat.

Practical settings: use headless mode for captures and leave RViz markers off when only Gazebo is being viewed. A separate `gz sim -g --force-version 8` client in the same partition can be closed without ending headless tracking. No further renderer, sensor-rate or physics changes are claimed.

**Validation verdict**

- **PASS:** sustained measured Y/Z evasion; bounded practical workspace; observed geometric reachability; selected B distance/pointing thresholds (mean .2 m / 10°, maximum .45 m / 25°); headless and GUI RTF ≥ .5 in these trials; command/position/first-link safety gates.
- **FAIL:** physical measured speed limits, including startup; C's mean pointing and no-worsening-speed requirements. The controller is not finalized.
- **OPEN:** timestamp ordering, actuator speed enforcement, longer-duration/statistical validation and GUI/WSL margin.

Build: 3 packages succeeded. Focused configuration/dynamics/marker tests: **19 passed**. One final functional regression: **784 passed, 18 existing lint checks deselected**. Independent reachability/native-pose tests: **4 passed**. Changed launch/new package tests pass flake8; `git diff --check` passes. All benchmark processes were cleaned up. The approved-stopped user launch remains stopped. Existing work was preserved.

**Exact launch commands**

From `/home/user/ros2_ws`, with other tracking launches stopped:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_control robot_simulation robot_bringup
source install/setup.bash
export ROS_DOMAIN_ID=176 GZ_PARTITION=aggressive_oscillatory GZ_IP=127.0.0.1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
```

Selected B, headless:

```bash
ros2 launch robot_bringup closed_loop.launch.py config:=config/reactive_tracking_oscillatory_aggressive.yaml strategy_config:=config/whole_body.yaml enable_actuation:=true publish_ground_truth:=true visualize_tracking:=false gui:=false
```

GUI, after stopping the headless launch:

```bash
ros2 launch robot_bringup closed_loop.launch.py config:=config/reactive_tracking_oscillatory_aggressive.yaml strategy_config:=config/whole_body.yaml enable_actuation:=true publish_ground_truth:=true visualize_tracking:=false gui:=true
```

For A select `config:=config/reactive_tracking_oscillatory.yaml`. C reproduction only: select `strategy_config:=config/whole_body_oscillatory_arm_preferred.yaml`; this candidate is rejected. Compute Only remains `enable_actuation:=false publish_commands:=false`.

Reproduce the direct B observation (nonzero exit is expected for physical speed failures):

```bash
python3 docs/validation/v2_tracking_gazebo.py --case v2 --config config/reactive_tracking_oscillatory_aggressive.yaml --strategy-config config/whole_body.yaml --physical-ground-truth --without-markers --shoulder-posture --duration 30 --wall-timeout 210 --output /tmp/aggressive_check
MPLCONFIGDIR=/tmp/aggressive-matplotlib python3 docs/validation/analyze_aggressive_oscillatory.py
```

The analyzer regenerates the saved A/B/C comparison; the fresh observation is in `/tmp/aggressive_check/result.json`.

**Created/modified source, configuration, test and report files**

- `docs/aggressive_oscillatory_tracking.md`
- `docs/validation/aggressive_modified_files.txt`
- `docs/validation/analyze_aggressive_oscillatory.py`
- `docs/validation/test_aggressive_analysis.py`
- `docs/validation/v2_tracking_gazebo.py`
- `src/robot_bringup/config/reactive_tracking_oscillatory_aggressive.yaml`
- `src/robot_bringup/launch/closed_loop.launch.py`
- `src/robot_bringup/test/test_aggressive_oscillatory.py`
- `src/robot_control/config/whole_body_oscillatory_arm_preferred.yaml`
- `src/robot_simulation/config/scenarios/environment_oscillatory_aggressive.yaml`
- `src/robot_simulation/config/scenarios/reactive_target_oscillatory_aggressive.yaml`
- `src/robot_simulation/test/test_oscillatory_experiment.py`

Generated evidence: `docs/validation/aggressive_*` directories/logs, `aggressive_oscillatory_comparison.json` and `aggressive_oscillatory_tracking.png`. [Exact file manifest](validation/aggressive_modified_files.txt). No commit or push.
