# Target context, control freshness and ROS/Gazebo lifecycle closure

Task attachment: `22770332-ffe1-43bc-8daa-04c87a99f1e2`. Design and frozen gates: `target_context_freshness_lifecycle_design.md`. Baseline hashes/status/index: `target_context_freshness_lifecycle_baseline.json` (1,597 existing files). No commits, staging or pushes. Historical experiment artifacts remain preserved.

## Established causes and correction boundaries

The current immutable target stepper discarded completed work when one required causal substep sample was absent. Retrying the original cursor could outlive its bounded history; future physical samples cannot repair that cursor. The deterministic historical reproduction confirms the mechanism, while the exact first missing callback in historic physical failures remains unknown.

The corrected optional partial-step mode returns completed substeps only. Simulation sends that candidate pose to Gazebo and commits/publishes it only after successful acknowledgement. Both base and tracking samples must be causal and fresh through the entire RK4 step. Delayed samples can be inserted in source order within retained history; duplicates cannot replace a sample. Numerical dynamics, integrator, physics step and maximum step count remain unchanged.

A recoverable delay resumes the same state/time trajectory. An irrecoverable history/time gap or stuck pose acknowledgement beyond the existing integration lookback becomes an explicit fatal run error. Essential-process launch supervision then shuts the experiment down. There is **no automatic reinitialization across such a gap**. No target position teleport, future-context read, stale restamp or convenient initial-state reset is used. This deliberately leaves automatic gap recovery CONDITIONAL: a controlled fresh experiment is required when continuity cannot be justified. No stale target observation is rebroadcast; existing prediction freshness expires normally.

The real pre-fix launch-root SIGINT test returned launch code0 with its owned Gazebo server still alive12s later. The vendor included launch used a shell wrapper; ROS supervised the wrapper rather than that server. The production simulation launch now starts `gz sim -s` directly without a shell. Optional GUI uses a separately supervised `gz sim -g`, avoiding the CLI forked GUI/server pair. Existing ROS launch process-exit handlers terminate the composition when an essential simulator, bridge or pipeline node exits. No new process manager/node/package is introduced.

The existing experiment runner now signals the launch root first during its graceful phase, leaving child shutdown ordering to ROS launch. Its existing marker, process create time and POSIX session checks remain the only ownership basis for bounded fallback. All signal recipients are recorded. No global process-name cleanup is used.

Validated older/duplicate control messages no longer evict a newer validated cache. The retained cache still passes the unchanged source-time age/frame/horizon checks every control decision. Malformed/future inputs retain fail-closed handling, and actuator command sequence/session replay protection is untouched. This corrects an avoidable interruption; it does not bypass localization trust holds.

Control telemetry adds `robot_state_time` and `prediction_reference_time`, in source-time seconds, with -1 for missing input. They are populated during both valid decisions and safe holds. These permit exact age calculations at decisions; they do not change input validity. Existing generic M08 normalization retains them without a second telemetry path.

A shutdown regression also demonstrated that `spin_once` after the final stop could dispatch a queued active callback. Control and actuator shutdown now cancel the periodic control timer and use a bounded transport acknowledgement wait, without dispatching more callbacks. Estimator/predictor entrypoints handle external shutdown idempotently; their mathematics and source acceptance policies remain unchanged.

## Timing/dependency contracts

All source epochs are ROS simulation time from one Gazebo transport clock authority bridged by one ROS `parameter_bridge`. Simulation rollback clears target state and physical histories, increments its local callback epoch and refuses old asynchronous commits. Existing estimator, predictor, controller and actuator rollback paths clear their own histories/sessions. ROS messages do not carry a globally unique run UUID; isolated domains/partitions plus lifecycle cleanup prevent previous-run contamination in the qualified tests. Arbitrarily replayed low timestamps indistinguishable from a new epoch remain a protocol limit.

| Input / owner → consumer | Type, source semantics | Configured timing / absence behavior |
|---|---|---|
| Gazebo → clock bridge → all simulation-time nodes | Clock; physical simulation time | One expected ROS publisher. Multiple authorities are a conflict, not a valid merged epoch. |
| Gazebo base → target context and physical TF | Odometry, world/base acquisition stamp |20Hz; contextual base age<=.30s, never future. |
| Gazebo joints → TF / estimator / independent actuator | JointState, named physical q/dq acquisition stamp | Physics-step source (2ms); component freshness/safety unchanged. |
| Physical base+joints → tracking TF → target | TFMessage; max component stamp, component skew<=.05s |20Hz, TF publisher component age<=.20s; selected target TF age<=.30s, zero future tolerance. |
| Wheel encoder odometry → EKF | Odometry, odom/base; measured v/omega, not world pose |50Hz; mandatory fresh component<=.20s. |
| Simulated mapped pose → EKF | Odometry, original acquisition stamp/covariance, sourceworld→odom TF |10Hz,40–50ms modeled delivery; arrival max age.15s; common localized reorder.08s at100Hz, capacity256/maxage.20s; trust5-sample/4-inlier >=.30s evidence and.35s source timeout. Untrusted full output withheld. |
| Optional IMU → EKF | Imu yaw rate, base frame/source stamp | Optional and not published in this deployment; no new sensor fallback. |
| Target pose acknowledgement → synthetic measurement → target KF | PointStamped world; candidate integration stamp retained |20Hz requested, service/callback dependent; missing context publishes nothing. Target KF retains original time propagation/frame validation. |
| Target KF → IMM/adaptation/prediction | StateEstimate → PredictedTrajectory world, forecast reference is estimate stamp | Observation-driven; strictly increasing predictor reference; rollback clears history. No old prediction republished with fresh time. |
| Trusted robot full state + forecast → control | StateEstimate odom / trajectory transformed at reference stamp |20Hz; robot age<=.20s, forecast reference<=.50s, future tolerance.01s, horizon covers now+.25s. Missing/stale input emits inactive zero command. |
| Safe command → actuator → native servo | SafeVelocityCommand, session/sequence/stamp | Existing watchdog, replay checks, physical bounds/holding PID unchanged. |

Physical history duration remains10s integration lookback plus source freshness(.30s in D). Capacity is derived in the simulation launch as ceil((lookback+2*max source age)/physics step)+2 =5,302 samples in D. `context_history_capacity` is a launch-derived internal ROS parameter, not another independent timing configuration. A source cannot acquire more frequently than the configured physical simulation step; the capacity also bounds memory under invalid excessive source density. Source histories never extrapolate. Bounded ahead-of-local-clock arrivals wait in that same history until causal; eviction uses receipt clock, and ahead-of-clock data beyond the existing source-age budget are rejected. This avoids dropping legitimate data solely because independent callbacks precede a local clock update; it never licenses future context for integration.

## Evidence accounting

Initial red run:6 failures/38 passes. Corrected focused run:82 passes before final additions. Historical baseline-module probes additionally reproduce missing partial commit, missing terminal-gap fault and child shutdown ordering. Final regression/lifecycle/physical result tables below use fresh artifacts. Earlier attempts remain preserved, including a diagnostic logging call-site severity error introduced during this task; it caused bounded essential-process shutdown and was corrected, then covered with a real ROS logger regression. A five-test fixture regression assumed the simulation include was always action0; it now discovers that include among registered handlers and verifies the same original behavior.

Trust health is sampled at5Hz. Input ages in new telemetry are exact for each decision, but a nearby untrusted health sample cannot establish the precise trust-transition time or distinguish every source-absence/transport/callback delay. Such attribution stays qualified or unresolved. Missing samples are never imputed as valid. Physical tracking metrics include inactive control states and use the same source windows/FK/skew/thresholds as rescored M03 evidence. Reactive before/after trajectories are not identical.

## Final nominal physical measurements

The final two trials ran sequentially on 2026-10-23, with no owned builds, regression tests, replay or analysis competing during measurement. Both ran90s after readiness and completed without run error. The table uses the same fixed0–90s physical/all-control window and20–90s localization window as rescored M03. Sources: `target_context_freshness_final_measurements.json` and each immutable run's `result.json`, `resolved_configuration.json`, telemetry and native trace. No failed attempt is discarded.

| Metric, seed42 /917 | Previous M03 robustness | Current | Interpretation |
|---|---:|---:|---|
| Published localization XY MAE m | .0397 / .0391 | .0392 / .0404 | Both<=.20 |
| Published localization yaw MAE deg |3.272 /2.613 |3.154 /2.903 | Both<=10 |
| Internal XY MAE including withheld m |.0385 / .0395 |.0416 / .0435 | Both<=.20 |
| Internal yaw MAE including withheld deg |2.686 /2.500 |3.990 /4.069 | Higher; both<=10 |
| Physical distance MAE m |.4075 / .3808 |.3741 / .4288 | Mixed; no uniform improvement |
| Physical distance max m |.9282 / .9672 |.8986 /1.0023 | True measured maxima |
| Physical pointing MAE deg |32.911 /28.512 |28.427 /30.229 | D42 improves; D917 worsens |
| Physical pointing max deg |96.938 /96.671 |96.625 /96.625 | Large remaining task errors |
| Compliance %, unchanged.20m/15deg |31.67 /35.43 |32.79 /30.07 | Conditional physical tracking |
| Physical compliance observation coverage % |97.37 /96.30 |97.94 /95.60 | Missing observations not filled |
| Control validity %,0–90s |96.99 /95.36 |96.94 /96.40 | Includes initialization |
| Active control validity % |99.19 /97.93 |98.865 /98.282 | Both meet>=98 in these trials |
| Longest active invalid interval s |.350 / .450 |.352 / .450 | Both<1 |
| Longest active control sample gap s |.084 / .070 |.066 / .068 | Not imputed as valid |
| RTF, total simulation delta / monotonic delta |.739 / .812 |.845 / .836 | Different wall sessions; no isolated speedup claim |

Internal diagnostic sampled maxima were XY.212/.234m and yaw20.44/31.02deg. Source diagnostics remain5Hz; these are sampled extrema, not continuous bounds or a covariance-consistency certificate. No localization mathematics, trust criterion, controller allocation, prediction algorithm, target dynamics, gains or physical limits were tuned. Reactive paths differ, so these two seeds cannot establish statistical performance improvements or identical-trajectory causation.

## Freshness and interruption attribution

Current present-input ages over0–90s: robot mean103.1/106.2ms, p95118/122ms; forecast mean60.1/62.0ms, p9596.0/98.6ms. Max robot ages500/608ms occur during reported invalid holds. In the complete nominal recordings, maximum robot age on a **valid** published telemetry decision was172/194ms, forecast180/158ms: zero recorded valid decisions exceeded the unchanged.20/.50s budgets. This is finite measured evidence, not a worst-case scheduling guarantee. `target_context_decision_contracts.json` also verifies the delay and irrecoverable-gap probes; the latter records205 `Target prediction is stale` decisions and no valid decision past its age bound.

All50 active invalid decisions in the two nominal0–90s windows name robot-state staleness, with a fresh forecast. Of these,17/26 coincide with a preceding sampled untrusted localization status;3/4 remain unresolved for source/transport/scheduling attribution. Thus43/50 are consistent with legitimate trust holds, not proof of exact transition timing. Do not infer that all50 were caused exclusively by localization. The30ms-versus80ms reorder choices,>=.30s localization evidence span and.20s control age remain unchanged. There is no evidence here to justify weakening those checks.

Every active invalid interval is listed below. Ages are exact source ages at the telemetry decisions in that interval; ending times are the next recorded valid decision. The tested controller action is inactive zero command; physical actuation uses the existing hold/watchdog. Raw command/actuation telemetry remains available to inspect delivery behavior.

| Seed | Source interval s | Duration s | Robot age s | Forecast age s | Condition |
|---|---:|---:|---:|---:|---|
| 42 | 4.300–4.650 | 0.350 | 0.200–0.500 | 0.014–0.074 | Robot state stale; sampled trust hold / unresolved onset |
| 42 | 73.400–73.752 | 0.352 | 0.200–0.500 | 0.040–0.094 | Robot state stale; sampled trust hold / unresolved onset |
| 42 | 76.752–77.050 | 0.298 | 0.220–0.470 | 0.020–0.072 | Robot state stale; sampled trust hold / unresolved onset |
| 917 | 4.900–5.250 | 0.350 | 0.200–0.500 | 0.028–0.078 | Robot state stale; sampled trust hold / unresolved onset |
| 917 | 36.200–36.650 | 0.450 | 0.208–0.608 | 0.020–0.104 | Robot state stale; sampled trust hold / unresolved onset |
| 917 | 85.150–85.550 | 0.400 | 0.240–0.590 | 0.048–0.060 | Robot state stale; sampled trust hold / unresolved onset |
| 917 | 88.150–88.450 | 0.300 | 0.240–0.490 | 0.038–0.098 | Robot state stale; sampled trust hold / unresolved onset |

Startup waiting is separate: first valid control1.950/2.700s. Recorded context availability waits were initialization-only,.084/2.212s; neither nominal run logged a post-initialization context wait, terminal gap, or integration-limit error. The old exception string alone is not evidence of availability; the report uses transition logs and advancing target observations. Target measurement, estimate and forecast median source periods were.058s (requested20Hz, achieved~17Hz), maximum source gaps.102/.084s, preserving identical source reference stamps across those boundaries. Wheel median period.020s; localization.100s; joints.002s; trusted robot output~.022s. All examined streams had zero recorded source-order regressions or duplicate stamps. These recorder measurements do not prove that every publisher callback was received: D917 joint recording has a.900s gap at11.728–12.628s and wheel odometry a.280s startup gap at.460–.740s, while native physics traces remain complete. `stream_timing` records all counts, epochs, periods and gaps.

## Physical safety and actuation

Both complete native2ms traces pass the unchanged.001rad/.001rad/s tolerances, including startup. No inward-demand native motor stall was observed. Maximum shoulder position excess was6.471531e-5rad; all other joint excesses<=1.006e-12rad. Maximum measured speeds across both trials were base yaw.717139, shoulder.9999999998, elbow1.0000000005, wrist.717229rad/s, against1/1/1/1.5rad/s. The elbow's~5.0e-10 numerical excess remains explicitly reported, within the existing acceptance tolerance.

Observed actuation recovery episodes:27/24. Of these26/23 return directly to active; one per seed ends in `Control invalid or disabled`, then subsequent valid operation resumes. Both runs are active near90s; zero sampled permanent-fault duration. In-window command-rejection deltas0/3, feedback-rejection deltas2/0, watchdog timeouts0/0. Future-timestamp protections still produce legitimate transient holds; no actuator check is bypassed. This is a finite position/speed/recovery qualification, not a motor-torque, serious-mechanical-fault or real-hardware safety certificate.

## Lifecycle, resets and installation

The final causal-buffering qualification uses the same leased ROS domain and Gazebo partition for five consecutive genuine cycles. Each ran6sim seconds after readiness with meaningful robot/target/control data. All had root return code0, no runner fallback signal, no owned process at the pre-fallback check, zero owned clock publishers and zero ROS world services after shutdown. A separately owned unrelated sleep process survived each cycle. Exact PIDs, creation times, inherited owner markers, sessions, command lines, clock endpoint GIDs, times and services are in the cycle JSONs. Child exits from launch logs are consolidated in `target_context_lifecycle_evidence.json`.

| Cycle | Shutdown method | Observed shutdown s | Clock publishers during/after | Owned survivors before fallback |
|---|---|---:|---:|---:|
|1 | Launch-root SIGINT |.777 |1/0 |0 |
|2 | Foreground-group SIGINT (Ctrl+C semantics) |.446 |1/0 |0 |
|3 | Launch-root SIGINT |.653 |1/0 |0 |
|4 | Foreground-group SIGINT |.577 |1/0 |0 |
|5 | Launch-root SIGINT |.809 |1/0 |0 |

Scoped essential-server/bridge SIGTERM probes also complete shutdown without runner fallback, within10.19/10.26s of the recorded check and the12s deadline. Gazebo service lists and `/clock` transport information show no remaining publishers/services afterward; ROS bridge publisher identity is recorded separately. Invalid-manifest startup exits before full assembly and leaves no survivor. A short delayed-context probe resumes the same target trajectory/control. The beyond-budget probe explicitly reports the terminal cursor gap, stops prediction-dependent control and cleans up without inventing dynamics or resetting target state. Recorded clock endpoints for its pause probe were5.258→15.860s; callback/discovery delays mean these are recorder epochs, not an exact physical signal-delivery timestamp.

All24 lifecycle attempts, including baseline orphan reproduction, logger-error attempts, the intermediate cursor-gap failure and an earlier.9s outage that recovered despite the test expecting a fault, remain preserved. The final five-cycle qualification is identified separately. Early cleanup does not turn unsuccessful attempts into passes. The initial orphan was only terminated through verified runner ownership; no unrelated process was killed.

**Shutdown limitation:** some requested SIGINT shutdowns still yield child status-2 or1 and Jazzy/rclpy callback-conversion/interrupt tracebacks. Final five-cycle evidence includes a simulation child1 and actuator child1; final D917 has a prediction child1 during requested shutdown. All occur after requested termination, with no surviving process; there is no claim that every child exits0. Root ROS launch may also return0 after an essential child fault; the existing runner treats any unexpected early launch exit as failure, regardless of code, and logs retain the child failure. Resource/ownership closure passes; completely clean child exit status remains CONDITIONAL. GUI lifecycle was not physically qualified.

Actual Gazebo world reset passes clock rollback, subsequent control recovery, both native trace epochs complete/safe and cleanup. Ordinary wheel-only10s simulation passes. The copied install built all10packages independently without symlinks, refreshed changed Python packages and installed final launch resources through setuptools `install_data`; source/installed hashes are checked in `target_context_copied_final_source_consistency.json`. It passes copied localization and prior M02/M07/M08/M09/challenge/native gates, plus a real localized10s Gazebo smoke with physical safety. A failed copied check first used an unwritable default ROS log path; its generated launch bytecode then tripped the existing resource-debris check. Both attempts remain recorded. Only this task-owned copied install's generated bytecode was removed, and final checks use a writable ROS log directory and `PYTHONDONTWRITEBYTECODE=1`.

## Independent gates and verdict

| Gate | Verdict | Evidence / remaining boundary |
|---|---|---|
| Target-context causal recovery | CONDITIONAL | Temporary/delayed recovery passes; irrecoverable gap explicitly terminates rather than automatically reinitializing. |
| Target time and epoch integrity | PASS for tested contracts | Causal selection, bounded ingress/history, no restamping, transaction/reset tests and real reset. Timestamp-only old-epoch ambiguity remains a protocol limit. |
| Control freshness and validity | PASS for tested numerical gates | Both>=98%; valid-decision source ages within bounds; all7 active holds documented. Seven individual invalid decisions retain unresolved deeper attribution. |
| Five consecutive lifecycle cycles | PASS for resource contract | Genuine normal stack, same domain/partition, each meaningful6s window, no pre-fallback survivor. |
| Ctrl+C shutdown ownership | PASS for ownership; CONDITIONAL clean exit codes | Both actual group SIGINT cycles reclaim all owned resources; occasional child-2/1 remains documented. |
| No stale owned clock publishers | PASS in qualified cycles | One ROS bridge identity during each cycle, none after; owned PIDs/services gone and next epoch coherent. Not a certificate for arbitrary unrelated publishers. |
| Nominal physical tracking regression | CONDITIONAL | D42 improves, D917 distance/pointing/compliance worsen; reactive task errors remain large. |
| Native physical safety | PASS for measured gates | Complete native position/speed traces; existing numerical tolerances and backend preserved. |
| Functional regression | PASS |1,333 functional tests;18 existing copyright/flake8/pep257 checks deselected, no lint-clean claim. |
| Architecture and maintainability | PASS within scope | Existing owners, no new production file/node/package/config profile; numerical owners/gains/limits untouched. |

**Overall: CONDITIONAL.** Demonstrated absorbing-cursor and shell-orphan failure mechanisms are corrected or converted into explicit bounded failure, normal control continuity meets the frozen threshold in both seeds, and physical enforcement is preserved. Remaining work is automatic scientifically valid continuation across an irrecoverable target gap (would require a designed epoch/discontinuity protocol), uniformly improved physical tracking, complete interruption attribution, and consistently clean child shutdown status. No controller/localization tuning or broader sensor/framework redesign is undertaken here.

## Exact changes and preservation

Task-only changes are measured against the captured dirty-tree baseline, **not** against HEAD; the Git diff still contains earlier M03 work. Existing production files changed here:

```text
src/robot_bringup/launch/closed_loop.launch.py
src/robot_control/robot_control/nodes/control_node.py
src/robot_estimation/robot_estimation/nodes/estimation_node.py
src/robot_experiments/robot_experiments/execution.py
src/robot_interfaces/msg/ControlTelemetry.msg
src/robot_prediction/robot_prediction/nodes/prediction_node.py
src/robot_simulation/launch/simulation.launch.py
src/robot_simulation/robot_simulation/gazebo/tracking_frame_provider.py
src/robot_simulation/robot_simulation/integration/time_stepper.py
src/robot_simulation/robot_simulation/nodes/simulation_node.py
src/robot_simulation/robot_simulation/nodes/velocity_actuation.py
src/robot_simulation/robot_simulation/state/physical_context.py
```

Existing tests extended:

```text
src/robot_bringup/test/test_aggressive_oscillatory.py
src/robot_control/test/test_control_node.py
src/robot_experiments/test/test_execution.py
src/robot_simulation/test/test_base_guidance.py
src/robot_simulation/test/test_configuration.py
src/robot_simulation/test/test_time_stepper.py
src/robot_simulation/test/test_tracking_frame_provider.py
```

New test: `src/robot_simulation/test/test_context_recovery.py`. **New production files: none.** New validation code: `tools/validation/target_context_lifecycle_run.py` and `target_context_freshness_analysis.py`. Design/closure are the two requested Markdown files. Exact new logs, JSON, documentation and experiment roots are inventoried in `target_context_freshness_lifecycle_modified_files.txt`; every generated results-file path is in `target_context_generated_results.txt`.

The19 changed pre-existing files are all expected; no baseline file is missing, no unexpected pre-existing file changed, and no historical nonignored artifact hash changed. HEAD/index match the captured baseline. Prior M03 estimator policy/filter/configuration hashes remain intact except the explicitly listed ROS entrypoint cleanup. The protected native backend still has SHA256 `59c4a3a5481bf9cad4480b9af1307d09d1444ccb1d22a0af1f6aee99e5d0fc5e`. All pre-existing description/actuation/physics/controller/target configuration and native sources are preserved. Baseline hashing covered1,597 **nonignored** files; it is not a claim that every ignored historical telemetry file was hashed at task start. Historical results were only read, and all new results use fresh uniquely generated study paths. The final host audit finds zero active ROS/Gazebo processes. Details: `target_context_freshness_lifecycle_preservation.json`, `target_context_freshness_lifecycle_final_processes.json`.

## Reproduction commands

Run from `/home/user/ros2_ws`. Rebuild interfaces and consumers because the read-only control telemetry message gained two fields. Existing launch arguments/profile inheritance are preserved.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  robot_interfaces robot_simulation robot_bringup robot_control \
  robot_estimation robot_prediction robot_experiments
source /home/user/ros2_ws/install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

# Normal localized simulation; no validation-only launch wrapper.
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_manipulation_D_localized.yaml \
  enable_actuation:=true gui:=false

# Same normal launch with the original wheel-only configuration.
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_manipulation_D.yaml \
  enable_actuation:=true gui:=false

# Safe Compute Only: command transport disabled by the existing explicit switch.
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_manipulation_D_localized.yaml \
  enable_actuation:=false publish_commands:=false gui:=false

ROS_LOG_DIR=/tmp/target_context_tests python3 -m pytest src -q \
  --import-mode=importlib -p no:launch_testing -p no:launch_ros \
  -k 'not copyright and not flake8 and not pep257'

# Existing raw atomic-step reproduction plus the new production-path regressions.
python3 tools/validation/tracking_loss_context_test.py
ROS_LOG_DIR=/tmp/target_context_focused python3 -m pytest \
  src/robot_simulation/test/test_context_recovery.py \
  src/robot_simulation/test/test_tracking_frame_provider.py \
  src/robot_control/test/test_control_node.py \
  src/robot_experiments/test/test_execution.py -q \
  --import-mode=importlib -p no:launch_testing -p no:launch_ros

python3 tools/validation/target_context_lifecycle_run.py \
  --output results/target_context_reproduce_cycles --cycles 5 --duration 6
python3 tools/validation/target_context_lifecycle_run.py \
  --output results/target_context_reproduce_delay --cycles 1 --duration 4 --failure context_delay
python3 tools/validation/target_context_lifecycle_run.py \
  --output results/target_context_reproduce_gap --cycles 1 --duration 4 --failure context_gap
# Other focused failure modes: --failure server, bridge, bad_config.
python3 tools/validation/m02_resolution_reset.py \
  --output results/target_context_reproduce_reset \
  --system-config config/system_manipulation_D_localized.yaml
python3 tools/validation/m03_localization_run.py \
  --output results/target_context_reproduce_nominal --duration 90 \
  --seeds 42 917 --modes localized
python3 tools/validation/m03_localization_run.py \
  --output results/target_context_reproduce_wheel_only --duration 10 \
  --seeds 42 --modes original
python3 tools/validation/target_context_freshness_analysis.py \
  results/target_context_nominal/m03_localization_20261023T040401_263745Z_wbg5yij4/localized_D42 \
  results/target_context_nominal/m03_localization_20261023T040401_263745Z_wbg5yij4/localized_D917 \
  --output /tmp/target_context_reproduced_metrics.json
```

Use a fresh output JSON for analysis. Lifecycle and experiment runners allocate unique study subdirectories and isolated leases; do not overlap nominal benchmarks with builds/tests or other simulations. The lifecycle qualification deliberately reuses its lease across cycles.

The copied installation was built without `--symlink-install` from a cleared overlay environment:

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH bash -c '
    source /opt/ros/jazzy/setup.bash
    colcon --log-base /tmp/target_context_copy/log build \
      --build-base /tmp/target_context_copy/build \
      --install-base /tmp/target_context_copy/install
  '
# In a clean shell, source only that copied overlay and run the same launch.
source /tmp/target_context_copy/install/setup.bash
ROS_LOG_DIR=/tmp/target_context_copy_verify PYTHONDONTWRITEBYTECODE=1 \
  python3 tools/validation/joint_limit_recovery_verify_installation.py \
  --prefix /tmp/target_context_copy/install --output /tmp/target_context_copy_gates.json
```

For a new checkout use a full ten-package build rather than `--packages-select`. No staging, commit or push was performed.
