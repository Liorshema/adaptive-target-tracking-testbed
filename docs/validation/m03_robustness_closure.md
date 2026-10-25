# M03 robust localization closure

Task: 05e41ac8-6155-467b-a366-3b61e0d2b8fd. The pre-implementation causal audit, alternatives, mathematics, assumptions and frozen numerical gates are in `m03_robustness_design.md`. Existing M02/M03 reports and experiment roots are preserved. The nonlocalized Scenario D remains unchanged; the existing localized profile selects the correction by inheritance.

**Overall: CONDITIONAL.** Startup and declared reacquisition checks pass; physical tracking/control-continuity acceptance remains conditional.

## Root causes and correction

The old pose update used a statistically valid single-observation NIS gate as if it could also establish source reliability. An honest broad initial pose prior admits a plausible corrupted first observation. Covariance then contracts around that bad reference; good observations fail subsequent gates. After skid/model inconsistency, the same gate can reject a run of good measurements until accumulated uncertainty finally permits a large correction. Neither aggregate wheel/joint freshness nor the old diagnostic `available` flag communicates absolute-pose trust to the controller; the controller does not use robot covariance to decide that trust.

The correction adds bounded causal source validation **inside the existing robot estimator**, not a second estimator or ROS node. `PoseAcceptance` lives beside the existing pose model/transform in `robot_estimation/robot/planar_pose.py`. It keeps at most five source-time innovations, requires four compatible observations including the newest, a minimum .30 s span, and expires evidence after .80 s. Innovations are compensated by the ordinary wheel/joint motion prior. A robust median selects a representative actual sample; a linear consistency scan uses both reported measurement covariances and the existing relative process covariance. Historical samples establish consistency only: **only the current observation is fused once**, with its original R and source stamp. No averaging gain or repeated independent use of calibration bias is claimed.

Source modes are `initializing`, `tracking`, `validating`, and `degraded`. A failed ordinary NIS gate enters validation; accepted-source age over .35 s degrades trust. The existing ROS adapter withholds the full stamped robot estimate while untrusted, so the unchanged controller freshness and actuation watchdog mechanisms can hold safely. Wheel/joint estimation, covariance propagation and the legacy diagnostic vector continue. An enabled but unavailable absolute source does not falsely establish localization validity. Explicitly disabled localization keeps ordinary wheel-only operation. No external source can guarantee absolute correctness if a coherent majority is corrupted; insufficient or inconsistent evidence never forces acceptance.

After corroborated model discrepancy, the robot estimator may add explicit positive-semidefinite uncertainty to its **pose prior**, then reapply the unchanged NIS gate and existing wrapped/Joseph EKF update. For r, H, S, original NIS n above the gate, add `deltaP=k(H'r)(H'r)'`, `k=1/3-1/n`; Sherman–Morrison gives the configured target NIS of three. The existing EKF owns a small validated `add_process_uncertainty` operation that preserves mean and timestamp. No truth reset, arbitrary gate enlargement, prior-certainty invention, cosmetic output smoothing, second filter or actuator-policy change is used. This fault-conditioned adaptive covariance rule is explicitly an approximation, not an exact Bayesian posterior or universal sensor-health proof.

The state remains `[x,y,yaw,v,omega,q,dq]`; messages/topics, frames, source-time queue, clock-reset contract and ordinary controller input remain unchanged. Frame conversion and yaw wrapping precede policy validation. Process Q, measurement R, simulator source noise/bias/latency/dropout, controller, predictor, target, geometry, limits, gains and native actuator/backend are unchanged in this task. Correlated calibration bias still limits exact covariance calibration. Diagnostics expose trust/mode, accepted/rejected/pending counts, recovery uncertainty, corrections, and internal pose/covariance even while full-state output is withheld.

## Production scope and authoritative configuration

| Existing module | Responsibility in this correction |
|---|---|
| `robot_estimation/robot/planar_pose.py` | Pure bounded source-consistency policy and configuration validation, beside the existing pose model. |
| `robot_estimation/robot/robot_state_estimator.py` | Apply policy to the same pose update, expose trust, perform explicit corroborated uncertainty addition and ordinary EKF update. |
| `robot_estimation/filters/extended_kalman_filter.py` | Validate/add PSD process uncertainty without resetting mean/epoch. |
| `robot_estimation/factory/estimator_factory.py` | Construct optional policy; preserve legacy and disabled configurations. |
| `robot_estimation/parameter_schema.py` | Typed optional policy parameters. |
| `robot_estimation/nodes/estimation_node.py` | Configure policy, withhold untrusted full output, publish source-health/internal-pose diagnostics, recreate policy on reset. |
| `robot_estimation/config/robot_localized.yaml` | One authoritative set of active policy settings, inherited by existing localized manifests. |

Parameters introduced: `localization_consensus_window=5`, `localization_consensus_inliers=4`, `localization_consensus_span=.30`, `localization_consensus_max_age=.80`, `localization_trust_timeout=.35`, `localization_recovery_nis_target=3.0`. A zero/absent window preserves legacy behavior; disabled localization does not construct the policy. Uninitialized optional ROS parameters are not read for disabled sources. Original NIS limit remains 11.345. No extra production file, source publisher, launch hierarchy or one-off system profile was added.

## Deterministic and recorded evidence

The focused red run failed all eight intended new behavior cases (`m03_robustness_red.log`), including immediate first-observation acceptance and missing recovery/trust behavior. Expanded focused estimation/composition/architecture checks then passed 197 tests. Full functional regression passed 1,311 tests with 18 copyright/flake8/pep257 checks explicitly deselected; no lint-clean claim is made. The exact logs are retained. Tests cover nominal initialization, first/consecutive/mixed outliers, absent source, extended inconsistent evidence, short/extended loss, large heading discrepancy, PSD covariance, wrapped heading, invalid/stale/duplicate observations, ROS publication withholding and clock reset. Existing tests retain delayed/reordered samples and nonzero world/odom rotation/translation coverage.

Identical-input source-ordered replay of the retained physical startup-failure recording establishes causality: legacy first commitment at 2.7 s had 1.449 m XY error and 29.79 deg yaw error; corrected first commitment at 3.1 s had .055 m XY error and 1.12 deg yaw error. Legacy rejected 100 observations; corrected had 125 accepted and 26 deferred observations, without poisoning the initial reference. The replay starts at the first usable original-source epoch inferred from recorded diagnostics, rather than assuming early TF availability. It replays source order, not ROS arrival scheduling; truth is used only for offline scoring.

The retained D917 heading recording replays to 31.28 deg peak yaw error under legacy acceptance versus 29.26 deg with the correction. The corrected path explicitly validates at 59.75–60.05s and corroborates recovery at 60.15 s; it does not conceal the innovation with smoothing. Full source/innovation/covariance/trust histories are in `m03_robustness_heading_replay.json`. Replay pose-update timings are sub-millisecond on average (~.36–.42ms in these runs), but the replay overlapped independent build activity and is **not** an isolated CPU-overhead benchmark.

## Physical scoring and causal control audit

Final physical trials run sequentially after builds/replay/regression complete. The preliminary 10 s smoke overlapped verification/build activity and is labeled only a smoke. Every attempt has a separate input snapshot, log, graph, telemetry, native 2 ms trace and cleanup record. A legacy-acceptance ablation is selected only through the existing validation runner's `--pose-policy legacy`; it leaves the original single-observation EKF math active and does not create a production recovery path. The prior actual physical startup-poisoning artifact remains the primary old failure evidence.

Physical tracking is recomputed with the existing `robot_experiments.metrics.manipulation.analyze_phases`, using authoritative FK, actual base/joints/target, causal .08s maximum skew, and **all control states including holds**. Compliance keeps `.20m / 15deg`; missing truth is exposed as coverage. Published full-state localization is scored at each message's source epoch. Internal diagnostic pose/covariance is also scored during untrusted intervals (5Hz sampled evidence), so withholding output cannot hide estimator excursions. Offline truth interpolation uses <=.15s brackets; no truth participates in online acceptance.

Control is reported for both 0–90s and the prespecified active interval[first valid control,90s]. Invalid reasons are separated into launch readiness, target prediction/context, localization withholding with fresh internal filter, and unresolved other robot/scheduling cases. A contemporaneous untrusted mode, fresh internal filter and valid raw joints support attribution to localization withholding; no claim assigns every gap to localization. Existing target-context synchronization is not rewritten.

## Architecture and maintainability review

Acceptance mathematics, EKF fusion/covariance, ROS publication, and experiment scoring have separate owners. The policy has bounded storage, median selection and a linear scan only while validating; normal accepted measurements retain the ordinary update. No blocking callback, future-data wait, extra TF query, simulator process, new node, competing estimator, or unbounded history is introduced. Existing source transformations and shared reorder queue are reused. Configuration values are defined once in the existing localized owner YAML; legacy/disabled behavior remains selectable.

A new engineer starts with the six parameters in `robot_localized.yaml`, `PoseAcceptance` in `planar_pose.py`, and the policy call in `RobotStateEstimator.update_planar_pose`. ROS publication gating and diagnostics are adjacent in the existing node. The existing validation runner is extended with one explicit legacy ablation option, and one focused validation tool reuses existing replay/physical geometry and metrics. The sensor package layout is untouched. Protected component hashes and exact task inventory are audited separately from functional tests.

## Final before/after measurements

Each nominal trial ran 90 simulated seconds after readiness; scoring below is the frozen 20–90 source-time interval for localization and 0–90 for physical tracking. Previous source artifacts were rescored with the same all-control-state physical metric. These values therefore differ from the older valid-control-only table; both remain preserved. Reactive trajectories are not identical.

| Metric (42 / 917) | Previous localized M03 | Corrected M03 | Acceptance |
|---|---:|---:|---|
| Published XY MAE m | 0.0395 / 0.0405 | 0.0397 / 0.0391 | <=.20; PASS |
| Published yaw MAE deg | 3.212 / 2.938 | 3.272 / 2.613 | <=10; PASS |
| Internal XY MAE including untrusted m | N/A | 0.0385 / 0.0395 | <=.20; PASS |
| Internal yaw MAE including untrusted deg | N/A | 2.686 / 2.500 | <=10; PASS |
| Physical distance MAE m, all control states | 0.3959 / 0.3997 | 0.4075 / 0.3808 | Mixed; CONDITIONAL |
| Physical pointing MAE deg, all control states | 29.849 / 28.699 | 32.911 / 28.512 | D42 increases; CONDITIONAL |
| Compliance % (unchanged .20m/15deg) | 31.03 / 32.71 | 31.67 / 35.43 | Improves at both seeds |
| Physical compliance observation coverage % | 96.76 / 94.62 | 97.37 / 96.30 | Missing intervals not imputed |
| Control validity %, 0–90s | 96.76 / 99.54 | 96.99 / 95.36 | Includes launch waiting |
| Active control validity % | 98.54 / 99.89 | 99.19 / 97.93 | >=98; D917 misses (97.934%) |
| Longest active invalid interval s | 0.632 / 0.034 | 0.350 / 0.450 | <1s; PASS |
| RTF | 0.640 / 0.761 | 0.739 / 0.812 | Conditions differ; no speedup claim |
| Startup-outlier outcome | FAIL: old first pose 1.449m wrong | Correct starts .066 / .034m error | PASS for prescribed cases |
| Trusted output after 5–7s outage | N/A (old output did not assert trust) | .456 / .450s after source returns | <=1.5s; PASS |
| Physical position/speed | Previously measured pass | Complete native traces pass | Same .001rad/.001rad/s |

The previous nominal RTF included build/verification contention; the corrected final trials did not overlap owned builds or tests. This comparison is not an isolated host-performance benchmark. Covariance is finite/positive in the scored base blocks: internal NEES means 2.443 / 2.056 (descriptive, correlated samples). No claim of exact consistency or independent-sample statistical significance is made.

Internal diagnostic sampled peak yaw errors were 12.73 / 27.57 degrees; sampled peak XY errors 0.0977 / 0.2636 m. Neither sampled internal series exceeded .5 m or 30 deg in 20–90 s. Diagnostic sampling is 5 Hz: these are sampled maxima, not continuous-time bounds. The replay supplies finer source-update evidence for the original heading episode.

### Degraded runs and operational consequences

| Seed | First trusted full-state epoch s | Delay from first recorded source s | Initial XY / yaw error | Trusted output after outage s | Internal peak XY / yaw | Accepted / deferred / hard rejected |
|---|---:|---:|---:|---:|---:|---:|
| 42 | 1.650 | 0.400 | 0.0661m / 0.43deg | 0.456 | 0.0782m / 4.54deg | 119 / 38 / 0 |
| 917 | 0.950 | 0.900 | 0.0338m / 1.45deg | 0.450 | 0.0882m / 6.11deg | 100 / 27 / 0 |

Incorrect initialization was 0/2 corrected physical starts and 0/5 prescribed deterministic startup patterns. The fresh legacy ablation also reproduced poisoning: its first full pose at 1.582 s was 1.449 m / 29.79 deg wrong, with no trust hold. This is additional physical evidence alongside the preserved old 2.7–15.1s failure and identical-input replay.

Both corrected degraded sources published zero measurements during 5–7 s. Trusted output returned at 7.456/7.450 s; both entire-window sampled internal errors remained below .20 m / 10 deg, satisfying the prespecified one-second settling condition at sampled epochs. The 0.4/0.9 s startup delays are measured from first recorded source publication, not assumed estimator/TF availability. No guaranteed initialization time is claimed when qualifying evidence does not exist.

The degraded tests deliberately cause safe control holds: active sampled control validity was 71.94 / 72.82%, with longest invalid intervals 2.548 / 2.650s. These include the planned two-second outage and causal revalidation. They are not promoted to nominal continuity passes. D917 also has a 3.396 s recorded control-sample gap; telemetry availability limits attribution during that gap.

Hard rejection and deferral are distinct: deferred observations are not fused, may support later consistency, and otherwise expire. Zero hard rejects does not mean every outlier was accepted. The accepted counts include only ordinary current-observation EKF updates. The reported `recoveries` counter counts explicit covariance-inflation updates; fresh physical runs required none, while the recorded heading replay exercised two.

### Control-gap attribution and corrections

| Nominal seed | Launch invalid samples | Localization holds with fresh internal state | Other robot freshness/unresolved | Active invalid duration max s |
|---|---:|---:|---:|---:|
| 42 | 39 | 4 | 10 | 0.350 |
| 917 | 47 | 24 | 12 | 0.450 |

No active invalid-control reason in these nominal recordings explicitly names target prediction; that does not certify the unrelated target-context subsystem. Health diagnostic sampling cannot prove the cause of every scheduling or recorder gap. Startup waits remain included in 0–90 s statistics and are separately reported rather than silently discarded.

The corrected nominal runs still include real discrete pose corrections: maximum recorded heading corrections were 13.26/22.62 deg; maximum adjacent published full-state heading changes were 12.00/12.55 deg (both over .030 s). These are reported, not cosmetically filtered away. Validation naturally withholds input for a >=.30 s evidence span before corroborated re-trust, longer than the currently configured .20 s control freshness budget. For a different controller/source cadence, preserve this guard relationship when configuring the span. All native position/speed checks still pass.

### Safety, reset and installation results

All nine owned launch attempts completed with native physical checks passing, including the preliminary contended smoke, legacy/degraded cases, two 90 s nominal cases, ordinary wheel-only launch, reset and copied-install smoke. No failed physical attempt was discarded. Observed longest inward-demand motor stalls were 0 s; observed actuation recovery episodes all released. These are finite tested conditions, not a motor-torque or serious-fault certificate.

| Joint | Maximum measured position excess rad (nominal/degraded/legacy scored cases) | Maximum measured speed rad/s | Configured speed limit rad/s |
|---|---:|---:|---:|
| base_yaw_joint | 1.64313008e-13 | 0.717248163 | 1 |
| shoulder_joint | 6.47153104e-05 | 1 | 1 |
| elbow_joint | 2.31814568e-13 | 1 | 1 |
| wrist_joint | 1.07602816e-12 | 0.700569278 | 1.5 |

Live reset passed clock-reset observation, control recovery, both native trace epochs complete/safe, and cleanup. The ordinary nonlocalized Scenario D launch and copied localized Gazebo smoke also passed. Copied module origins, new policy configuration and all existing installation gates were verified; normal and copied installs select the same repaired native backend.

## Independent gates and final verdict

| Gate | Verdict | Evidence / limit |
|---|---|---|
| Robust startup initialization | PASS for prescribed corruption patterns | Identical-input replay, legacy physical reproduction, corrected starts 0/2 wrong; no guarantee for coherent majority corruption. |
| Slip/outage reacquisition | PASS for declared evidence/noise assumptions | .45s physical re-trust after outage; explicit uncertainty/recovery in heading replay; insufficient evidence remains untrusted. |
| Nominal localization | PASS | Both seeds pass XY/yaw mean gates, including untrusted diagnostic samples; covariance assumptions remain documented. |
| Physical tracking regression | CONDITIONAL | D42 physical pointing/distance mean increases; D917 active validity97.934% misses98%; tracking remains much better than nonlocalized but is not a uniform improvement over previous localized policy. |
| Physical safety | PASS for measured position/speed/release gates | Native complete traces and unchanged limits/backend; unmeasured torque and M02 serious-fault coverage remain outside this gate. |
| Architecture and maintainability | PASS | Existing owners, one EKF, no new production file/node/profile, bounded causal state, configuration inheritance and normal/copied validation. |

**Overall: CONDITIONAL.** Startup poisoning and absorbing single-observation recovery are addressed under the stated source assumptions. The conservative validation holds have a measured continuity cost, and one nominal reactive trajectory tracks slightly worse. The frozen gates are not weakened or tuned after observation. Future closure should evaluate the trust-versus-continuity tradeoff on rejected observations separately from target-context synchronization; this task does not broaden into either controller tuning or that synchronization rewrite.

Exact numerical artifacts: `m03_robustness_measurements.json`, `m03_robustness_extra_measurements.json`, the two replay JSON files, and `m03_robustness_execution.json`. Exact task-only files and preservation checks: `m03_robustness_modified_files.txt`, `m03_robustness_preservation.json`, `m03_robustness_final_processes.json`. No staging, commit or push.

Physical result roots:

- `results/m03_robustness_copied_smoke`
- `results/m03_robustness_degraded`
- `results/m03_robustness_legacy`
- `results/m03_robustness_nominal`
- `results/m03_robustness_reset`
- `results/m03_robustness_smoke`
- `results/m03_robustness_wheel_only`

## Remaining boundaries

This correction cannot certify a coherently biased/majority-corrupted single source, unknown persistent map error, or arbitrarily incorrect relative motion uncertainty. It does not provide real VIO/SLAM or collision avoidance. Genuine corroborated corrections remain discrete and must be reported with their physical/control consequences. Exact covariance consistency under correlated calibration bias/contact slip remains approximate.

Target-context synchronization, whole-stack scheduling/freshness, late-orbit WLS rate infeasibility, sensor architecture restructuring, IMM/adaptation benefit, measured motor torque and serious M02 mechanical-fault injection remain separate concerns. A passing localization test is not hardware-safety certification. No unrelated tuning or limits were changed.

## Reproduction

```bash
source /home/user/ros2_ws/install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_manipulation_D_localized.yaml \
  enable_actuation:=true gui:=false

ROS_LOG_DIR=/tmp/m03_robust_tests python3 -m pytest src -q --import-mode=importlib \
  -p no:launch_testing -p no:launch_ros \
  -k 'not copyright and not flake8 and not pep257'
python3 tools/validation/m03_robustness_validation.py \
  --replay results/m03_localization_degraded/m03_localization_20261022T110513_857158Z_sd6faju9/degraded_D42 \
  --output /tmp/m03_robust_replay.json
python3 tools/validation/m03_localization_run.py \
  --output results/m03_robust_reproduce_degraded --duration 18 --seeds 42 917 --modes degraded
python3 tools/validation/m03_localization_run.py \
  --output results/m03_robust_reproduce_nominal --duration 90 --seeds 42 917 --modes localized
python3 tools/validation/m03_localization_run.py \
  --output results/m03_robust_reproduce_legacy --duration 18 --seeds 42 --modes degraded --pose-policy legacy
python3 tools/validation/m02_resolution_reset.py \
  --output results/m03_robust_reproduce_reset --system-config config/system_manipulation_D_localized.yaml
```

Output files/directories must be fresh; historical artifacts are never overwritten. Original nonlocalized launch remains `system_config:=config/system_manipulation_D.yaml`.

Copied installation was built independently at `/tmp/m03_robust_copy`, without workspace overlay variables or symlinks. All ten packages built; the estimator was recopied after final diagnostic edits. Module/configuration origins and previous M07/M08/M09/M02/native installation gates passed from `/tmp`. Reproduce with the copied-install commands in the previous M03 report, substituting `/tmp/m03_robust_copy` and using the same localized manifest.
