# Tracking-loss diagnostic plan

Goal: isolate tracking degradation without changing production algorithms or the validated physics repair.
Spec: /home/user/.codex/attachments/718caf89-5a9c-47c3-af7b-31d4a3acc25e/Pasted text.txt

Architecture: read-only offline reconstruction using recorded physical truth, odometry, estimates, forecasts and control; test-only interventions only where existing interfaces permit clean exclusive substitution.
Tech stack: ROS 2 Jazzy, Gazebo 8, existing M08 recording/metrics, NumPy/SciPy/matplotlib.

Constraints: preserve all baseline hashes, physical limits, gains, watchdogs and algorithms. No staging/commits/push. No future truth or silent stale extrapolation. Failed runs remain evidence. No broad repair implementation.

Review focus: world/odom spawn transform; signed quaternion/yaw; causal timestamp alignment and reset epochs; current versus future target; invalid control/missing evidence must not count as success.

- [x] Capture baseline and current processes; inspect selected configuration and source flow.
- [x] Reconstruct D42 physical/internal tracking, localization, prediction, realization and 0–10/10–20/20–40/40–90 s stages. Pin diagnostics with synthetic frame/time/axis tests.
- [x] Evaluate low-impact causal interventions: offline same-state command recomputation, then live oracle robot/current-target input substitution only if safe exclusive ownership is feasible. Otherwise document the reviewed design and evidence limits.
- [x] Audit physical-context bootstrap/history/cursor/reset separately and reproduce causal failure minimally without changing time policy.
- [x] Generate source-backed tables/plots, hypothesis verdicts and proposed prioritized repairs with regression specifications.
- [x] Run focused tests/essential regressions, verify diagnostic isolation and original source hashes, clean owned processes, finalize exact inventory and report.
