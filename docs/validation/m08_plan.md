# M08 design and implementation plan

Baseline b16b649: committed M07 on main; existing feature/m08-experiments branch.
Preserve historical untracked evidence. No commits/push. User's autonomous delivery
instruction overrides skill approval/commit gates; no further authorization needed.

Goal: one experiment YAML composes existing M07 manifests, runs isolated owned
sessions and produces auditable portable results for future read-only consumers.
Keep algorithm/physics/numerical implementation in existing owners.

Design: robot_experiments owns flat cohesive configuration.py, execution.py,
benchmarking.py, reporting.py and cli.py; recording/ separates ROS acquisition
from normalization; metrics/ contains pure alignment and evaluation mathematics.
Existing bringup builds the deployment; the runner invokes its console launch.
Persist append-only JSONL original messages (source/arrival/clock/frame), config
snapshots, versioned metrics/time series/events/status, plus study JSON/CSV.
Evaluate using physically measured base+joints FK in a declared common frame;
never substitute estimated pose for truth. Bound alignment and mark unavailable
metrics. Prediction uses source epoch + absolute future times and target truth.
Measurement RNG is the only deployed random consumer; materialize a seeded copy
of its owner YAML and a referencing profile/system without altering source files.
Capture seed, source hashes, physics/owner config and actual runtime type logs.

Lifecycle: exclusive domain leases plus unique Gazebo partitions, process session,
readiness from required streams, sim-time settling/duration, monotonic start/stall/
wall deadlines, reject clock resets as interrupted runs, finite group shutdown.
Only owned process descendants/group may be signalled. Preserve partial results.
Completed execution and failed safety acceptance are distinct. Sequential matched
seed variants aggregate only compatible scenario/physical/time contracts; reactive
closed-loop target trajectories are not promised identical or bitwise deterministic.

Review focus: irregular/stale/out-of-order timestamps; prediction future alignment;
wrong-frame or missing truth; seed propagation to actual consumer; failed launch,
clock reset and child cleanup; immutable output; fair comparison and safety gating.

- [x] 1. Configuration/resources/materialization + package/CLI tests RED→GREEN.
- [x] 2. Pure metrics/alignment with analytical timestamp/safety fixtures RED→GREEN.
- [x] 3. Recorder normalization/readiness/raw timestamps + ROS adapter tests.
- [x] 4. Owned lifecycle, deadline failures, isolation, persistence + fake process tests.
- [x] 5. Benchmark aggregation, artifacts/schema, CLI, tenth-package architecture.
- [x] 6. Focused/full regressions, ten-package builds and copied installation.
- [x] 7. Real nominal+CV paired seed runner checks and negative cleanup validation.
- [x] 8. Whole-change review, required fixes, documentation, exact inventory/verdict.

## Final closure — 2026-10-18

Completed all eight steps on the existing feature/m08-experiments branch without
commits or pushes. Source baseline remains b16b649. Read-only independent review
findings were corrected using RED→GREEN tests and actual live acquisition.
Final: 1071 functional tests passed (18 existing style/copyright checks excluded),
137 focused/architecture tests passed, both ten-package builds passed, and copied
installation outside checkout verified all eight console entries and seed917.
Real seed42/917 IMM/CV pairs completed with failed physical-speed acceptance;
compute-only completed with no command publication; invalid input and actual
wall timeout preserve explicit failures/cleanup. RTF/sampling/feedback limitations
remain measured and visible. Verdict CONDITIONAL. No M09/M10 work started.
See m08_final_verification.json, m08_live_evidence.json and the main M08 report.
