# R1B1 observer instrumentation implementation plan

Execute inline (superpowers:executing-plans); user has authorized implementation
without repeated approval. Preserve the current working tree; no worktree/index
mutations or commits. Spec: attachment ac1a5151-1e7a-407d-9c85-f6264d6c7697.

Goal: default-off bounded diagnostics on the existing observer, followed by one
unchanged paired IMM/CV study. R1B2 and S3B2 remain unauthorized.

Architecture: one recorder-owned diagnostic collector; no second observation
store or scientific clock. CLI flag passes only runtime options, outside M08
materialization/contracts. Instrument existing normalization, serialization and
synchronous write exactly once, and existing bounded executor loop without
changing timeout arguments, order or budget checks. Use two-argument public
subscription callbacks only in diagnostic mode; unavailable MessageInfo fields
remain explicitly unavailable. No private executor monkey-patching. Independent
low-frequency root-process monitoring; bounded summaries/rings; atomic shutdown
JSON. Diagnostic failure must fail an otherwise successful run and preserve an
already-existing scientific failure.

Alternatives rejected: global tracing/private executor patches (too invasive),
per-message diagnostic logging (perturbs hot path), YAML switches (changes study
inputs), persistence/executor redesign (R1B2). Existing R1A probe remains a control.

- [x] Freeze HEAD/index/status, 1,664 initial files, historical metadata and
  original recorder/runner/CLI code before production edits.
- [x] Baseline relevant tests and installed rclpy/RMW API capability inspection.
- [x] RED tests: bounded collector, exact scientific envelopes, duration stages,
  batching/order/timeout semantics, count/time limits, middleware missing fields,
  writer/collector/finalization failures, CLI opt-in and interrupted-run evidence.
- [x] Implement recording/observer_timing.py; modify recording/storage.py,
  recording/ros.py, runner.py and cli.py only. No YAML/metric/algorithm changes.
- [x] GREEN focused/M08/full regression; scoped lint; package build and installed
  CLI. Compare all original materialized inputs/contracts against frozen source.
- [x] Matched synthetic disabled/enabled replay overhead (alternating order,
  multiple repeats, variability, same input/output work). Retain uncertainty;
  potentially perturbed live results if budget not demonstrated.
- [x] Independent review, source freeze, host process preflight. One bounded
  diagnostic paired_prediction_composition run with original 80 ms policy,
  six-second window/three-second settling and deadlines. Preserve failures.
- [x] Analyze batch/callback/stage/resource/metadata evidence and R1A contrast;
  optional one sensor study only with an explicit unanswered causal question.
- [x] Finish runtime_reliability_r1b1_closure.md with exact changes, schema,
  measurements, limits, R1B2 recommendation, rollback, preservation and cleanup.

Review focus: enabled metadata callback must require two arguments (rclpy bind
otherwise selects MessageOnly); diag clock calls must not change deadline placement;
failed/partial flush must not overwrite an original timeout/writer/interrupt;
telemetry byte equivalence including raw headers; all rings/history bounded;
unsupported middleware timestamps/sequence never inferred from source time;
resource thread shutdown bounded; no extra ROS subscription/event waitable.

Outcome: instrumentation CONDITIONAL (27.51% median matched replay overhead,
above 5% aim); live results potentially perturbed/diagnosis only. Both runs
readiness_timeout; no scientific evaluation window or repeat attempts. Overall
root cause PROBABLE observer servicing/backlog, with individual stage locations
identified and DDS/drop/OS attribution still incomplete. Optional sensor study
not justified because the remaining questions need missing boundaries/identity,
not more unchanged trajectories. Full regression 1,566 passed; final focused
38 passed; builds and copied install verified. Scoped collector/recording/test
lint clean; broad CLI/runner formatting findings are quantified in closure.
No R1B2/S3B2 work, Git index changes, commit or push.
