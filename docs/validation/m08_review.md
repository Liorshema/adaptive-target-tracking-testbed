# M08 review closure

Baseline b16b649; full M08 requirements and m08_plan.md reviewed against the new
package, tests, existing launch and live outputs. Independent read-only review
reported four concrete defects and a safety-evidence coverage gap. All received
focused RED→GREEN corrections; no physical/numerical robotics changes were made.

| Finding | Correction / evidence |
|---|---|
| Compute-only waited for a publisher intentionally disabled by M07 | required_streams follows effective publish_commands; test_runner; actual compute-only CLI completed with zero command messages |
| Immediately orphaned setsid child could escape polling discovery | Private inherited ownership UUID and identity checks; five immediate orphan trials; m08_execution_orphan_red/green.log; unsupported deliberately clearing marker/session documented |
| Ctrl-C could continue to another variant | Interrupt persists current failure and always aborts study; test_interrupt_aborts_study_even_with_continue_policy |
| Different forecast horizon contracts were pooled as comparable | Comparison fingerprint includes predictor dt/horizon; configuration regression |
| Sparse safe feedback could produce accepted safety | Full bounded physical-coverage gate, pre-start forward hold; sparse is unknown, any observed overspeed still fails; metrics tests |
| Expensive process polling starved recorder acquisition | Cheap Popen poll; descendant watcher reduced; bounded callback drain; m08_execution_poll_benchmark.log; successful second real pair with hundreds of tracking samples |
| Nanosecond timestamp conversion split nominal forecast horizons | 1e-7-s reporting bins only; exact future/issuance epochs preserved; horizon quantization RED/GREEN |
| Pooled prediction aggregation used mean error instead of requested RMSE | Benchmark uses per-run rmse, preserves horizon RMSE aggregates; test_prediction_aggregation_uses_rmse_and_preserves_horizons |
| Readiness accepted ever-seen stale source epochs | Required source epochs must be within selected max_skew of clock; test_readiness_can_require_fresh_source_epochs |

The first physical pair under results/experiments/m08_verified completed but had
severely starved acquisition; it is preserved and is not accepted tracking evidence.
The seed42 rerun and final seed917 pair have actual estimates, predictions, physical
truth, finite safe commands and completed owned shutdown. Their failed physical
speed acceptance is retained. Current CV sampling/performance limitations are
explicit in the main report, including one feedback rejection and incomplete
physical coverage. No additional tuning or broad refactoring was attempted.

Final functional regression: 1071 passed, 18 existing style/copyright checks
excluded. Focused suite: 137 passed. Normal and copied ten-package builds passed;
outside-source copied-install checker verifies eight console entries and actual
M07/M08 examples. Final real invalid, compute-only and early-timeout checks retain
status/artifacts and no surviving owned process. No final second independent
review was requested; the fix pass is supported by targeted regressions and live
measurements. No commit/push.
