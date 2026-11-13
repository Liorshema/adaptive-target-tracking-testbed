# D4.7 — Koopman (EDMD)

**Overall verdict: CONDITIONAL.** EDMD is implemented, causal, numerically tested,
registered, and verified through a real ROS/Gazebo Run and MPC. It did **not** beat
CV on the controlled recorded-input comparison, and many candidate operators were
rejected. No commit or push. No GRU, GP or Predictor Comparison Session implemented.

## Independent acceptance

| Area | Verdict | Scope / evidence |
|---|---|---|
| EDMD mathematics | PASS | Independent ridge least-squares reference, known linear matrix powers, nonlinear closed-subspace example, exact decoder and repeated lifted propagation |
| Causal learning / isolation | PASS | Existing timestamped update hook; bounded past estimates; irregular-time resampling; duplicate/time reversal/gap/reset tests; no truth in learning |
| Prediction contract | PASS | Canonical position/velocity dimensions, requested timestamps, explicit model/fallback identity, finite checks; no invented covariance/confidence |
| Factory / Lab / immutable authoring | PASS | Actual registered class, schema controls, saved/pinned revisions, generic UI label, real MPC consumer |
| Numerical robustness / work bounds | PASS | Ridge condition guard, rejected operator growth, finite fallback validation, configurable per-call transition budget, failure tests |
| Computation | PASS | Paired replay: fit mean 1.033 ms, inference mean 0.208 ms / max 0.969 ms; six-state degree-two model and 20-point horizon |
| ROS/Gazebo execution | PASS | Genuine eight-second scored window, 160 valid MPC computations, saved results, Session reference, readiness predicate unchanged, no cleanup survivors |
| Compatibility / preservation | PASS with rebuild | Historical owner/JSON compatibility tested; D4.6 source unchanged. ROS message schema extension requires rebuilding/restarting consumers |
| Demonstrated predictive usefulness | CONDITIONAL | EDMD 0.640 m RMSE versus CV 0.496 m on identical inputs/score samples; learned-only subset also worse; substantial explicit CV fallback |

Successful execution is separate from predictive improvement and physical
actuation safety. Physical actuation was explicitly **OFF** in these checks.
This milestone makes no new robot-tracking or hardware-safety acceptance claim.

## Architecture and shared extensions

`robot_prediction/baseline/koopman_edmd.py` owns the generic polynomial dictionary,
ridge fit, bounded online lifecycle and forecast. It implements the existing
`Predictor` interface and is discovered as `koopman_edmd`; owner display name is
**Koopman (EDMD)**. The existing trusted predictor namespace is named `baseline`;
no second namespace or prediction engine was introduced.

Reused: `PredictionInput`, `TrajectoryPrediction`, `StateLayout`,
`TrajectoryPredictionSystem.update_at`, monitoring, Factory/Registry, Author,
ExperimentBackend/Jobs/Runner, Recorder and scientific metrics. Dashboard and MPC
production source were not modified in D4.7. The D4.6 metadata-driven label support
already renders the new display name and schema controls.

Necessary shared changes:

- Optional `configured_only` discovery allows an immutable old prediction owner
  snapshot to omit algorithms introduced later. Present declarations still validate
  strictly. Selecting an absent model fails; it never becomes IMM/CV silently.
  Default discovery behavior for other callers remains strict.
- Generic `model_name` appended to ROS `PredictedTrajectory`. The existing field in
  Python `TrajectoryPrediction` previously vanished at the ROS boundary. Recorder
  normalization already retains additional message fields, so no recording branch
  was added. Old JSON without this field remains readable.
- Correction fusion retains the baseline identity as
  `<baseline>+weighted_correction`, preserving warm-up visibility. Its numerical
  behavior is unchanged. The scientific comparison disables correction/fusion for
  both predictors.

**ROS compatibility:** this is a message schema/type-support change, not wire ABI
compatibility with old binaries. Build `robot_interfaces`, `robot_configuration`
and `robot_prediction`, source the resulting workspace, and restart ROS consumers.
All deployed prediction consumers here are Python; source-level consumers ignore
or record the extra metadata without changing numerical algorithms.

## Mathematical formulation

Primary reference: [Williams, Kevrekidis & Rowley, *A Data-Driven Approximation of
the Koopman Operator: Extending Dynamic Mode Decomposition*](https://arxiv.org/abs/1408.4408).

The supported common state is `x=[position, velocity]` in any configured spatial
dimension. Pure tests cover 2-, 4- and 6-coordinate states. No 3D matrices or robot
joint counts are embedded in EDMD.

At each refit, compute center `mu` and per-coordinate standard deviation `s` from
only the retained, already received training states. Apply a configurable positive
scale floor. Let `a=(x-mu)/s`. The dictionary contains:

```
psi(x) = [1, a_1, ..., a_d, all total-degree monomials up to degree p]
```

Degree two includes squares and cross terms. Degree three is supported; `linear`
basis always uses degree one. Feature count is `choose(d+p,p)` and is checked
against `max_features` before allocating the dictionary. All observables are
normalized/dimensionless. Physical state reconstruction is exactly linear:

```
C = [mu, diag(s), zeros]
C psi(x) = x
```

For column snapshots `X=[psi(x_k)]`, `Y=[psi(x_{k+1})]` separated by fixed training
interval `h`, solve ridge EDMD:

```
K = Y X.T (X X.T + lambda I)^-1
X = U diag(sigma) V.T
K = (Y V) diag(sigma/(sigma²+lambda)) U.T
```

The implementation uses SVD, never an explicit inverse. It checks finite features,
operator values and the condition number of the **regularized** Gram matrix,
including missing singular directions when snapshots are rank deficient. Ridge
also acts on the constant feature; no eigenvalue or operator coefficient is
silently overwritten after fitting.

Each prediction starts from `psi(current estimate)` and repeatedly propagates
`z <- K z`; it does **not** reconstruct and relift at every step. Integer horizons
return `C K^j psi(x)`. Requested horizons falling between training ticks use
explicit linear interpolation between neighboring decoded lifted iterates.
Absolute output timestamps remain exactly the caller's `timestamp + dt*[1..N]`.
This is a discrete-time model with a declared interpolation convention, not an
identified continuous-time generator.

## Causal online lifecycle

The first prediction seeds the first observed state. Later learning is performed
through the existing `TrajectoryPredictionSystem.update_at` hook before predicting
at that same observation timestamp. Repeating a prediction does not add data.

Received estimates may be irregular. An estimate at `t_b` completes all training
grid points between the previous estimate `t_a` and `t_b`; those states are linearly
interpolated **only after the bracketing estimate has arrived**. No estimate after
the current prediction origin, target truth or external training data is used.
The grid is anchored to the first estimate and computed from integer tick indices.
Machine-precision integer snapping handles roundoff; it does not modify source
message timestamps or relax Runner freshness/readiness tolerances.

A deque retains at most `history_capacity+1` grid states, defining at most
`history_capacity` transition pairs. A first fit requires `min_training_samples`
pairs. Further refits require `refit_interval` new pairs. Each fit freezes its own
normalization/dictionary/decoder/operator. New Run construction and ROS clock
reset create fresh instances. Explicit `reset()` clears all learned state.

Duplicate/decreasing or inconsistent observation times raise and reset the pure
predictor. The existing Node rejects duplicate/out-of-order messages before this
hook and reconstructs the selected system on a clock epoch change. A long missing-
data gap starts a fresh epoch rather than bridging it with fabricated dense data.
A prediction origin earlier than training data is rejected and clears that epoch;
callers must provide a timestamped update before advancing the prediction origin.
Frames are fixed by the existing Node validation/TF contract; pure prediction
interfaces themselves have no frame field. Direct callers must reset on frame
changes and preserve the same fixed-coordinate interpretation.

Actual emitted identities:

- `koopman_edmd/cv_warmup`: insufficient causal training pairs.
- `koopman_edmd/cv_conditioning`: numerical/conditioning fit rejection, with reason
  available in the predictor's detached `diagnostics`.
- `koopman_edmd/cv_unstable`: candidate learned exponential growth exceeds policy.
- `koopman_edmd`: actual lifted model propagation.

A rejected fit discards the operator and waits for the next configured refit,
using explicitly labeled CV in the meantime. A trained nonfinite rollout raises
instead of returning an unchecked forecast. CV fallback is also finite-validated.
No covariance/confidence is fabricated; both remain `None`. Existing residual
monitoring is retained and should not be interpreted as calibrated EDMD uncertainty.

## External defaults and numerical safeguards

All tuning belongs to `robot_prediction/config/prediction.yaml` and its registered
schema. No second YAML profile or predictor-specific dashboard panel was added.

| Parameter | Default | Interpretation |
|---|---|---|
| `basis` | polynomial | Constant + original linear coordinates + optional polynomial terms |
| `polynomial_degree` | 2 | Total degree 1..3; ignored by linear basis |
| `regularization` | 0.0001 | Ridge lambda in normalized observable coordinates |
| `training_interval` | 0.1 s | Identified discrete transition interval |
| `history_capacity` | 200 pairs | Bounded retained history, approximately 20 s |
| `min_training_samples` | 30 pairs | First fit after at least approximately 3 s |
| `refit_interval` | 5 new pairs | Nominal 0.5 s refit cadence |
| `max_condition` | 1e10 | Regularized Gram condition ceiling |
| `minimum_scale` | 0.001 | Std floor applied in each coordinate's physical units |
| `max_observation_gap` | 0.5 s | Longer gaps restart learning |
| `max_features` | 256 | Dictionary allocation limit |
| `max_transition_steps` | 1000 | Per-call resampling/rollout and output-count work budget |
| `max_spectral_radius` | 1.05 | Maximum accepted candidate radius per training interval |

Conditioning alone did not make the initial default model useful: preliminary
Runs produced very large but finite long-horizon outputs. A recorded-estimate-only
probe found radius up to **1.639 per 0.1 s transition**. Therefore the final model
adds a declared operator-growth eligibility policy. It rejects excessive growth
and labels fallback; it never clips eigenvalues or changes the ridge solution.

This policy permits moderate growth and is **not** a global stability guarantee:
non-normal transient amplification and inaccurate finite-dimensional closures can
remain. It can reject genuinely expanding dynamics; other experiments must choose
an appropriate externally declared policy. No control weights, physical limits,
training-horizon sweep, covariance tuning or target-dynamics changes were used.

The transition budget is checked before forecast allocation or dense resampling.
It prevents tiny positive intervals from requesting billions of operations despite
a bounded deque. Oversized resampling requests reset and fail explicitly; oversized
forecast requests fail before propagation. Typical six-state/degree-two operation
uses 28 observables and 20 transitions, well below these bounds.

## Independent tests and review

Focused mathematical/lifecycle tests cover:

- Ridge K against an independently constructed augmented least-squares problem.
- A known coupled linear system against independent matrix powers.
- A nonlinear triangular map with quadratic dependence: polynomial lifted rollout
  error below 1e-8, while the linear dictionary exceeds 0.01 on the same example.
  This is a mathematical test, not a robot prediction-improvement claim.
- Exact reconstruction, cross terms, repeated K propagation versus relifting,
  physical layout/dimensions and fractional timestamp grids.
- Causal bracketing, no training from repeated forecasts, bounded eviction/refits,
  Run isolation, explicit reset, clock reset, bad time sequences and long gaps.
- Conditioning failure, excessive operator growth, nonfinite learned and CV
  outputs, feature limits and transition-work budgets, including tiny intervals
  and huge integer horizons before allocation.
- Factory/schema selection, pinned immutable Scenario revisions, actual generic
  Experiment Lab controls, ROS model labels through reset/fusion, and the real MPC
  command consumer with unchanged independent safety projection.

Fresh read-only review supported the ridge orientation, decoder, lifted rollout
and causal lifecycle. It found unbounded derived transition work (Important) and
unchecked CV-fallback finiteness (reported Minor, treated as a contract defect).
Three reproducing tests failed before correction and passed afterward. No findings
were deferred. The later measured growth issue was addressed with the explicit
policy and a known expanding-system rejection test.

Final checks:

- **177 passed**: prediction functional tests and robot_configuration tests,
  including all **33** focused EDMD cases (`final_functional_tests.txt`).
- **559 passed**: existing control/MPC consumers, Recorder and public experiment
  components (`consumer_regression.txt`).
- **2 passed**: actual Scenario persistence/MPC integration and generic Experiment
  Lab rendering (`lab_integration_tests.txt`).
- New production module and focused integration/test files pass Flake8 at the
  project's 99-character convention (`final_new_file_lint.txt`). Existing files
  were not broadly reformatted; historical whole-package style gates were not
  represented as functional regressions.
- Selected-package build succeeded for robot_interfaces, robot_configuration and
  robot_prediction (`final_build.txt`). Only the ordinary installed-overlay warning
  was emitted. Isolated `/tmp` imports use installed modules matching source and
  the generated ROS message contains `model_name` (`installed_verification.json`).

## Real ROS/Gazebo verification

Saved Scenario: **EXP-KOOPMAN-D47 revision 1**, seed 42, nominal physical setup,
actual `KoopmanEDMDPredictor`, correction/fusion explicitly `none`, actual MPC.
The inherited variant directory is named `imm_seed42`; this historical variant
label does not determine the constructed predictor. Actual types are in frozen
configuration and launch logs. Physical actuation is explicitly **OFF**.

Final authoritative Run: **adf5eac5-68b4-4d46-a449-ce60e3e19feb**.

- Runner readiness genuinely observed at simulation **0.376 s** with unchanged
  `max_skew=0.08 s`; maximum required-stream age was 0.056 s.
- Three seconds settling, scored window **3.376–11.376 s**, stopped at 11.378 s.
- Genuine execution state **Completed**. Overall acceptance is `null` for this
  compute-only execution; it must not be relabeled as actuation acceptance.
- 192 recorded forecasts, each with explicit model identity; 20 future six-state
  samples per forecast. Scored window: 137 forecasts, 160 valid MPC computations,
  zero invalid scored control samples.
- RTF **0.939**. Whole MPC/control pipeline mean **16.857 ms**, p95 **28.426 ms**,
  maximum **36.699 ms** against the existing 50 ms period.
- Readiness-required streams all produced data. Empty command stream is expected
  with publication/actuation disabled; optional localization/diagnostic streams
  are not claimed as recorded sensor evidence.
- Existing Recorder saved metrics, time series and raw telemetry; Results API
  returns them. Session **73e0abee-b545-413b-9c36-0008f8011258** references this
  immutable Run directory. No duplicate result store or comparison Session.
- Owned cleanup returned 0, empty survivor list. Final process audit is separate.

Two preliminary bounded Runs and the diagnostic growth probe are preserved under
`runs/` and `final/runs/`. They exposed unacceptable unguarded extrapolation and
are not the final validated configuration. The guarded Run is under
`verified/runs/`; no preliminary evidence was deleted or overwritten.

## Controlled CV comparison

`results/koopman_edmd/compare.py` replays the final Run's recorded target estimates
in their original arrival-record order into fresh CV and EDMD systems. Both use
identical estimates, frames, origins, dt=0.1 s, horizon=20, correction/fusion=`none`,
and causal prefix-only updates. Truth is accessed only after all forecasts are
fixed, for scoring by the **existing** scientific metric engine. This is offline
analysis of one Run, not a second execution engine or scored Run.

Common evaluation window is 3.376–11.376 s, unchanged truth-bracket skew 0.08 s;
no end-of-window extrapolation. Horizon-wise sample counts are explicitly matched.
190 matched live/replay forecast origins have **zero maximum state difference**
and identical model labels, supporting replay fidelity. Two unmatched early live
origins are not invented. The final live and replay aggregate prediction RMSE
also agree exactly over the scored samples.

| Scope | CV position RMSE | EDMD position RMSE | Matched forecast/truth samples |
|---|---:|---:|---:|
| All common scored origins, including visible fallback | 0.49553 m | 0.64007 m | 2,389 |
| Only origins where an accepted learned model was used | 0.40816 m | 0.79976 m | 829 |

The learned-only subset is chosen from model eligibility before observing future
truth and applied equally to CV. Fallback is not hidden or counted as learned EDMD.
No IMM comparison, improvement percentage or automated ranking is claimed.

| Horizon | CV RMSE (m) | EDMD RMSE (m) | Paired samples |
|---|---:|---:|---:|
| 0.1 s | 0.10280 | 0.11959 | 136 |
| 0.5 s | 0.22276 | 0.32074 | 129 |
| 1.0 s | 0.41962 | 0.55272 | 120 |
| 1.5 s | 0.63741 | 0.82121 | 112 |
| 2.0 s | 0.86944 | 1.08560 | 103 |

All 20 horizons, raw error samples, complete forecast records and input SHA-256
are in `comparison/summary.json` and companion files.

Learning / fallback observations:

- First fit after **3.022 s** of received estimates; first accepted trained forecast
  after **6.040 s** because early operators failed the growth policy.
- Live modes: **55 warm-up CV**, **78 growth-rejection CV**, **59 learned EDMD** out
  of 192 forecasts: total fallback **69.27%**. Scored live fallback: 78/137 = **56.93%**.
- Replay modes: 55 / 78 / 60 out of 193 inputs, including the last observation
  received around shutdown. Replay fallback: **68.91%** overall, **56.52%** in its
  scored origins. These different denominators are reported separately.
- 17 replay fit attempts, **9 explicit growth rejections**, 8 accepted fits;
  zero replay exceptions/invalid forecasts for either predictor. Rejected fits
  are model eligibility outcomes, not silently successful numerical learning.

Measured wall times in this paired replay (including ordinary system monitoring):

| Operation | Mean | p95 | Maximum |
|---|---:|---:|---:|
| EDMD fit, 17 attempts | 1.033 ms | 1.762 ms | 2.742 ms |
| EDMD update, 193 inputs | 0.292 ms | 1.041 ms | 3.152 ms |
| EDMD predict, 193 forecasts | 0.208 ms | 0.499 ms | 0.969 ms |
| CV predict, 193 forecasts | 0.066 ms | 0.135 ms | 0.186 ms |

This is one noisy reactive recording with a stationary robot during compute-only
verification, not an independent broad holdout study. The growth policy was added
following diagnostic Runs; these results do not establish generalization or an
optimal regularization/dictionary. No further tuning was attempted to manufacture
an improvement claim. Sparse excitation, noisy velocity estimates, finite closure
and extrapolation remain practical limitations. CV is better on this evidence.

## Preservation and reproduction

`verification.json` confirms all **7,321** pre-existing result JSON/YAML documents
are byte-identical, plus **115** MPC/control/dashboard source/config/test assets.
D4.6 implementation and report are retained. No existing files removed or relocated.
Generated Runs/analyses are under `results/koopman_edmd/`, separate from production.

Build and verify from `/home/user/ros2_ws`:

```bash
source install/setup.bash
build/dashboard_venv/bin/python /usr/bin/colcon build \
  --packages-select robot_interfaces robot_configuration robot_prediction
source install/setup.bash
PYTHONPATH="$PWD/src/robot_configuration:$PWD/src/robot_prediction:$PYTHONPATH" \
  build/dashboard_venv/bin/python -m pytest src/robot_prediction/test \
  --ignore=src/robot_prediction/test/test_flake8.py \
  --ignore=src/robot_prediction/test/test_pep257.py \
  --ignore=src/robot_prediction/test/test_copyright.py src/robot_configuration/test -q
```

Repeat the same saved immutable Scenario through the authoritative Runner, with a
**new** output directory and physical actuation OFF:

```bash
build/dashboard_venv/bin/python results/koopman_edmd/verified_run_check.py \
  --output-dir results/koopman_edmd/reproduction_01
```

The helper reads `verified/workspace/catalog.yaml`, revision 1; it does not recreate
or alter that Scenario. It retains the Runner's duration, settling, readiness,
process isolation and owned cleanup, with a bounded outer cancellation guard.

Reproduce the paired analysis of the existing final Run without rerunning Gazebo:

```bash
build/dashboard_venv/bin/python results/koopman_edmd/compare.py \
  --output results/koopman_edmd/comparison_reproduction_01
build/dashboard_venv/bin/python results/koopman_edmd/verify_evidence.py
```

New analysis/output directories must not exist. In Experiment Lab, select
**Koopman (EDMD)** normally; all model parameters come from the owner schema.
For the actual saved validation Scenario and Results, start the existing dashboard
on a verified free port:

```bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir /home/user/ros2_ws/results/koopman_edmd/verified/workspace \
  --catalog /home/user/ros2_ws/results/koopman_edmd/verified/workspace/catalog.yaml \
  --result-root /home/user/ros2_ws/results/koopman_edmd/verified/runs
```

Select `Koopman EDMD integration`, revision 1. No server is left running by this task.

## Exact D4.7 changed files

Production:

1. `src/robot_prediction/robot_prediction/baseline/koopman_edmd.py` — dictionary,
   ridge fitting, causal bounded predictor, diagnostics and registration.
2. `src/robot_prediction/config/prediction.yaml` — authoritative EDMD numerical defaults.
3. `src/robot_prediction/robot_prediction/factory/component_registry.py` — discover only
   configured extensions for historical owner snapshots.
4. `src/robot_configuration/robot_configuration/model_registration.py` — optional
   configured-only discovery, default strict behavior retained.
5. `src/robot_prediction/robot_prediction/nodes/prediction_node.py` — preserve model
   identity on full ROS forecasts.
6. `src/robot_prediction/robot_prediction/fusion/weighted_correction.py` — preserve
   baseline/warm-up identity through correction metadata.
7. `src/robot_interfaces/msg/PredictedTrajectory.msg` — generic model identity field.

Tests/documentation:

8. `src/robot_prediction/test/test_koopman_edmd.py` — independent mathematics/lifecycle/failure tests.
9. `src/robot_prediction/test/test_prediction_node.py` — ROS identity and selected-model reset.
10. `src/robot_prediction/test/test_prediction_components.py` — preserve old defaults
    while recognizing the new optional family.
11. `src/robot_prediction/test/test_stateless_model.py` — expanded production inventory.
12. `src/robot_experiments/test/test_koopman_integration.py` — Scenario/Lab/MPC integration.
13. `docs/validation/koopman_edmd_d47.md` — this design/evidence/verdict record.

Implementation plan complete: mathematical/online tests → normal registry and
provenance integration → measured growth diagnosis and explicit rejection → final
bounded Run, paired scientific comparison, tests/build/preservation/cleanup.
Remaining work is predictive-quality research, not an unfinished integration.
Stop at D4.7; no commit/push or next predictor milestone.
