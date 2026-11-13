# D4.8 — Online Gaussian Process predictor

Overall verdict: **CONDITIONAL**. The mathematical, causal and integration contracts
pass. Predictive usefulness remains conditional on broader scientific evaluation;
one short compute-only Run cannot establish general superiority or closed-loop safety.

## Architecture and mathematics

`GaussianProcessPredictor` is discovered from `robot_prediction.baseline` by the
existing registry, using ID `gaussian_process` and display name **Gaussian Process**.
Numerical defaults belong only to `robot_prediction/config/prediction.yaml`.
No Factory, dashboard, controller, estimator, simulation or ROS interface production
changes were needed for D4.8. SciPy is now an explicit package dependency.

For each completed pair of consecutive estimated states, the training feature is
`x_previous=[p_previous,v_previous]` and the label is
`a=(v_current-v_previous)/(t_current-t_previous)`. The second estimate must have
arrived before the transition can enter history. Actual irregular intervals are used;
there is no truth access, future interpolation or offline initialization.

Features are normalized using the current causal fit window's mean and standard
deviation, with a configurable scale floor. Targets remain in acceleration units.
Independent output GPs share normalized features and one covariance factorization;
all have a zero acceleration prior. There is no output centering that would create
a nonzero acceleration prior far from data.

For normalized inputs, `k(x,z)=signal_variance*exp(-||x-z||²/(2*length_scale²))`.
The fitted matrix is `C=K+(observation_noise+jitter)I`. A positive-definiteness and
condition check precedes Cholesky factorization `C=LLᵀ`.
Posterior acceleration mean is `k_*ᵀ C⁻¹Y`; latent acceleration variance is
`signal_variance-||L⁻¹k_*||²`. Implementation uses triangular solves and cached
coefficients, without forming an inverse. This follows the regression formulation
in [GPML, chapter 2, Algorithm 2.1](https://gaussianprocess.org/gpml/chapters/RW2.pdf).

Recursive forecast steps evaluate the fitted GP at the newly propagated state:
`p_next=p+dt*v+0.5*dt²*a`, `v_next=v+dt*a`. This supports any even configured
position/velocity state dimension within the resource cap; tests cover 1, 2, 3 and 5
spatial dimensions. Forecast timestamps are precisely `origin+dt*[1,...,horizon]`.
The ROS node preserves the accepted estimate's fixed coordinate frame.

## Online lifecycle and uncertainty

The actual hook is `TrajectoryPredictionSystem.update(...)` dispatching to
`GaussianProcessPredictor.update_at(...)`. Training continues even if the next
estimate lies outside the previous forecast horizon. Prediction alone adds no
transitions. History is a bounded deque; refits are scheduled by completed pairs.
First fitting happens at minimum history, then every configured refit interval.
Rejected fits clear the fitted model and report `gaussian_process/cv_conditioning`.
Warm-up is `gaussian_process/cv_warmup`; arithmetic rejection is
`gaussian_process/cv_numerical`; accepted forecasts are `gaussian_process`.
The existing CV implementation supplies fallback states. Nonfinite fallback is rejected.

Invalid observation values, inconsistent dt, ordering or dimension changes discard
learning and raise. Long observation gaps discard history and seed a new warm-up.
The ROS node rejects queued duplicates/out-of-order/future messages before learning;
its existing clock-jump handler reconstructs the selected system. Independent
Factory instances share no learned data. No experiment or Run storage is duplicated.

`acceleration_posterior(state)` returns mean and nonnegative latent variance, one
value per independent output. This is conditional on deterministic query inputs,
fixed kernel parameters and the assumed noise likelihood. Tiny negative roundoff
is clipped within a machine-precision tolerance; larger negative variance is rejected.
Trajectory `covariance` and `confidence` remain **None**. The current ROS trajectory
message has no acceleration-variance field; no unsupported covariance is invented.
The existing residual-monitor confidence is a separate empirical metric, not GP calibration.

## Authoritative configuration

| Parameter | Default | Meaning |
|---|---:|---|
| kernel | rbf | Registered kernel family; only RBF currently supported |
| length_scale | 2.0 | Normalized feature coordinates |
| signal_variance | 1.0 | Acceleration variance, m²/s⁴ |
| observation_noise | 1.0 | Finite-difference acceleration observation variance, m²/s⁴ |
| jitter | 1e-8 | Numerical covariance diagonal nugget, m²/s⁴ |
| history_capacity | 64 | Completed pairs in sliding window |
| min_training_samples | 16 | Pairs before first fit |
| refit_interval | 5 | New pairs per refit |
| normalize_features | true | Window-local mean/std normalization |
| minimum_scale | 0.1 | Std floor in each state's coordinate units |
| max_condition | 1e10 | Covariance condition ceiling |
| max_observation_gap | 0.5 s | Reset learning after longer gaps |
| max_state_dimension | 64 | Input feature resource budget |
| max_horizon | 200 | Recursive forecast work budget |

The owner schema supplies units/bounds and the generic Experiment Lab controls.
Constructor checks also enforce cross-parameter validity. Implementation resource
ceilings are 256 history pairs, 128 state coordinates and 1,000 future steps; these
are workload limits, not robot physical constraints. Exact fitting costs O(N³),
memory O(N²+N*state_dimension), mean rollout O(horizon*N*state_dimension).
Optional acceleration variance adds triangular-solve work per queried state.
No kernel search, sparse framework, pretrained model or additional adaptation is introduced.

## Tests, build and review

- Initial independent GP tests failed on the absent module, then passed after implementation.
- **232 passed**: prediction/configuration functional suites, actual MPC tests, and
  GP/EDMD Scenario + Lab integration (79.16 s).
- **557 passed**: remaining control suite plus Recorder, component and scientific
  metric regressions (4.01 s). These exclude the MPC tests already in the first command.
- The 22 GP cases include an independent dense posterior reference (mean/variance
  agreement at 1e-13), kernel construction, duplicate/nearly duplicate inputs,
  conditioning rejection, analytical CV/acceleration, nonlinear recursive rollout,
  causality, bounded history/refits, noisy irregular estimates, resets, invalid data,
  future-grid/resource rejection and the real observation-update orchestrator.
- Scenario tests verify immutable revisions and pinned restore, strict schema validation,
  actual MPC forecast consumption and the rendered generic Lab selector/parameter fields.
  ROS tests verify GP/EDMD warm-up versus trained identity and clock-reset isolation.
- `colcon build --packages-select robot_prediction`: PASS. Standard overlay override
  warning only. Isolated installed-package check from `/tmp`: PASS; installed/source
  GP SHA256 identical, learned forecast produced, uncertainty fields unset.
- New GP source and both new test files pass Flake8 with the repository's 99-column limit.
  Existing style suites were excluded from the functional commands; no claim that the
  whole historical repository is lint-clean is made.
- Fresh read-only review found no critical or important defects; independently reran
  all 22 GP tests. The reviewer also forced finite CV numerical fallback and recovery.

## Real ROS/Gazebo evidence

One bounded **headless**, actuation-OFF Run used the existing Backend/Jobs/Runner,
nominal scenario, seed 42, GP + MPC, and no residual adaptation/fusion.
The inherited variant directory label is `imm`; the immutable configuration and ROS
`model_name` establish actual GP selection. No GUI or physical actuation acceptance
is claimed by this compute-only verification.

- Experiment: `EXP-GP-D48`, immutable revision 1.
- Run: `d0813ff9-ea41-42f9-9eb6-c0b3e88cb3da`.
- Session: `0cbb6217-7f94-4ca9-9f36-54f415e4c397`; references this existing Run.
- Job: `d00b5de0-f58e-4a69-bd13-9ce1557f47a1`.
- Runner readiness: origin **1.626 s**, maximum required-stream age **0.036 s**,
  unchanged `max_skew=0.08 s`.
- Settling **3 s**; scoring **4.626–12.626 s** (configured **8 s**); stop **12.628 s**.
- Execution and Job genuinely **completed**. Acceptance remains **null** because
  actuation was explicitly disabled, with **0 recorded command messages**.
- **188 live predictions**: 16 warm-up CV, 172 trained GP; all 135 scored-origin
  predictions were trained GP. **160 valid MPC computations**, 0 invalid.
- Mean RTF **0.964551**.
  MPC compute mean **15.215 ms**,
  maximum **42.272 ms**.
- Results API returns recorded tracking/pointing, estimation, prediction, control,
  physical-joint and RTF series. Prediction error has **2,345 samples**.
- For context only, stationary compute-only robot distance error mean
  **0.732553 m**, pointing error mean
  **0.128688 rad**. These are not tracking-quality acceptance.
- Recorded/replayed forecasts have **186 exactly matching
  timestamp origins**, with maximum state difference **0** and identical mode labels.
  This equality assertion applies to that matched set, not all 188 origins.
- Owned launch exited with return code 0 after SIGINT; **no survivors**. Independent
  process inventory found no remaining ROS launch, prediction node or Gazebo owner.
- **11,104 pre-task artifacts** checked by SHA256, **0 changed or missing**.
  Prior MPC/EDMD sources, Runs and experiment configurations are preserved.

Raw artifacts: `results/gaussian_process/verified/runs/nominal_20261111T202035_372001Z_znmd52o3/runs/imm_seed42`.
Backend/Session/readiness/history audit: `results/gaussian_process/verification.json`.

## Matched scientific comparison

Replay preserves recorded callback order, processes each estimate before forecasting,
and constructs fresh systems for CV and GP. Truth is read only after forecasts are
fixed. Both use the frozen owner input, dt=0.1 s, horizon=20, identical source origins,
frame, window and `max_skew=0.08 s`. Scoring uses existing
`robot_experiments.metrics.evaluate`; no new scoring implementation or ranked Session.
The estimate-file SHA256 is `e353bd041ffcb496cc7cf1d06c646e0a8fa78c52cd8238215d325e8471092218`.

| Scope | Scored future/truth samples per model | CV position RMSE (m) | GP position RMSE (m) |
|---|---:|---:|---:|
| All common origins | 2345 | 0.510018 | 0.486227 |
| Learned-only common origins | 2345 | 0.510018 | 0.486227 |

There are 188 common origins and 172 learned common origins over the complete recording.
The two scoring scopes coincide because warm-up ended before the scoring window.
Warm-up lasted **0.922 simulated seconds**; fallback was **8.51%** over all origins
and **0%** in the scoring window. All **35 fits accepted**, maximum retained history
**64**, **0 numerical prediction rejections**, **0 invalid/exception forecasts**.
This single short trace showed lower GP error, not general superiority.

| Horizon (s) | Matched samples | CV RMSE (m) | GP RMSE (m) |
|---:|---:|---:|---:|
| 0.1 | 133 | 0.103559 | 0.103094 |
| 0.2 | 132 | 0.129638 | 0.127831 |
| 0.3 | 130 | 0.159269 | 0.155435 |
| 0.4 | 128 | 0.191862 | 0.185372 |
| 0.5 | 127 | 0.227011 | 0.217573 |
| 0.6 | 125 | 0.265404 | 0.252869 |
| 0.7 | 123 | 0.305914 | 0.290435 |
| 0.8 | 121 | 0.347150 | 0.328273 |
| 0.9 | 120 | 0.390081 | 0.367645 |
| 1.0 | 118 | 0.433576 | 0.408928 |
| 1.1 | 116 | 0.476954 | 0.450633 |
| 1.2 | 115 | 0.522077 | 0.493583 |
| 1.3 | 113 | 0.567630 | 0.537643 |
| 1.4 | 111 | 0.615637 | 0.584564 |
| 1.5 | 110 | 0.663577 | 0.630743 |
| 1.6 | 108 | 0.710161 | 0.678874 |
| 1.7 | 106 | 0.753620 | 0.720401 |
| 1.8 | 105 | 0.797675 | 0.762079 |
| 1.9 | 103 | 0.838308 | 0.800429 |
| 2.0 | 101 | 0.878379 | 0.845663 |

Performance figures below come from a second **offline** replay after regression
processes finished, isolating the timing measurement. It did not launch another
simulator, fit to truth or change the scientific samples/results. The initial replay
is preserved under `comparison/`; isolated replay under `comparison_isolated/`.
They produce identical scientific RMSE. These are measured Python wall durations,
not instrumented per-fit ROS callback timing.

| Operation | Count | Mean (ms) | p95 (ms) | Maximum (ms) |
|---|---:|---:|---:|---:|
| GP fit | 35 | 0.434 | 0.649 | 0.855 |
| GP system update (includes fits) | 188 | 0.290 | 0.708 | 1.317 |
| GP forecast, 20 future states | 188 | 0.860 | 1.317 | 2.542 |
| CV forecast | 188 | 0.080 | 0.150 | 0.959 |

Recorded estimate intervals were 50–62 ms, median 60 ms. Fit and forecast timings
are below this measured cycle for the tested 6D / 64-history / 20-step setup.
No deadline guarantee is inferred for arbitrary resource caps or deployment machines.

## Independent verdicts and limitations

| Area | Verdict | Evidence / qualification |
|---|---|---|
| Mathematical correctness | PASS | Independent mean/variance reference, analytical trajectories, recursive query and conditioning checks |
| Causal learning | PASS | Completed estimate pairs only; resets and independent instances; same-stream causal replay |
| Uncertainty handling | PASS | Valid conditional acceleration variance; no invented trajectory covariance/confidence |
| Runtime performance | PASS for tested defaults | Bounded workload; measured fit/inference below observed cycle; real RTF 0.965 |
| Integration | PASS | Actual ROS/Gazebo completion, MPC computations, saved Results/Session, installed package and clean shutdown |
| Predictive usefulness | CONDITIONAL | Short matched trace only; no broad motion/noise/generalization or closed-loop superiority claim |
| Backward compatibility | PASS | CV/IMM/EDMD and downstream functional regressions; historical artifacts unchanged |

Finite differences amplify noisy estimated velocities, especially for short irregular
intervals. Labels are interval-average acceleration associated with the preceding
state; rapidly varying dynamics introduce approximation error. Adjacent differences
can have correlated errors, and state features themselves are uncertain. The current
independent, constant observation-noise likelihood does not model these effects or
learn noise online. Kernel hyperparameters are fixed, without calibration claims.
Data may leave the local window's support; RBF mean then tends toward zero acceleration.
Recursive mean rollout does not integrate uncertain state distributions. No full
trajectory uncertainty, confidence calibration, active-actuation safety or general
real-time scheduling guarantee is established by this milestone.

## Reproduction

From `/home/user/ros2_ws`:

```bash
source install/setup.bash
build/dashboard_venv/bin/python /usr/bin/colcon build --packages-select robot_prediction
source install/setup.bash
# New directory required; do not overwrite previous evidence.
build/dashboard_venv/bin/python results/gaussian_process/run_check.py \
  --output-dir results/gaussian_process/reproduction
# Replay the preserved verified Run; writes a new analysis directory.
build/dashboard_venv/bin/python results/gaussian_process/compare.py \
  --output results/gaussian_process/replay_reproduction
```

`run_check.py` creates an isolated Catalog, saves an immutable GP/MPC Scenario,
sets 8 s evaluation + 3 s settling, explicitly disables actuation, uses the authoritative
Runner with a 200 s outer cancellation guard, and adds the Run reference to a Session.
In an existing Experiment Lab, select **Gaussian Process**, save a new immutable
revision, and run through the normal lifecycle; no predictor-specific UI path is used.

To inspect the verified evidence in the existing dashboard (choose a free port):

```bash
ros2 run robot_dashboard dashboard --port 8060 \
  --data-dir "$PWD/results/gaussian_process/verified/workspace" \
  --catalog "$PWD/results/gaussian_process/verified/workspace/catalog.yaml" \
  --result-root "$PWD/results/gaussian_process/verified/runs"
```

Functional regression commands (source paths ensure current source is tested):

```bash
export PYTHONPATH="$PWD/src/robot_prediction:$PWD/src/robot_configuration:$PWD/src/robot_experiments:$PYTHONPATH"
build/dashboard_venv/bin/python -m pytest -q \
  src/robot_prediction/test src/robot_configuration/test \
  src/robot_control/test/test_model_predictive.py \
  src/robot_experiments/test/test_gaussian_process_integration.py \
  src/robot_experiments/test/test_koopman_integration.py \
  --ignore=src/robot_prediction/test/test_flake8.py \
  --ignore=src/robot_prediction/test/test_pep257.py \
  --ignore=src/robot_prediction/test/test_copyright.py \
  --ignore=src/robot_configuration/test/test_flake8.py \
  --ignore=src/robot_configuration/test/test_pep257.py \
  --ignore=src/robot_configuration/test/test_copyright.py
build/dashboard_venv/bin/python -m pytest -q src/robot_control/test \
  src/robot_experiments/test/test_recording.py \
  src/robot_experiments/test/test_components.py \
  src/robot_experiments/test/test_metrics.py \
  --ignore=src/robot_control/test/test_model_predictive.py \
  --ignore=src/robot_control/test/test_flake8.py \
  --ignore=src/robot_control/test/test_pep257.py \
  --ignore=src/robot_control/test/test_copyright.py
```

Logs: `results/gaussian_process/{build,regression,consumer_regression,lint,live}.log`.
Installed-source identity: `results/gaussian_process/installed_verification.json`.

## Exact D4.8 changed-file list

- `src/robot_prediction/robot_prediction/baseline/gaussian_process.py` — new model,
  numerical regression and owner schema registration.
- `src/robot_prediction/config/prediction.yaml` — GP numerical defaults only.
- `src/robot_prediction/setup.py` — declare SciPy dependency.
- `src/robot_prediction/package.xml` — declare ROS SciPy runtime dependency.
- `src/robot_prediction/test/test_gaussian_process.py` — new independent GP tests.
- `src/robot_prediction/test/test_prediction_node.py` — extend online ROS/reset contract to GP.
- `src/robot_prediction/test/test_prediction_components.py` — preserve legacy numerical defaults
  while allowing the additional registered family.
- `src/robot_prediction/test/test_stateless_model.py` — expected production inventory includes GP.
- `src/robot_experiments/test/test_gaussian_process_integration.py` — new immutable
  Scenario, MPC and generic Lab integration tests.
- `docs/validation/gaussian_process_d48.md` — this report.

Generated scripts, immutable configurations, Runs, Session metadata, raw telemetry,
replay forecasts and analysis JSON live under ignored `results/gaussian_process/`,
separate from production. No files removed; no unrelated cleanup or algorithm changes.
Pre-existing dirty MPC/EDMD changes are not attributed to D4.8. No commit or push.

## Execution ledger

1. Math and causal learner complete; initial missing-module RED, then GREEN, 22 cases.
2. Owner/Scenario/Lab/ROS/MPC integration complete; numerical defaults of existing
   algorithms unchanged. Production consumers required no predictor-specific branch.
3. One real actuation-OFF Run completed and saved; paired metrics and exact matched
   live/replay checks passed. Isolated offline replay resolved timing contention uncertainty.
4. Fresh review and all 789 selected functional tests passed; installed package and
   historical-artifact checks passed. Final verdict remains conditional for usefulness.

Ruling: use the requested current working tree without commits; keep the execution
ledger in this report instead of a duplicate planning structure. Cost: no separate
committed task checkpoints; the existing work/evidence is preserved in place.

Final review: no critical/important findings. Pending empirical judgments (live evidence,
runtime, usefulness and report completeness) were explicitly assessed above. Two minor
items deferred: automate the separately exercised numerical-fallback/recovery branch;
polish the repeated internal `_state`/`_fit` docstrings. Neither affects numerical behavior;
the reviewer manually verified finite fallback and subsequent recovery.
