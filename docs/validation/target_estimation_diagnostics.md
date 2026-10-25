# M09 target estimation diagnostics

**CONDITIONAL — the orange trajectory contains real measurement-driven posterior corrections.** In five preserved captures the filter reduces position norm RMSE by 49–55%, while retaining substantially more short-timescale roughness than physical truth. The data do not support an R mismatch or an implementation failure. Reactive-motion mismatch and the Q/R accuracy–roughness tradeoff remain measurable. Estimator corrections and forecast revisions are strongly related; physical wrist spikes cannot be attributed to estimation from these recordings.

No estimator, predictor, controller, PID, physics or deployed configuration was changed. No new Gazebo run was needed.

## Evidence and alignment

The authoritative inputs are normalized `telemetry/*.jsonl` envelopes (`source_time`, `frame_id`, `data`) and each run's archived `inputs/estimation.yaml`, `target_estimation.yaml`, and `measurements.yaml`:

- Seed 42: `results/experiments/m08_verified_final/paired_prediction_20261017T035322_605496Z_mykcu7wt/runs/{cv_seed42,imm_seed42}`.
- Seed 917: `results/experiments/m08_final_seed917/paired_seed917_20261018T061032_929053Z_750v4ffu/runs/{cv_seed917,imm_seed917}`.
- M09: `results/m09_headless_verified/headless_20261018T095903_806517Z_e0e8kb9e`.

[Machine-readable evidence, source hashes and event rows](../../results/validation/m09_target_estimation_20261018_refinement_final/m09_target_diagnostic_evidence.json) retain unrounded values. These are short, reactive closed-loop recordings; repeated seeds share synthetic noise sequences and are not independent experiments or identical physical paths. CV/IMM names select the **predictor**, while target estimation uses the same CV Kalman filter in every capture.

All target truth, measurement, posterior and prediction samples declare `world`. Measurement/posterior epochs pair one-to-one within 2 ns, allowing the observed 1 ns floating-point/ROS stamp serialization loss. Truth is evaluated at measurement source epochs using exact observations or linear interpolation between truth samples separated by at most 60 ms. There is no extrapolation or recorder-wall-time substitution. The first 1 s after the first target observation is excluded from scored windows; replay still starts from the first observation. M09 truth stops at 11.628 s, leaving **16** later target observations without truth; these are excluded from all truth comparisons.

| Capture | Scored source-time window (s) | Pairs | Truth effective Hz | Measurement/estimate effective Hz | Largest truth/update gap (ms) |
|---|---:|---:|---:|---:|---:|
| CV 42 | 2.520–10.536 | 138 | 84.66 | 17.18 | 22 / 64 |
| IMM 42 | 2.652–10.594 | 136 | 84.91 | 17.05 | 18 / 62 |
| CV 917 | 2.306–10.444 | 140 | 67.95 | 17.17 | 56 / 94 |
| IMM 917 | 2.760–10.888 | 140 | 76.76 | 17.12 | 38 / 78 |
| M09 | 2.264–11.628 | 163 | 76.72 | 17.40 | 50 / 78 |

Rates/gaps describe each full recorded target stream, not nominal YAML rates (measurement requests 20 Hz). No target duplicate/backward/nonfinite source epochs or frame changes were found. Repeated inactive control/command heartbeats at source epoch zero are counted in JSON and excluded from valid-control comparisons. These checks establish the recorded time/frame contract, without certifying every unrecorded callback or TF transformation.

## Accuracy, roughness and turns

RMSE is `sqrt(mean(sum(error_xyz**2)))`; axis RMSE and signed mean bias use exactly the same paired observations. Units below are metres; axis entries are x/y/z.

| Capture | Measurement norm RMSE | Estimate norm RMSE | Measurement axis RMSE | Estimate axis RMSE | Estimate bias |
|---|---:|---:|---|---|---|
| CV 42 | .1700 | .0769 | .1017 / .0959 / .0968 | .0357 / .0435 / .0524 | −.0016 / +.0029 / −.0042 |
| IMM 42 | .1686 | .0767 | .1015 / .0949 / .0956 | .0353 / .0432 / .0526 | −.0003 / +.0029 / −.0045 |
| CV 917 | .1798 | .0892 | .1050 / .1060 / .1005 | .0363 / .0655 / .0484 | −.0118 / +.0029 / −.0159 |
| IMM 917 | .1798 | .0900 | .1050 / .1060 / .1005 | .0367 / .0657 / .0494 | −.0097 / +.0003 / −.0178 |
| M09 | .1720 | .0776 | .1042 / .0965 / .0971 | .0350 / .0504 / .0475 | −.0026 / −.0072 / −.0002 |

Synthetic measurement error standard deviations are .0947–.1057 m across capture/axis, consistent in scale with the configured .1 m standard deviation. The same seed-917 measurement norm RMSE is expected from its shared additive noise sequence; it does not establish identical trajectories.

Roughness is the RMS Euclidean norm of the second difference divided by grid-step squared, in m/s². Truth is first evaluated at paired measurement epochs, so all streams have identical observation support. Bounded linear interpolation is used **only for numerical comparison**, on shared 100 ms and 200 ms grids; no bracket exceeds 120 ms and no difference bridges missing grid points. The 100 ms grid (10 Hz) is coarser than the observed 17 Hz update rate; the coarser 200 ms grid checks sensitivity. No band-limited derivative or physical-acceleration inference is made. This statistic measures grid-dependent curvature/noise and is not physical acceleration inferred from noisy samples.

| Capture | 100 ms: truth / measurement / estimate | 200 ms: truth / measurement / estimate |
|---|---|---|
| CV 42 | .422 / 34.084 / 4.901 | .420 / 9.004 / 1.862 |
| IMM 42 | .419 / 36.387 / 5.010 | .416 / 9.202 / 1.852 |
| CV 917 | .432 / 35.958 / 4.984 | .430 / 8.380 / 1.947 |
| IMM 917 | .418 / 37.716 / 4.622 | .415 / 8.591 / 1.936 |
| M09 | .429 / 35.045 / 4.911 | .428 / 8.603 / 1.835 |

The estimate is 11–12 times rougher than truth at 100 ms, while measurements are 7–8 times rougher than the estimate. The excess persists at 200 ms. A smooth truth trail alongside a jagged estimate is therefore consistent with the recorded estimator output, before RViz path construction.

Turn analysis finds axis extrema with at least .01 m excursion on either side within ±.5 s. It compares the local truth shape with `estimate(t + delay)`, searching 0–1 s at 50 ms resolution. Among interior optima with the complete search supported, per-capture median fitted shifts are .300/.150/.375/.350/.150 s in the table order; individual shifts span .05–.75 s. JSON retains every event, search-support boundary and local RMSE. These descriptive shifts mix filter lag, noise, amplitude bias and local shape. They are not identified causal latency; several shifts give only small error improvements and some optima lie at search boundaries.

![M09 truth, noisy measurement, posterior and covariance scale](../../results/validation/m09_target_estimation_20261018_refinement_final/m09_headless_target_timeseries.png)

## Actual filter and covariance contracts

`TargetStateEstimator` delegates to the linear `KalmanFilter`; `estimation_node._target_measurement_callback` predicts to each incoming measurement source epoch, updates, then publishes that posterior in the measurement frame. The first observation updates the initial prior without prediction. The process state is `[px,py,pz,vx,vy,vz]`; the actual Q is

`Q(dt) = sigma_a² [[dt⁴ I/4, dt³ I/2], [dt³ I/2, dt² I]]`.

The effective archived deployment parameters are **sigma_a=.5**, **R=diag(.01,.01,.01)**, initial mean zero and initial covariance identity. `target_estimator.yaml` alone has sigma_a=.1; using that instead of the `simulation_estimation.yaml` deployment override would produce a false diagnosis. Synthetic `SyntheticMeasurementGenerator.measure` draws additive multivariate Gaussian noise with covariance .01 I, matching R. The discrete Q contract above is reproduced as implemented; this audit does not reinterpret sigma_a as a continuous-time spectral density.

An independent pure NumPy replay reproduces **every captured posterior mean and covariance exactly (maximum absolute differences 0.0)** in all five runs, from their archived initial priors and complete measurement sequences. This verifies the initialization, predict/update order and noise contracts for these captures, making reconstructed **pre-update** innovation `z−Hx_prior`, innovation covariance `HP_prior Hᵀ+R`, gain, and correction valid. Post-update measurement residuals are never called innovations.

After startup, mean diagonal position gain is .169–.171. Median position correction norm is .0297–.0316 m, with p95 .0545–.0574 m. Centimetre-scale discrete corrections occur approximately every 58 ms, explaining much of the visual jaggedness. Position covariance trace falls from .029703 m² at the first published posterior to about .005 m²; scored covariance eigenvalues remain positive (minimum ≥7.53e−4), with asymmetry ≤1.74e−18.

| Capture | Mean NIS (3D) | Fraction NIS ≤7.815 | Mean position NEES (3D) | Fraction NEES ≤7.815 |
|---|---:|---:|---:|---:|
| CV 42 | 3.342 | .928 | 3.488 | .949 |
| IMM 42 | 3.299 | .926 | 3.461 | .949 |
| CV 917 | 3.674 | .929 | 4.690 | .836 |
| IMM 917 | 3.634 | .921 | 4.770 | .836 |
| M09 | 3.388 | .920 | 3.602 | .939 |

Position NEES uses only the position marginal covariance and contemporaneous world-frame position error. The three-dimensional chi-square 95% reference is descriptive: reactive unmodeled acceleration, short runs, shared seeds and temporally correlated errors preclude a formal consistency certification. Innovation lag-one axis correlations span approximately −.139 to +.245; seed-917 position NEES shows greater overconfidence than seed 42. This supports investigating motion-model adequacy, without proving a unique Q defect.

A controlled offline 3×3 sensitivity grid uses **identical complete recorded measurements and initial states**, with sigma_a=.25/.5/1 and R multipliers .5/1/2; all metrics use the original scored windows. R is physically supported by the injected noise, so changing it would deliberately violate that known sensor contract. With R unchanged:

| Capture | sigma_a=.25: RMSE / roughness | Deployed .5: RMSE / roughness | sigma_a=1: RMSE / roughness |
|---|---|---|---|
| CV 42 | .1134 / 3.583 | .0769 / 4.901 | .0669 / 6.770 |
| IMM 42 | .1117 / 3.654 | .0767 / 5.010 | .0670 / 6.936 |
| CV 917 | .1266 / 3.663 | .0892 / 4.984 | .0794 / 6.872 |
| IMM 917 | .1235 / 3.425 | .0900 / 4.622 | .0805 / 6.373 |
| M09 | .1149 / 3.577 | .0776 / 4.911 | .0675 / 6.804 |

RMSE units are m and roughness is the 100 ms statistic. Making the posterior smoother worsens these errors; increasing process response improves these in-sample errors while making the path rougher. No parameter was selected for deployment; these short, correlated scenarios are insufficient for tuning.

## Temporal relation to prediction and wrist commands

Predictions share target-update source epochs and world frame. Forecast revision compares the new first future position with the preceding forecast interpolated at **the same absolute future epoch**; comparing consecutive first points directly would confound revision with target motion. For each estimate event, the first valid control epoch at/after it must be within 120 ms, with a previous valid control sample within 120 ms. The physical response statistic is the peak named `wrist_joint` speed actually recorded in the following 120 ms; unsupported windows remain missing. Joint names come from the archived layout, not feedback array order.

Control telemetry is in `odom`, so world-frame target points are not subtracted from controller target points. The following Pearson correlations compare scalar magnitudes and compatible joint velocities only:

| Capture | Events | Correction → forecast revision | Revision → absolute raw wrist change | Safe wrist change → following observed wrist peak (N) |
|---|---:|---:|---:|---:|
| CV 42 | 138 | 1.000 | .296 | .112 (134) |
| IMM 42 | 136 | .974 | .201 | .118 (136) |
| CV 917 | 140 | 1.000 | .099 | .413 (131) |
| IMM 917 | 140 | .970 | .213 | .051 (136) |
| M09 | 163 | .973 | .343 | .219 (160) |

Raw and safe wrist values coincide for these aligned events. Control telemetry does **not** expose its consumed prediction epoch/sequence, so a neighboring control sample may still use an earlier input. Feedback windows overlap, rates differ, and incomplete physical sampling can miss peaks. Consequently these correlations have no causal or independent-sample significance interpretation.

The largest position-correction event in each scored capture illustrates the variable physical response:

| Capture | Estimate epoch (s) | Correction / forecast revision (m) | Raw=safe wrist (rad/s) | Following observed physical peak (rad/s) |
|---|---:|---:|---:|---:|
| CV 42 | 8.782 | .0763 / .0883 | −.1435 | 1.4196 |
| IMM 42 | 6.400 | .0761 / .0939 | −.3941 | 1.5000 |
| CV 917 | 8.954 | .0838 / .0971 | +.0080 | 1.0980 |
| IMM 917 | 9.380 | .0802 / .1113 | −.0182 | .4069 |
| M09 | 8.340 | .0686 / .0904 | +.0729 | 1.5000 |

Values rounded to 1.5000 are not automatically physical limit violations; the unchanged tolerance and physical spike analysis are in [joint diagnostics](joint_motion_and_actuation_diagnostics.md). These events support real estimator-to-prediction coupling and some temporal association with command changes, while leaving the cause of the much larger physical wrist excursions unresolved.

![M09 innovations, forecast revisions, commands and physical wrist samples](../../results/validation/m09_target_estimation_20261018_refinement_final/m09_headless_coupling_timeseries.png)

All five target and coupling plots are retained beside the JSON. Physical velocities are displayed as individual measured points, avoiding invented interpolation across feedback gaps.

## Hypotheses, action and limits

| Hypothesis | Evidence and disposition |
|---|---|
| Synthetic noise | Strong support: known .1 m noise, measured axis spread near .1 m, discrete posterior corrections near .03 m norm. |
| Incorrect Kalman implementation | No evidence in these captures: full posterior/covariance reproduction is exact; filter substantially reduces RMSE. |
| Inconsistent R | Contradicted by archived sensor covariance and measured spread. |
| Insufficient Q / CV motion mismatch | Plausible tradeoff: turn-shape shifts, higher seed-917 NEES, and increased-Q offline error improvement. No unique defect or validated replacement setting established. |
| Timestamp/frame errors | No target-stream reset/order/frame fault observed; small stamp serialization loss is bounded. Unrecorded delivery/TF behavior is not certified. |
| Visualization construction | Cannot explain roughness already present in raw source-time posteriors. Rendering continuity checks belong to the M09 visualization implementation. |
| Estimation causes physical wrist spikes | Not established. Strong forecast coupling, weak-to-moderate command associations, missing input lineage and low-level dynamics prevent causal attribution. |

Next work belongs to **robot_estimation**: evaluate the existing CV prior on longer held-out seeds and maneuver scenarios, retain the known sensor R, and score accuracy, turn behavior, innovation consistency and roughness together before proposing any tuning. **robot_experiments / robot_control** need the smallest extra lineage field: the consumed prediction reference timestamp in control telemetry (and its resulting command sequence), enabling exact input-to-command association. **robot_simulation** owns the separately observed physical velocity response; consult the joint report before any PID/physics change.

[M03's existing limitations](../m03_estimation_localization.md) remain: skid-related localization drift, non-real-time scheduling, intermittent target availability and physical speed enforcement. This audit does not relabel these as new estimator findings or certify covariances for safety. Its conclusions apply to the preserved observations, not all future executions.

## Reproduction and checks

```bash
cd /home/user/ros2_ws
source install/setup.bash
PYTHONPATH=src/robot_experiments:$PYTHONPATH python3 -m pytest \
  src/robot_experiments/test/test_target_diagnostics.py -q \
  -p no:launch_testing -p no:launch_ros
MPLCONFIGDIR=/tmp/m09_target_mpl PYTHONPATH=src/robot_experiments:$PYTHONPATH \
  python3 tools/validation/m09_target_diagnostics.py \
  --output /tmp/m09_target_diagnostics_reproduction
```

The CLI refuses to overwrite an existing output directory; choose a fresh path. It validates frame/epoch contracts and exact posterior replay before deriving innovations, then emits a JSON evidence file and ten standalone PNG plots. Pure analytical tests cover norm/axis RMSE, bias, bounded interpolation/no extrapolation, duplicate/reset rejection, common-grid acceleration and gap handling, known Kalman innovations/gains/Joseph covariance, the deployed dt⁴ Q contract, causal previous-sample alignment, invalid noise and a known turn delay. **23 focused tests passed**; all five recorded replays and the complete evidence-generation command completed successfully. No ROS/Gazebo node is launched by this diagnostic workflow.
