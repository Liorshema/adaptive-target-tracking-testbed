# Reliable localization: design and prespecified acceptance

Date: 2026-10-22. Specification: attachment aa63bca2-0b8f-48a8-8f01-2d5ac1a47d64.
Baseline: m03_localization_baseline.json, 1,657 pre-existing files. No staging/commits/push; preserve all prior experiments and DART native sources/binaries.

## Alternatives and decision (before implementation)

A. Wheel-only geometry/encoder correction: geometry/signs already checked against actual encoders. Four fixed wheels at fore/aft offsets require lateral contact slip for yaw. Better calibration can reduce systematic error but does not observe absolute XY/yaw; wheel-only global drift is not generally bounded.
B. Wheel + IMU: real independent angular rate improves turns, but rate bias integrates into yaw and no absolute XY measurement exists. Existing IMU path is rate-only, with no simulated source. Implementing bias/orientation instrumentation alone does not meet the stated observability objective.
C. Wheel + independent planar pose: chosen. An explicit external-localization simulation abstraction represents intermittently available mapped-fiducial/external-tracker localization, not implemented VIO/SLAM. Gazebo truth is consumed only by robot_simulation's sensor node. The EKF consumes stamped imperfect Odometry pose/covariance through a normal replaceable sensor topic; it never subscribes to Gazebo truth and the controller consumes its unchanged ordinary estimate.

## Frozen sensor and filter assumptions

External localization: 10 Hz acquisition from configured stamped physical base source; independent Gaussian XY white noise sigma 0.04 m each and yaw sigma 2 degrees. Per-run calibration bias sigma 0.01 m and 0.5 degrees, retained across measurements. Reported R includes white and calibration-bias marginal variance, but repeated bias is temporally correlated, a documented approximation requiring consistency evaluation. Nominal latency 0.04 s plus uniform [0,0.01] s jitter; 5% independent dropout; configurable availability intervals, outlier probability/offset and RNG seed. None is presented as actual hardware performance certification. Output source stamp is acquisition time, never release/receipt time. Pose-only Odometry has source fixed frame and explicit tracked child frame. Unsupported roll/pitch frame transforms are rejected; yaw rotation/translation and nonzero spawn offsets are handled, including covariance cross terms.

EKF state unchanged: [x,y,yaw,v,omega,q,dq]. Independent pose h(x)=[x,y,yaw], H selects the first three state components. Heading innovation wraps to (-pi,pi]; the update uses an equivalent observation around predicted yaw, preserving continuous state and the existing Joseph covariance update. NIS gate 11.345 for 3 DOF (~99%) rejects outliers before filter mutation. Transport time rejection is separate from statistical rejection. The observation enters the existing common source-time reorder queue; the localized profile uses 0.08 s holdback, below the unchanged 0.2 s robot/controller component age budget. Older-than-committed measurements remain rejected, not rewound or relabeled.

The localized profile increases continuous linear/angular acceleration-noise amplitudes from the ideal-wheel values to 0.3 each (m or rad per s^(3/2)), reflecting unmodeled contact/slip changes rather than claiming wheel motion exact. Joint Q, all physical/safety/controller/target/predictor parameters remain unchanged. The new sensor-disabled ablation retains this profile, exposing its fallback honestly. Pose absence does not invent corrections: wheel/joint estimation continues, covariance evolves, diagnostics report localization degradation. Long dropout and subsequent outlier-gated reacquisition may remain limitations.

## Ownership and implementation boundaries

- robot_estimation: pure planar measurement/covariance transform + wrapped innovation/gate, optional typed ROS parameters, TF adapter, existing queue integration and source-health diagnostics. No Gazebo imports.
- robot_simulation: generic noisy/delayed/dropout pose sensor mathematics, thin ROS sensor node and explicit optional sensor config in simulation deployment; one measurement publisher.
- robot_bringup: additive localized D composition using YAML inheritance; preflight sensor and estimator contracts. Existing D remains the baseline.
- robot_experiments: record/materialize the optional measurement source and its seeded configuration; use existing recorder/lifecycle/full-step checks and read-only tracking diagnostics.

## Numerical acceptance frozen before final comparisons

D42 and D917, >=90 sim seconds each. Use historical baseline only after unchanged provenance checks; fresh original and localized-with-sensor-disabled ablations where needed. Reactive paths differ; these are policy comparisons under matched initial/exogenous conditions, not identical trajectories.

Localization (20–90 source seconds): XY MAE <=0.20 m, absolute yaw MAE <=10 degrees; no sustained >0.5 m or >30 degree divergence, no unexplained covariance collapse or discrete pose jumps; inspect NEES and 95% marginal coverage rather than asserting covariance correctness from PSD alone.
Tracking (0–90 valid observed source time): distance MAE and pointing MAE lower than matched-seed baseline, higher joint compliance at |range error|<=0.20 m and pointing<=15 degrees. Preserve valid-control sample fraction >=98%, no sustained invalid interval >=1 s, native physical safety and recovery release. Missing evidence is not a pass. End-to-end tracking may remain CONDITIONAL because instantaneous task-rate infeasibility is independently demonstrated.
Existing native acceptance remains 0.001 rad and 0.001 rad/s; position, speed and effort limits are unchanged. No torque-measurement claim. Serious-fault injection remains an independent prior gap.

## Plan / review focus

1. Red/green tests: transform/spawn/yaw wrap, covariance SPD, gating and no mutation, sensor reproducibility/bias/delay/dropout/reset, typed disabled behavior, late/duplicate/reset queue integration.
2. Implement the smallest reusable sensor and EKF update using the existing pipeline; validate configuration and enable only through explicit localized profile.
3. Build and run focused + architecture checks; short normal-launch smoke before longer runs.
4. Preserve every D42/D917 original/improved/disabled run; record sensor errors, arrivals, covariance, ordinary-controller input and native traces. Add brief degraded-source/reset checks where needed.
5. Full functional regression, copied-install origin/sensor launch, native binary/source preservation and final report/file inventory.

No user permission checkpoint is added: the task explicitly authorizes implementation and validation, with the user's standing instruction to proceed autonomously. No separate estimator/control pipeline, ground-truth remap, controller tuning or target repair is included.

Pre-comparison smoke finding (retained under results/m03_localization_smoke): acceleration-only Q still rejected 43/82 otherwise valid pose samples by 10.4 s; last acceptance was 6.15 s. Heading error reached ~0.40 rad while predicted sigma was ~0.05 rad. Wheel R inflation alone does not represent sustained contact-slip pose error. Before final comparisons, add optional zero-default planar/yaw pose diffusion to the same process model, amplitudes 0.15 m/sqrt(s), 0.20 rad/sqrt(s), chosen conservatively from the prior ~0.129 m and ~0.189 rad one-second wheel/body velocity discrepancy scale. This represents unresolved velocity bias/lateral contact motion over a one-second correlation horizon, not extra sensor precision. Frozen continuous Qc has independent XY density .15^2 and yaw density .20^2; exact frozen-Jacobian integration includes yaw/XY cross terms. No mean, measurement gate or acceptance threshold changes. Original configs have zero diffusion and identical behavior. Disabled ablation retains these same densities. This engineering stochastic approximation must still be checked against actual pose error/covariance; it is not an identified wheel-slip dynamics model.

Additional source-audit finding before implementation: the deployed calibration overrides wheel velocity variances to 1e-6, describing nearly noiseless encoders. Encoder precision is not chassis-velocity accuracy under slip. The historical wheel-versus-body audit measured forward/yaw rate RMSE about 0.129 m/s and 0.189 rad/s. The localized calibration therefore uses chassis-motion R=diag(0.02,0.04), conservative rounded squares of those independent observed discrepancies, while retaining exact joint and target calibration. This change is owned by the existing deployment-calibration override, not embedded in estimator code. The sensor-disabled ablation retains the same R/Q/holdback; global pose remains unobservable without the independent source.
