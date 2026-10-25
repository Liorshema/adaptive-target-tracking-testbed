# Finding and configuring sensors

Start here to identify sensing paths and effective configuration. For the rationale and deferred changes, see the [architecture review](sensor_architecture_review.md) and [proposed design](sensor_architecture_proposed_design.md). Phase 1 adds discoverability only: models, conversion callbacks, physics and safety behavior remain unchanged.

## Architecture overview

```text
Physical / simulated source      robot_simulation (or external publisher)
          ↓
ROS message / transport          producer node / ros_gz_bridge
          ↓
Measurement conversion          robot_estimation/nodes/estimation_node.py
          ↓
Observation h(x), H(x), R        robot_estimation pure models
          ↓
Estimator / consumer             robot EKF or target KF
```

`robot_bringup` composes and preflights owner configurations; `robot_configuration` loads inheritance/resources; `robot_experiments` snapshots the resolved system and records runs. Simulator physical truth also feeds explicit imperfect sensor generation and target-dynamics context. Tracking TF is derived from base feedback and measured joints. These context/truth paths are not independent measurements fused directly into the robot EKF.

## Sensor directory

Paths below link to current owners. Status describes built-in wiring, **not live availability**; inspect the selected manifest's computed inventory for configured selection.

| Source | Implementation | Configuration | Consumer | Status |
|---|---|---|---|---|
| Wheel odometry | [Gazebo DiffDrive builder](../../src/robot_simulation/robot_simulation/gazebo/robot_builder.py), [ROS bridge](../../src/robot_simulation/launch/simulation.launch.py) | [Robot deployment](../../src/robot_simulation/config/gazebo/robot.yaml): `robot.differential_drive`; estimator wheel topic/frame/variance/age | [Estimation callbacks](../../src/robot_estimation/robot_estimation/nodes/estimation_node.py), [wheel h/H](../../src/robot_estimation/robot_estimation/robot/models/measurement_model.py), actuator feedback; optionally tracking FK | Built-in; fused when received. Mandatory wheel feedback for present full-state/active-actuation contracts. |
| Joint feedback | Same builder/bridge: `JointStatePublisher` | Deployment `robot.joint_states`; [description](../../src/robot_description/config/robot/robot.yaml) names/limits; estimator joint assumptions | Same estimator callbacks and joint h/H; raw-bound gate, actuator, tracking FK | Built-in; not an independent noisy encoder simulator. Required safety feedback. |
| IMU yaw rate | Estimation node `_imu_callback`; `ImuYawRateMeasurementModel` in the model file above | [Robot estimator](../../src/robot_estimation/config/robot_estimator.yaml): `imu_topic`, `imu_frame`, `imu_yaw_rate_variance` | Robot EKF scalar yaw-rate observation | Subscriber exists; **no built-in producer**. Compatible external messages may be fused. A schematic camera is not an IMU or image source. |
| External planar localization | [Pure generator](../../src/robot_simulation/robot_simulation/measurements/planar_localization.py), [ROS publisher](../../src/robot_simulation/robot_simulation/nodes/localization_sensor.py) | [Simulated source](../../src/robot_simulation/config/sensors/planar_localization.yaml); [fusion selection](../../src/robot_estimation/config/robot_localized.yaml) | Estimation pose callback → [pose transform/model/trust](../../src/robot_estimation/robot_estimation/robot/planar_pose.py) → [robot fusion](../../src/robot_estimation/robot_estimation/robot/robot_state_estimator.py) | Optional built-in simulated publisher; fusion selected independently. Not real SLAM/VIO. |
| Synthetic target position | [Simulation node](../../src/robot_simulation/robot_simulation/nodes/simulation_node.py), [Gaussian generator](../../src/robot_simulation/robot_simulation/measurements/synthetic_measurement.py) | [Target measurements](../../src/robot_simulation/config/measurements.yaml); [target estimator](../../src/robot_estimation/config/target_estimator.yaml) | Target callback → [position H](../../src/robot_estimation/robot_estimation/target/models/measurement_model.py) → target KF → prediction/control | Selected by current M07 deployment contract; PointStamped has no covariance field. |
| Physical base context | Builder `OdometryPublisher`/bridge → simulation `_on_physical_base`; [causal history](../../src/robot_simulation/robot_simulation/state/physical_context.py) | Deployment `robot.ground_truth_odometry`; simulation `physical_context`; M07 `runtime.publish_ground_truth` | Contextual target dynamics, optional imperfect localization generator, physical tracking FK and diagnostics | Physical truth, not direct EKF/control input. Contextual consumption depends on the selected environment. |
| Tracking-frame TF | [Measured-feedback FK broadcaster](../../src/robot_bringup/robot_bringup/nodes/tracking_feedback_tf.py), [TF provider/history](../../src/robot_simulation/robot_simulation/gazebo/tracking_frame_provider.py) | Bringup `tracking_tf`; simulation `tracking`; [authoritative mount](../../src/robot_description/urdf/tracking_frame.xacro) | Reactive target tracking/context | Derived FK context; wheel-derived or physical base pose according to selection. Static tracking uses no broadcaster. |

The simulation clock is timing/epoch infrastructure. Physical base truth and committed target truth are evaluation/context sources; they are not additional robot sensors. Target truth is published only after Gazebo acknowledges the numerical target pose. Physical tracking/visualization truth must remain distinct from estimated robot state.

## Where to change things

| Question | Owner / entry |
|---|---|
| Localization noise or bias? | Inherit `robot_simulation/config/sensors/planar_localization.yaml`; change `white_noise_std` or `calibration_bias_std`. Select the child through the deployment's `localization_sensor_config`. |
| Dropout, outliers, outages or latency? | Same source owner: `dropout_probability`, `outages`, `outlier_probability`, `outlier_offset`, `latency`, `latency_jitter`. Respect source delivery age and the selected estimator's reorder/admission budgets. |
| Enable/disable simulated pose publisher? | Source owner's `localization_sensor.enabled`. Disabled publisher does not disable estimator fusion. |
| Select localization fusion? | Inherited estimator owner: `estimation_node.ros__parameters.localization_enabled`; select using manifest `estimation.robot.config`. Fusion off can leave the publisher on for a fusion ablation. Fusion on without a producer requires an external provider or remains unavailable under existing trust/freshness gates. |
| Change estimator measurement assumptions? | Robot estimator's wheel/joint/IMU variances; target estimator's `target_position_variance`. Pose fusion uses the transported selected covariance. These are observation R settings, not process Q settings. |
| Select a system? | [M07 manifest](../../src/robot_bringup/config/system_nominal.yaml), optionally inherited like [localized manipulation](../../src/robot_bringup/config/system_manipulation_D_localized.yaml); `closed_loop.launch.py system_config:=...`. M08 variants select the same manifests through `system_config`. |
| Why does the effective value differ from an estimator YAML? | The selected bringup profile may apply `estimation_overrides` last. Inventory `configuration.parameter_sources` identifies the selected owner or final override resource; it does not trace every inherited key to its declaring ancestor. |
| Mandatory actuator feedback? | [Actuation YAML](../../src/robot_simulation/config/actuation.yaml), [actuation factory](../../src/robot_simulation/robot_simulation/factory/actuation_factory.py), [actuation node](../../src/robot_simulation/robot_simulation/nodes/velocity_actuation.py); full-state freshness/raw-joint gates remain in estimation. Limits remain in robot description. Disabling fusion is not permission to remove physical safety feedback. |
| Inspect final effective values? | `load_system_configuration(...)["sensor_inventory"]`; concise summary at normal M07 startup; M08 `resolved_configuration.json` → `system.sensor_inventory`. |

Resource selection stays in the existing chain: M07 manifest → bringup profile → simulation deployment → sensor owner, with estimator owners selected separately. Use existing `extends` and package resources. There is no new editable sensor catalog or top-level `sensors:` YAML field.

### Inspect a plan without starting ROS/Gazebo

After building and sourcing the workspace:

```bash
source /home/user/ros2_ws/install/setup.bash
python3 - <<'PY'
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from robot_bringup.factory.system_configuration import load_system_configuration
share = lambda package: Path(get_package_share_directory(package))
plan = load_system_configuration(
    share('robot_bringup') / 'config/system_manipulation_D_localized.yaml', share)
print(json.dumps(plan['sensor_inventory'], indent=2))
PY
```

[Inventory construction and presentation](../../src/robot_bringup/robot_bringup/factory/sensor_inventory.py) are pure metadata functions, called by the existing [M07 assembler](../../src/robot_bringup/robot_bringup/factory/system_configuration.py). The builder reports configured producer/subscription/fusion separately and sets live status to `not observed`. Publication rates are configured schedules, not measured delivery rates; missing settings stay explicit. M08 recomputes the same inventory from materialized run inputs, so seed/resource values describe that run rather than the original template.

Normal Compute Only launch:

```bash
ros2 launch robot_bringup closed_loop.launch.py \
  system_config:=config/system_nominal.yaml \
  enable_actuation:=false publish_commands:=false gui:=false
```

The legacy profile-only `config:=...` workflow is preserved; the summary belongs to the M07 manifest path. Detailed inventory is configuration evidence, not proof that messages arrived or were accepted. M08 comparisons remain strict by default; intentional sensor-effect differences require its [explicit scientific comparison policy](../m08_experiments_and_benchmarking.md#explicit-sensor-experiments).

## Adding a new sensor

1. Specify fields/dimension, units, covariance, frames/extrinsics, acquisition time, delay, reset epoch and source independence/correlation. Decide whether feedback is optional or physically required.
2. Add a producer only if needed. Keep synthetic imperfections in simulation `measurements/`, ROS delivery in `nodes/`, and Gazebo plugins/bridge in their existing integration owners. An equivalent external publisher may need no generator or new observation model.
3. Add ROS conversion/subscription at the current estimation node boundary. Preserve source stamps/R and frame validation. A new robot observation must join the bounded shared robot chronology, not bypass it.
4. Add pure h/H (and circular innovation where needed), estimator update/factory wiring, typed parameters and owner YAML. A changed state dimension is a separate estimator extension.
5. Update M07 preflight and inventory explicitly for the new actual topology. No registry or class-name loading is implied. Extend M08 recording/readiness only if the experiment needs the new stream; IMU and TF are not currently recorder streams.
6. Test dimensions, covariance, frames, malformed/future/stale/reordered input, clock reset, source absence, safety gates, inheritance/resources and installed integration. A sensor cannot always be added through YAML alone.

## Existing compatibility interfaces

[SensorInterface](../../src/robot_estimation/robot_estimation/interfaces/sensor_interface.py) and [wheel](../../src/robot_estimation/robot_estimation/robot/sensors/wheel_odometry.py), [joint](../../src/robot_estimation/robot_estimation/robot/sensors/joint_encoder.py), [IMU](../../src/robot_estimation/robot_estimation/robot/sensors/imu.py) wrappers accept **raw arrays** and are tested compatibility APIs. Production ROS callbacks currently construct `Measurement` directly; they do not call these wrappers. In particular, the wrapper IMU defaults to six acceleration/rate values, while production observes only yaw rate. Their public contracts are preserved. This phase neither extracts callbacks nor removes/redesigns these APIs.
