"""ROS 2 orchestration for validated, compute-only robot control.

Receive estimates and predictions, validate inputs, and optionally
compute safety-constrained tracking commands.

No actuator commands are published by this node.
"""

from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from robot_interfaces.msg import (
    PredictedTrajectory as PredictedTrajectoryMsg,
)
from robot_interfaces.msg import StateEstimate as StateEstimateMsg
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener

from robot_control.adapters.control_config_adapter import (
    ControlConfigAdapter,
)
from robot_control.adapters.prediction_message_adapter import (
    PredictionMessageAdapter,
)
from robot_control.adapters.robot_state_message_adapter import (
    RobotStateMessageAdapter,
)
from robot_control.factory.control_factory import ControlFactory
from robot_control.factory.project_components import (
    create_project_registries,
)
from robot_control.factory.project_configuration import (
    project_control_configuration,
)
from robot_control.interfaces.control_data import (
    PredictedTargetTrajectory,
    RobotState,
)
from robot_control.pipeline.control_execution_pipeline import (
    ControlExecutionPipeline,
)
from robot_control.safety.input_status import InputStatusTracker
from robot_control.safety.input_validator import (
    InputValidationConfig,
    InputValidator,
)


class ControlNode(Node):
    """Validate ROS inputs and optionally compute safe tracking commands."""

    _REQUIRED_PARAMETERS = {
        "joint_count": Parameter.Type.INTEGER,
        "robot_state_topic": Parameter.Type.STRING,
        "prediction_topic": Parameter.Type.STRING,
        "max_robot_state_age": Parameter.Type.DOUBLE,
        "max_prediction_age": Parameter.Type.DOUBLE,
        "future_tolerance": Parameter.Type.DOUBLE,
        "validation_rate": Parameter.Type.DOUBLE,
    }

    _CONTROL_PARAMETERS = {
        "base_model_type": Parameter.Type.STRING,
        "manipulator_model_type": Parameter.Type.STRING,
        "kinematics_adapter_type": Parameter.Type.STRING,
        "controller_type": Parameter.Type.STRING,
        "controller_damping": Parameter.Type.DOUBLE,
        "controller_task_axes": Parameter.Type.INTEGER_ARRAY,
        "controller_task_gains": Parameter.Type.DOUBLE_ARRAY,
        "reference_geometry_type": Parameter.Type.STRING,
        "reference_policy_type": Parameter.Type.STRING,
        "tracking_radius": Parameter.Type.DOUBLE,
        "tracking_lookahead_time": Parameter.Type.DOUBLE,
        "tracking_forward_axis": Parameter.Type.DOUBLE_ARRAY,
        "tracking_up_axis": Parameter.Type.DOUBLE_ARRAY,
        "world_up_axis": Parameter.Type.DOUBLE_ARRAY,
        "base_velocity_lower": Parameter.Type.DOUBLE_ARRAY,
        "base_velocity_upper": Parameter.Type.DOUBLE_ARRAY,
    }

    def __init__(self) -> None:
        """Initialize ROS inputs and optional compute-only pipeline."""
        super().__init__("control_node")

        self._declare_parameters()

        self._compute_enabled = bool(
            self.get_parameter("compute_enabled").value
        )

        if self.get_parameter("publish_commands").value:
            raise ValueError(
                "Command publishing is not supported by ControlNode"
            )

        joint_count = int(
            self.get_parameter("joint_count").value
        )

        self._robot_adapter = RobotStateMessageAdapter(
            joint_count=joint_count,
        )
        self._prediction_adapter = PredictionMessageAdapter()

        self._validator = InputValidator(
            InputValidationConfig(
                max_robot_state_age=float(
                    self.get_parameter("max_robot_state_age").value
                ),
                max_prediction_age=float(
                    self.get_parameter("max_prediction_age").value
                ),
                future_tolerance=float(
                    self.get_parameter("future_tolerance").value
                ),
                lookahead_time=float(
                    self.get_parameter("tracking_lookahead_time").value or 0.0
                ),
            )
        )

        self._status_tracker = InputStatusTracker()

        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(
            self._tf_buffer, self
        )

        self._robot_state: Optional[RobotState] = None
        self._prediction: Optional[PredictedTargetTrajectory] = None
        self._prediction_reference_timestamp: Optional[float] = None
        self._last_robot_timestamp: Optional[float] = None
        self._last_prediction_timestamp: Optional[float] = None

        self._pipeline: Optional[ControlExecutionPipeline] = None
        self._last_compute_time: Optional[float] = None
        self._last_output_log_time: Optional[float] = None

        self._clock_jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None,
                min_backward=Duration(nanoseconds=-1),
                on_clock_change=True,
            ),
            post_callback=self._on_time_jump,
        )

        validation_rate = float(
            self.get_parameter("validation_rate").value
        )

        if not np.isfinite(validation_rate) or validation_rate <= 0.0:
            raise ValueError(
                "validation_rate must be finite and positive"
            )

        self._nominal_dt = 1.0 / validation_rate

        if self._compute_enabled:
            self._pipeline = self._build_pipeline(joint_count)

        self.create_subscription(
            StateEstimateMsg,
            str(self.get_parameter("robot_state_topic").value),
            self._robot_callback,
            10,
        )

        self.create_subscription(
            PredictedTrajectoryMsg,
            str(self.get_parameter("prediction_topic").value),
            self._prediction_callback,
            10,
        )

        self._timer = self.create_timer(
            self._nominal_dt,
            self._validate_inputs,
        )

        mode = (
            "COMPUTE ONLY"
            if self._compute_enabled
            else "INPUT VALIDATION ONLY"
        )

        self.get_logger().info(
            f"ControlNode initialized: {mode}. "
            "Actuator command publishing disabled."
        )

    def _declare_parameters(self) -> None:
        """Declare required, control, and execution parameters."""
        for name, parameter_type in self._REQUIRED_PARAMETERS.items():
            self.declare_parameter(name, parameter_type)

        self.declare_parameter("compute_enabled", False)
        self.declare_parameter("publish_commands", False)
        self.declare_parameter("safety_enabled", True)

        for name, parameter_type in self._CONTROL_PARAMETERS.items():
            self.declare_parameter(name, parameter_type)

        missing = [
            name
            for name in self._REQUIRED_PARAMETERS
            if self.get_parameter(name).value is None
        ]

        if missing:
            raise ValueError(
                "Missing required control parameters: "
                + ", ".join(missing)
            )

    def _build_pipeline(
        self,
        joint_count: int,
    ) -> ControlExecutionPipeline:
        """Construct real-model tracking with physical safety limits."""
        if not self.get_parameter("safety_enabled").value:
            raise ValueError(
                "Compute-only mode requires safety_enabled=true"
            )

        parameters = {
            name: self.get_parameter(name).value
            for name in (
                *self._REQUIRED_PARAMETERS,
                *self._CONTROL_PARAMETERS,
            )
        }

        base_lower = parameters["base_velocity_lower"]
        base_upper = parameters["base_velocity_upper"]

        if (
            base_lower is None
            or base_upper is None
            or len(base_lower) != 2
            or len(base_upper) != 2
        ):
            raise ValueError(
                "Compute-only mode requires explicit "
                "base_velocity_lower and base_velocity_upper "
                "arrays in [v, omega] order"
            )

        description_path = (
            Path(get_package_share_directory("robot_description"))
            / "config"
            / "robot"
            / "robot.yaml"
        )

        with description_path.open(encoding="utf-8") as stream:
            description = yaml.safe_load(stream)

        robot_config, safety_config, transform_et = (
            project_control_configuration(
                description=description,
                base_velocity_lower=base_lower,
                base_velocity_upper=base_upper,
            )
        )

        description_joint_count = len(
            description["robot"]["manipulator"]["joints"]
        )

        if description_joint_count != joint_count:
            raise ValueError(
                "ROS joint_count does not match robot description"
            )

        factory_config = ControlConfigAdapter.convert(
            ros_parameters=parameters,
            robot_configuration=robot_config,
            safety_configuration=safety_config,
        )

        factory_config["tracking"]["frame"] = {
            "transform_end_effector_tracking": (
                transform_et.tolist()
            ),
        }

        factory = ControlFactory(
            registries=create_project_registries(),
        )

        return factory.build(factory_config)

    def _prediction_in_robot_frame(
        self,
        prediction: PredictedTargetTrajectory,
        robot_frame: str,
        reference_timestamp: float,
    ) -> PredictedTargetTrajectory:
        """Transform predicted [position, velocity] into robot frame.

        Assumes state layout [x, y, z, vx, vy, vz].
        Resolve TF at the prediction reference time, never at a future
        prediction sample or an unrelated latest time. The frames must
        have a fixed inertial relationship: moving-frame velocity terms
        are not represented by this rigid coordinate conversion.
        """
        if prediction.frame_id == robot_frame:
            return prediction

        transform = self._tf_buffer.lookup_transform(
            robot_frame,
            prediction.frame_id,
            Time(seconds=reference_timestamp),
        )

        translation = transform.transform.translation
        quaternion = transform.transform.rotation

        rotation = Rotation.from_quat([
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ]).as_matrix()

        offset = np.array([
            translation.x,
            translation.y,
            translation.z,
        ], dtype=float)

        states = prediction.states.copy()

        if states.shape[1] != 6:
            raise ValueError(
                "Frame conversion requires target state "
                "[x, y, z, vx, vy, vz]"
            )

        states[:, :3] = states[:, :3] @ rotation.T + offset
        states[:, 3:6] = states[:, 3:6] @ rotation.T

        return PredictedTargetTrajectory(
            states=states,
            timestamps=prediction.timestamps,
            frame_id=robot_frame,
        )

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        """Convert a ROS timestamp to seconds."""
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _stable_reason(error: ValueError) -> str:
        """Remove time-dependent details from freshness errors."""
        message = str(error)

        if message.startswith("Robot state is stale"):
            return "Robot state is stale"

        if message.startswith("Target prediction is stale"):
            return "Target prediction is stale"

        return message

    def _set_invalid(self, reason: str) -> None:
        """Report input validity transitions."""
        if self._status_tracker.set_invalid(reason):
            now = self.get_clock().now().nanoseconds * 1e-9
            robot_time = (
                self._robot_state.timestamp if self._robot_state is not None else None
            )
            self.get_logger().warning(
                f"Control inputs unavailable: {reason} | now={now:.6f} | "
                f"robot_t={robot_time} | "
                f"prediction_t={self._prediction_reference_timestamp}"
            )

    def _set_valid(self) -> None:
        """Report when inputs become valid."""
        if self._status_tracker.set_valid():
            self.get_logger().info(
                "Control inputs VALID: "
                "robot state and prediction are ready."
            )

    def _on_time_jump(self, _jump) -> None:
        """Invalidate cached data when ROS time changes epoch."""
        self._robot_state = None
        self._prediction = None
        self._prediction_reference_timestamp = None
        self._last_robot_timestamp = None
        self._last_prediction_timestamp = None
        self._last_compute_time = None
        self._last_output_log_time = None
        self._status_tracker.set_waiting('Waiting for inputs after clock reset')
        self.get_logger().warning('ROS clock changed; cleared control inputs.')

    def _check_reference_time(self, timestamp: float, name: str) -> None:
        """Reject future messages before updating ordering caches."""
        now = self.get_clock().now().nanoseconds * 1e-9
        tolerance = float(self.get_parameter('future_tolerance').value)
        if timestamp > now + tolerance:
            raise ValueError(
                f'{name} timestamp is in the future: '
                f'stamp={timestamp:.6f}, now={now:.6f}'
            )

    def _robot_callback(self, message: StateEstimateMsg) -> None:
        """Accept same-time fused updates; reject older estimates."""
        try:
            state = self._robot_adapter.convert(message)
            self._check_reference_time(state.timestamp, 'Robot state')

            if (
                self._last_robot_timestamp is not None
                and state.timestamp < self._last_robot_timestamp
            ):
                self.get_logger().warning(
                    "Ignoring out-of-order robot estimate: "
                    f"stamp={state.timestamp:.6f}, "
                    f"last={self._last_robot_timestamp:.6f}",
                    throttle_duration_sec=5.0,
                )
                return

            self._robot_state = state
            self._last_robot_timestamp = state.timestamp

        except (TypeError, ValueError) as error:
            self.get_logger().error(
                f"Invalid robot state estimate: {error}",
                throttle_duration_sec=5.0,
            )

    def _prediction_callback(
        self,
        message: PredictedTrajectoryMsg,
    ) -> None:
        """Accept a prediction with increasing reference timestamps."""
        try:
            prediction = self._prediction_adapter.convert(message)

            reference_timestamp = self._stamp_to_seconds(
                message.header.stamp
            )
            self._check_reference_time(reference_timestamp, 'Target prediction')

            if (
                self._last_prediction_timestamp is not None
                and reference_timestamp <= self._last_prediction_timestamp
            ):
                self.get_logger().warning(
                    "Ignoring duplicate or out-of-order prediction.",
                    throttle_duration_sec=5.0,
                )
                return

            self._prediction = prediction
            self._prediction_reference_timestamp = reference_timestamp
            self._last_prediction_timestamp = reference_timestamp

        except (TypeError, ValueError) as error:
            self.get_logger().error(
                f"Invalid target prediction: {error}",
                throttle_duration_sec=5.0,
            )

    def _validate_inputs(self) -> None:
        """Validate inputs and optionally compute constrained control."""
        missing = []

        if self._robot_state is None:
            missing.append("robot state")

        if (
            self._prediction is None
            or self._prediction_reference_timestamp is None
        ):
            missing.append("target prediction")

        if missing:
            reason = "Waiting for " + ", ".join(missing)

            if self._status_tracker.set_waiting(reason):
                self.get_logger().info(reason)

            self._last_compute_time = None
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        try:
            prediction = self._prediction_in_robot_frame(
                self._prediction,
                self._robot_state.frame_id,
                self._prediction_reference_timestamp,
            )
            self._validator.validate(
                robot_state=self._robot_state,
                prediction=prediction,
                prediction_reference_timestamp=(
                    self._prediction_reference_timestamp
                ),
                now=now,
            )

        except (TypeError, ValueError, TransformException) as error:
            self._set_invalid(self._stable_reason(error))
            self._last_compute_time = None
            return

        self._set_valid()

        if not self._compute_enabled:
            return

        self._compute_control(now, prediction)

    def _compute_control(
        self,
        now: float,
        prediction: PredictedTargetTrajectory,
    ) -> None:
        """Compute and report safe commands without publishing them."""
        if self._pipeline is None:
            return

        # Joint-position constraints need a forward time interval.
        # Use the nominal period for the first successful cycle.
        dt = self._nominal_dt

        if self._last_compute_time is not None:
            elapsed = now - self._last_compute_time
            if elapsed <= 0.0:
                return
            if np.isfinite(elapsed):
                dt = max(elapsed, self._nominal_dt)

        try:
            result = self._pipeline.compute(
                robot_state=self._robot_state,
                prediction=prediction,
                now=now,
                dt=dt,
            )

        except (TypeError, ValueError, np.linalg.LinAlgError) as error:
            self._last_compute_time = None
            self.get_logger().warning(
                f"Control computation unavailable: {error}",
                throttle_duration_sec=5.0,
            )
            return

        self._last_compute_time = now

        if (
            self._last_output_log_time is not None
            and now >= self._last_output_log_time
            and now - self._last_output_log_time < 1.0
        ):
            return

        self._last_output_log_time = now

        raw = np.array2string(
            result.raw_output.command,
            precision=4,
            separator=", ",
        )
        safe = np.array2string(
            result.safe_output.command,
            precision=4,
            separator=", ",
        )

        self.get_logger().info(
            f"CONTROL COMPUTE ONLY | raw={raw} | safe={safe}"
        )


def main(args=None) -> None:
    """Start the ROS 2 control node."""
    rclpy.init(args=args)
    node = None

    try:
        node = ControlNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
