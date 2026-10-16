from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from robot_interfaces.msg import (
    PredictedTrajectory as PredictedTrajectoryMsg,
    StateEstimate as StateEstimateMsg,
)

from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.interfaces.prediction_data import (
    PredictionInput,
    TrajectoryPrediction,
)
from robot_prediction.interfaces.state_layout import StateLayout
from robot_prediction.factory.system_factory import PredictionSystemFactory
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


class PredictionNode(Node):
    """Run target trajectory prediction from estimated target states."""

    def __init__(
        self,
    ):
        super().__init__(
            'prediction_node'
        )

        config_path = (
            Path(
                get_package_share_directory(
                    'robot_prediction'
                )
            )
            / 'config'
            / 'prediction.yaml'
        )

        self._config = PredictionConfig(
            config_path
        )

        prediction_config = self._config.get(
            'prediction'
        )

        self._dt = float(
            prediction_config['dt']
        )

        self._horizon = int(
            prediction_config['horizon']
        )

        self._system = (
            PredictionSystemFactory(
                config=self._config
            ).create()
        )

        self._last_timestamp = None
        self._state_dimension = None
        self._frame_id = None

        self.declare_parameter('future_tolerance', 0.01)
        tolerance = float(self.get_parameter('future_tolerance').value)
        if not np.isfinite(tolerance) or tolerance < 0.0:
            raise ValueError('future_tolerance must be finite and nonnegative.')
        self._clock_jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None,
                min_backward=Duration(nanoseconds=-1),
                on_clock_change=True,
            ),
            post_callback=self._on_time_jump,
        )

        self.declare_parameter(
            'estimated_state_topic',
            '/estimation/target_state_full',
        )
        estimated_state_topic = str(
            self.get_parameter('estimated_state_topic').value
        )

        self._state_subscription = (
            self.create_subscription(
                StateEstimateMsg,
                estimated_state_topic,
                self._state_callback,
                10,
            )
        )

        self._trajectory_publisher = (
            self.create_publisher(
                Float64MultiArray,
                'predicted_trajectory',
                10,
            )
        )

        self.declare_parameter(
            "trajectory_full_topic",
            "/prediction/trajectory",
        )
        self._trajectory_full_publisher = self.create_publisher(
            PredictedTrajectoryMsg,
            str(self.get_parameter("trajectory_full_topic").value),
            10,
        )

        self._confidence_publisher = (
            self.create_publisher(
                Float64MultiArray,
                'prediction_confidence',
                10,
            )
        )

        self.get_logger().info(
            'Prediction node started.'
        )

    def _on_time_jump(self, _jump) -> None:
        """Discard prediction/adaptation history when ROS time changes epoch."""
        self._system = PredictionSystemFactory(config=self._config).create()
        self._last_timestamp = None
        self._state_dimension = None
        self._frame_id = None
        self.get_logger().warning('ROS clock changed; reset prediction history.')

    def _state_callback(
        self,
        message: StateEstimateMsg,
    ) -> None:
        """Predict from a stamped state estimate and its full covariance."""
        state = np.asarray(message.mean, dtype=float)
        timestamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )
        frame_id = message.header.frame_id

        try:
            layout = StateLayout.from_state_dimension(state.size)
            covariance = np.asarray(message.covariance, dtype=float)
            if covariance.size != state.size ** 2:
                raise ValueError(
                    'Invalid covariance length: expected '
                    f'{state.size ** 2}, received {covariance.size}.'
                )
            covariance = covariance.reshape(state.size, state.size)

            if not np.isfinite(timestamp) or not np.all(np.isfinite(state)):
                raise ValueError('State and timestamp must be finite.')
            if not np.all(np.isfinite(covariance)):
                raise ValueError('Covariance must contain finite values.')
            if not np.allclose(covariance, covariance.T, atol=1e-8):
                raise ValueError('Covariance must be symmetric.')
            if not frame_id:
                raise ValueError('Estimated state frame_id is empty.')
            now = self.get_clock().now().nanoseconds * 1e-9
            tolerance = float(self.get_parameter('future_tolerance').value)
            if timestamp > now + tolerance:
                raise ValueError(
                    f'Target estimate timestamp is in the future: '
                    f'stamp={timestamp:.6f}, now={now:.6f}'
                )
            if self._last_timestamp is not None and timestamp <= self._last_timestamp:
                self.get_logger().warning(
                    'Ignoring out-of-order or duplicate target estimate.',
                    throttle_duration_sec=5.0,
                )
                return
            if self._frame_id is not None and frame_id != self._frame_id:
                raise ValueError(
                    'Target reference frame changed without a transform.'
                )

            self._validate_state_dimension(layout.state_dimension)

            if self._system.last_prediction is not None:
                observation_dt = timestamp - self._last_timestamp
                performance = self._system.update(
                    observed_state=state,
                    observation_dt=observation_dt,
                    observation_timestamp=timestamp,
                )
                if performance is not None:
                    self._publish_confidence(
                        confidence=performance.confidence,
                        residual=performance.residual,
                    )

            prediction_input = PredictionInput(
                state=state,
                timestamp=timestamp,
                dt=self._dt,
                horizon=self._horizon,
                covariance=covariance,
            )
            prediction = self._system.predict(prediction_input)
            self._publish_full_trajectory(
                prediction=prediction,
                reference_timestamp=timestamp,
                frame_id=frame_id,
            )
            self._publish_trajectory(prediction.states)

            self._last_timestamp = timestamp
            self._frame_id = frame_id

        except (RuntimeError, ValueError) as error:
            self.get_logger().error(f'Prediction failed: {error}', throttle_duration_sec=5.0)

    def _validate_state_dimension(
        self,
        state_dimension: int,
    ) -> None:
        """Ensure the input state dimension remains consistent."""
        if self._state_dimension is None:
            self._state_dimension = (
                state_dimension
            )

            spatial_dimension = (
                state_dimension // 2
            )

            self.get_logger().info(
                'Detected target state layout: '
                f'spatial_dimension={spatial_dimension}, '
                f'state_dimension={state_dimension}.'
            )

            return

        if state_dimension != self._state_dimension:
            raise ValueError(
                'target state dimension changed during execution.'
            )

    def _publish_full_trajectory(
        self,
        prediction: TrajectoryPrediction,
        reference_timestamp: float,
        frame_id: str,
    ) -> None:
        """Publish the same prediction with timestamps and frame metadata."""
        states = np.asarray(prediction.states, dtype=float)
        timestamps = np.asarray(prediction.timestamps, dtype=float)

        if states.ndim != 2 or min(states.shape) == 0:
            raise ValueError("Predicted states must be a nonempty matrix.")

        horizon, state_dimension = states.shape
        if timestamps.shape != (horizon,):
            raise ValueError("Prediction timestamps must match the horizon.")
        if not np.all(np.isfinite(states)):
            raise ValueError("Predicted states must be finite.")
        if not np.all(np.isfinite(timestamps)):
            raise ValueError("Prediction timestamps must be finite.")
        if not np.isfinite(reference_timestamp):
            raise ValueError("Prediction reference timestamp must be finite.")
        if timestamps[0] <= reference_timestamp or np.any(np.diff(timestamps) <= 0):
            raise ValueError("Prediction timestamps must increase after reference time.")

        message = PredictedTrajectoryMsg()
        message.header.stamp = rclpy.time.Time(seconds=reference_timestamp).to_msg()
        message.header.frame_id = frame_id
        message.horizon = horizon
        message.state_dimension = state_dimension
        message.timestamps = timestamps.tolist()
        message.states = states.ravel(order="C").tolist()

        self._trajectory_full_publisher.publish(message)

    def _publish_trajectory(
        self,
        states: np.ndarray,
    ) -> None:
        """Publish the predicted trajectory."""
        if states.ndim != 2:
            raise ValueError(
                'predicted trajectory must be a matrix.'
            )

        horizon, state_dimension = (
            states.shape
        )

        message = Float64MultiArray()

        message.layout.dim = [
            MultiArrayDimension(
                label='horizon',
                size=horizon,
                stride=horizon * state_dimension,
            ),
            MultiArrayDimension(
                label='state',
                size=state_dimension,
                stride=state_dimension,
            ),
        ]

        message.data = (
            states.reshape(-1).tolist()
        )

        self._trajectory_publisher.publish(
            message
        )

    def _publish_confidence(
        self,
        confidence: float,
        residual: np.ndarray,
    ) -> None:
        """Publish prediction confidence and residual vector."""
        residual = np.asarray(
            residual,
            dtype=float,
        )

        message = Float64MultiArray()

        message.layout.dim = [
            MultiArrayDimension(
                label='confidence_and_residual',
                size=1 + residual.size,
                stride=1 + residual.size,
            )
        ]

        message.data = np.concatenate((
            np.array(
                [confidence],
                dtype=float,
            ),
            residual,
        )).tolist()

        self._confidence_publisher.publish(
            message
        )


def main(
    args=None,
) -> None:
    """Run the prediction node."""
    rclpy.init(
        args=args
    )

    node = PredictionNode()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
