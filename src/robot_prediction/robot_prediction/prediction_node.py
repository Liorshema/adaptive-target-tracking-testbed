from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node

from robot_prediction.config_loader import PredictionConfig
from robot_prediction.interfaces.prediction_data import PredictionInput
from robot_prediction.interfaces.state_layout import StateLayout
from robot_prediction.system_factory import PredictionSystemFactory
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

        self._timestamp = 0.0
        self._state_dimension = None

        self._state_subscription = (
            self.create_subscription(
                Float64MultiArray,
                'estimated_state',
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

    def _state_callback(
        self,
        message: Float64MultiArray,
    ) -> None:
        """Process a new estimated target state."""
        state = np.asarray(
            message.data,
            dtype=float,
        )

        try:
            layout = (
                StateLayout.from_state_dimension(
                    state.size
                )
            )

            self._validate_state_dimension(
                layout.state_dimension
            )

            if self._system.last_prediction is not None:
                performance = (
                    self._system.update(
                        observed_state=state
                    )
                )

                self._publish_confidence(
                    confidence=performance.confidence,
                    residual=performance.residual,
                )

            prediction_input = PredictionInput(
                state=state,
                timestamp=self._timestamp,
                dt=self._dt,
                horizon=self._horizon,
            )

            prediction = (
                self._system.predict(
                    prediction_input
                )
            )

            self._publish_trajectory(
                prediction.states
            )

            self._timestamp += (
                self._dt
            )

        except (
            RuntimeError,
            ValueError,
        ) as error:
            self.get_logger().error(
                f'Prediction failed: {error}'
            )

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
