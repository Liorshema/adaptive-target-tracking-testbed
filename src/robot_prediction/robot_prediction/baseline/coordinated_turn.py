import numpy as np

from robot_prediction.interfaces.prediction_data import (
    PredictionInput,
    TrajectoryPrediction,
)
from robot_prediction.interfaces.predictor import Predictor
from robot_prediction.interfaces.state_layout import StateLayout


class CoordinatedTurnPredictor(Predictor):
    """Predict coordinated-turn motion in a selected motion plane."""

    def __init__(
        self,
        turn_rate_epsilon: float,
    ):
        if turn_rate_epsilon <= 0.0:
            raise ValueError(
                'turn_rate_epsilon must be positive.'
            )

        self._turn_rate_epsilon = turn_rate_epsilon

    def predict(
        self,
        prediction_input: PredictionInput,
    ) -> TrajectoryPrediction:
        """Predict a coordinated-turn trajectory."""
        state = np.asarray(
            prediction_input.state,
            dtype=float,
        )

        self._validate_input(
            prediction_input=prediction_input,
            state=state,
        )

        spatial_dimension = (
            state.size - 1
        ) // 2

        layout = StateLayout(
            spatial_dimension=spatial_dimension
        )

        position = state[
            :spatial_dimension
        ].copy()

        velocity = state[
            spatial_dimension:
            2 * spatial_dimension
        ].copy()

        turn_rate = float(
            state[-1]
        )

        states = np.empty(
            (
                prediction_input.horizon,
                layout.state_dimension,
            ),
            dtype=float,
        )

        for step in range(
            prediction_input.horizon
        ):
            position, velocity = self._propagate(
                position=position,
                velocity=velocity,
                turn_rate=turn_rate,
                dt=prediction_input.dt,
            )

            states[step] = np.concatenate((
                position,
                velocity,
            ))

        timestamps = (
            prediction_input.timestamp
            + prediction_input.dt
            * np.arange(
                1,
                prediction_input.horizon + 1,
                dtype=float,
            )
        )

        return TrajectoryPrediction(
            states=states,
            timestamps=timestamps,
            model_name='coordinated_turn',
        )

    def _propagate(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        turn_rate: float,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Propagate position and velocity by one time step."""
        next_position = position.copy()
        next_velocity = velocity.copy()

        planar_position = position[:2]
        planar_velocity = velocity[:2]

        if abs(turn_rate) < self._turn_rate_epsilon:
            next_position = (
                position
                + dt * velocity
            )

            return (
                next_position,
                next_velocity,
            )

        angle = (
            turn_rate * dt
        )

        sine = np.sin(
            angle
        )

        cosine = np.cos(
            angle
        )

        rotation = np.array([
            [
                cosine,
                -sine,
            ],
            [
                sine,
                cosine,
            ],
        ])

        integration_matrix = (
            1.0
            / turn_rate
            * np.array([
                [
                    sine,
                    -(1.0 - cosine),
                ],
                [
                    1.0 - cosine,
                    sine,
                ],
            ])
        )

        next_position[:2] = (
            planar_position
            + integration_matrix
            @ planar_velocity
        )

        next_velocity[:2] = (
            rotation
            @ planar_velocity
        )

        if position.size > 2:
            next_position[2:] = (
                position[2:]
                + dt * velocity[2:]
            )

            next_velocity[2:] = (
                velocity[2:]
            )

        return (
            next_position,
            next_velocity,
        )

    @staticmethod
    def _validate_input(
        prediction_input: PredictionInput,
        state: np.ndarray,
    ) -> None:
        """Validate coordinated-turn prediction input."""
        if state.ndim != 1:
            raise ValueError(
                'state must be a one-dimensional vector.'
            )

        if state.size < 5:
            raise ValueError(
                'coordinated-turn state must contain '
                '[position, velocity, turn_rate] '
                'with at least two spatial dimensions.'
            )

        if (
            state.size - 1
        ) % 2 != 0:
            raise ValueError(
                'coordinated-turn state must contain '
                '[position, velocity, turn_rate].'
            )

        spatial_dimension = (
            state.size - 1
        ) // 2

        if spatial_dimension < 2:
            raise ValueError(
                'coordinated-turn motion requires '
                'at least two spatial dimensions.'
            )

        if prediction_input.dt <= 0.0:
            raise ValueError(
                'dt must be positive.'
            )

        if prediction_input.horizon <= 0:
            raise ValueError(
                'horizon must be positive.'
            )
