import numpy as np

from robot_prediction.interfaces.prediction_data import (
    PredictionInput,
    TrajectoryPrediction,
)
from robot_prediction.interfaces.predictor import Predictor
from robot_prediction.interfaces.state_layout import StateLayout


class ConstantVelocityPredictor(Predictor):
    """Predict motion using a constant-velocity model."""

    def predict(
        self,
        prediction_input: PredictionInput,
    ) -> TrajectoryPrediction:
        """Predict a trajectory with constant velocity."""
        state = np.asarray(
            prediction_input.state,
            dtype=float,
        )

        layout = StateLayout.from_state_dimension(
            state.size
        )

        self._validate_input(
            prediction_input=prediction_input,
            state=state,
        )

        transition_matrix = (
            self._build_transition_matrix(
                spatial_dimension=layout.spatial_dimension,
                dt=prediction_input.dt,
            )
        )

        states = np.empty(
            (
                prediction_input.horizon,
                layout.state_dimension,
            ),
            dtype=float,
        )

        predicted_state = state.copy()

        for step in range(
            prediction_input.horizon
        ):
            predicted_state = (
                transition_matrix
                @ predicted_state
            )

            states[step] = (
                predicted_state
            )

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
            model_name='constant_velocity',
        )

    @staticmethod
    def _build_transition_matrix(
        spatial_dimension: int,
        dt: float,
    ) -> np.ndarray:
        """Build the constant-velocity transition matrix."""
        identity = np.eye(
            spatial_dimension,
            dtype=float,
        )

        zeros = np.zeros_like(
            identity
        )

        return np.block([
            [
                identity,
                dt * identity,
            ],
            [
                zeros,
                identity,
            ],
        ])

    @staticmethod
    def _validate_input(
        prediction_input: PredictionInput,
        state: np.ndarray,
    ) -> None:
        """Validate prediction input."""
        if state.ndim != 1:
            raise ValueError(
                'state must be a one-dimensional vector.'
            )

        if state.size == 0:
            raise ValueError(
                'state must not be empty.'
            )

        if state.size % 2 != 0:
            raise ValueError(
                'constant-velocity state must contain '
                '[position, velocity].'
            )

        if prediction_input.dt <= 0.0:
            raise ValueError(
                'dt must be positive.'
            )

        if prediction_input.horizon <= 0:
            raise ValueError(
                'horizon must be positive.'
            )
