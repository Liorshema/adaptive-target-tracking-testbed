import numpy as np

from robot_prediction.interfaces.prediction_data import (
    TrajectoryPrediction,
)


class WeightedCorrection:
    """Apply a weighted correction to a trajectory prediction."""

    def apply(
        self,
        baseline: TrajectoryPrediction,
        correction: np.ndarray,
        weight: float | np.ndarray = 1.0,
    ) -> TrajectoryPrediction:
        """Apply a weighted correction to a baseline trajectory."""
        correction = np.asarray(
            correction,
            dtype=float,
        )

        if correction.shape != baseline.states.shape:
            raise ValueError(
                'correction must have the same shape '
                'as baseline states.'
            )

        weight_array = np.asarray(
            weight,
            dtype=float,
        )

        if np.any(weight_array < 0.0):
            raise ValueError(
                'weight must be non-negative.'
            )

        if weight_array.ndim == 0:
            weighted_correction = (
                weight_array * correction
            )

        elif weight_array.shape == (
            baseline.states.shape[1],
        ):
            weighted_correction = (
                correction
                * weight_array[np.newaxis, :]
            )

        else:
            raise ValueError(
                'weight must be a scalar or have shape '
                '(state_dimension,).'
            )

        corrected_states = (
            baseline.states
            + weighted_correction
        )

        return TrajectoryPrediction(
            states=corrected_states,
            timestamps=baseline.timestamps.copy(),
            covariance=baseline.covariance,
            confidence=baseline.confidence,
            model_name='weighted_correction',
        )
