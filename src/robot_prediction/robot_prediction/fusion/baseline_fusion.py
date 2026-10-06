from collections.abc import Sequence

import numpy as np

from robot_prediction.interfaces.prediction_data import (
    TrajectoryPrediction,
)


class BaselineFusion:
    """Fuse multiple trajectory predictions using fixed weights."""

    def fuse(
        self,
        predictions: Sequence[TrajectoryPrediction],
        weights: np.ndarray,
    ) -> TrajectoryPrediction:
        """Fuse trajectory predictions using normalized weights."""
        if not predictions:
            raise ValueError(
                'predictions must contain at least one trajectory.'
            )

        weights = np.asarray(
            weights,
            dtype=float,
        )

        if weights.ndim != 1:
            raise ValueError(
                'weights must be a one-dimensional vector.'
            )

        if weights.size != len(predictions):
            raise ValueError(
                'weights size must match number of predictions.'
            )

        if np.any(weights < 0.0):
            raise ValueError(
                'weights must be non-negative.'
            )

        weight_sum = np.sum(weights)

        if weight_sum <= 0.0:
            raise ValueError(
                'weights must have positive sum.'
            )

        normalized_weights = (
            weights / weight_sum
        )

        reference_shape = predictions[0].states.shape
        reference_timestamps = predictions[0].timestamps

        for prediction in predictions:
            if prediction.states.shape != reference_shape:
                raise ValueError(
                    'all predictions must have the same state shape.'
                )

            if not np.allclose(
                prediction.timestamps,
                reference_timestamps,
            ):
                raise ValueError(
                    'all predictions must share the same timestamps.'
                )

        stacked_states = np.stack(
            [
                prediction.states
                for prediction in predictions
            ],
            axis=0,
        )

        fused_states = np.tensordot(
            normalized_weights,
            stacked_states,
            axes=(0, 0),
        )

        return TrajectoryPrediction(
            states=fused_states,
            timestamps=reference_timestamps.copy(),
            model_name='baseline_fusion',
        )
