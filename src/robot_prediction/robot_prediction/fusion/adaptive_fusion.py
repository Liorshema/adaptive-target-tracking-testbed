from collections.abc import Sequence

import numpy as np

from robot_prediction.fusion.baseline_fusion import BaselineFusion
from robot_prediction.fusion.confidence_weighting import (
    ConfidenceWeighting,
)
from robot_prediction.interfaces.prediction_data import (
    TrajectoryPrediction,
)


class AdaptiveFusion:
    """Fuse predictions using confidence-derived weights."""

    def __init__(self):
        self._weighting = ConfidenceWeighting()
        self._fusion = BaselineFusion()

    def fuse(
        self,
        predictions: Sequence[TrajectoryPrediction],
        confidences: np.ndarray,
    ) -> TrajectoryPrediction:
        """Fuse predictions using adaptive confidence weights."""
        if len(predictions) != len(confidences):
            raise ValueError(
                'number of confidences must match number of predictions.'
            )

        weights = self._weighting.compute(
            confidences
        )

        return self._fusion.fuse(
            predictions=predictions,
            weights=weights,
        )
