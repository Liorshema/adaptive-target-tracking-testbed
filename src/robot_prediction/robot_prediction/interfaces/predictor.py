from abc import ABC, abstractmethod

from .prediction_data import PredictionInput, TrajectoryPrediction


class Predictor(ABC):
    """Abstract interface for target-motion predictors."""

    @abstractmethod
    def predict(
        self,
        prediction_input: PredictionInput,
    ) -> TrajectoryPrediction:
        """Predict the target trajectory over a finite horizon."""
        raise NotImplementedError
