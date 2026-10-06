import numpy as np

from robot_prediction.confidence.performance_monitor import (
    PerformanceMonitor,
    PerformanceSnapshot,
)
from robot_prediction.fusion.weighted_correction import (
    WeightedCorrection,
)
from robot_prediction.interfaces.prediction_data import (
    PredictionInput,
    TrajectoryPrediction,
)
from robot_prediction.interfaces.predictor import Predictor
from robot_prediction.online_adaptation.classical.residual_adapter import (
    ResidualAdapter,
)


class TrajectoryPredictionSystem:
    """Coordinate prediction, adaptation, and performance monitoring."""

    def __init__(
        self,
        predictor: Predictor,
        performance_monitor: PerformanceMonitor,
        residual_adapter: ResidualAdapter | None = None,
    ):
        self._predictor = predictor
        self._performance_monitor = performance_monitor
        self._residual_adapter = residual_adapter
        self._weighted_correction = WeightedCorrection()

        self._last_prediction = None
        self._last_performance = None

    def predict(
        self,
        prediction_input: PredictionInput,
    ) -> TrajectoryPrediction:
        """Generate a target trajectory prediction."""
        prediction = self._predictor.predict(
            prediction_input
        )

        if (
            self._residual_adapter is not None
            and self._performance_monitor.sample_count > 0
        ):
            correction = (
                self._residual_adapter.compute_correction(
                    statistics=self._performance_monitor.statistics,
                    horizon=prediction_input.horizon,
                )
            )

            prediction = (
                self._weighted_correction.apply(
                    baseline=prediction,
                    correction=correction,
                )
            )

        self._last_prediction = prediction

        return prediction

    def update(
        self,
        observed_state: np.ndarray,
    ) -> PerformanceSnapshot:
        """Update prediction performance and adaptive predictor state."""
        if self._last_prediction is None:
            raise RuntimeError(
                'predict() must be called before update().'
            )

        observation = np.asarray(
            observed_state,
            dtype=float,
        )

        predicted_state = (
            self._last_prediction.states[0]
        )

        performance = (
            self._performance_monitor.update(
                observed_state=observation,
                predicted_state=predicted_state,
            )
        )

        predictor_update = getattr(
            self._predictor,
            'update',
            None,
        )

        if callable(
            predictor_update
        ):
            predictor_update(
                observation
            )

        self._last_performance = (
            performance
        )

        return performance

    @property
    def last_prediction(
        self,
    ) -> TrajectoryPrediction | None:
        """Return the latest trajectory prediction."""
        return self._last_prediction

    @property
    def last_performance(
        self,
    ) -> PerformanceSnapshot | None:
        """Return the latest performance snapshot."""
        return self._last_performance
