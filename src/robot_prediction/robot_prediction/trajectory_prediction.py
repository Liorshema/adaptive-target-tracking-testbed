
"""Orchestrate trajectory prediction, adaptation, and monitoring."""

import numpy as np

from robot_prediction.confidence.performance_monitor import (
    PerformanceMonitor,
    PerformanceSnapshot,
)
from robot_prediction.confidence.temporal_alignment import (
    interpolate_predicted_state,
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
    ) -> None:
        self._predictor = predictor
        self._performance_monitor = performance_monitor
        self._residual_adapter = residual_adapter
        self._weighted_correction = WeightedCorrection()

        self._last_prediction: TrajectoryPrediction | None = None
        self._last_input: PredictionInput | None = None
        self._last_performance: PerformanceSnapshot | None = None

    def predict(
        self,
        prediction_input: PredictionInput,
    ) -> TrajectoryPrediction:
        """Generate and store a trajectory prediction."""
        prediction = self._predictor.predict(prediction_input)

        if (
            self._residual_adapter is not None
            and self._performance_monitor.sample_count > 0
        ):
            correction = self._residual_adapter.compute_correction(
                statistics=self._performance_monitor.statistics,
                horizon=prediction_input.horizon,
            )

            prediction = self._weighted_correction.apply(
                baseline=prediction,
                correction=correction,
            )

        self._last_input = prediction_input
        self._last_prediction = prediction

        return prediction

    def update(
        self,
        observed_state: np.ndarray,
        observation_dt: float | None = None,
        observation_timestamp: float | None = None,
    ) -> PerformanceSnapshot | None:
        """Update the predictor and time-aligned performance monitor.

        Returns None when a measurement cannot be compared against
        the stored prediction horizon. Predictor adaptation still runs.
        """
        if self._last_prediction is None or self._last_input is None:
            raise RuntimeError(
                "predict() must be called before update()."
            )

        observation = np.asarray(observed_state, dtype=float)

        if observation_dt is not None:
            observation_dt = float(observation_dt)
            if not np.isfinite(observation_dt) or observation_dt <= 0.0:
                raise ValueError(
                    "observation_dt must be finite and positive."
                )

        if observation_timestamp is None:
            if observation_dt is None:
                raise ValueError(
                    "observation_timestamp or observation_dt is required."
                )
            observation_timestamp = (
                self._last_input.timestamp + observation_dt
            )

        observation_timestamp = float(observation_timestamp)

        if not np.isfinite(observation_timestamp):
            raise ValueError("observation_timestamp must be finite.")

        if observation_timestamp <= self._last_input.timestamp:
            raise ValueError(
                "Observation time must follow prediction time."
            )

        # Always update the online predictor, even if the
        # observation is outside the previous prediction horizon.
        predictor_update = getattr(self._predictor, "update", None)

        if callable(predictor_update):
            predictor_update(
                observation,
                observation_dt=observation_dt,
            )

        timestamps = np.asarray(
            self._last_prediction.timestamps,
            dtype=float,
        )

        if (
            timestamps.size == 0
            or observation_timestamp > timestamps[-1]
        ):
            return None

        predicted_state = interpolate_predicted_state(
            prediction=self._last_prediction,
            initial_state=self._last_input.state,
            initial_timestamp=self._last_input.timestamp,
            observation_timestamp=observation_timestamp,
        )

        performance = self._performance_monitor.update(
            observed_state=observation,
            predicted_state=predicted_state,
        )

        self._last_performance = performance
        return performance

    @property
    def last_prediction(self) -> TrajectoryPrediction | None:
        return self._last_prediction

    @property
    def last_performance(self) -> PerformanceSnapshot | None:
        return self._last_performance
