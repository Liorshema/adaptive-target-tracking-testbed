from abc import ABC, abstractmethod

import numpy as np


class OnlineSequenceLearner(ABC):
    """Define an interface for online sequence prediction models."""

    def __init__(
        self,
        state_dimension: int,
        history_length: int,
        horizon: int,
    ):
        if state_dimension <= 0:
            raise ValueError(
                'state_dimension must be positive.'
            )

        if history_length <= 0:
            raise ValueError(
                'history_length must be positive.'
            )

        if horizon <= 0:
            raise ValueError(
                'horizon must be positive.'
            )

        self._state_dimension = state_dimension
        self._history_length = history_length
        self._horizon = horizon

    def predict(
        self,
        history: np.ndarray,
    ) -> np.ndarray:
        """Predict a future trajectory from a state-history window."""
        history = self._validate_history(
            history
        )

        prediction = self._predict_impl(
            history
        )

        prediction = np.asarray(
            prediction,
            dtype=float,
        )

        expected_shape = (
            self._horizon,
            self._state_dimension,
        )

        if prediction.shape != expected_shape:
            raise ValueError(
                'prediction has incompatible shape.'
            )

        return prediction

    def update(
        self,
        history: np.ndarray,
        target: np.ndarray,
    ) -> None:
        """Update the sequence model from a new training example."""
        history = self._validate_history(
            history
        )

        target = np.asarray(
            target,
            dtype=float,
        )

        expected_shape = (
            self._horizon,
            self._state_dimension,
        )

        if target.shape != expected_shape:
            raise ValueError(
                'target has incompatible shape.'
            )

        self._update_impl(
            history=history,
            target=target,
        )

    @abstractmethod
    def _predict_impl(
        self,
        history: np.ndarray,
    ) -> np.ndarray:
        """Implement model-specific sequence prediction."""
        raise NotImplementedError

    @abstractmethod
    def _update_impl(
        self,
        history: np.ndarray,
        target: np.ndarray,
    ) -> None:
        """Implement model-specific online learning."""
        raise NotImplementedError

    def _validate_history(
        self,
        history: np.ndarray,
    ) -> np.ndarray:
        """Validate a state-history matrix."""
        history = np.asarray(
            history,
            dtype=float,
        )

        expected_shape = (
            self._history_length,
            self._state_dimension,
        )

        if history.shape != expected_shape:
            raise ValueError(
                'history has incompatible shape.'
            )

        return history

    @property
    def state_dimension(
        self,
    ) -> int:
        """Return the state dimension."""
        return self._state_dimension

    @property
    def history_length(
        self,
    ) -> int:
        """Return the required history length."""
        return self._history_length

    @property
    def horizon(
        self,
    ) -> int:
        """Return the prediction horizon."""
        return self._horizon
