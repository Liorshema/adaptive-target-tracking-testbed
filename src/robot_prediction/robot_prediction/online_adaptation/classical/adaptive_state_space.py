import numpy as np

from robot_prediction.online_adaptation.classical.rls_adapter import (
    RLSAdapter,
)


class AdaptiveStateSpace:
    """Learn and predict linear state dynamics online."""

    def __init__(
        self,
        state_dimension: int,
        forgetting_factor: float,
        initial_covariance: float,
    ):
        if state_dimension <= 0:
            raise ValueError(
                'state_dimension must be positive.'
            )

        self._state_dimension = state_dimension

        self._adapter = RLSAdapter(
            input_dimension=state_dimension,
            output_dimension=state_dimension,
            forgetting_factor=forgetting_factor,
            initial_covariance=initial_covariance,
        )

    def update(
        self,
        current_state: np.ndarray,
        next_state: np.ndarray,
    ) -> np.ndarray:
        """Update the learned state-transition model."""
        current_state = self._validate_state(
            current_state
        )

        next_state = self._validate_state(
            next_state
        )

        residual = self._adapter.update(
            features=current_state,
            target=next_state,
        )

        return residual

    def predict_next(
        self,
        state: np.ndarray,
    ) -> np.ndarray:
        """Predict the next state."""
        state = self._validate_state(
            state
        )

        return self._adapter.predict(
            features=state
        )

    def predict_trajectory(
        self,
        initial_state: np.ndarray,
        horizon: int,
    ) -> np.ndarray:
        """Predict a state trajectory recursively."""
        if horizon <= 0:
            raise ValueError(
                'horizon must be positive.'
            )

        state = self._validate_state(
            initial_state
        ).copy()

        trajectory = np.empty(
            (
                horizon,
                self._state_dimension,
            ),
            dtype=float,
        )

        for step in range(horizon):
            state = self.predict_next(
                state
            )

            trajectory[step] = state

        return trajectory

    @property
    def transition_matrix(
        self,
    ) -> np.ndarray:
        """Return the learned state-transition matrix."""
        return self._adapter.parameters

    @property
    def covariance(
        self,
    ) -> np.ndarray:
        """Return the RLS parameter covariance matrix."""
        return self._adapter.covariance

    def _validate_state(
        self,
        state: np.ndarray,
    ) -> np.ndarray:
        """Validate and return a state vector."""
        state = np.asarray(
            state,
            dtype=float,
        )

        if state.shape != (
            self._state_dimension,
        ):
            raise ValueError(
                'state has incompatible dimension.'
            )

        return state
