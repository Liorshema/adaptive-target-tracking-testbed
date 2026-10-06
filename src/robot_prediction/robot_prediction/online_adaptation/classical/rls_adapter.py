import numpy as np


class RLSAdapter:
    """Learn a linear vector mapping online using recursive least squares."""

    def __init__(
        self,
        input_dimension: int,
        output_dimension: int,
        forgetting_factor: float,
        initial_covariance: float,
    ):
        if input_dimension <= 0:
            raise ValueError(
                'input_dimension must be positive.'
            )

        if output_dimension <= 0:
            raise ValueError(
                'output_dimension must be positive.'
            )

        if not 0.0 < forgetting_factor <= 1.0:
            raise ValueError(
                'forgetting_factor must be in the interval (0, 1].'
            )

        if initial_covariance <= 0.0:
            raise ValueError(
                'initial_covariance must be positive.'
            )

        self._input_dimension = input_dimension
        self._output_dimension = output_dimension
        self._forgetting_factor = forgetting_factor

        self._parameters = np.zeros(
            (
                output_dimension,
                input_dimension,
            ),
            dtype=float,
        )

        self._covariance = (
            initial_covariance
            * np.eye(
                input_dimension,
                dtype=float,
            )
        )

    def predict(
        self,
        features: np.ndarray,
    ) -> np.ndarray:
        """Predict an output vector from the current parameter estimate."""
        features = self._validate_features(
            features
        )

        return self._parameters @ features

    def update(
        self,
        features: np.ndarray,
        target: np.ndarray,
    ) -> np.ndarray:
        """Update the parameter estimate from one observation."""
        features = self._validate_features(
            features
        )

        target = np.asarray(
            target,
            dtype=float,
        )

        if target.shape != (
            self._output_dimension,
        ):
            raise ValueError(
                'target has incompatible dimension.'
            )

        covariance_features = (
            self._covariance @ features
        )

        denominator = (
            self._forgetting_factor
            + features.T
            @ covariance_features
        )

        gain = (
            covariance_features
            / denominator
        )

        prediction = (
            self._parameters @ features
        )

        residual = (
            target - prediction
        )

        self._parameters = (
            self._parameters
            + np.outer(
                residual,
                gain,
            )
        )

        self._covariance = (
            self._covariance
            - np.outer(
                gain,
                features,
            )
            @ self._covariance
        ) / self._forgetting_factor

        return residual

    def reset(
        self,
        initial_covariance: float,
    ) -> None:
        """Reset learned parameters and covariance."""
        if initial_covariance <= 0.0:
            raise ValueError(
                'initial_covariance must be positive.'
            )

        self._parameters.fill(
            0.0
        )

        self._covariance = (
            initial_covariance
            * np.eye(
                self._input_dimension,
                dtype=float,
            )
        )

    @property
    def parameters(
        self,
    ) -> np.ndarray:
        """Return the learned parameter matrix."""
        return self._parameters.copy()

    @property
    def covariance(
        self,
    ) -> np.ndarray:
        """Return the current RLS covariance matrix."""
        return self._covariance.copy()

    def _validate_features(
        self,
        features: np.ndarray,
    ) -> np.ndarray:
        """Validate and return the feature vector."""
        features = np.asarray(
            features,
            dtype=float,
        )

        if features.shape != (
            self._input_dimension,
        ):
            raise ValueError(
                'features have incompatible dimension.'
            )

        return features
