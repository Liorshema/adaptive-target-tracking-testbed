import numpy as np


class ConfidenceWeighting:
    """Convert prediction confidences into normalized fusion weights."""

    def compute(
        self,
        confidences: np.ndarray,
    ) -> np.ndarray:
        """Compute normalized weights from confidence values."""
        confidences = np.asarray(
            confidences,
            dtype=float,
        )

        if confidences.ndim != 1:
            raise ValueError(
                'confidences must be a one-dimensional vector.'
            )

        if confidences.size == 0:
            raise ValueError(
                'confidences must contain at least one value.'
            )

        if np.any(confidences < 0.0):
            raise ValueError(
                'confidences must be non-negative.'
            )

        confidence_sum = np.sum(
            confidences
        )

        if confidence_sum <= 0.0:
            return np.full(
                confidences.size,
                1.0 / confidences.size,
            )

        return confidences / confidence_sum
