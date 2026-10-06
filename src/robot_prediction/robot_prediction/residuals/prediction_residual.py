import numpy as np


def compute_residual(
    observed_state: np.ndarray,
    predicted_state: np.ndarray,
) -> np.ndarray:
    """Compute the prediction residual."""
    observed = np.asarray(
        observed_state,
        dtype=float,
    )

    predicted = np.asarray(
        predicted_state,
        dtype=float,
    )

    if observed.ndim != 1:
        raise ValueError(
            'observed_state must be a one-dimensional vector.'
        )

    if predicted.ndim != 1:
        raise ValueError(
            'predicted_state must be a one-dimensional vector.'
        )

    if observed.shape != predicted.shape:
        raise ValueError(
            'observed_state and predicted_state '
            'must have the same shape.'
        )

    return observed - predicted
