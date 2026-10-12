
"""Time alignment utilities for predicted state trajectories."""

import numpy as np

from robot_prediction.interfaces.prediction_data import (
    TrajectoryPrediction,
)


def interpolate_predicted_state(
    prediction: TrajectoryPrediction,
    initial_state: np.ndarray,
    initial_timestamp: float,
    observation_timestamp: float,
) -> np.ndarray:
    """Interpolate a predicted state at an observation timestamp.

    The initial estimated state anchors the trajectory at its start.
    Interpolation is permitted only within the prediction horizon.
    """
    states = np.asarray(prediction.states, dtype=float)
    timestamps = np.asarray(prediction.timestamps, dtype=float)

    initial_state = np.asarray(initial_state, dtype=float)

    if states.ndim != 2 or states.shape[0] == 0:
        raise ValueError("Prediction states must be a nonempty matrix.")

    if timestamps.shape != (states.shape[0],):
        raise ValueError("Prediction timestamps have invalid dimensions.")

    if initial_state.shape != (states.shape[1],):
        raise ValueError("Initial state dimension does not match prediction.")

    times = np.concatenate((
        [float(initial_timestamp)],
        timestamps,
    ))

    trajectory = np.vstack((
        initial_state,
        states,
    ))

    query_time = float(observation_timestamp)

    if (
        not np.all(np.isfinite(times))
        or not np.all(np.isfinite(trajectory))
        or not np.isfinite(query_time)
    ):
        raise ValueError("States and timestamps must be finite.")

    if np.any(np.diff(times) <= 0.0):
        raise ValueError("Prediction timestamps must be strictly increasing.")

    if query_time < times[0] or query_time > times[-1]:
        raise ValueError(
            "Observation timestamp is outside the prediction horizon."
        )

    upper = int(np.searchsorted(times, query_time, side="left"))

    if upper == 0:
        return trajectory[0].copy()

    lower = upper - 1

    alpha = (
        (query_time - times[lower])
        / (times[upper] - times[lower])
    )

    return (
        (1.0 - alpha) * trajectory[lower]
        + alpha * trajectory[upper]
    )
