from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class PredictionInput:
    """Input data required by a trajectory predictor."""

    state: np.ndarray
    timestamp: float
    dt: float
    horizon: int
    covariance: Optional[np.ndarray] = None


@dataclass(frozen=True)
class TrajectoryPrediction:
    """Predicted target trajectory over a finite horizon."""

    states: np.ndarray
    timestamps: np.ndarray
    covariance: Optional[np.ndarray] = None
    confidence: Optional[np.ndarray] = None
    model_name: Optional[str] = None
