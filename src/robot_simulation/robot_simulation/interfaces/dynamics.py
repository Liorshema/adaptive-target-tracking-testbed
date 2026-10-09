
"""Generic interface for continuous-time dynamical systems."""

from abc import ABC, abstractmethod
from typing import Any, Mapping

import numpy as np


class Dynamics(ABC):
    """Abstract continuous-time state-space model."""

    @abstractmethod
    def derivative(
        self,
        state: np.ndarray,
        control: np.ndarray,
        time: float,
        context: Mapping[str, Any],
    ) -> np.ndarray:
        """Compute the continuous-time state derivative."""
        raise NotImplementedError
