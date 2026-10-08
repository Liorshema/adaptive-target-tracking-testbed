"""Interfaces for generic tracking-reference geometry."""

from abc import ABC, abstractmethod

import numpy as np


class ReferenceGeometry(ABC):
    """Common contract for geometric tracking references."""

    @abstractmethod
    def desired_point(
        self,
        target_position_world: np.ndarray,
        geometry_parameters: np.ndarray,
    ) -> np.ndarray:
        """Return desired tracking-frame position in the world frame."""

    @abstractmethod
    def residual(
        self,
        current_position_world: np.ndarray,
        target_position_world: np.ndarray,
        geometry_parameters: np.ndarray,
    ) -> np.ndarray:
        """Return geometric tracking residual."""
