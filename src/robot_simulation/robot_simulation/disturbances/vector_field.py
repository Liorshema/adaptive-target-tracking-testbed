
"""Configurable linear restoring vector field."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VectorFieldParameters:
    """Parameters defining a restoring acceleration field."""

    center: np.ndarray
    gain_matrix: np.ndarray

    def __post_init__(self) -> None:
        center = np.asarray(self.center, dtype=float)
        gain = np.asarray(self.gain_matrix, dtype=float)

        if center.ndim != 1 or center.size == 0:
            raise ValueError("center must be a non-empty vector.")

        dimension = center.size

        if gain.shape != (dimension, dimension):
            raise ValueError("gain_matrix dimension mismatch.")

        if not np.all(np.isfinite(center)):
            raise ValueError("center must contain finite values.")

        if not np.all(np.isfinite(gain)):
            raise ValueError("gain_matrix must contain finite values.")

        if not np.allclose(gain, gain.T):
            raise ValueError("gain_matrix must be symmetric.")

        if np.min(np.linalg.eigvalsh(gain)) <= 0.0:
            raise ValueError("gain_matrix must be positive definite.")

        object.__setattr__(self, "center", center.copy())
        object.__setattr__(self, "gain_matrix", gain.copy())


class RestoringVectorField:
    """Acceleration field attracting positions toward a center."""

    def __init__(self, parameters: VectorFieldParameters) -> None:
        self.parameters = parameters

    def acceleration(
        self,
        position: np.ndarray,
        time: float = 0.0,
    ) -> np.ndarray:
        """Compute restoring acceleration at a given position."""
        del time

        position = np.asarray(position, dtype=float)

        if position.shape != self.parameters.center.shape:
            raise ValueError("position dimension mismatch.")

        if not np.all(np.isfinite(position)):
            raise ValueError("position must contain finite values.")

        displacement = position - self.parameters.center

        return -self.parameters.gain_matrix @ displacement
