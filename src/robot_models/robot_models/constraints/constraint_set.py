"""ConstraintSet domain records with validated numerical shapes."""

from dataclasses import dataclass

import numpy as np



@dataclass
class ConstraintSet:
    """
    Linear inequality constraint representation.

    Represents constraints of the form:

        lower <= matrix @ x <= upper
    """

    matrix: np.ndarray
    lower: np.ndarray
    upper: np.ndarray

    def __post_init__(self) -> None:
        self.matrix = np.asarray(
            self.matrix,
            dtype=float,
        )

        self.lower = np.asarray(
            self.lower,
            dtype=float,
        )

        self.upper = np.asarray(
            self.upper,
            dtype=float,
        )

        if self.matrix.ndim != 2:
            raise ValueError(
                'matrix must be two-dimensional'
            )

        if self.lower.ndim != 1:
            raise ValueError(
                'lower must be a 1D vector'
            )

        if self.upper.ndim != 1:
            raise ValueError(
                'upper must be a 1D vector'
            )

        if self.matrix.shape[0] != self.lower.size:
            raise ValueError(
                'matrix row count must match lower size'
            )

        if self.lower.shape != self.upper.shape:
            raise ValueError(
                'lower and upper must have the same shape'
            )

        if np.any(self.lower > self.upper):
            raise ValueError(
                'lower bounds must not exceed upper bounds'
            )
