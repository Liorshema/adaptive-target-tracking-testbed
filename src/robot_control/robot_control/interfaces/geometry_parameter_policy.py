"""Contract for selecting reference-geometry parameters."""

from typing import Protocol

import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]

class GeometryParameterPolicy(Protocol):
    """Interface for selecting tracking geometry parameters."""

    def select_parameters(
        self,
        current_position: Array,
        target_position: Array,
    ) -> Array:
        """Return parameters accepted by a ReferenceGeometry."""
        ...
