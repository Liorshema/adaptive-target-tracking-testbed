from collections import deque

import numpy as np


class VectorBuffer:
    """Store a fixed-length history of vectors."""

    def __init__(
        self,
        capacity: int,
    ):
        if capacity <= 0:
            raise ValueError(
                'capacity must be positive.'
            )

        self._capacity = capacity
        self._values = deque(
            maxlen=capacity
        )

        self._vector_dimension = None

    def append(
        self,
        value: np.ndarray,
    ) -> None:
        """Append a vector to the buffer."""
        vector = np.asarray(
            value,
            dtype=float,
        )

        if vector.ndim != 1:
            raise ValueError(
                'value must be a one-dimensional vector.'
            )

        if self._vector_dimension is None:
            self._vector_dimension = vector.size

        elif vector.size != self._vector_dimension:
            raise ValueError(
                'all vectors must have the same dimension.'
            )

        self._values.append(
            vector.copy()
        )

    def clear(
        self,
    ) -> None:
        """Clear all stored vectors."""
        self._values.clear()
        self._vector_dimension = None

    def as_array(
        self,
    ) -> np.ndarray:
        """Return stored vectors as a matrix."""
        if not self._values:
            if self._vector_dimension is None:
                return np.empty(
                    (0, 0),
                    dtype=float,
                )

            return np.empty(
                (0, self._vector_dimension),
                dtype=float,
            )

        return np.stack(
            self._values,
            axis=0,
        )

    @property
    def latest(
        self,
    ) -> np.ndarray | None:
        """Return the most recently stored vector."""
        if not self._values:
            return None

        return self._values[-1].copy()

    @property
    def oldest(
        self,
    ) -> np.ndarray | None:
        """Return the oldest stored vector."""
        if not self._values:
            return None

        return self._values[0].copy()

    @property
    def capacity(
        self,
    ) -> int:
        """Return the maximum number of stored vectors."""
        return self._capacity

    @property
    def size(
        self,
    ) -> int:
        """Return the current number of stored vectors."""
        return len(self._values)

    @property
    def is_full(
        self,
    ) -> bool:
        """Return whether the buffer is full."""
        return len(self._values) == self._capacity

    def __len__(
        self,
    ) -> int:
        return len(self._values)
