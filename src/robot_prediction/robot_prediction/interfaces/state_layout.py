from dataclasses import dataclass


@dataclass(frozen=True)
class StateLayout:
    """Describe a position-velocity state for any spatial dimension."""

    spatial_dimension: int

    def __post_init__(self) -> None:
        if self.spatial_dimension <= 0:
            raise ValueError(
                'spatial_dimension must be positive.'
            )

    @property
    def position_dimension(self) -> int:
        """Return position-vector dimension."""
        return self.spatial_dimension

    @property
    def velocity_dimension(self) -> int:
        """Return velocity-vector dimension."""
        return self.spatial_dimension

    @property
    def state_dimension(self) -> int:
        """Return dimension of the common [position, velocity] state."""
        return 2 * self.spatial_dimension

    @property
    def acceleration_state_dimension(self) -> int:
        """Return dimension of an internal [position, velocity, acceleration] state."""
        return 3 * self.spatial_dimension

    @classmethod
    def from_state_dimension(
        cls,
        state_dimension: int,
    ) -> 'StateLayout':
        """Infer spatial dimension from a common [position, velocity] state."""
        if state_dimension <= 0:
            raise ValueError(
                'state_dimension must be positive.'
            )

        if state_dimension % 2 != 0:
            raise ValueError(
                'common state dimension must be even '
                'for [position, velocity].'
            )

        return cls(
            spatial_dimension=state_dimension // 2
        )

    def position_slice(
        self,
    ) -> slice:
        """Return the position slice."""
        return slice(
            0,
            self.spatial_dimension,
        )

    def velocity_slice(
        self,
    ) -> slice:
        """Return the velocity slice."""
        return slice(
            self.spatial_dimension,
            2 * self.spatial_dimension,
        )

    def acceleration_slice(
        self,
    ) -> slice:
        """Return the acceleration slice of an internal acceleration state."""
        return slice(
            2 * self.spatial_dimension,
            3 * self.spatial_dimension,
        )
