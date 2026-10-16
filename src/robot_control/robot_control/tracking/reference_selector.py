from robot_control.interfaces.geometry_parameter_policy import GeometryParameterPolicy

"""Geometry-independent tracking reference selection.

Selection policies choose geometry parameters.
Robot models convert those parameters into desired positions.

This module has no ROS or Gazebo dependencies.
"""


import numpy as np
from numpy.typing import NDArray
from robot_models.interfaces.reference_geometry import (
    ReferenceGeometry,
)

from robot_control.interfaces.control_data import TrackingReference

Array = NDArray[np.float64]


def _validate_position(value: Array, name: str) -> Array:
    """Validate a finite position vector."""
    result = np.asarray(value, dtype=np.float64)

    if result.ndim != 1 or result.size == 0:
        raise ValueError(f"{name} must be a nonempty 1D vector")

    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains nonfinite values")

    return result.copy()


class FixedGeometryParameters:
    """Policy using externally supplied, constant parameters."""

    def __init__(self, parameters: Array) -> None:
        """Store a validated copy of the parameters."""
        values = np.asarray(parameters, dtype=np.float64)

        if values.ndim != 1 or values.size == 0:
            raise ValueError("parameters must be a nonempty vector")

        if not np.all(np.isfinite(values)):
            raise ValueError("parameters contain nonfinite values")

        self._parameters = values.copy()

    def select_parameters(
        self,
        current_position: Array,
        target_position: Array,
    ) -> Array:
        """Return the configured geometry parameters."""
        return self._parameters.copy()


class ReferenceSelector:
    """Combine a selection policy with reference geometry."""

    def __init__(
        self,
        geometry: ReferenceGeometry,
        policy: GeometryParameterPolicy,
    ) -> None:
        """Inject interchangeable geometry and selection policy."""
        self._geometry = geometry
        self._policy = policy

    def select(
        self,
        current_position: Array,
        target_position: Array,
    ) -> TrackingReference:
        """Select the desired task-space position.

        Both positions must be expressed in the same world frame.

        Args:
            current_position: Current tracking-frame position.
            target_position: Current or predicted target position.

        Returns:
            Validated desired tracking position.
        """
        current = _validate_position(
            current_position, "current_position"
        )
        target = _validate_position(
            target_position, "target_position"
        )

        if current.shape != target.shape:
            raise ValueError(
                "Current and target position dimensions differ"
            )

        parameters = self._policy.select_parameters(
            current_position=current,
            target_position=target,
        )

        parameters = np.asarray(parameters, dtype=np.float64)

        if parameters.ndim != 1 or not np.all(
            np.isfinite(parameters)
        ):
            raise ValueError(
                "Policy must return a finite parameter vector"
            )

        desired_position = self._geometry.desired_point(
            target_position_world=target,
            geometry_parameters=parameters,
        )

        return TrackingReference(
            position=desired_position,
        )
