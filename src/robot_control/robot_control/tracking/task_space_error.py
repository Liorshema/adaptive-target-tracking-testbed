
"""Controller-independent task-space error computation.

Pose mathematics are provided by robot_models.
This module selects the task components relevant to control.
"""

import numpy as np
from numpy.typing import NDArray
from robot_models.task_space.pose_error import PoseError

Array = NDArray[np.float64]


def _validate_array(
    value: Array,
    shape: tuple,
    name: str,
) -> Array:
    """Validate and copy a finite numerical array."""
    result = np.array(value, dtype=np.float64, copy=True)

    if result.shape != shape:
        raise ValueError(
            f"{name} must have shape {shape}, got {result.shape}"
        )

    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain finite values")

    return result


class TaskSpaceError:
    """Compute and select components of a spatial pose error.

    The complete pose error uses the convention:

        e = [position_error, rotation_error]

    Both components are expressed in the world frame.

    A task selection matrix S maps the complete error to
    the selected task:

        e_selected = S @ e

    The same selection matrix can later be applied to the
    whole-body Jacobian:

        J_selected = S @ J
    """

    SPATIAL_DIMENSION = 6

    def __init__(self, selection_matrix: Array) -> None:
        """Configure a task-space selection matrix."""
        selection = np.array(
            selection_matrix,
            dtype=np.float64,
            copy=True,
        )

        if selection.ndim != 2:
            raise ValueError("selection_matrix must be 2D")

        if selection.shape[1] != self.SPATIAL_DIMENSION:
            raise ValueError(
                "selection_matrix must have 6 columns"
            )

        if selection.shape[0] == 0:
            raise ValueError(
                "selection_matrix cannot have zero rows"
            )

        if not np.all(np.isfinite(selection)):
            raise ValueError(
                "selection_matrix must contain finite values"
            )

        self._selection_matrix = selection

    @classmethod
    def full_pose(cls) -> "TaskSpaceError":
        """Create a six-dimensional pose tracking task."""
        return cls(np.eye(cls.SPATIAL_DIMENSION))

    @classmethod
    def position_only(cls) -> "TaskSpaceError":
        """Create a three-dimensional position tracking task."""
        return cls(
            np.hstack(
                (
                    np.eye(3),
                    np.zeros((3, 3)),
                )
            )
        )

    @property
    def selection_matrix(self) -> Array:
        """Return an independent copy of the task selection."""
        return self._selection_matrix.copy()

    def compute(
        self,
        current_pose: Array,
        desired_pose: Array,
    ) -> Array:
        """Compute the selected task-space pose error."""
        current = _validate_array(
            current_pose,
            (4, 4),
            "current_pose",
        )

        desired = _validate_array(
            desired_pose,
            (4, 4),
            "desired_pose",
        )

        full_error = PoseError.compute(
            transform_world_frame=current,
            transform_world_frame_desired=desired,
        )

        return self._selection_matrix @ full_error

    def select_jacobian(self, jacobian: Array) -> Array:
        """Project a spatial Jacobian into the selected task."""
        matrix = np.asarray(jacobian, dtype=np.float64)

        if matrix.ndim != 2 or matrix.shape[0] != 6:
            raise ValueError(
                "jacobian must have shape (6, n)"
            )

        if matrix.shape[1] == 0:
            raise ValueError(
                "jacobian must contain at least one column"
            )

        if not np.all(np.isfinite(matrix)):
            raise ValueError(
                "jacobian must contain finite values"
            )

        return self._selection_matrix @ matrix
