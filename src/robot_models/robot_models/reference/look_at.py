"""Look-at orientation generation for a generic tracking frame."""

import numpy as np

from robot_models.common.rotations import (
    is_rotation_matrix,
)


class LookAtReference:
    """Generate a desired frame orientation toward a target."""

    @staticmethod
    def compute(
        frame_position_world: np.ndarray,
        target_position_world: np.ndarray,
        forward_axis_local: np.ndarray = np.array(
            [0.0, 0.0, 1.0],
            dtype=float,
        ),
        up_axis_local: np.ndarray = np.array(
            [0.0, -1.0, 0.0],
            dtype=float,
        ),
        world_up: np.ndarray = np.array(
            [0.0, 0.0, 1.0],
            dtype=float,
        ),
    ) -> np.ndarray:
        """
        Return desired frame rotation R_W_F.

        The local forward axis is aligned with the direction from
        the frame position to the target.

        The local up axis is used to resolve rotation about the
        forward direction.
        """
        frame_position_world = np.asarray(
            frame_position_world,
            dtype=float,
        )

        target_position_world = np.asarray(
            target_position_world,
            dtype=float,
        )

        forward_axis_local = np.asarray(
            forward_axis_local,
            dtype=float,
        )

        up_axis_local = np.asarray(
            up_axis_local,
            dtype=float,
        )

        world_up = np.asarray(
            world_up,
            dtype=float,
        )

        for name, vector in (
            (
                'frame_position_world',
                frame_position_world,
            ),
            (
                'target_position_world',
                target_position_world,
            ),
            (
                'forward_axis_local',
                forward_axis_local,
            ),
            (
                'up_axis_local',
                up_axis_local,
            ),
            (
                'world_up',
                world_up,
            ),
        ):
            if vector.shape != (3,):
                raise ValueError(
                    f'{name} must have shape (3,)'
                )

        direction_world = (
            target_position_world
            - frame_position_world
        )

        direction_norm = np.linalg.norm(
            direction_world
        )

        if np.isclose(
            direction_norm,
            0.0,
        ):
            raise ValueError(
                'frame and target positions must be different'
            )

        forward_world = (
            direction_world
            / direction_norm
        )

        forward_local_norm = np.linalg.norm(
            forward_axis_local
        )

        up_local_norm = np.linalg.norm(
            up_axis_local
        )

        world_up_norm = np.linalg.norm(
            world_up
        )

        if np.isclose(
            forward_local_norm,
            0.0,
        ):
            raise ValueError(
                'forward_axis_local must be non-zero'
            )

        if np.isclose(
            up_local_norm,
            0.0,
        ):
            raise ValueError(
                'up_axis_local must be non-zero'
            )

        if np.isclose(
            world_up_norm,
            0.0,
        ):
            raise ValueError(
                'world_up must be non-zero'
            )

        forward_local = (
            forward_axis_local
            / forward_local_norm
        )

        up_local = (
            up_axis_local
            / up_local_norm
        )

        if not np.isclose(
            np.dot(
                forward_local,
                up_local,
            ),
            0.0,
        ):
            raise ValueError(
                'forward_axis_local and up_axis_local '
                'must be orthogonal'
            )

        up_world_reference = (
            world_up
            / world_up_norm
        )

        lateral_world = np.cross(
            forward_world,
            up_world_reference,
        )

        if np.isclose(
            np.linalg.norm(
                lateral_world
            ),
            0.0,
        ):
            fallback_up = np.array(
                [0.0, 1.0, 0.0],
                dtype=float,
            )

            lateral_world = np.cross(
                forward_world,
                fallback_up,
            )

        lateral_world /= np.linalg.norm(
            lateral_world
        )

        up_world = np.cross(
            lateral_world,
            forward_world,
        )

        up_world /= np.linalg.norm(
            up_world
        )

        lateral_local = np.cross(
            forward_local,
            up_local,
        )

        lateral_local /= np.linalg.norm(
            lateral_local
        )

        local_basis = np.column_stack(
            (
                lateral_local,
                up_local,
                forward_local,
            )
        )

        world_basis = np.column_stack(
            (
                lateral_world,
                up_world,
                forward_world,
            )
        )

        rotation_world_frame = (
            world_basis
            @ local_basis.T
        )

        if not is_rotation_matrix(
            rotation_world_frame
        ):
            raise RuntimeError(
                'failed to construct a valid rotation matrix'
            )

        return rotation_world_frame
