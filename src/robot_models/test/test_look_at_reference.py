"""Tests for generic look-at orientation references."""

import numpy as np

from robot_models.reference.look_at import (
    LookAtReference,
)


def test_look_at_with_positive_z_forward() -> None:
    """Verify +Z local axis points toward the target."""
    frame_position_world = np.array([
        0.0,
        0.0,
        0.0,
    ])

    target_position_world = np.array([
        1.0,
        0.0,
        0.0,
    ])

    rotation_world_frame = (
        LookAtReference.compute(
            frame_position_world=frame_position_world,
            target_position_world=target_position_world,
            forward_axis_local=np.array([
                0.0,
                0.0,
                1.0,
            ]),
            up_axis_local=np.array([
                0.0,
                -1.0,
                0.0,
            ]),
        )
    )

    forward_world = (
        rotation_world_frame
        @ np.array([
            0.0,
            0.0,
            1.0,
        ])
    )

    expected_direction = np.array([
        1.0,
        0.0,
        0.0,
    ])

    assert np.allclose(
        forward_world,
        expected_direction,
    )


def test_look_at_with_positive_x_forward() -> None:
    """Verify +X local axis can be used as forward."""
    frame_position_world = np.array([
        0.0,
        0.0,
        0.0,
    ])

    target_position_world = np.array([
        0.0,
        1.0,
        0.0,
    ])

    rotation_world_frame = (
        LookAtReference.compute(
            frame_position_world=frame_position_world,
            target_position_world=target_position_world,
            forward_axis_local=np.array([
                1.0,
                0.0,
                0.0,
            ]),
            up_axis_local=np.array([
                0.0,
                0.0,
                1.0,
            ]),
        )
    )

    forward_world = (
        rotation_world_frame
        @ np.array([
            1.0,
            0.0,
            0.0,
        ])
    )

    expected_direction = np.array([
        0.0,
        1.0,
        0.0,
    ])

    assert np.allclose(
        forward_world,
        expected_direction,
    )
