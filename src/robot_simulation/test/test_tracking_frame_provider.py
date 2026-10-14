
"""Tests for tracking-frame position extraction."""

from unittest.mock import Mock, patch

import numpy as np
import pytest

from tf2_ros import LookupException

from robot_simulation.gazebo.tracking_frame_provider import (
    TrackingFrameProvider,
)


def make_provider(**parameters):
    """Create a provider without a running ROS graph."""
    with (
        patch(
            "robot_simulation.gazebo.tracking_frame_provider.Buffer"
        ) as buffer_class,
        patch(
            "robot_simulation.gazebo.tracking_frame_provider."
            "TransformListener"
        ),
    ):
        buffer = buffer_class.return_value

        provider = TrackingFrameProvider(
            node=Mock(),
            reference_frame="world",
            tracking_frame="tracking_frame",
            **parameters,
        )

    return provider, buffer


def test_extracts_3d_position():
    """Read the position vector from a TF transform."""
    provider, buffer = make_provider()

    transform = Mock()
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = -2.0
    transform.transform.translation.z = 3.0

    buffer.lookup_transform.return_value = transform

    position = provider.position_world()

    np.testing.assert_allclose(
        position,
        [1.0, -2.0, 3.0],
    )

    assert buffer.lookup_transform.call_args.args[:2] == (
        "world",
        "tracking_frame",
    )


def test_missing_transform():
    """Return None until the required TF becomes available."""
    provider, buffer = make_provider()

    buffer.lookup_transform.side_effect = LookupException(
        "Tracking frame not available"
    )

    assert provider.position_world() is None


def test_rejects_invalid_frame_names():
    """Frame identifiers must be explicitly configured."""
    with pytest.raises(ValueError):
        TrackingFrameProvider(
            node=Mock(),
            reference_frame="",
            tracking_frame="tracking_frame",
        )


def test_rejects_non_finite_position():
    """Invalid TF translation values must be rejected."""
    provider, buffer = make_provider()

    transform = Mock()
    transform.transform.translation.x = np.nan
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0

    buffer.lookup_transform.return_value = transform

    with pytest.raises(ValueError, match="non-finite"):
        provider.position_world()


@pytest.mark.parametrize('timestamp,available', [(9.9, True), (9.7, False), (10.1, False)])
def test_reactive_tracking_rejects_stale_and_future_transforms(timestamp, available):
    """A stopped broadcaster must not silently leave a reactive target using old FK."""
    provider, buffer = make_provider(max_age=.2, future_tolerance=.01)
    provider._clock.now.return_value.nanoseconds = 10_000_000_000
    transform = Mock()
    transform.header.stamp.sec = int(timestamp)
    transform.header.stamp.nanosec = round((timestamp - int(timestamp)) * 1e9)
    transform.transform.translation.x = 1.
    transform.transform.translation.y = 2.
    transform.transform.translation.z = 3.
    buffer.lookup_transform.return_value = transform
    assert (provider.position_world() is not None) == available
