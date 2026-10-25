
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


def test_causal_tf_read_does_not_use_latest_future_sample():
    provider,buffer=make_provider(max_age=.2,future_tolerance=0.)
    provider._clock.now.return_value.nanoseconds=1_000_000_000
    def transform(time,x):
        value=Mock()
        value.header.stamp.sec=int(time)
        value.header.stamp.nanosec=round((time-int(time))*1e9)
        value.transform.translation.x=x
        value.transform.translation.y=0.
        value.transform.translation.z=0.
        return value
    buffer.lookup_transform.return_value=transform(1.,1.)
    np.testing.assert_equal(provider.position_at(1.),[1.,0.,0.])
    provider._clock.now.return_value.nanoseconds=1_100_000_000
    buffer.lookup_transform.return_value=transform(1.1,2.)
    np.testing.assert_equal(provider.position_at(1.05),[1.,0.,0.])
    provider.clear_history()
    assert provider.position_at(1.05) is None


def test_direct_tf_history_retains_causal_samples_while_pose_request_is_pending():
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    provider,_=make_provider(max_age=.2,future_tolerance=0.,history_duration=2.)
    provider._clock.now.return_value.nanoseconds=1_200_000_000
    def sample(t,x,frame='world'):
        value=TransformStamped();value.header.stamp.sec=1
        value.header.stamp.nanosec=round((t-1.)*1e9)
        value.header.frame_id=frame;value.child_frame_id='tracking_frame'
        value.transform.translation.x=x
        return value
    provider._receive_direct(TFMessage(transforms=[sample(1.,1.),sample(1.1,2.),
        sample(1.15,99.,'odom'),sample(1.3,100.)]))
    np.testing.assert_equal(provider.position_at(1.05,refresh=False),[1.,0.,0.])
    np.testing.assert_equal(provider.position_at(1.15,refresh=False),[2.,0.,0.])
    provider.clear_history()
    provider._clock.now.return_value.nanoseconds=1_000_000_000
    provider._receive_direct(TFMessage(transforms=[sample(1.3,100.),sample(1.,3.)]))
    np.testing.assert_equal(provider.position_at(1.,refresh=False),[3.,0.,0.])


def test_direct_tf_ahead_of_clock_waits_for_causal_integration_time():
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    provider,_=make_provider(max_age=.2,future_tolerance=0.,history_duration=1.)
    provider._clock.now.return_value.nanoseconds=1_000_000_000
    value=TransformStamped();value.header.frame_id='world';value.child_frame_id='tracking_frame'
    value.header.stamp.sec=1;value.header.stamp.nanosec=10_000_000
    value.transform.translation.x=4.
    provider._receive_direct(TFMessage(transforms=[value]))
    assert provider.position_at(1.,refresh=False) is None
    np.testing.assert_equal(provider.position_at(1.01,refresh=False),[4.,0.,0.])
