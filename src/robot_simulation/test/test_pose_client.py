
"""Unit tests for the Gazebo ROS pose client."""

from concurrent.futures import Future
from unittest.mock import Mock

import numpy as np
import pytest

from ros_gz_interfaces.msg import Entity

from robot_simulation.gazebo.pose_client import GazeboPoseClient
from robot_simulation.gazebo.target_adapter import TargetPose


def make_client():
    """Construct a pose client with a mocked ROS node."""
    node = Mock()
    ros_client = Mock()
    ros_client.service_is_ready.return_value = True
    ros_client.call_async.return_value = Future()
    node.create_client.return_value = ros_client

    client = GazeboPoseClient(
        node=node,
        service_name="/world/test_world/set_pose",
        entity_name="test_target",
    )

    return client, node, ros_client


def test_client_uses_configured_service():
    """The ROS service name must be externally configurable."""
    _, node, _ = make_client()

    assert node.create_client.call_args.args[1] == (
        "/world/test_world/set_pose"
    )


def test_pose_request():
    """A 3D pose must map correctly into the ROS request."""
    client, _, ros_client = make_client()

    pose = TargetPose(
        position=np.array([1.0, 2.0, 3.0]),
        orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    client.send_pose(pose)

    request = ros_client.call_async.call_args.args[0]

    assert request.entity.name == "test_target"
    assert request.entity.type == Entity.MODEL

    assert request.pose.position.x == pytest.approx(1.0)
    assert request.pose.position.y == pytest.approx(2.0)
    assert request.pose.position.z == pytest.approx(3.0)
    assert request.pose.orientation.w == pytest.approx(1.0)


def test_service_unavailable():
    """Sending must fail clearly if the service is unavailable."""
    client, _, ros_client = make_client()
    ros_client.service_is_ready.return_value = False

    with pytest.raises(RuntimeError, match="not ready"):
        client.send_pose(
            TargetPose(
                position=np.zeros(3),
                orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            )
        )


def test_invalid_pose():
    """Invalid position dimensions must be rejected."""
    client, _, _ = make_client()

    with pytest.raises(ValueError):
        client.send_pose(
            TargetPose(
                position=np.zeros(2),
                orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            )
        )
