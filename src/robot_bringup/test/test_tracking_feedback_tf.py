"""Verify measured feedback FK and named-joint mapping without a ROS graph."""

import importlib
from types import SimpleNamespace

from nav_msgs.msg import Odometry
import numpy as np
from rclpy.time import Time
from robot_bringup.factory.model_factory import create_robot_model
from sensor_msgs.msg import JointState


def test_tracking_tf_uses_measured_named_joints_and_one_mount():
    """Reordering wheels/joints cannot replace measured motion with references."""
    module = 'robot_bringup.nodes.tracking_feedback_tf'
    assert importlib.util.find_spec(module), 'Measured tracking TF is missing'
    model = create_robot_model()
    names = ['base_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint']
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.
    odometry.pose.pose.position.x = 1.
    joints = JointState()
    joints.name = ['wheel', *names[::-1]]
    joints.position = [10., 0., 0., 0., 0.]
    transform = importlib.import_module(module).tracking_pose(model, odometry, joints, names)
    assert np.allclose(transform[:3, 3], [1.88, 0., .18])


def test_measured_tf_does_not_regress_a_sensor_timestamp():
    """Out-of-order packets cannot overwrite newer FK input within an epoch."""
    module = importlib.import_module('robot_bringup.nodes.tracking_feedback_tf')
    receiver = SimpleNamespace(_latest={}, _future_tolerance=.01,
                               get_clock=lambda: SimpleNamespace(
                                   now=lambda: Time(seconds=10.)))
    newest, old = JointState(), JointState()
    newest.header.stamp = Time(seconds=10.).to_msg()
    old.header.stamp = Time(seconds=9.).to_msg()
    module.TrackingFeedbackTF._receive(receiver, 'joints', newest)
    module.TrackingFeedbackTF._receive(receiver, 'joints', old)
    assert receiver._latest['joints'] is newest
    receiver._latest.clear()
    module.TrackingFeedbackTF._receive(receiver, 'joints', old)
    assert receiver._latest['joints'] is old


def test_future_feedback_cannot_poison_tracking_tf_after_reset():
    """A former epoch's queued packet must not block lower-time fresh feedback."""
    module = importlib.import_module('robot_bringup.nodes.tracking_feedback_tf')
    receiver = SimpleNamespace(_latest={}, _future_tolerance=.01,
                               get_clock=lambda: SimpleNamespace(
                                   now=lambda: Time(seconds=1.)))
    future, fresh = JointState(), JointState()
    future.header.stamp = Time(seconds=100.).to_msg()
    fresh.header.stamp = Time(seconds=1.).to_msg()
    module.TrackingFeedbackTF._receive(receiver, 'joints', future)
    module.TrackingFeedbackTF._receive(receiver, 'joints', fresh)
    assert receiver._latest['joints'] is fresh
