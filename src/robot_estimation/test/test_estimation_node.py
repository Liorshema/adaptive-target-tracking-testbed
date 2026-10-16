"""Regress sensor transport ordering with the real estimation filters."""

from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from rclpy.time import Time
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
import yaml

from robot_estimation.nodes.estimation_node import EstimationNode


@pytest.fixture
def node():
    """Provide local parameters and publication sinks without DDS."""
    estimate = object.__new__(EstimationNode)
    parameters = {}
    for name in ['robot_estimator', 'target_estimator']:
        path = Path(__file__).parents[1] / 'config' / f'{name}.yaml'
        parameters.update(yaml.safe_load(path.read_text())['estimation_node']['ros__parameters'])
    parameters.setdefault('joint_names', [
        'base_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint',
    ])
    parameters.setdefault('future_tolerance', 0.01)
    parameters.setdefault('max_wheel_odometry_age', 0.2)
    parameters.setdefault('max_joint_state_age', 0.2)
    parameters['reorder_holdback'] = 0.0  # Legacy immediate-fusion cases.
    estimate.test_parameters = parameters
    estimate.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    estimate.test_now = 10.05
    estimate.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=estimate.test_now))
    estimate.logs = []

    def record(message, **kwargs):
        estimate.logs.append(message)

    estimate.get_logger = lambda: SimpleNamespace(warning=record, error=record, info=record)
    estimate.robot_messages = []
    estimate.target_messages = []
    estimate._robot_state_publisher = SimpleNamespace(publish=lambda message: None)
    estimate._target_state_publisher = SimpleNamespace(publish=lambda message: None)
    estimate._robot_state_full_publisher = SimpleNamespace(publish=estimate.robot_messages.append)
    estimate._target_state_full_publisher = SimpleNamespace(publish=estimate.target_messages.append)
    estimate._robot_estimator = estimate._create_robot_estimator()
    estimate._target_estimator = estimate._create_target_estimator()
    estimate._last_robot_time = None
    estimate._last_target_time = None
    estimate._target_frame_id = ''
    estimate._last_wheel_time = None
    estimate._last_joint_time = None
    estimate._configure_robot_timing()
    return estimate


def wheel_message(timestamp=10.0, velocity=0.0):
    """Build a wheel velocity observation."""
    message = Odometry()
    message.header.stamp = Time(seconds=timestamp).to_msg()
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_link'
    message.twist.twist.linear.x = velocity
    return message


def test_full_state_publication_budget_preserves_fusion_without_duplicate_output(node):
    """Fuse equal-time measurements while keeping the full-message rate bounded."""
    node._full_state_period = .02
    node._last_full_publish_time = None
    node._wheel_odometry_callback(wheel_message())
    node._joint_state_callback(joint_message())
    initial = len(node.robot_messages)
    node._joint_state_callback(joint_message(10.005))
    assert len(node.robot_messages) == initial
    node._joint_state_callback(joint_message(10.02))
    assert len(node.robot_messages) == initial + 1
    node._wheel_odometry_callback(wheel_message(10.02, .2))
    assert len(node.robot_messages) == initial + 1
    assert node._robot_estimator.state.mean[3] > 0
    for _ in range(100):
        node._joint_state_callback(joint_message(10.02))
    assert len(node.robot_messages) == initial + 1


def test_rejected_time_counts_are_kept_without_per_packet_logging(node):
    """Keep rejection evidence while avoiding repeated ROS throttling introspection."""
    for _ in range(10):
        assert not node._measurement_time_valid(9., 10., 'robot')
    assert len(node.logs) == 1
    assert node._time_rejections == 10


def joint_message(timestamp=10.0):
    """Include shuffled arm joints and a wheel as Gazebo can publish."""
    message = JointState()
    message.header.stamp = Time(seconds=timestamp).to_msg()
    message.name = ['elbow_joint', 'front_left_wheel_joint', 'base_yaw_joint',
                    'wrist_joint', 'shoulder_joint']
    message.position = [0.3, 5.0, 0.1, 0.4, 0.2]
    message.velocity = [0.03, 10.0, 0.01, 0.04, 0.02]
    return message


def test_named_arm_joints_ignore_wheels_and_preserve_model_order(node):
    """Fuse the configured arm joints instead of trusting topic array order."""
    node._wheel_odometry_callback(wheel_message())
    node._joint_state_callback(joint_message())
    assert len(node.robot_messages) == 1
    np.testing.assert_allclose(node._robot_estimator.state.mean[5:9],
                               [0.0980392157, 0.1960784314, 0.2941176471, 0.3921568627])
    np.testing.assert_allclose(node._robot_estimator.state.mean[9:],
                               [0.0098039216, 0.0196078431, 0.0294117647, 0.0392156863])


def test_equal_time_wheel_and_joint_updates_both_reach_estimate(node):
    """Same-clock sensor fusion preserves the second sensor's information."""
    node._wheel_odometry_callback(wheel_message())
    node._joint_state_callback(joint_message())
    assert len(node.robot_messages) == 1
    assert node._robot_estimator.state.timestamp == 10.0
    assert node._robot_estimator.state.mean[6] > 0.19


def test_older_measurement_does_not_roll_filter_backward(node):
    """Reject an older sensor before mutating the filter or publication."""
    node._wheel_odometry_callback(wheel_message(10.0, 0.2))
    previous = node._robot_estimator.state
    node._wheel_odometry_callback(wheel_message(9.0, -2.0))
    assert node._robot_estimator.state.timestamp == 10.0
    np.testing.assert_array_equal(node._robot_estimator.state.mean, previous.mean)
    assert not node.robot_messages


@pytest.mark.parametrize('malformed', ['missing', 'duplicate', 'velocity', 'nonfinite'])
def test_bad_joint_message_does_not_advance_filter(node, malformed):
    """Validate joint metadata and data before advancing filter time."""
    node._wheel_odometry_callback(wheel_message())
    previous = node._robot_estimator.state
    message = joint_message(10.04)
    if malformed == 'missing':
        message.name[0] = 'other_joint'
    elif malformed == 'duplicate':
        message.name[0] = 'shoulder_joint'
    elif malformed == 'velocity':
        message.velocity = []
    else:
        message.position[0] = np.nan
    node._joint_state_callback(message)
    assert node._last_robot_time == 10.0
    np.testing.assert_array_equal(node._robot_estimator.state.mean, previous.mean)
    np.testing.assert_array_equal(node._robot_estimator.state.covariance, previous.covariance)
    assert not node.robot_messages


def test_clock_reset_reinitializes_filters_for_new_epoch(node):
    """Rebuild filter and timing state before lower-time measurements arrive."""
    node._wheel_odometry_callback(wheel_message(10.0, 0.2))
    node.test_now = 1.05
    node._on_time_jump(None)
    assert node._last_robot_time is None
    assert node._last_target_time is None
    np.testing.assert_array_equal(node._robot_estimator.state.mean, np.zeros(13))
    node._wheel_odometry_callback(wheel_message(1.0, 0.0))
    assert node._robot_estimator.state.timestamp == 1.0


def test_future_sensor_does_not_poison_filter_time(node):
    """Reject a former epoch's queued sensor packet after a reset."""
    node._wheel_odometry_callback(wheel_message(100.0, 2.0))
    node._wheel_odometry_callback(wheel_message(10.0, 0.0))
    assert not node.robot_messages
    assert node._last_robot_time == 10.0


def test_older_target_observation_does_not_regress_timestamp(node):
    """Target freshness and prediction timestamps remain monotonic."""
    message = PointStamped()
    message.header.frame_id = 'world'
    message.header.stamp = Time(seconds=10.0).to_msg()
    message.point.x = 2.0
    node._target_measurement_callback(message)
    previous = node._target_estimator.state
    message.header.stamp = Time(seconds=9.0).to_msg()
    message.point.x = -2.0
    node._target_measurement_callback(message)
    assert node._target_estimator.state.timestamp == 10.0
    np.testing.assert_array_equal(node._target_estimator.state.mean, previous.mean)
    assert len(node.target_messages) == 1


@pytest.mark.parametrize('sensor', ['wheel', 'imu', 'target'])
def test_nonfinite_sensor_does_not_poison_estimator(node, sensor):
    """Reject invalid transport data before it contaminates future states."""
    if sensor == 'wheel':
        node._wheel_odometry_callback(wheel_message(velocity=np.nan))
    elif sensor == 'imu':
        message = Imu()
        message.header.stamp = Time(seconds=10.0).to_msg()
        message.angular_velocity.z = np.inf
        node._imu_callback(message)
    else:
        message = PointStamped()
        message.header.stamp = Time(seconds=10.0).to_msg()
        message.header.frame_id = 'world'
        message.point.x = np.nan
        node._target_measurement_callback(message)
    np.testing.assert_array_equal(node._robot_estimator.state.mean, np.zeros(13))
    np.testing.assert_array_equal(node._target_estimator.state.mean, np.zeros(6))
    assert node._last_robot_time is None
    assert node._last_target_time is None
    assert not node.robot_messages
    assert not node.target_messages


def test_full_robot_estimate_requires_wheel_and_joint_feedback(node):
    """Initial joint values cannot masquerade as measured robot configuration."""
    node._wheel_odometry_callback(wheel_message())
    assert not node.robot_messages
    node._joint_state_callback(joint_message())
    assert len(node.robot_messages) == 1


@pytest.mark.parametrize('stale_sensor', ['wheel', 'joints'])
def test_one_sensor_cannot_hide_missing_updates_from_another(node, stale_sensor):
    """Aggregate filter timestamps cannot refresh unobserved robot components."""
    node._wheel_odometry_callback(wheel_message())
    node._joint_state_callback(joint_message())
    assert len(node.robot_messages) >= 1
    count = len(node.robot_messages)
    node.test_now = 10.3
    if stale_sensor == 'joints':
        node._wheel_odometry_callback(wheel_message(10.3))
    else:
        node._joint_state_callback(joint_message(10.3))
    assert len(node.robot_messages) == count


def test_interleaved_streams_commit_at_measurement_time_behind_the_clock(node):
    """Wheel receipt must not force the joint update to the wrong epoch."""
    node.test_parameters['reorder_holdback'] = .03
    node._configure_robot_timing()
    node._wheel_odometry_callback(wheel_message(10.02))
    node._joint_state_callback(joint_message(10.01))
    assert node._last_robot_time is None
    node.test_now = 10.06
    node._drain_robot()
    assert node._last_robot_time == 10.02
    assert node._last_joint_time == 10.01
    assert len(node.robot_messages) == 1
    assert node._robot_buffer.counters['joints']['accepted'] == 1
    assert node.robot_messages[0].header.stamp == Time(seconds=10.02).to_msg()


def test_raw_physical_excursion_blocks_full_output_instead_of_being_hidden(node):
    """The high-level consumer must not receive a newly disguised legal state."""
    node._wheel_odometry_callback(wheel_message())
    message = joint_message()
    message.position[message.name.index('shoulder_joint')] = -.001
    node._joint_state_callback(message)
    assert not node.robot_messages
    assert not node._joint_constraint.physical_valid
    assert node._joint_constraint.excess[1] == .001


def test_malformed_stamp_cannot_advance_or_crash_filter(node):
    message = wheel_message()
    message.header.stamp.nanosec = 1_500_000_000
    node._wheel_odometry_callback(message)
    assert node._last_robot_time is None
    assert node._robot_buffer.counters['wheel']['invalid'] == 1


def test_wrong_odometry_frame_is_counted_without_filter_mutation(node):
    message = wheel_message()
    message.child_frame_id = 'wrong_body'
    node._wheel_odometry_callback(message)
    assert node._last_robot_time is None
    assert node._robot_buffer.counters['wheel']['invalid'] == 1


def test_optional_imu_with_explicit_body_frame_can_update_shared_state(node):
    node._wheel_odometry_callback(wheel_message())
    node._joint_state_callback(joint_message())
    message = Imu()
    message.header.frame_id = node.test_parameters['imu_frame']
    message.header.stamp = Time(seconds=10.).to_msg()
    message.angular_velocity.z = .2
    node._imu_callback(message)
    assert node._robot_estimator.state.mean[4] > .1


def test_stale_bad_joint_observation_does_not_poison_current_physical_status(node):
    node._wheel_odometry_callback(wheel_message())
    node._joint_state_callback(joint_message())
    message = joint_message(9.8)
    message.position[message.name.index('shoulder_joint')] = -.1
    node._joint_state_callback(message)
    assert node._raw_joint_valid
    assert node._joint_constraint.physical_valid
    assert node._robot_buffer.counters['joints']['stale'] == 1
