"""ROS boundary checks using real messages and an offline TF/transport fixture."""
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from robot_interfaces.msg import PredictedTrajectory, StateEstimate
from sensor_msgs.msg import JointState
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import TransformException
from visualization_msgs.msg import Marker

from robot_bringup.nodes.tracking_visualization import TrackingVisualization


class Output:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def sendTransform(self, message):
        self.messages.append(message)


class SceneInput:
    def __init__(self):
        self.received = []

    def receive(self, key, stamp, frame, data, now):
        self.received.append((key, stamp, frame, data, now))
        return True

    def snapshot(self, now):
        return self.output

    def reset(self):
        self.received.clear()

    def invalidate(self, key, stamp):
        pass


class StaticTF:
    def __init__(self):
        self.requests = []
        self.unavailable = False

    def lookup_transform(self, target, source, stamp):
        self.requests.append((target, source, stamp.nanoseconds))
        if self.unavailable:
            raise TransformException('missing fixture TF')
        result = TransformStamped()
        result.transform.translation.x = 10.
        result.transform.rotation.z = np.sqrt(.5)
        result.transform.rotation.w = np.sqrt(.5)
        return result


@pytest.fixture
def adapter():
    node = TrackingVisualization.__new__(TrackingVisualization)
    node._configuration = {'frame_id': 'world', 'model_frame_prefix': 'visual/',
                           'lifetime': .3, 'future_tolerance': .01}
    node._frame = 'world'
    node._reference = 'world'
    node._odometry_frame = 'odom'
    node._base_frame = 'base_link'
    node._names = ['a', 'b']
    node._scene = SceneInput()
    node._buffer = StaticTF()
    node._broadcaster = Output()
    node._publisher = Output()
    node._path_publishers = {name: Output() for name in [
        'target_truth', 'target_estimate', 'target_prediction', 'robot_truth', 'robot_estimate']}
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=10.))
    return node


def stamped(message, frame='odom', seconds=9.):
    message.header.frame_id = frame
    message.header.stamp = Time(seconds=seconds).to_msg()
    return message


def test_target_transform_uses_source_time_and_real_position(adapter):
    message = stamped(PointStamped())
    message.point.x, message.point.y, message.point.z = 1., 2., 3.
    assert adapter._receive('target_truth', message)
    key, stamp, frame, data, now = adapter._scene.received[-1]
    assert (key, stamp, frame, now) == ('target_truth', 9., 'world', 10.)
    np.testing.assert_allclose(data['position'], [8., 1., 3.])
    assert adapter._buffer.requests == [('world', 'odom', 9000000000)]


def test_forecast_preserves_future_epochs_but_transforms_at_issuance(adapter):
    message = stamped(PredictedTrajectory())
    message.horizon, message.state_dimension = 2, 6
    message.timestamps = [11.125, 12.25]
    message.states = [1., 2., 3., 4., 5., 6., 2., 3., 4., 5., 6., 7.]
    assert adapter._receive('prediction', message)
    _, stamp, _, data, _ = adapter._scene.received[-1]
    assert stamp == 9.
    assert data['timestamps'] == [11.125, 12.25]
    np.testing.assert_allclose(data['states'], [
        [8., 1., 3., -5., 4., 6.], [7., 2., 4., -6., 5., 7.]])
    assert adapter._buffer.requests == [('world', 'odom', 9000000000)]


@pytest.mark.parametrize('problem', ['empty_frame', 'missing_tf', 'nonfinite', 'bad_dimension',
                                     'bad_horizon', 'moving_forecast_frame'])
def test_invalid_input_does_not_reach_scene_or_raise(adapter, problem):
    message = stamped(PredictedTrajectory())
    message.horizon, message.state_dimension = 1, 6
    message.timestamps = [11.]
    message.states = [1., 2., 3., 4., 5., 6.]
    if problem == 'empty_frame':
        message.header.frame_id = ''
    elif problem == 'missing_tf':
        adapter._buffer.unavailable = True
    elif problem == 'nonfinite':
        message.states[0] = float('nan')
    elif problem == 'bad_dimension':
        message.state_dimension = 3
    elif problem == 'bad_horizon':
        message.horizon = 2
    else:
        message.header.frame_id = 'base_link'
    assert adapter._receive('prediction', message) is False
    assert not adapter._scene.received


def test_robot_estimate_uses_stamped_estimator_pose(adapter):
    message = stamped(StateEstimate())
    message.mean = [1., 2., 0., .1, .2, .3, .4, .5, .6]
    assert adapter._receive('robot_estimate', message)
    data = adapter._scene.received[-1][3]
    np.testing.assert_allclose(data['position'], [8., 1., 0.])
    np.testing.assert_allclose(Rotation.from_quat(data['orientation']).as_matrix(),
                               [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]], atol=1e-14)
    assert set(data) == {'position', 'orientation'}
    assert not adapter._broadcaster.messages


def test_only_accepted_physical_pose_broadcasts_prefixed_model_tf(adapter):
    message = stamped(Odometry(), seconds=9.5)
    message.pose.pose.position.x = 1.
    message.pose.pose.orientation.w = 1.
    assert adapter._receive('robot_truth', message)
    transform = adapter._broadcaster.messages[-1]
    assert transform.header.frame_id == 'world'
    assert transform.child_frame_id == 'visual/base_link'
    assert transform.header.stamp == message.header.stamp
    assert transform.transform.translation.x == pytest.approx(10.)
    assert transform.transform.translation.y == pytest.approx(1.)
    adapter._scene.receive = lambda *args: False
    assert adapter._receive('robot_truth', message) is False
    assert len(adapter._broadcaster.messages) == 1


def test_publish_replaces_markers_and_clears_all_empty_paths(adapter):
    adapter._scene.output = {
        'frame_id': 'world',
        'paths': {'target_truth': [], 'target_estimate': [], 'robot_truth': [],
                  'robot_estimate': [], 'target_prediction': [
                      {'stamp': 11.125, 'position': [1., 2., 3.]},
                      {'stamp': 12.25, 'position': [4., 5., 6.]}]},
        'markers': [{'ns': 'physical_camera', 'id': 0, 'kind': 'arrow',
                     'points': [[1., 2., 3.], [4., 5., 6.]], 'color': [0., 1., 1., 1.],
                     'scale': [.01, .02, .02], 'stamp': 9.5}]}
    adapter._publish()
    markers = adapter._publisher.messages[-1].markers
    assert markers[0].action == Marker.DELETEALL
    assert markers[1].action == Marker.ADD
    assert markers[1].header.frame_id == 'world'
    assert markers[1].header.stamp == Time(seconds=9.5).to_msg()
    assert (markers[1].ns, markers[1].id) == ('physical_camera', 0)
    for name, publisher in adapter._path_publishers.items():
        path = publisher.messages[-1]
        assert path.header.frame_id == 'world'
        assert path.header.stamp == Time(seconds=10.).to_msg()
        if name == 'target_prediction':
            assert [pose.header.stamp for pose in path.poses] == [
                Time(seconds=11.125).to_msg(), Time(seconds=12.25).to_msg()]
            assert all(pose.header.frame_id == 'world' for pose in path.poses)
        else:
            assert path.poses == []


def test_joint_input_keeps_names_and_physical_velocity(adapter):
    message = stamped(JointState(), frame='')
    message.name = ['b', 'a']
    message.position = [.2, .1]
    message.velocity = [.4, .3]
    assert adapter._receive('joints', message)
    data = adapter._scene.received[-1][3]
    assert data == {'names': ['b', 'a'], 'position': [.2, .1], 'velocity': [.4, .3]}


def test_real_scene_receives_estimated_state_and_resets_published_paths(adapter):
    """Exercise strict pure-engine contracts so boundary doubles cannot hide schema drift."""
    from robot_bringup.factory.model_factory import create_robot_model
    from robot_bringup.visualization import Scene, validate_configuration
    from robot_description.model_parameters import load_robot_model_parameters

    parameters = load_robot_model_parameters()
    model = create_robot_model(parameters)
    adapter._names = [f'joint_{index}' for index in range(model.manipulator.dof)]
    cfg = validate_configuration({'layers': {'robot_estimate': True}})
    adapter._configuration = cfg
    adapter._scene = Scene(cfg, model, adapter._names, parameters.joint_limits, [0., 0., 1.], .8)
    estimate = stamped(StateEstimate(), frame='world', seconds=9.9)
    estimate.mean = [1., 2., .3, .1, .2] + [0.] * (2 * model.manipulator.dof)
    assert adapter._receive('robot_estimate', estimate)
    adapter._publish()
    path = adapter._path_publishers['robot_estimate'].messages[-1]
    assert len(path.poses) == 1
    assert path.poses[0].pose.position.x == 1.
    assert path.poses[0].pose.position.y == 2.
    assert not any(marker.ns.startswith('camera')
                   for marker in adapter._publisher.messages[-1].markers)
    adapter._scene.reset()
    adapter._publish()
    assert adapter._path_publishers['robot_estimate'].messages[-1].poses == []
    assert [marker.action for marker in adapter._publisher.messages[-1].markers] == [Marker.DELETEALL]


def test_real_scene_physical_camera_publishes_finite_geometry(adapter):
    from robot_bringup.factory.model_factory import create_robot_model
    from robot_bringup.visualization import Scene, validate_configuration
    from robot_description.model_parameters import load_robot_model_parameters

    parameters = load_robot_model_parameters()
    model = create_robot_model(parameters)
    adapter._names = [f'joint_{index}' for index in range(model.manipulator.dof)]
    adapter._configuration = validate_configuration({})
    adapter._scene = Scene(adapter._configuration, model, adapter._names,
                           parameters.joint_limits, [0., 0., 1.], .8)
    base = stamped(Odometry(), frame='world', seconds=9.95)
    base.pose.pose.orientation.w = 1.
    joints = stamped(JointState(), seconds=9.95)
    joints.name = adapter._names
    joints.position = ((parameters.joint_limits.lower + parameters.joint_limits.upper) / 2).tolist()
    joints.velocity = [0.] * model.manipulator.dof
    target = stamped(PointStamped(), frame='world', seconds=9.95)
    target.point.x, target.point.z = 1., 1.
    for key, message in [('robot_truth', base), ('joints', joints), ('target_truth', target)]:
        assert adapter._receive(key, message)
    adapter._publish()
    markers = adapter._publisher.messages[-1].markers
    assert {'camera_optical', 'line_of_sight', 'tracking_radius', 'joint_limits'} <= {
        marker.ns for marker in markers}
    assert len({(marker.ns, marker.id) for marker in markers}) == len(markers)
    for marker in markers:
        assert all(np.isfinite([point.x, point.y, point.z]).all() for point in marker.points)
        assert marker.header.frame_id == 'world'


def test_physical_quaternion_is_not_silently_normalized(adapter):
    physical = stamped(Odometry(), frame='world', seconds=9.95)
    physical.pose.pose.orientation.w = 2.
    assert adapter._receive('robot_truth', physical) is False
    assert not adapter._scene.received
    assert not adapter._broadcaster.messages


def test_target_estimate_uses_declared_six_state_contract(adapter):
    estimate = stamped(StateEstimate(), seconds=9.95)
    estimate.mean = [1., 2., 3., .1, .2, .3]
    assert adapter._receive('target_estimate', estimate)
    np.testing.assert_allclose(adapter._scene.received[-1][3]['position'], [8., 1., 3.])
    estimate.mean = [1., 2., 3.]
    assert adapter._receive('target_estimate', estimate) is False
    assert len(adapter._scene.received) == 1


@pytest.fixture
def configured_node(monkeypatch, tmp_path):
    """Replace DDS and timers only, preserving constructor, messages, and real Scene."""
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    import robot_bringup.nodes.tracking_visualization as module
    from rclpy.node import Node

    configuration = tmp_path / 'visualization.yaml'
    configuration.write_text('{}\n')
    values = {'configuration_file': str(configuration),
              'description_file': str(Path(get_package_share_directory('robot_description')) /
                                      'config' / 'robot' / 'robot.yaml'),
              'joint_names': ['j0', 'j1', 'j2', 'j3'], 'joint_topic': '/physical/joints',
              'odom_topic': '/wheel/odom', 'physical_pose_topic': '/physical/pose',
              'reference_frame': 'world', 'odometry_frame': 'odom', 'base_frame': 'base_link',
              'target_estimate_topic': '/estimate/target', 'robot_estimate_topic': '/estimate/robot',
              'prediction_topic': '/forecast/target', 'forward_axis': [0., 0., 1.],
              'tracking_radius': .8}
    subscriptions, publishers, jumps = {}, {}, []
    monkeypatch.setattr(Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, *args: None)
    monkeypatch.setattr(Node, 'get_parameter', lambda self, name: SimpleNamespace(value=values[name]))

    def publisher(self, kind, topic, qos):
        result = Output()
        result.kind, result.qos = kind, qos
        publishers[topic] = result
        return result

    def subscription(self, kind, topic, callback, qos):
        subscriptions[topic] = (kind, callback)

    clock = SimpleNamespace(now=lambda: Time(seconds=10.),
                            create_jump_callback=lambda threshold, post_callback: jumps.append(post_callback))
    monkeypatch.setattr(Node, 'create_publisher', publisher)
    monkeypatch.setattr(Node, 'create_subscription', subscription)
    monkeypatch.setattr(Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(Node, 'create_timer', lambda self, period, callback: (period, callback))
    monkeypatch.setattr(module, 'Buffer', lambda node: StaticTF())
    monkeypatch.setattr(module, 'TransformListener', lambda *args: None)
    monkeypatch.setattr(module, 'TransformBroadcaster', lambda node: Output())
    return values, subscriptions, publishers, jumps, module


def test_constructor_binds_owner_topics_read_only_and_clock_reset_clears_scene(configured_node):
    values, subscriptions, publishers, jumps, module = configured_node
    node = module.TrackingVisualization()
    assert set(subscriptions) == {'/physical/joints', '/physical/pose',
                                  '/simulation/target_position', '/estimate/target',
                                  '/estimate/robot', '/forecast/target'}
    assert set(publishers) == {'/tracking/markers', *[
        '/tracking/paths/' + name for name in node.PATH_LAYERS]}
    assert subscriptions['/estimate/target'][0] is StateEstimate
    assert subscriptions['/forecast/target'][0] is PredictedTrajectory
    target = stamped(PointStamped(), frame='world', seconds=9.95)
    target.point.x = 1.
    subscriptions['/simulation/target_position'][1](target)
    node._publish()
    assert len(publishers['/tracking/paths/target_truth'].messages[-1].poses) == 1
    jumps[0](None)
    node._publish()
    assert publishers['/tracking/paths/target_truth'].messages[-1].poses == []
    assert [marker.action for marker in publishers['/tracking/markers'].messages[-1].markers] == [
        Marker.DELETEALL]


def test_startup_rejects_a_moving_display_frame(configured_node):
    values, _, _, _, module = configured_node
    from pathlib import Path
    Path(values['configuration_file']).write_text('frame_id: base_link\n')
    with pytest.raises(ValueError, match='fixed reference_frame or odometry_frame'):
        module.TrackingVisualization()


@pytest.mark.parametrize('problem', ['nonfinite', 'malformed', 'missing_tf'])
def test_adapter_rejections_break_real_scene_trails(configured_node, problem):
    _, _, _, _, module = configured_node
    node = module.TrackingVisualization()
    first = stamped(StateEstimate(), frame='world', seconds=9.9)
    first.mean = [1., 2., 3., 0., 0., 0.]
    assert node._receive('target_estimate', first)
    invalid = stamped(StateEstimate(), frame='world', seconds=9.95)
    invalid.mean = first.mean
    if problem == 'nonfinite':
        invalid.mean[0] = float('nan')
    elif problem == 'malformed':
        invalid.mean = [1., 2., 3.]
    else:
        invalid.header.frame_id = 'odom'
        node._buffer.unavailable = True
    assert not node._receive('target_estimate', invalid)
    node._buffer.unavailable = False
    last = stamped(StateEstimate(), frame='world', seconds=10.)
    last.mean = [1.1, 2., 3., 0., 0., 0.]
    assert node._receive('target_estimate', last)
    assert node._scene.snapshot(10.)['paths']['target_estimate'] == [
        {'stamp': 10., 'position': [1.1, 2., 3.]}]
