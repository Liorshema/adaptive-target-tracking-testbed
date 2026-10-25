"""Exercise ROS control boundaries with real messages, models and TF.

Node construction/DDS is reserved for the live smoke check. These tests
provide only a local clock and logger; all control mathematics is real.
"""

from pathlib import Path
from types import SimpleNamespace
from uuid import uuid4

from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
from rclpy.time import Time
from robot_interfaces.msg import PredictedTrajectory, StateEstimate
from tf2_ros import Buffer
import yaml

from robot_control.adapters.prediction_message_adapter import PredictionMessageAdapter
from robot_control.adapters.robot_state_message_adapter import RobotStateMessageAdapter
from robot_control.nodes.control_node import ControlNode
from robot_control.safety.input_status import InputStatusTracker
from robot_control.safety.input_validator import InputValidationConfig, InputValidator


def robot_message(timestamp=10.0, shoulder=0.0):
    """Build a complete 13-component stationary robot estimate."""
    message = StateEstimate()
    message.header.stamp = Time(seconds=timestamp).to_msg()
    message.header.frame_id = 'odom'
    mean = np.zeros(13)
    mean[6] = shoulder
    message.mean = mean.tolist()
    message.covariance = np.eye(13).ravel().tolist()
    return message


def prediction_message(timestamp=10.0):
    """Build a two-second world-frame prediction."""
    message = PredictedTrajectory()
    message.header.stamp = Time(seconds=timestamp).to_msg()
    message.header.frame_id = 'world'
    message.horizon = 20
    message.state_dimension = 6
    message.timestamps = (timestamp + 0.1 * np.arange(1, 21)).tolist()
    message.states = np.tile([2., 0., 1., 0., 0., 0.], (20, 1)).ravel().tolist()
    return message


@pytest.fixture
def node():
    """Supply local ROS boundaries without creating DDS resources."""
    control = object.__new__(ControlNode)
    config = Path(__file__).parents[1] / 'config' / 'control.yaml'
    parameters = yaml.safe_load(config.read_text())['control_node']['ros__parameters']
    # Exercise the existing implementation before shipped config is repaired.
    parameters.setdefault('base_velocity_lower', [-0.5, -1.0])
    parameters.setdefault('base_velocity_upper', [0.5, 1.0])
    control.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    control.test_now = 10.05
    control.get_clock = lambda: SimpleNamespace(
        now=lambda: Time(seconds=control.test_now),
    )
    control.logs = []

    def record(message, **kwargs):
        control.logs.append(message)

    control.get_logger = lambda: SimpleNamespace(
        info=record, warning=record, error=record, debug=record,
    )
    control._robot_adapter = RobotStateMessageAdapter(4)
    control._prediction_adapter = PredictionMessageAdapter()
    control._validator = InputValidator(InputValidationConfig(0.2, 0.5, 0.01))
    control._status_tracker = InputStatusTracker()
    control._tf_buffer = Buffer()
    transform = TransformStamped()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'world'
    transform.transform.rotation.w = 1.0
    control._tf_buffer.set_transform_static(transform, 'test')
    control._robot_state = None
    control._prediction = None
    control._prediction_reference_timestamp = None
    control._last_robot_timestamp = None
    control._last_prediction_timestamp = None
    control._last_compute_time = None
    control._last_output_log_time = None
    control._compute_enabled = True
    control._nominal_dt = 0.05
    control._pipeline = control._build_pipeline(4)
    control._command_publisher = None
    control._telemetry_publisher = None
    control._session_id = str(uuid4())
    control._sequence = 0
    control._command_names = ['base_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint']
    control._command_frame = 'base_link'
    return control


def test_opt_in_publishes_safe_output_and_stale_input_stops(node):
    """Publish the constrained result; invalidity immediately commands a stop."""
    commands = []
    node._command_publisher = SimpleNamespace(publish=commands.append)
    node._robot_callback(robot_message(shoulder=1.56))
    node._prediction_callback(prediction_message())
    node._validate_inputs()
    result = node._pipeline.compute(node._robot_state,
                                    node._prediction_in_robot_frame(node._prediction, 'odom', 10.),
                                    node.test_now, dt=.05)
    assert commands[-1].active
    assert np.allclose(commands[-1].velocity, result.safe_output.command)
    node.test_now = 11.
    node._validate_inputs()
    assert not commands[-1].active
    assert not any(commands[-1].velocity)


def test_reset_changes_command_session_and_requires_new_feedback(node):
    """A reset cannot republish a command from the previous clock epoch."""
    commands = []
    node._command_publisher = SimpleNamespace(publish=commands.append)
    old_session = node._session_id
    node._on_time_jump(None)
    assert node._session_id != old_session
    node._validate_inputs()
    assert all(not command.active for command in commands)


def test_default_compute_only_creates_no_commands(node):
    """The default configuration keeps command transport disabled."""
    assert not node.get_parameter('publish_commands').value
    node._robot_callback(robot_message())
    node._prediction_callback(prediction_message())
    node._validate_inputs()
    assert node._command_publisher is None


def test_equal_time_estimate_retains_new_sensor_fusion(node):
    """Keep the arm update published at the same time as wheel odometry."""
    node._robot_callback(robot_message(shoulder=0.0))
    node._robot_callback(robot_message(shoulder=0.4))
    assert node._robot_state.configuration[4] == 0.4


def test_older_estimate_does_not_replace_new_state(node):
    """Do not regress the accepted state to a delayed message."""
    node._robot_callback(robot_message(10.02, shoulder=0.4))
    node._robot_callback(robot_message(10.0, shoulder=-0.4))
    assert node._robot_state.configuration[4] == 0.4


@pytest.mark.parametrize('stream', ['robot', 'prediction'])
def test_future_message_does_not_poison_timestamp_cache(node, stream):
    """A queued message from an old clock epoch cannot lock out live data."""
    if stream == 'robot':
        node._robot_callback(robot_message(100.0))
        node._robot_callback(robot_message(10.0))
        assert node._robot_state.timestamp == 10.0
    else:
        node._prediction_callback(prediction_message(100.0))
        node._prediction_callback(prediction_message(10.0))
        assert node._prediction_reference_timestamp == 10.0


def test_clock_reset_requires_new_inputs_then_resumes(node):
    """Clear the old epoch and compute again with lower valid timestamps."""
    node._robot_callback(robot_message())
    node._prediction_callback(prediction_message())
    node._validate_inputs()
    assert any('CONTROL COMPUTE ONLY' in log for log in node.logs)
    node.test_now = 1.05
    node._on_time_jump(None)
    assert node._robot_state is None
    assert node._prediction is None
    node.logs.clear()
    node._validate_inputs()
    assert not any('CONTROL COMPUTE ONLY' in log for log in node.logs)
    node._robot_callback(robot_message(1.0))
    node._prediction_callback(prediction_message(1.0))
    node._validate_inputs()
    assert any('CONTROL COMPUTE ONLY' in log for log in node.logs)


def test_tf_uses_prediction_reference_time_and_rotates_velocity(node):
    """Select historical TF and apply translation only to position."""
    node._tf_buffer = Buffer()
    for timestamp, offset in [(10.0, 1.0), (11.0, 9.0)]:
        transform = TransformStamped()
        transform.header.stamp = Time(seconds=timestamp).to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'world'
        transform.transform.translation.x = offset
        transform.transform.rotation.z = np.sqrt(0.5)
        transform.transform.rotation.w = np.sqrt(0.5)
        node._tf_buffer.set_transform(transform, 'test')
    message = prediction_message()
    message.states = np.tile([2., 0., 1., 1., 0., 0.], (20, 1)).ravel().tolist()
    original = node._prediction_adapter.convert(message)
    converted = node._prediction_in_robot_frame(original, 'odom', 10.0)
    np.testing.assert_allclose(converted.states[0], [1., 2., 1., 0., 1., 0.], atol=1e-12)
    np.testing.assert_array_equal(original.states[0], [2., 0., 1., 1., 0., 0.])
    np.testing.assert_array_equal(converted.timestamps, original.timestamps)
    assert converted.frame_id == 'odom'


@pytest.mark.parametrize('failure', ['stale', 'missing_tf', 'unsafe_joints'])
def test_invalid_input_or_safety_failure_does_not_compute(node, failure):
    """No successful output is reported when prerequisites fail."""
    node._robot_callback(robot_message(shoulder=2.0 if failure == 'unsafe_joints' else 0.0))
    node._prediction_callback(prediction_message())
    if failure == 'stale':
        node.test_now = 10.6
    elif failure == 'missing_tf':
        node._tf_buffer = Buffer()
    node._validate_inputs()
    assert not any('CONTROL COMPUTE ONLY' in log for log in node.logs)
    assert node._last_compute_time is None


def test_shipped_configuration_computes_with_real_model(node):
    """The installed example supplies a complete safe computation setup."""
    parameters = yaml.safe_load(
        (Path(__file__).parents[1] / 'config' / 'control.yaml').read_text()
    )['control_node']['ros__parameters']
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    assert parameters['compute_enabled']
    assert not parameters['publish_commands']
    pipeline = node._build_pipeline(4)
    result = pipeline.compute(
        robot_state=node._robot_adapter.convert(robot_message()),
        prediction=node._prediction_in_robot_frame(
            node._prediction_adapter.convert(prediction_message()), 'odom', 10.0,
        ),
        now=10.05, dt=0.05,
    )
    assert result.raw_output.command.shape == (6,)
    assert result.safe_output.command.shape == (6,)
    assert np.all(np.isfinite(result.raw_output.command))
    upper = np.array([*parameters['base_velocity_upper'], 1., 1., 1., 1.5])
    lower = np.array([*parameters['base_velocity_lower'], -1., -1., -1., -1.5])
    assert np.all(result.safe_output.command >= lower)
    assert np.all(result.safe_output.command <= upper)


def test_inherited_strategy_uses_same_loader_at_preflight_and_runtime(node, tmp_path):
    parameters = yaml.safe_load(
        (Path(__file__).parents[1] / 'config/control.yaml').read_text()
    )['control_node']['ros__parameters']
    parent = Path(__file__).resolve().parents[1] / 'config/whole_body.yaml'
    child = tmp_path / 'strategy.yaml'; child.write_text(f'extends: {parent}\n')
    parameters['strategy_config_file'] = str(child)
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    assert node._build_pipeline(4).component_types['controller'].endswith('.WeightedLeastSquaresController')


def test_delayed_cycle_enforces_joint_limits_for_elapsed_interval(node):
    """A late timer cannot shorten the safety interval and cross a joint limit."""
    node._robot_callback(robot_message(shoulder=1.565))
    message = prediction_message()
    message.states = np.tile([-2., 0., 1., 0., 0., 0.], (20, 1)).ravel().tolist()
    prediction = node._prediction_in_robot_frame(
        node._prediction_adapter.convert(message), 'odom', 10.0,
    )
    outputs = []
    original = node._pipeline.compute

    def record_compute(**kwargs):
        result = original(**kwargs)
        outputs.append(result)
        return result

    node._pipeline.compute = record_compute
    node._compute_control(10.05, prediction)
    node._compute_control(10.19, prediction)
    assert len(outputs) == 2
    # Shoulder limit from robot.yaml is 1.57; elapsed time is 0.14s.
    assert outputs[-1].raw_output.command[3] > 0.005 / 0.14
    assert 1.565 + 0.14 * outputs[-1].safe_output.command[3] <= 1.57 + 1e-12
    node._compute_control(10.19, prediction)
    assert len(outputs) == 2


@pytest.mark.parametrize('stream', ['robot', 'prediction'])
def test_old_valid_message_does_not_evict_newer_fresh_cache(node, stream):
    """Transport reordering is not revocation of a newer validated observation."""
    commands = []
    node._command_publisher = SimpleNamespace(publish=commands.append)
    node._robot_callback(robot_message())
    node._prediction_callback(prediction_message())
    if stream == 'robot': node._robot_callback(robot_message(9.95))
    else:
        node._prediction_callback(prediction_message(10.))
        node._prediction_callback(prediction_message(9.95))
    node._validate_inputs()
    assert commands[-1].active
    node.test_now = 10.6
    node._validate_inputs()
    assert not commands[-1].active


def test_telemetry_exposes_actual_input_reference_times_during_stale_hold(node):
    messages = []
    node._telemetry_publisher = SimpleNamespace(publish=messages.append)
    node._robot_callback(robot_message())
    node._prediction_callback(prediction_message(9.9))
    node.test_now = 10.6
    node._validate_inputs()
    assert not messages[-1].valid
    assert getattr(messages[-1], 'robot_state_time', None) == 10.
    assert getattr(messages[-1], 'prediction_reference_time', None) == 9.9


def test_shutdown_does_not_dispatch_queued_active_control(monkeypatch):
    """A stop must remain the final output even with a queued timer callback."""
    from robot_control.nodes import control_node as module
    outputs=[]
    publisher=SimpleNamespace(wait_for_all_acked=lambda timeout:True)
    fake=SimpleNamespace(_timer=SimpleNamespace(cancel=lambda:None),
        _command_publisher=publisher, _stop=lambda reason:outputs.append(False),
        destroy_node=lambda:None)
    monkeypatch.setattr(module,'ControlNode',lambda:fake)
    monkeypatch.setattr(module.signal,'signal',lambda *args:None)
    monkeypatch.setattr(module.rclpy,'init',lambda **kwargs:None)
    def interrupted(node): raise KeyboardInterrupt
    monkeypatch.setattr(module.rclpy,'spin',interrupted)
    monkeypatch.setattr(module.rclpy,'spin_once',lambda *args,**kwargs:outputs.append(True))
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    monkeypatch.setattr(module.rclpy,'shutdown',lambda:None)
    module.main()
    assert outputs == [False]
