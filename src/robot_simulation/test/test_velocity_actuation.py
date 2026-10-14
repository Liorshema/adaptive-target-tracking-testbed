"""Exercise command ownership, epochs, timeouts and startup gating."""

from types import SimpleNamespace

import numpy as np

import pytest

from rclpy.time import Time

from robot_interfaces.msg import SafeVelocityCommand

from robot_simulation.actuation.position_reference import PositionReference
from robot_simulation.actuation.velocity_actuation import VelocityActuationNode


@pytest.fixture
def actuator():
    """Supply local callbacks without DDS or Gazebo."""
    node = object.__new__(VelocityActuationNode)
    node._enabled = True
    node._names = ['a', 'b']
    node._frame = 'body'
    node._reference = PositionReference([-1., -1.], [1., 1.], [1., 1.], .08, .1)
    node._lower = np.array([-.5, -1., -1., -1.])
    node._upper = -node._lower
    node._config = {'command_max_age': .25, 'feedback_max_age': .2,
                    'wall_timeout': .5, 'future_tolerance': .01}
    node.test_now = 10.
    node._now = lambda: node.test_now
    node._wall = lambda: 10.
    node.get_publishers_info_by_topic = lambda _: [object()]
    node._command_topic = '/safe'
    node._output_topics = []
    node.get_logger = lambda: SimpleNamespace(warning=lambda *a, **kw: None)
    node.outputs = []
    node._send = lambda velocity=None: node.outputs.append(velocity)
    node._session = None
    node._retired = set()
    node._sequence = -1
    node._command_stamp = None
    node._command = None
    node._last_tick = None
    node._joint = (10., 10., np.array([.2, -.3]))
    node._odom = (10., 10.)
    node._last_joint_stamp = None
    node._last_odom_stamp = None
    node._was_active = False
    node._reason = None
    return node


def command(session='session', sequence=1, active=True):
    """Construct a named body-frame test command."""
    message = SafeVelocityCommand()
    message.header.stamp = Time(seconds=10.).to_msg()
    message.header.frame_id = 'body'
    message.session_id = session
    message.sequence = sequence
    message.joint_names = ['a', 'b']
    message.velocity = [.2, .3, .4, -.5] if active else [0.] * 4
    message.active = active
    return message


def test_startup_discards_commands_without_complete_feedback(actuator):
    """Startup cannot queue active commands before feedback is complete."""
    actuator._odom = None
    actuator._command_callback(command())
    assert actuator._command is None
    assert actuator.outputs[-1] is None


def test_timeout_holds_reference_and_new_command_reinitializes(actuator):
    """A timeout stops integration while retaining the last safe reference."""
    actuator._command_callback(command())
    actuator._tick()
    actuator.test_now = 10.05
    actuator._tick()
    held = actuator._reference.position.copy()
    assert np.allclose(held, [.22, -.325])
    actuator.test_now = 10.4
    actuator._tick()
    assert actuator._command is None
    assert actuator.outputs[-1] is None
    assert np.allclose(actuator._reference.position, held)


@pytest.mark.parametrize('field,value', [
    ('velocity', [2., 0., 0., 0.]), ('joint_names', ['b', 'a']),
])
def test_unsafe_command_invalidates_tracking(actuator, field, value):
    """Reject limits and joint-identity mismatches."""
    message = command()
    setattr(message, field, value)
    actuator._command_callback(message)
    assert actuator._command is None


def test_reset_retires_session_and_replay_cannot_activate(actuator):
    """Require a fresh session and feedback after a clock reset."""
    actuator._command_callback(command())
    actuator._on_time_jump(None)
    actuator._joint = (10., 10., np.zeros(2))
    actuator._odom = (10., 10.)
    actuator._command_callback(command(sequence=2))
    assert actuator._command is None
    actuator._command_callback(command('new-session'))
    assert actuator._command is not None


def test_disabled_mode_never_emits_motor_commands(actuator):
    """Opt-out suppresses all actuator output."""
    actuator._enabled = False
    actuator._command_callback(command())
    actuator._tick()
    assert not actuator.outputs


def test_newer_sequence_with_older_time_stops_and_does_not_poison_order(actuator):
    """Sequence alone cannot make a delayed command current."""
    actuator._command_callback(command())
    older = command(sequence=2)
    older.header.stamp = Time(seconds=9.9).to_msg()
    actuator._command_callback(older)
    assert actuator._command is None
    assert actuator._sequence == 1
    actuator._command_callback(command(sequence=2))
    assert actuator._command is not None


def test_competitor_cannot_retire_the_valid_source(actuator):
    """Once a second publisher leaves, the original session can recover."""
    actuator._command_callback(command())
    actuator.get_publishers_info_by_topic = lambda _: [object(), object()]
    actuator._command_callback(command('competitor'))
    assert actuator._session == 'session'
    actuator.get_publishers_info_by_topic = lambda _: [object()]
    actuator._command_callback(command(sequence=2))
    assert actuator._command is not None


def test_stale_reset_stop_cannot_adopt_the_next_session_early(actuator):
    """A delayed local clock cannot retire the controller's new epoch."""
    actuator._command_callback(command())
    early = command('next-epoch', active=False)
    early.header.stamp = Time(seconds=1.).to_msg()
    actuator._command_callback(early)
    assert actuator._session == 'session'


def test_bad_feedback_does_not_erase_its_ordering_watermark(actuator):
    """A malformed packet followed by an older packet remains blocked."""
    from sensor_msgs.msg import JointState
    valid = JointState()
    valid.header.stamp = Time(seconds=10.).to_msg()
    valid.name = ['b', 'a']
    valid.position = [-.3, .2]
    valid.velocity = [0., 0.]
    actuator._joint_callback(valid)
    actuator._joint_callback(JointState())
    valid.header.stamp = Time(seconds=9.9).to_msg()
    actuator._joint_callback(valid)
    assert actuator._joint is None
    assert actuator._last_joint_stamp == 10.


def test_wall_watchdog_stops_even_when_simulation_time_pauses(actuator):
    """A clock pause cannot leave an old moving command active."""
    actuator._command_callback(command())
    actuator._wall = lambda: 10.6
    actuator._watchdog_tick()
    assert actuator._command is None
    assert actuator.outputs[-1] is None
