"""Causal target continuation and Gazebo pose transaction regressions."""
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from robot_simulation.gazebo.target_adapter import GazeboTargetAdapter
from robot_simulation.nodes.simulation_node import SimulationNode
from robot_simulation.state.physical_context import PhysicalPositionHistory
from robot_simulation.integration.time_stepper import TimeStepper
from robot_simulation.engine.simulation_engine import SimulationEngine
from robot_simulation.integration.rk4 import RK4Integrator
from robot_simulation.interfaces.dynamics import Dynamics


class ConstantVelocity(Dynamics):
    def derivative(self, state, control, time, context):
        return np.r_[state[3:], np.zeros(3)]


@pytest.fixture
def node():
    n = object.__new__(SimulationNode)
    n._reference_frame = 'world'; n._tracking_source = 'tf'
    n._initial_position = np.zeros(3); n._initial_velocity = np.array([1.,0.,0.])
    n._physical_history = PhysicalPositionHistory('world','base',.025,.125,100)
    camera = PhysicalPositionHistory('world','camera',.025,.125,100)
    n.camera_history = camera
    def camera_at(t, refresh=True, valid_until=None):
        sample = camera.at(t)
        if sample is None: return None
        try: sample.position_at(t if valid_until is None else valid_until,'world')
        except ValueError: return None
        return sample.position_world
    n._tracking_provider = SimpleNamespace(position_at=camera_at,clear_history=camera.clear)
    for t in [0.,.02,.04]: n._physical_history.add(np.zeros(3),t,'world')
    camera.add(np.zeros(3),0.,'world')
    n._state = n._make_initial_state(0.,np.zeros(3))
    n._last_clock = 0.; n._epoch = 0; n._pending = False; n._pending_since = None
    n._last_measurement_time = None; n._dt = .01; n._context_history_duration = .1
    n._stepper = TimeStepper(SimulationEngine(ConstantVelocity(),RK4Integrator()),.01,10)
    n._adapter = GazeboTargetAdapter(); n.requests = []
    def send(pose):
        future = Future(); n.requests.append((pose,future)); return future
    n._pose_client = SimpleNamespace(service_ready=lambda:True,send_pose=send)
    n._publish_ground_truth = Mock(); n._publish_measurement = Mock()
    n.get_logger = lambda:Mock()
    n.test_now = .05
    n.get_clock = lambda: SimpleNamespace(now=lambda:Time(seconds=n.test_now))
    n.get_parameter = lambda name:SimpleNamespace(value=0.)
    return n


def clock(n,t):
    n.test_now = t
    n._on_clock(Clock(clock=Time(seconds=t).to_msg()))


def acknowledge(n,success=True):
    n.requests[-1][1].set_result(SimpleNamespace(success=success))


def test_partial_pose_is_committed_only_after_acknowledgement_and_resumes(node):
    clock(node,.05)
    assert node._state.time == 0.
    assert node._pending
    np.testing.assert_allclose(node.requests[-1][0].position,[.02,0.,0.])
    acknowledge(node)
    assert node._state.time == pytest.approx(.02)
    for t in [.02,.04]: node.camera_history.add(np.zeros(3),t,'world')
    clock(node,.06)
    acknowledge(node)
    assert node._state.time == pytest.approx(.06)
    np.testing.assert_allclose(node._state.target.position,[.06,0.,0.])
    assert node._publish_measurement.call_count == 2


@pytest.mark.parametrize('source',['base','camera'])
def test_missing_context_does_not_publish_or_retimestamp_target(node,source):
    (node._physical_history if source=='base' else node.camera_history).clear()
    clock(node,.05)
    assert node._state.time == 0.
    assert not node.requests
    node._publish_measurement.assert_not_called()
    with pytest.raises(RuntimeError,match='Irrecoverable'):
        clock(node,.11)
    assert node._state.time == 0.


def test_rejected_pose_does_not_commit_partial_progress(node):
    clock(node,.05); acknowledge(node,False)
    assert node._state.time == 0.
    node._publish_ground_truth.assert_not_called()


def test_clock_reset_discards_old_async_candidate_and_both_histories(node):
    clock(node,.05)
    clock(node,.001)
    acknowledge(node)
    assert node._state is None and node._epoch == 1
    assert node._physical_history.at(0.) is None
    assert node.camera_history.at(0.) is None
    node._publish_measurement.assert_not_called()


def test_pending_pose_has_bounded_time_budget(node):
    clock(node,.05)
    with pytest.raises(RuntimeError,match='acknowledgement'):
        clock(node,.16)
    node._publish_measurement.assert_not_called()


def test_future_physical_message_cannot_poison_current_history(node):
    message=Odometry(); message.header.frame_id='world'
    message.header.stamp=Time(seconds=2.).to_msg()
    node._on_physical_base(message)
    assert node._physical_history._samples[-1].timestamp == .04


def test_capacity_is_independent_of_source_density():
    history=PhysicalPositionHistory('world','source',.2,1.,capacity=8)
    for t in np.linspace(0.,.01,1000): history.add(np.zeros(3),float(t),'world')
    assert len(history._samples)==8
    history.add(np.ones(3),0.,'world')
    assert len(history._samples)==8


def test_repeated_delayed_context_recovers_without_reinitializing_target(node):
    for cycle in range(1,5):
        start=(cycle-1)*.1; end=cycle*.1
        for t in np.arange(start,end+.001,.02):
            node._physical_history.add(np.zeros(3),round(float(t),8),'world')
        clock(node,end)
        acknowledge(node)
        assert node._state.time < end
        for t in np.arange(start,end+.001,.02):
            node.camera_history.add(np.zeros(3),round(float(t),8),'world')
        clock(node,end)
        acknowledge(node)
        assert node._state.time == pytest.approx(end)
        np.testing.assert_allclose(node._state.target.position,[end,0.,0.])
    assert node._epoch == 0


def test_context_transition_logging_uses_valid_ros_logger_callsite(node):
    from rclpy.logging import get_logger
    logger=get_logger('context_transition_regression')
    node.get_logger=lambda:logger
    node._report_context_wait(True,.05)
    node._report_context_wait(False,.06)


def test_base_callback_ahead_of_clock_is_held_until_causal(node):
    """A transport callback before /clock is not a missing physical sample."""
    node.test_now=.05
    message=Odometry();message.header.frame_id='world'
    message.header.stamp=Time(seconds=.06).to_msg()
    message.pose.pose.position.x=3.
    node._on_physical_base(message)
    assert node._physical_history.at(.05).timestamp == .04
    assert node._physical_history.at(.061).timestamp == pytest.approx(.06)
    assert node._physical_history.at(.061).position_world[0] == 3.


def test_ahead_of_clock_receipt_does_not_evict_needed_past_context():
    history=PhysicalPositionHistory('world','source',.3,10.3,capacity=5302)
    history.add(np.zeros(3),0.,'world',receipt_time=0.)
    history.add(np.ones(3),10.2,'world',receipt_time=10.)
    assert history.at(0.) is not None
    assert history.at(10.) is None
    assert history.at(10.21).timestamp == 10.2
    history.add(np.ones(3),20.,'world',receipt_time=10.)
    assert history._samples[-1].timestamp == 10.2
