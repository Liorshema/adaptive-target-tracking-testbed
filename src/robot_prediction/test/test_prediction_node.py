"""Regress clock epochs through the real prediction system."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.time import Time
from robot_interfaces.msg import StateEstimate

from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.nodes.prediction_node import PredictionNode
from robot_prediction.factory.system_factory import PredictionSystemFactory


@pytest.fixture
def node():
    """Supply local ROS boundaries with the actual configured predictor."""
    predictor = object.__new__(PredictionNode)
    predictor._config = PredictionConfig(Path(__file__).parents[1] / 'config' / 'prediction.yaml')
    predictor._selection = {
        'predictor_type': 'imm', 'adaptation_type': 'residual',
        'fusion_type': 'weighted_correction'}
    predictor._system = PredictionSystemFactory(config=predictor._config).create(
        **predictor._selection)
    predictor._dt = 0.1
    predictor._horizon = 20
    predictor._last_timestamp = None
    predictor._state_dimension = None
    predictor._frame_id = None
    predictor.test_now = 10.05
    predictor.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=predictor.test_now))
    predictor.get_parameter = lambda name: SimpleNamespace(value=0.01)

    def record(message, **kwargs):
        pass

    predictor.get_logger = lambda: SimpleNamespace(info=record, warning=record, error=record)
    predictor.messages = []
    predictor._trajectory_full_publisher = SimpleNamespace(publish=predictor.messages.append)
    predictor._trajectory_publisher = SimpleNamespace(publish=lambda message: None)
    predictor._confidence_publisher = SimpleNamespace(publish=lambda message: None)
    return predictor


def state_message(timestamp):
    """Build a complete six-dimensional target estimate."""
    message = StateEstimate()
    message.header.stamp = Time(seconds=timestamp).to_msg()
    message.header.frame_id = 'world'
    message.mean = [2., 0., 1., 0., 0., 0.]
    message.covariance = np.eye(6).ravel().tolist()
    return message


def test_predictor_reset_discards_old_adaptation_and_resumes(node):
    """A lower clock epoch receives a fresh prediction and future horizon."""
    node._state_callback(state_message(10.0))
    assert len(node.messages) == 1
    node.test_now = 1.05
    node._on_time_jump(None)
    assert node._system.last_prediction is None
    node._state_callback(state_message(1.0))
    assert len(node.messages) == 2
    assert node.messages[-1].header.stamp.sec == 1
    assert node.messages[-1].timestamps[0] == pytest.approx(1.1)


def test_future_target_does_not_block_new_prediction(node):
    """Queued estimates from before reset cannot poison ordering state."""
    node._state_callback(state_message(100.0))
    node._state_callback(state_message(10.0))
    assert len(node.messages) == 1
    assert node._last_timestamp == 10.0


@pytest.mark.parametrize('model', ['constant_velocity', 'none'])
def test_full_forecast_preserves_explicit_model_identity(node, model):
    """ROS observers must distinguish the actual model, including warm-up labels."""
    node._system = PredictionSystemFactory(config=node._config).create(
        predictor_type=model, adaptation_type='none', fusion_type='none')
    node._state_callback(state_message(10.0))
    assert node.messages[-1].model_name == model
    assert node.messages[-1].header.frame_id == 'world'
    if model == 'none':
        assert np.all(np.asarray(node.messages[-1].states).reshape(-1, 6)[:, 3:] == 0.)


@pytest.mark.parametrize('name', ['koopman_edmd', 'gaussian_process'])
def test_online_reset_restarts_visible_warmup_without_learning_leak(node, name):
    """A new clock epoch reconstructs the selected model with no old training."""
    node._selection = dict(predictor_type=name, adaptation_type='none', fusion_type='none')
    node._system = PredictionSystemFactory(config=node._config).create(**node._selection)
    for k in range(41):
        timestamp=5.+k*.1
        node.test_now=timestamp+.01
        node._state_callback(state_message(timestamp))
    assert node.messages[0].model_name == name + '/cv_warmup'
    assert node.messages[-1].model_name == name
    node._on_time_jump(None)
    node.test_now=1.01
    node._state_callback(state_message(1.))
    assert node.messages[-1].model_name == name + '/cv_warmup'
    assert node._system._predictor.diagnostics['training_pairs'] == 0
