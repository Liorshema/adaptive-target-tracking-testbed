"""Selection constructs real predictors and preserves the deployed trajectory contract."""

from pathlib import Path

import numpy as np
import pytest

from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.factory.system_factory import PredictionSystemFactory
from robot_prediction.interfaces.prediction_data import PredictionInput


def factory():
    return PredictionSystemFactory(PredictionConfig(Path(__file__).parents[1] / 'config/prediction.yaml'))


def test_imm_and_cv_select_distinct_real_implementations_and_valid_trajectories():
    imm = factory().create(
        predictor_type='imm', adaptation_type='residual', fusion_type='weighted_correction')
    cv = factory().create(predictor_type='constant_velocity', adaptation_type='none', fusion_type='none')
    assert type(imm._predictor).__name__ == 'IMMPredictor'
    assert type(cv._predictor).__name__ == 'ConstantVelocityPredictor'
    state = np.array([2., 0., 1., .2, -.1, .05])
    prediction = cv.predict(PredictionInput(state=state, timestamp=12., dt=.1, horizon=4))
    np.testing.assert_allclose(prediction.states[:, :3], state[:3] + np.arange(1, 5)[:, None]*.1*state[3:])
    np.testing.assert_allclose(prediction.timestamps, [12.1, 12.2, 12.3, 12.4])
    assert prediction.states.shape == (4, 6)
    assert cv._residual_adapter is None
    assert cv._weighted_correction is None


@pytest.mark.parametrize('model', ['immm', 'constant_acceleration', 'coordinated_turn', 'learned'])
def test_unsupported_deployed_predictor_is_never_silently_imm(model):
    with pytest.raises(ValueError, match='predictor|prediction.type'):
        factory().create(
            predictor_type=model, adaptation_type='residual', fusion_type='weighted_correction')


def test_incompatible_adaptation_fusion_rejected():
    with pytest.raises(ValueError, match='fusion'):
        factory().create(predictor_type='imm', adaptation_type='residual', fusion_type='none')


def test_cv_selection_survives_node_clock_reset():
    from robot_prediction.nodes.prediction_node import PredictionNode
    from types import SimpleNamespace
    node = object.__new__(PredictionNode)
    node._config = factory()._config
    node._selection = dict(predictor_type='constant_velocity', adaptation_type='none', fusion_type='none')
    node.get_logger = lambda: SimpleNamespace(warning=lambda _: None)
    node._on_time_jump(None)
    assert type(node._system._predictor).__name__ == 'ConstantVelocityPredictor'
    assert node._system.last_prediction is None


def test_factory_does_not_choose_models_without_explicit_selection():
    with pytest.raises(TypeError):
        factory().create()


@pytest.mark.parametrize('section', ['prediction', 'adaptations', 'fusions'])
def test_parameter_documents_cannot_select_runtime_components(tmp_path, section):
    import yaml
    data = PredictionConfig(Path(__file__).parents[1] / 'config/prediction.yaml').data
    data[section]['type'] = 'imm'
    source = tmp_path / 'invalid_selection.yaml'
    source.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='parameters|unsupported'):
        PredictionConfig(source)
