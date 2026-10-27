"""Owner defaults, registration, and deployed prediction contracts."""

from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.factory.system_factory import PredictionSystemFactory
from robot_prediction.interfaces.prediction_data import PredictionInput, TrajectoryPrediction
from robot_prediction.interfaces.predictor import Predictor


CONFIG = Path(__file__).parents[1] / 'config/prediction.yaml'
LEGACY = {
    'prediction': {'dt': .1, 'horizon': 20},
    'history': {'window_duration': 1.0},
    'confidence': {'residual_scale': .5},
    'imm': {'model_probabilities': [.333333, .333333, .333334],
            'transition_matrix': [[.90, .05, .05], [.05, .90, .05], [.05, .05, .90]],
            'likelihood_variance': 1.0},
    'coordinated_turn': {'turn_rate_epsilon': 1.e-6},
    'fusion': {'default_weights': [1.0]},
    'adaptation': {'residual': {'correction_gain': 1.0},
                   'rls': {'forgetting_factor': .99, 'initial_covariance': 100.0}},
}


def write_config(tmp_path, value):
    path = tmp_path / 'prediction.yaml'
    path.write_text(yaml.safe_dump(value))
    return PredictionConfig(path)


def test_canonical_defaults_and_legacy_saved_inputs_resolve_identically(tmp_path):
    current = PredictionConfig(CONFIG)
    legacy = write_config(tmp_path, LEGACY)
    assert set(current.data) == {
        'prediction', 'monitoring', 'predictors', 'adaptations', 'fusions'}
    assert current.data == legacy.data
    for section, expected in LEGACY.items():
        assert current.get(section) == expected
    detached = current.data
    detached['predictors']['imm']['parameters']['model_probabilities'][0] = 900.
    assert current.get('imm')['model_probabilities'] == [.333333, .333333, .333334]


@pytest.mark.parametrize('path,value', [
    (('prediction', 'parameters', 'dt'), True),
    (('prediction', 'parameters', 'horizon'), 0),
    (('monitoring', 'parameters', 'window_duration'), -1.),
    (('monitoring', 'parameters', 'residual_scale'), '0.5'),
    (('predictors', 'coordinated_turn', 'parameters', 'turn_rate_epsilon'), float('nan')),
    (('predictors', 'imm', 'parameters', 'model_probabilities'), [1., 0.]),
    (('predictors', 'imm', 'parameters', 'transition_matrix'), [[1., 0., 0.]]),
    (('adaptations', 'rls', 'parameters', 'forgetting_factor'), 1.1),
    (('fusions', 'baseline', 'parameters', 'default_weights'), [-1.]),
])
def test_all_family_parameters_are_strict_even_when_inactive(tmp_path, path, value):
    data = PredictionConfig(CONFIG).data
    parent = data
    for key in path[:-1]:
        parent = parent[key]
    parent[path[-1]] = value
    with pytest.raises(ValueError, match=path[-1]):
        write_config(tmp_path, data)


def test_missing_and_unknown_parameter_leaves_rejected(tmp_path):
    data = PredictionConfig(CONFIG).data
    del data['monitoring']['parameters']['window_duration']
    with pytest.raises(ValueError, match='window_duration'):
        write_config(tmp_path, data)
    data = PredictionConfig(CONFIG).data
    data['predictors']['imm']['parameters']['typo'] = 1.
    with pytest.raises(ValueError, match='typo'):
        write_config(tmp_path, data)


def test_old_owner_overrides_translate_without_inserting_defaults():
    from robot_prediction.configuration.prediction_config import normalize_prediction_overrides
    overrides = {'imm': {'likelihood_variance': .5}, 'prediction': {'dt': .2},
                 'adaptation': {'residual': {'correction_gain': .7}}}
    original = deepcopy(overrides)
    assert normalize_prediction_overrides(overrides) == {
        'predictors': {'imm': {'parameters': {'likelihood_variance': .5}}},
        'prediction': {'parameters': {'dt': .2}},
        'adaptations': {'residual': {'parameters': {'correction_gain': .7}}},
    }
    assert overrides == original
    with pytest.raises(ValueError, match='ambiguous|mixed|conflict'):
        normalize_prediction_overrides({
            'imm': {'likelihood_variance': .5},
            'predictors': {'imm': {'parameters': {'likelihood_variance': .7}}}})


class OffsetPredictor(Predictor):
    """Test-only component with a visible configurable positional offset."""

    def __init__(self, offset):
        self.offset = offset

    def predict(self, prediction_input):
        states = np.tile(prediction_input.state, (prediction_input.horizon, 1))
        states[:, :states.shape[1] // 2] += self.offset
        return TrajectoryPrediction(
            states=states,
            timestamps=prediction_input.timestamp + prediction_input.dt * np.arange(
                1, prediction_input.horizon + 1),
            model_name='test_offset')


def test_owner_registered_predictor_runs_through_real_factory():
    from robot_configuration.components import schema_for
    from robot_prediction.factory.component_registry import create_prediction_registries
    config = PredictionConfig(CONFIG)
    registries = create_prediction_registries(config)
    registries['predictor'].register(
        'test_offset', OffsetPredictor, defaults={'offset': .2},
        schema=schema_for({'offset': .2}, units={'offset': 'm'},
                          constraints={'offset': {'minimum': 0.}}),
        capabilities={'output_layout': 'position_velocity', 'test_only': True},
        requirements={'state_layout': 'position_velocity'})
    assert 'test_offset' in registries['predictor'].names()
    metadata = registries['predictor'].describe('test_offset')
    assert metadata['schema']['properties']['offset']['unit'] == 'm'
    system = PredictionSystemFactory(config, registries=registries).create(
        predictor_type='test_offset', adaptation_type='none', fusion_type='none',
        parameters={'predictor': {'offset': .7}})
    prediction = system.predict(PredictionInput(
        state=np.array([1., 2., 3., .1, .2, .3]), timestamp=4., dt=.1, horizon=2))
    np.testing.assert_array_equal(prediction.states, [
        [1.7, 2.7, 3.7, .1, .2, .3], [1.7, 2.7, 3.7, .1, .2, .3]])
    np.testing.assert_allclose(prediction.timestamps, [4.1, 4.2])
    assert registries['predictor'].describe('test_offset')['defaults'] == {'offset': .2}
    with pytest.raises(ValueError, match='offset'):
        PredictionSystemFactory(config, registries=registries).create(
            predictor_type='test_offset', adaptation_type='none', fusion_type='none',
            parameters={'predictor': {'offset': -1.}})


def test_adaptation_and_fusion_selection_uses_injected_components():
    from robot_configuration.components import schema_for
    from robot_prediction.factory.component_registry import create_prediction_registries
    config = PredictionConfig(CONFIG)
    registries = create_prediction_registries(config)

    class Bias:
        def compute_correction(self, statistics, horizon):
            return np.tile(statistics.mean * .5, (horizon, 1))

    class DoubleCorrection:
        def apply(self, baseline, correction):
            return TrajectoryPrediction(states=baseline.states + 2. * correction,
                                        timestamps=baseline.timestamps.copy())

    registries['adaptation'].register(
        'test_bias', Bias, schema=schema_for({}),
        capabilities={'correction_representation': 'state_bias', 'test_only': True})
    registries['fusion'].register(
        'test_double', DoubleCorrection, schema=schema_for({}),
        requirements={'correction_representation': 'state_bias'},
        capabilities={'test_only': True})
    system = PredictionSystemFactory(config, registries=registries).create(
        predictor_type='constant_velocity', adaptation_type='test_bias', fusion_type='test_double')
    system.predict(PredictionInput(np.array([0., 0., 0., 0., 0., 0.]), 0., .1, 2))
    system.update(np.array([1., 0., 0., 0., 0., 0.]), observation_timestamp=.1)
    prediction = system.predict(PredictionInput(
        np.array([1., 0., 0., 0., 0., 0.]), .1, .1, 2))
    np.testing.assert_array_equal(prediction.states[:, 0], [2., 2.])


def test_inventory_keeps_library_components_outside_deployed_factory():
    from robot_prediction.factory.component_registry import create_prediction_registries
    config = PredictionConfig(CONFIG)
    registries = create_prediction_registries(config)
    assert registries['predictor'].describe('imm')['capabilities']['runtime_validation'] == 'conditional'
    for family, name in [('predictor', 'constant_acceleration'),
                         ('predictor', 'coordinated_turn'), ('adaptation', 'rls')]:
        assert registries[family].describe(name)['capabilities']['deployable'] is False
        selection = dict(predictor_type='constant_velocity', adaptation_type='none', fusion_type='none')
        selection[family + '_type' if family != 'predictor' else 'predictor_type'] = name
        with pytest.raises(ValueError, match='incompatible|library'):
            PredictionSystemFactory(config, registries=registries).create(**selection)
    compatibility = registries['predictor'].compatibility(
        'imm', {'state_layout': 'position_velocity', 'spatial_dimension': 1,
                'deployment': 'trajectory_system'})
    assert compatibility['compatible'] is False


def test_deployed_residual_correction_retains_original_numerical_behavior(tmp_path):
    system = PredictionSystemFactory(write_config(tmp_path, LEGACY)).create(
        predictor_type='constant_velocity', adaptation_type='residual', fusion_type='weighted_correction')
    system.predict(PredictionInput(np.array([0., 0., 0., 1., 0., 0.]), 2., .1, 2))
    system.update(np.array([.3, 0., 0., 1.5, 0., 0.]), observation_timestamp=2.1)
    result = system.predict(PredictionInput(np.array([.3, 0., 0., 1.5, 0., 0.]), 2.1, .1, 2))
    np.testing.assert_allclose(result.states, [[.65, 0., 0., 2., 0., 0.], [.8, 0., 0., 2., 0., 0.]],
                               rtol=0., atol=1.e-15)
    np.testing.assert_array_equal(result.timestamps, [2.2, 2.3000000000000003])
