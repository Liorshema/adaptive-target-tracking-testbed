"""Mathematical and owner-factory checks for the optional predictor adapter."""

from dataclasses import replace
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest
from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.factory.component_registry import create_prediction_registries
from robot_prediction.factory.system_factory import PredictionSystemFactory
from robot_prediction.interfaces.prediction_data import PredictionInput, TrajectoryPrediction
from robot_prediction.interfaces.predictor import Predictor
import yaml

SCHEMA = {
    'type': 'object',
    'properties': {'acceleration': {'type': 'number', 'unit': 'm/s^2', 'minimum': 0.0}},
    'required': ['acceleration'],
    'additionalProperties': False,
}
CONFIG = Path(__file__).parents[1] / 'config/prediction.yaml'


def declaration(function=None, **options):
    from robot_prediction.baseline.stateless_model import predictor_model

    if function is None:

        def function(state, horizons, parameters):
            acceleration = parameters['acceleration']
            positions = state[:3] + horizons[:, None] * state[3:]
            positions += 0.5 * acceleration * horizons[:, None] ** 2
            velocities = state[3:] + acceleration * horizons[:, None]
            return np.concatenate([positions, velocities], axis=1)

    return predictor_model('test_parabolic', parameters_schema=SCHEMA, **options)(function)


def sample():
    return PredictionInput(np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3]), 11.0, 0.25, 3)


def test_exact_relative_horizons_absolute_timestamps_and_absent_uncertainty():
    model = declaration()
    predictor = model.builder(acceleration=2.0)
    assert isinstance(predictor, Predictor)
    result = predictor.predict(sample())
    expected = np.array(
        [
            [1.0875, 2.1125, 3.1375, 0.6, 0.7, 0.8],
            [1.3, 2.35, 3.4, 1.1, 1.2, 1.3],
            [1.6375, 2.7125, 3.7875, 1.6, 1.7, 1.8],
        ]
    )
    np.testing.assert_allclose(result.states, expected, rtol=0.0, atol=1.0e-15)
    np.testing.assert_array_equal(result.timestamps, [11.25, 11.5, 11.75])
    assert result.covariance is None
    assert result.confidence is None
    assert result.model_name == 'test_parabolic'
    assert model.requirements == {'state_layout': 'position_velocity', 'spatial_dimension': 3}
    assert model.capabilities['coordinate_frames'] == 'same fixed frame as input'


def test_genuine_uncertainty_is_preserved_and_detached():
    covariance = np.tile(np.eye(6), (3, 1, 1))
    confidence = np.array([0.9, 0.8, 0.7])

    def mathematical(state, horizons, parameters):
        return TrajectoryPrediction(
            np.tile(state, (3, 1)), 11.0 + horizons, covariance, confidence, 'genuine'
        )

    result = declaration(mathematical).builder(acceleration=0.0).predict(sample())
    np.testing.assert_array_equal(result.covariance, covariance)
    np.testing.assert_array_equal(result.confidence, confidence)
    covariance[:] = 0.0
    confidence[:] = 0.0
    assert result.covariance[0, 0, 0] == 1.0
    assert result.confidence[0] == 0.9
    assert result.model_name == 'genuine'


@pytest.mark.parametrize(
    'changes,match',
    [
        ({'state': np.ones(4)}, 'state'),
        ({'state': np.ones((1, 6))}, 'state'),
        ({'state': np.full(6, np.nan)}, 'state'),
        ({'timestamp': np.inf}, 'timestamp'),
        ({'timestamp': True}, 'timestamp'),
        ({'timestamp': 1.0e300}, 'timestamps'),
        ({'dt': 0.0}, 'dt'),
        ({'dt': np.nan}, 'dt'),
        ({'dt': True}, 'dt'),
        ({'horizon': 0}, 'horizon'),
        ({'horizon': 2.5}, 'horizon'),
        ({'horizon': True}, 'horizon'),
        ({'covariance': np.ones((3, 3))}, 'covariance'),
        ({'covariance': -np.eye(6)}, 'covariance'),
        ({'covariance': np.triu(np.ones((6, 6)))}, 'covariance'),
        ({'covariance': np.full((6, 6), np.inf)}, 'covariance'),
    ],
)
def test_invalid_input_rejected_before_math(changes, match):
    calls = []

    def mathematical(*arguments):
        calls.append(arguments)
        return np.zeros((3, 6))

    with pytest.raises(ValueError, match=match):
        declaration(mathematical).builder(acceleration=0.0).predict(replace(sample(), **changes))
    assert not calls


@pytest.mark.parametrize(
    'output,match',
    [
        (np.zeros((4, 6)), 'states'),
        (np.zeros((3, 4)), 'states'),
        (np.full((3, 6), np.inf), 'states'),
        (TrajectoryPrediction(np.zeros((3, 6)), np.array([11.0, 11.25, 11.5])), 'timestamps'),
        (TrajectoryPrediction(np.zeros((3, 6)), np.array([11.25, 11.75, 11.5])), 'timestamps'),
        (
            TrajectoryPrediction(
                np.zeros((3, 6)), np.array([11.25, 11.5, 11.75]), covariance=np.eye(6)
            ),
            'covariance',
        ),
        (
            TrajectoryPrediction(
                np.zeros((3, 6)),
                np.array([11.25, 11.5, 11.75]),
                covariance=np.tile(-np.eye(6), (3, 1, 1)),
            ),
            'covariance',
        ),
        (
            TrajectoryPrediction(
                np.zeros((3, 6)),
                np.array([11.25, 11.5, 11.75]),
                confidence=np.array([0.5, np.nan, 0.9]),
            ),
            'confidence',
        ),
        (
            TrajectoryPrediction(
                np.zeros((3, 6)),
                np.array([11.25, 11.5, 11.75]),
                confidence=np.array([0.5, 1.1, 0.9]),
            ),
            'confidence',
        ),
        (
            TrajectoryPrediction(
                np.zeros((3, 6)), np.array([11.25, 11.5, 11.75]), confidence=np.ones((3, 6))
            ),
            'confidence',
        ),
    ],
)
def test_invalid_output_rejected(output, match):
    with pytest.raises(ValueError, match=match):
        declaration(lambda *_: output).builder(acceleration=0.0).predict(sample())


@pytest.mark.parametrize(
    'parameters',
    [{'acceleration': True}, {'acceleration': -1.0}, {'acceleration': '1'}, {'typo': 1.0}, {}],
)
def test_direct_builder_validates_parameter_contract(parameters):
    with pytest.raises(ValueError):
        declaration().builder(**parameters)


@pytest.mark.parametrize('dimension', [True, 0, -1, 2.5])
def test_dimension_declaration_is_positive_integer(dimension):
    with pytest.raises(ValueError, match='spatial_dimension'):
        declaration(spatial_dimension=dimension)


def test_callback_cannot_mutate_caller_state_or_parameter_defaults():
    def mathematical(state, horizons, parameters):
        with pytest.raises(ValueError):
            state[0] = 100.0
        with pytest.raises(ValueError):
            horizons[0] = 100.0
        with pytest.raises(TypeError):
            parameters['acceleration'] = 100.0
        return np.tile(state, (len(horizons), 1))

    prediction_input = sample()
    declaration(mathematical).builder(acceleration=0.0).predict(prediction_input)
    assert prediction_input.state[0] == 1.0


def test_trusted_owner_discovery_and_selected_yaml_defaults(tmp_path, monkeypatch):
    import robot_prediction.baseline as namespace
    import robot_prediction.factory.component_registry as owner

    plugin = tmp_path / 'trusted_fixture'
    plugin.mkdir()
    (plugin / 'mathematics.py').write_text('COMPONENTS = ()\n')
    module = ModuleType('robot_prediction.baseline.mathematics')
    module.COMPONENTS = (declaration(),)
    monkeypatch.setitem(__import__('sys').modules, module.__name__, module)
    monkeypatch.setattr(namespace, '__path__', [str(plugin)])
    monkeypatch.setattr(owner, 'PREDICTOR_MODEL_NAMESPACE', 'robot_prediction.baseline')
    data = PredictionConfig(CONFIG).data
    data['predictors']['test_parabolic'] = {'parameters': {'acceleration': 2.0}}
    path = tmp_path / 'prediction.yaml'
    path.write_text(yaml.safe_dump(data))
    config = PredictionConfig(path)
    registries = create_prediction_registries(config)
    # Registry defaults can differ from the selected runtime owner configuration.
    runtime = data.copy()
    runtime['predictors'] = dict(data['predictors'])
    runtime['predictors']['test_parabolic'] = {'parameters': {'acceleration': 4.0}}
    path.write_text(yaml.safe_dump(runtime))
    factory = PredictionSystemFactory(PredictionConfig(path), registries=registries)
    selection = {
        'predictor_type': 'test_parabolic',
        'adaptation_type': 'none',
        'fusion_type': 'none',
    }
    expected = declaration().builder(acceleration=4.0).predict(sample())
    np.testing.assert_array_equal(
        factory.create(**selection).predict(sample()).states, expected.states
    )
    explicit = factory.create(**selection, parameters={'predictor': {'acceleration': 1.0}})
    np.testing.assert_array_equal(
        explicit.predict(sample()).states,
        declaration().builder(acceleration=1.0).predict(sample()).states,
    )
    runtime['predictors']['test_parabolic']['parameters']['acceleration'] = -1.0
    path.write_text(yaml.safe_dump(runtime))
    with pytest.raises(ValueError, match='acceleration'):
        PredictionSystemFactory(PredictionConfig(path), registries=registries).create(**selection)
    assert registries['predictor'].describe('test_parabolic')['defaults'] == {'acceleration': 2.0}


def test_production_inventory_has_no_test_models():
    registries = create_prediction_registries(PredictionConfig(CONFIG))
    assert registries['predictor'].names() == (
        'constant_acceleration',
        'constant_velocity',
        'coordinated_turn',
        'gaussian_process',
        'imm',
        'koopman_edmd',
        'none',
    )


def test_factory_merges_nested_explicit_parameters_over_owner_values(tmp_path):
    from robot_configuration.components import schema_for

    data = PredictionConfig(CONFIG).data
    data['predictors']['test_nested'] = {'parameters': {'law': {'first': 2.0, 'second': 3.0}}}
    path = tmp_path / 'nested.yaml'
    path.write_text(yaml.safe_dump(data))
    config = PredictionConfig(path)
    registries = create_prediction_registries(PredictionConfig(CONFIG))
    seen = []

    def build(law):
        seen.append(law)
        return declaration().builder(acceleration=0.0)

    registries['predictor'].register(
        'test_nested',
        build,
        defaults={'law': {'first': 0.0, 'second': 0.0}},
        schema=schema_for({'law': {'first': 0.0, 'second': 0.0}}),
    )
    PredictionSystemFactory(config, registries=registries).create(
        predictor_type='test_nested',
        adaptation_type='none',
        fusion_type='none',
        parameters={'predictor': {'law': {'first': 4.0}}},
    )
    assert seen == [{'first': 4.0, 'second': 3.0}]


def test_complex_forecast_is_rejected_without_discarding_imaginary_part():
    model = declaration(lambda *args: np.ones((3, 6), dtype=complex) * (1.0 + 2.0j))
    with pytest.raises(ValueError, match='real'):
        model.builder(acceleration=0.0).predict(sample())


def test_complex_state_is_rejected():
    value = replace(sample(), state=np.ones(6, dtype=complex))
    with pytest.raises(ValueError, match='real'):
        declaration().builder(acceleration=0.0).predict(value)


@pytest.mark.parametrize('origin', [11., 123.])
def test_uncertainty_mapping_uses_adapter_owned_absolute_grid(origin):
    def mathematical(state, horizons, parameters):
        return {'states': np.tile(state, (3, 1)),
                'covariance': np.tile(np.eye(6), (3, 1, 1)),
                'confidence': np.array([.9, .8, .7])}
    result = declaration(mathematical).builder(acceleration=0.).predict(
        replace(sample(), timestamp=origin))
    np.testing.assert_array_equal(result.timestamps, origin + np.array([.25, .5, .75]))
    np.testing.assert_array_equal(result.covariance, np.tile(np.eye(6), (3, 1, 1)))
    np.testing.assert_array_equal(result.confidence, [.9, .8, .7])


def test_uncertainty_mapping_rejects_unknown_time_or_result_fields():
    def mathematical(state, horizons, parameters):
        return {'states': np.tile(state, (3, 1)), 'timestamps': horizons}
    with pytest.raises(ValueError, match='result.*unknown'):
        declaration(mathematical).builder(acceleration=0.).predict(sample())
