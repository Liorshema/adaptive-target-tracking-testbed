"""Parameterized algorithms traverse Author, frozen inputs and real estimator factories."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from robot_configuration.yaml_loader import load_yaml
from robot_estimation.factory.estimator_factory import create_target_estimator
from robot_estimation.measurement import Measurement
from robot_experiments.authoring import ExperimentAuthor
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.components import list_components
from robot_experiments.configuration import materialize_run
import yaml

SRC = Path(__file__).resolve().parents[2]
fixture_spec = importlib.util.spec_from_file_location(
    'estimator_fixture_context', Path(__file__).parent / 'model_fixtures/context.py',
)
fixture_module = importlib.util.module_from_spec(fixture_spec)
fixture_spec.loader.exec_module(fixture_module)


def share(package):
    return SRC / package


def document(identifier='EXP-FADING', factor=None, base='EXP-NOMINAL'):
    experiment = {
        'id': identifier, 'name': 'Frozen fading-memory fixture', 'base_case': base,
        'components': {'robot_estimator': 'ekf', 'target_estimator': 'example_fading_kf'},
        'execution': {'seeds': [42], 'duration': 1.0},
        'runtime': {'enable_actuation': False},
    }
    if factor is not None:
        experiment['parameters'] = {
            'target_estimator': {'algorithm_parameters': {'fading_factor': factor}},
        }
    return {'schema_version': 1, 'experiment': experiment}


def numerical_result(parameters):
    estimator = create_target_estimator(parameters, 'example_fading_kf', require_frozen=True)
    initial = estimator.state
    estimator.predict(0.2)
    # Independent expected A, Q and H for the inherited 3D CV model.
    transition = np.eye(6)
    transition[:3, 3:] = 0.2 * np.eye(3)
    g = np.vstack([0.5 * 0.2 ** 2 * np.eye(3), 0.2 * np.eye(3)])
    sigma = parameters['target_acceleration_noise_std']
    expected_p = (estimator._filter.fading_factor * transition @ initial.covariance
                  @ transition.T + sigma ** 2 * g @ g.T)
    np.testing.assert_allclose(estimator.state.covariance, expected_p, atol=1e-14)
    h = np.hstack([np.eye(3), np.zeros((3, 3))])
    r = np.diag(parameters['target_position_variance'])
    gain = expected_p @ h.T @ np.linalg.inv(h @ expected_p @ h.T + r)
    measurement = Measurement(np.array([1.0, -2.0, 3.0]), r, 0.2)
    predicted_mean = transition @ initial.mean
    expected_mean = predicted_mean + gain @ (measurement.value - h @ predicted_mean)
    actual = estimator.update(measurement)
    np.testing.assert_allclose(actual.mean, expected_mean, atol=1e-14)
    return actual.mean, actual.covariance


def test_parameterized_author_snapshot_replay_and_inheritance(tmp_path):
    before = list_components()
    with fixture_module.model_context(tmp_path / 'owners', share) as isolated_share:
        catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', isolated_share)
        catalog.initialize()
        author = ExperimentAuthor(catalog)
        preview = author.preview(document())
        assert preview['components']['target_estimator']['effective']['algorithm_parameters'] == {
            'fading_factor': 1.5,
        }
        original = author.save(document(factor=2.0))
        spec = catalog.prepare('EXP-FADING', 1)
        run = tmp_path / 'run'
        run.mkdir()
        prepared = materialize_run(spec['variants'][0], 42, run, isolated_share, spec['runtime'])
        frozen = load_yaml(run / 'inputs/estimation.yaml')['estimation_node']['ros__parameters']
        assert json.loads(frozen['target_algorithm_parameters']) == {
            'algorithm': 'example_fading_kf', 'parameters': {'fading_factor': 2.0},
        }
        assert json.loads(frozen['robot_algorithm_parameters']) == {
            'algorithm': 'ekf', 'parameters': {},
        }
        first = numerical_result(frozen)
        assert prepared['plan']['estimation_parameters']['target_algorithm_parameters'] == (
            frozen['target_algorithm_parameters'])
        # A child inherits settings; a local edit wins; the base stays immutable.
        child = document('EXP-FADING-CHILD', base='EXP-FADING')
        assert author.preview(child)['components']['target_estimator']['effective'][
            'algorithm_parameters'] == {'fading_factor': 2.0}
        child['experiment']['parameters'] = {
            'target_estimator': {'algorithm_parameters': {'fading_factor': 3.0}},
        }
        author.save(child)
        assert author.preview(child)['components']['target_estimator']['effective'][
            'algorithm_parameters'] == {'fading_factor': 3.0}
        author.save(document(factor=2.5), expected_revision=1)
        assert catalog.get('EXP-FADING', 1) == original
        assert catalog.reset(catalog.draft('EXP-FADING'))['overrides'] == {}
        # A different role's settings stay independently frozen.
        assert json.loads(frozen['robot_algorithm_parameters'])['parameters'] == {}
        path = isolated_share('robot_estimation') / 'config/estimation.yaml'
        data = load_yaml(path)
        data['estimators']['example_fading_kf']['parameters']['fading_factor'] = 3.75
        path.write_text(yaml.safe_dump(data))
        from robot_bringup.factory.system_configuration import load_system_configuration
        replay = load_system_configuration(run / 'inputs/system.yaml', isolated_share)
        second = numerical_result(replay['estimation_parameters'])
        np.testing.assert_array_equal(first[0], second[0])
        np.testing.assert_array_equal(first[1], second[1])
        changed = dict(frozen)
        envelope = json.loads(changed['target_algorithm_parameters'])
        envelope['parameters']['fading_factor'] = 3.0
        changed['target_algorithm_parameters'] = json.dumps(envelope)
        assert not np.array_equal(first[0], numerical_result(changed)[0])
    assert list_components() == before


@pytest.mark.parametrize('factor', [0.0, 4.1, True, '2', float('nan'), float('inf')])
def test_bad_intrinsic_override_rejects_before_save(tmp_path, factor):
    with fixture_module.model_context(tmp_path / 'owners', share) as isolated_share:
        catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', isolated_share)
        catalog.initialize()
        with pytest.raises((ValueError, TypeError)):
            ExperimentAuthor(catalog).save(document(factor=factor))
        assert 'EXP-FADING' not in [item['id'] for item in catalog.list()]


@pytest.mark.parametrize('include_mapping', [False, True])
def test_public_author_rejects_reserved_transport_instead_of_overwriting_intrinsic_edits(
    tmp_path, include_mapping,
):
    with fixture_module.model_context(tmp_path / 'owners', share) as isolated_share:
        catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', isolated_share)
        catalog.initialize()
        doc = document(factor=2.0 if include_mapping else None)
        doc['experiment'].setdefault('parameters', {}).setdefault('target_estimator', {})[
            'target_algorithm_parameters'] = json.dumps({
                'algorithm': 'example_fading_kf', 'parameters': {'fading_factor': 3.0},
            })
        with pytest.raises(ValueError, match='reserved|transport'):
            ExperimentAuthor(catalog).compile(doc)
