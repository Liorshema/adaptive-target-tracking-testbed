"""Role-neutral estimator choices use the public authoring and deployment path."""

from pathlib import Path

import numpy as np
import pytest
from robot_configuration.yaml_loader import load_yaml
from robot_estimation.factory.estimator_factory import create_target_estimator
from robot_experiments.authoring import ExperimentAuthor
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.configuration import materialize_run
import yaml

SRC = Path(__file__).resolve().parents[2]


def share(package):
    return SRC / package


def document():
    return {
        'schema_version': 1,
        'experiment': {
            'id': 'EXP-ESTIMATION',
            'name': 'Canonical estimation',
            'base_case': 'EXP-NOMINAL',
            'components': {'robot_estimator': 'ekf', 'target_estimator': 'kf'},
            'execution': {'seeds': [42], 'duration': 1.0},
            'runtime': {'enable_actuation': False},
        },
    }


def test_canonical_authoring_actual_construction_revisions_and_snapshot(tmp_path):
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    author = ExperimentAuthor(catalog)
    old = catalog.get('EXP-NOMINAL')
    before = {p: p.read_bytes() for p in SRC.rglob('*.yaml')}
    doc = document()
    choices = author.discover()['components']
    assert {'ekf', 'robot_ekf'} <= {v['id'] for v in choices['robot_estimator']}
    assert {'kf', 'kalman'} <= {v['id'] for v in choices['target_estimator']}
    doc['experiment']['estimation_bindings'] = {
        'robot': {'process_model': 'robot_motion', 'measurement_model': 'robot_sensors'},
        'target': {
            'process_model': 'target_cv',
            'measurement_model': 'target_position',
            'adapter': 'target_state',
        },
    }
    path = tmp_path / 'experiment.yaml'
    path.write_text(yaml.safe_dump(doc))
    preview = author.preview(path)
    assert preview['components']['robot_estimator']['id'] == 'ekf'
    assert preview['components']['target_estimator']['id'] == 'kf'
    record = author.save(path)
    spec = catalog.prepare('EXP-ESTIMATION', 1)
    run = tmp_path / 'run'
    run.mkdir()
    result = materialize_run(spec['variants'][0], 42, run, share, spec['runtime'])
    params = result['plan']['estimation_parameters']
    assert result['plan']['selection']['robot_estimator_type'] == 'ekf'
    assert result['plan']['selection']['target_estimator_type'] == 'kf'
    assert params['robot_process_model'] == 'robot_motion'
    assert params['target_measurement_model'] == 'target_position'
    frozen = load_yaml(run / 'inputs/estimation.yaml')['estimation_node']['ros__parameters']
    assert frozen['target_process_model'] == 'target_cv'
    first = create_target_estimator(frozen, 'kf')
    second = create_target_estimator(frozen, 'kalman')
    np.testing.assert_array_equal(first.predict(0.2).covariance, second.predict(0.2).covariance)
    doc['experiment'].setdefault('parameters', {})['target_estimator'] = {
        'target_acceleration_noise_std': 0.2
    }
    author.save(doc, expected_revision=1)
    assert catalog.get('EXP-ESTIMATION', 1) == record
    assert catalog.get('EXP-NOMINAL') == old
    assert catalog.reset(catalog.draft('EXP-ESTIMATION'))['overrides'] == {}
    assert '/tmp/robot_composition_' not in yaml.safe_dump(record['definition'])
    assert before == {p: p.read_bytes() for p in before}


@pytest.mark.parametrize(
    'binding',
    [
        {'process_model': 'missing', 'measurement_model': 'target_position'},
        {'process_model': 'robot_motion', 'measurement_model': 'target_position'},
        {'process_model': 'target_cv', 'measurement_model': 'robot_sensors'},
        {
            'process_model': 'target_cv',
            'measurement_model': 'target_position',
            'adapter': 'robot_state',
        },
        {'measurement_model': 'target_position'},
    ],
)
def test_incompatible_explicit_binding_rejects_before_save(tmp_path, binding):
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    doc = document()
    doc['experiment']['estimation_bindings'] = {'target': binding}
    with pytest.raises((ValueError, KeyError)):
        ExperimentAuthor(catalog).save(doc)
    assert 'EXP-ESTIMATION' not in {record['id'] for record in catalog.list()}


def test_trusted_algorithm_registration_reaches_public_role_selection(tmp_path, monkeypatch):
    import sys
    import robot_estimation.configuration as configuration
    import robot_estimation.filters as namespace
    from robot_experiments.components import list_components

    owner = tmp_path / 'owner'
    owner.mkdir()
    data = load_yaml(SRC / 'robot_estimation/config/estimation.yaml')
    data['estimators']['test_linear_observer'] = {'parameters': {}}
    path = owner / 'estimation.yaml'
    path.write_text(yaml.safe_dump(data))
    (owner / 'observer.py').write_text("""
from robot_configuration.model_registration import ModelRegistration
from robot_estimation.factory.algorithm_factory import COMPONENTS as BUILTINS
original = next(d for d in BUILTINS if d.identifier == 'kf')
COMPONENTS = (ModelRegistration('test_linear_observer', original.builder,
    original.schema, {**original.capabilities, 'runtime_validation': 'unit_tested_extension'},
    dependencies=original.dependencies),)
""")
    before = list_components()
    monkeypatch.setattr(namespace, '__path__', [*namespace.__path__, str(owner)])
    monkeypatch.setattr(configuration, 'estimation_configuration_path', lambda: path)
    try:
        catalog = ExperimentCatalog(tmp_path / 'plugin_catalog.yaml', share)
        catalog.initialize()
        original = catalog.get('EXP-NOMINAL')
        overrides = original['overrides']
        overrides.setdefault('estimation', {})['target_acceleration_noise_std'] = 0.37
        catalog.create(
            'EXP-ESTIMATION-BASE', 'Observer model baseline', original['definition'], overrides
        )
        api = ExperimentAuthor(catalog)
        doc = document()
        doc['experiment']['base_case'] = 'EXP-ESTIMATION-BASE'
        doc['experiment']['components']['target_estimator'] = 'test_linear_observer'
        preview = api.preview(doc)
        assert preview['components']['target_estimator']['id'] == 'test_linear_observer'
        assert preview['identities']['target_estimator'].endswith('.TargetStateEstimator')
        assert (
            preview['components']['target_estimator']['effective']['target_acceleration_noise_std']
            == 0.37
        )
        result = create_target_estimator({}, 'test_linear_observer')
        assert type(result._filter).__name__ == 'KalmanFilter'
    finally:
        sys.modules.pop('robot_estimation.filters.observer', None)
        monkeypatch.undo()
    assert list_components() == before
