"""Test-only equations must traverse public authoring and real owner factories."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
from robot_configuration.yaml_loader import load_yaml
from robot_experiments.authoring import ExperimentAuthor
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.components import list_components
from robot_experiments.configuration import materialize_run
from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.factory.system_factory import PredictionSystemFactory
from robot_prediction.interfaces.prediction_data import PredictionInput
import yaml

SRC = Path(__file__).resolve().parents[2]
fixture_spec = importlib.util.spec_from_file_location(
    'model_fixture_context', Path(__file__).parent / 'model_fixtures/context.py'
)
fixture_module = importlib.util.module_from_spec(fixture_spec)
fixture_spec.loader.exec_module(fixture_module)


def share(package):
    return SRC / package


def simple():
    return {
        'schema_version': 1,
        'experiment': {
            'id': 'EXP-MATH',
            'name': 'Test-only mathematical composition',
            'base_case': 'EXP-NOMINAL',
            'components': {
                'target_dynamics': 'example_spring',
                'predictor': 'example_forecast',
                'adaptation': 'none',
                'fusion': 'none',
                'controller': 'example_feedback',
                'task': 'cartesian',
                'robot_estimator': 'robot_ekf',
                'target_estimator': 'kalman',
            },
            'parameters': {
                'target_dynamics': {'damping': 0.3},
                'predictor': {'acceleration': 0.2},
                'controller': {'gain': 0.03},
            },
            'execution': {'seeds': [42], 'duration': 1.0},
            'runtime': {'enable_actuation': False},
        },
    }


def test_public_one_file_selects_builds_previews_saves_and_revises_models(tmp_path):
    before = list_components()
    defaults = {p: p.read_bytes() for p in SRC.rglob('*.yaml')}
    with fixture_module.model_context(tmp_path / 'owners', share) as isolated_share:
        catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', isolated_share)
        catalog.initialize()
        original = catalog.get('EXP-NOMINAL')
        author = ExperimentAuthor(catalog)
        choices = author.discover()['components']
        for slot, identifier in simple()['experiment']['components'].items():
            assert identifier in [item['id'] for item in choices[slot]]
        path = tmp_path / 'experiment.yaml'
        path.write_text(yaml.safe_dump(simple()))
        preview = author.preview(path)
        assert preview['components']['predictor']['effective']['acceleration'] == 0.2
        assert preview['components']['controller']['effective']['gain'] == 0.03
        assert preview['components']['target_dynamics']['effective']['damping'] == 0.3
        assert preview['identities']['predictor'].endswith('.StatelessPredictor')
        assert preview['identities']['controller'].endswith('._VelocityFeedbackController')
        assert preview['identities']['dynamics'].endswith('.AccelerationDynamics')
        record = author.save(path)
        spec = catalog.prepare('EXP-MATH', 1)
        run = tmp_path / 'run'
        run.mkdir()
        prepared = materialize_run(spec['variants'][0], 42, run, isolated_share, spec['runtime'])
        configuration = PredictionConfig(run / 'inputs/prediction.yaml')
        prediction = PredictionSystemFactory(configuration).create(
            predictor_type='example_forecast', adaptation_type='none', fusion_type='none'
        )
        value = prediction.predict(
            PredictionInput(np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]), 10.0, 0.5, 2)
        )
        np.testing.assert_array_equal(value.timestamps, [10.5, 11.0])
        np.testing.assert_allclose(value.states[1], [5.1, 7.1, 9.1, 4.2, 5.2, 6.2])
        assert value.covariance is None and value.confidence is None
        dynamics = load_yaml(run / 'inputs/target.yaml')['target']['dynamics']
        assert dynamics['type'] == 'example_spring'
        assert dynamics['parameters'] == {'stiffness': 0.4, 'damping': 0.3}
        assert not spec['runtime']['enable_actuation']
        assert prepared['plan']['identities'] == spec['variants'][0]['plan']['identities']
        changed = simple()
        changed['experiment']['parameters']['predictor']['acceleration'] = 0.4
        author.save(changed, expected_revision=1)
        assert catalog.get('EXP-MATH', 1) == record
        assert catalog.get('EXP-NOMINAL') == original
        assert catalog.reset(catalog.draft('EXP-MATH'))['overrides'] == {}
        assert 'Authoring provenance' in record['definition']['experiment']['description']
        assert '/tmp/robot_composition_' not in yaml.safe_dump(record['definition'])
    assert list_components() == before
    assert defaults == {p: p.read_bytes() for p in defaults}


@pytest.mark.parametrize(
    'slot,values',
    [
        ('target_dynamics', {'damping': -1.0}),
        ('predictor', {'acceleration': 'fast'}),
        ('controller', {'gain': float('inf')}),
        ('controller', {'unknown': 1.0}),
    ],
)
def test_invalid_math_overrides_reject_before_save(tmp_path, slot, values):
    with fixture_module.model_context(tmp_path / 'owners', share) as isolated_share:
        catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', isolated_share)
        catalog.initialize()
        doc = simple()
        doc['experiment']['parameters'][slot] = values
        with pytest.raises(ValueError):
            ExperimentAuthor(catalog).save(doc)
        assert 'EXP-MATH' not in [item['id'] for item in catalog.list()]
