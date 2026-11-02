"""Real Author/Catalog/preflight/frozen evidence integration for YAML-only models."""

from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from robot_configuration.yaml_loader import load_yaml
from robot_experiments.authoring import ExperimentAuthor
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.configuration import materialize_run
from robot_experiments.components import describe_component, validate_component
from robot_experiments.comparison import comparison_views
from robot_simulation.factory.simulation_factory import SimulationFactory

SRC = Path(__file__).resolve().parents[2]
share = lambda name: SRC / name


def authored():
    m = deepcopy(
        SimulationFactory.with_defaults().dynamics_registry.describe('equation_state_space')[
            'defaults'
        ]['model']
    )
    m.update(
        id='authored_spring',
        parameters={
            'stiffness': {'value': 2.0, 'unit': '1/s^2', 'shape': [], 'minimum': 0.0},
            'damping': {'value': 0.5, 'unit': '1/s', 'shape': [], 'minimum': 0.0},
        },
        equations={'p': 'v', 'v': '-stiffness*p-damping*v'},
    )
    return {
        'schema_version': 1,
        'experiment': {
            'id': 'EXP-EQUATION',
            'name': 'Equation test',
            'base_case': 'EXP-NOMINAL',
            'components': {'target_dynamics': 'equation_state_space'},
            'parameters': {'target_dynamics': {'model': m}},
            'execution': {'duration': 1.0, 'settling_time': 0.0, 'seeds': [42]},
            'runtime': {'enable_actuation': False, 'gui': False},
        },
    }


def test_owner_discovery_and_malformed_equations_reject_before_execution():
    info = describe_component('dynamics', 'equation_state_space')
    assert info['defaults']['model']['id'] == 'constant_velocity'
    bad = authored()['experiment']['parameters']['target_dynamics']
    bad['model']['equations']['v'] = 'missing_symbol'
    assert not validate_component('dynamics', 'equation_state_space', bad)['valid']


def test_author_catalog_revision_materialization_freezes_complete_model(tmp_path):
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    author = ExperimentAuthor(catalog)
    source = tmp_path / 'experiment.yaml'
    value = authored()
    source.write_text(yaml.safe_dump(value))
    first = author.save(source)
    assert first['revision'] == 1
    spec = catalog.prepare('EXP-EQUATION', 1)
    directory = tmp_path / 'run'
    directory.mkdir()
    prepared = materialize_run(spec['variants'][0], 42, directory, share, spec['runtime'])
    frozen = load_yaml(directory / 'inputs/target.yaml')['target']['dynamics']
    assert frozen['parameters'] == value['experiment']['parameters']['target_dynamics']
    assert prepared['plan']['identities']['dynamics'].endswith('.EquationDynamics')
    value['experiment']['parameters']['target_dynamics']['model']['parameters']['stiffness'][
        'value'
    ] = 3.0
    second = author.save(value, expected_revision=1)
    assert second['revision'] == 2 and second['sha256'] != first['sha256']
    assert catalog.get('EXP-EQUATION', 1) == first
    source.write_text('invalid: overwritten by user after save\n')
    assert (
        catalog.prepare('EXP-EQUATION', 1)['experiment_identity']['definition_sha256']
        == first['sha256']
    )
    assert load_yaml(directory / 'inputs/target.yaml')['target']['dynamics'] == frozen
    # Re-edit owner defaults; replay of the frozen complete model remains identical.
    owner = load_yaml(SRC / 'robot_simulation/config/dynamics.yaml')
    owner['dynamics']['equation_state_space']['parameters']['model']['equations']['v'] = '[0,0,0]'
    owner['dynamics']['equation_state_space']['parameters']['model']['parameters'][
        'new_default'
    ] = {'value': 1.0, 'shape': [], 'unit': '1'}
    changed = tmp_path / 'changed_defaults.yaml'
    changed.write_text(yaml.safe_dump(owner))
    registry = SimulationFactory.with_defaults(changed).dynamics_registry
    replay = registry.create(frozen)
    assert replay.definition == frozen['parameters']['model']


def test_eight_state_and_wrong_frame_rejected_during_catalog_preflight(tmp_path):
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    value = authored()
    m = value['experiment']['parameters']['target_dynamics']['model']
    m['state'] += [
        {'name': 'yaw', 'shape': [], 'unit': 'rad', 'initial': 0.0},
        {'name': 'rate', 'shape': [], 'unit': 'rad/s', 'initial': 0.0},
    ]
    m['equations'].update(yaw='rate', rate='0')
    m.pop('deployment')
    with pytest.raises(ValueError, match='deploy'):
        ExperimentAuthor(catalog).save(value)
    assert not any(r['id'] == 'EXP-EQUATION' for r in catalog.list())
    value = authored()
    value['experiment']['parameters']['target_dynamics']['model']['frame'] = 'body'
    with pytest.raises(ValueError, match='frame'):
        ExperimentAuthor(catalog).save(value)


@pytest.mark.parametrize('change', ['equation', 'parameter', 'identity'])
def test_changed_definitions_are_protected_from_predictor_comparison(change):
    model = authored()['experiment']['parameters']['target_dynamics']['model']
    base = {
        'target_dynamics': {
            'target': {
                'dynamics': {'type': 'equation_state_space', 'parameters': {'model': model}}
            }
        }
    }
    other = deepcopy(base)
    m = other['target_dynamics']['target']['dynamics']['parameters']['model']
    if change == 'equation':
        m['equations']['v'] = '-stiffness*p'
    elif change == 'parameter':
        m['parameters']['stiffness']['value'] = 4.0
    else:
        m['id'] = 'different_model'
    # No masking policy: full protected dynamics definitions and numeric values
    # participate in the existing original/invariant evidence hashes.
    a = comparison_views(base, None, {})
    b = comparison_views(other, None, {})
    assert a['original_hash'] != b['original_hash']
    from robot_prediction.configuration.prediction_config import PredictionConfig

    policy = {
        'schema_version': 2,
        'domain': 'component_models',
        'variables': [
            {
                'name': 'predictor',
                'path': 'components.predictor',
                'values': ['imm', 'constant_velocity'],
            }
        ],
    }
    conditions = {
        'component_selection': {'predictor_type': 'imm'},
        'prediction_configuration': PredictionConfig(
            SRC / 'robot_prediction/config/prediction.yaml'
        ).data,
    }
    a = comparison_views(base, policy, conditions)
    b = comparison_views(other, policy, conditions)
    assert a['invariant_hash'] != b['invariant_hash']


def test_inherited_equation_model_replacement_does_not_retain_hidden_parameters(tmp_path):
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    author = ExperimentAuthor(catalog)
    author.save(authored())
    replacement = deepcopy(
        describe_component('dynamics', 'equation_state_space')['defaults']['model']
    )
    second = authored()
    second['experiment'].update(
        id='EXP-REPLACEMENT',
        base_case='EXP-EQUATION',
        parameters={'target_dynamics': {'model': replacement}},
    )
    saved = author.save(second)
    actual = saved['overrides']['target']['target']['dynamics']['parameters']['model']
    assert actual == replacement
