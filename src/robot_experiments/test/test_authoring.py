"""Simple authoring must use real catalog preparation and owner construction."""

from copy import deepcopy
from pathlib import Path

import pytest
from robot_configuration.yaml_loader import load_yaml
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
            'id': 'EXP-AUTHORED',
            'name': 'A simple study',
            'base_case': 'EXP-NOMINAL',
            'components': {
                'predictor': 'constant_velocity',
                'adaptation': 'none',
                'fusion': 'none',
                'controller': 'jacobian',
                'task': 'cartesian',
            },
            'parameters': {'target_dynamics': {'damping': 0.23}},
            'execution': {'seeds': [42], 'duration': 2.0, 'settling_time': 0.0},
        },
    }


def author(catalog=None):
    import robot_experiments.authoring as api

    return api.ExperimentAuthor(catalog or ExperimentCatalog(share=share))


def test_one_document_builds_saves_revises_and_preserves_source(tmp_path):
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    original = catalog.get('EXP-NOMINAL')
    before = {p: p.read_bytes() for p in SRC.rglob('*.yaml')}
    path = tmp_path / 'simple.yaml'
    path.write_text(yaml.safe_dump(document()))
    api = author(catalog)
    preview = api.preview(path)
    assert preview['components']['predictor']['id'] == 'constant_velocity'
    assert preview['components']['controller']['id'] == 'jacobian'
    assert preview['components']['target_dynamics']['effective']['damping'] == 0.23
    assert preview['components']['target_dynamics']['defaults']['damping'] == 0.5
    assert catalog.get('EXP-NOMINAL') == original
    record = api.save(path)
    assert record['revision'] == 1
    assert '/tmp/robot_composition_' not in yaml.safe_dump(record['definition'])
    assert 'sha256' in record['definition']['experiment']['description']
    spec = catalog.prepare('EXP-AUTHORED')
    run = tmp_path / 'run'
    run.mkdir()
    prepared = materialize_run(spec['variants'][0], 42, run, share, spec['runtime'])
    assert prepared['plan']['identities']['controller'].endswith('JacobianController')
    assert prepared['plan']['identities']['predictor'].endswith('ConstantVelocityPredictor')
    assert (
        load_yaml(run / 'inputs/target.yaml')['target']['dynamics']['parameters']['damping']
        == 0.23
    )
    changed = document()
    changed['experiment']['parameters']['target_dynamics']['damping'] = 0.31
    revised = api.save(changed, expected_revision=1)
    assert revised['revision'] == 2
    assert catalog.get('EXP-AUTHORED', 1) == record
    assert catalog.reset(catalog.draft('EXP-AUTHORED'))['overrides'] == {}
    assert before == {p: p.read_bytes() for p in before}


@pytest.mark.parametrize(
    'slot,selection,identity,implementation',
    [
        ('predictor', 'imm', 'predictor', 'IMMPredictor'),
        ('controller', 'weighted_least_squares', 'controller', 'WeightedLeastSquaresController'),
        ('target_dynamics', 'fight_flight', 'dynamics', 'FightFlightDynamics'),
    ],
)
def test_registered_component_choices_reach_owner_construction(
    slot, selection, identity, implementation
):
    value = document()
    value['experiment']['components'][slot] = selection
    preview = author().preview(value)
    assert preview['components'][slot]['id'] == selection
    assert preview['identities'][identity].endswith('.' + implementation)


@pytest.mark.parametrize(
    'section,key,value,path',
    [
        ('components', 'predictor', 'missing', 'components.predictor'),
        ('components', 'predictor', 'coordinated_turn', 'components.predictor'),
        ('components', 'fusion', 'weighted_correction', 'components'),
        ('parameters', 'target_dynamics', {'unknown': 1}, 'parameters.target_dynamics'),
        ('parameters', 'controller', {'gains': [1, 2]}, 'parameters.controller'),
        ('parameters', 'target_dynamics', {'damping': -1}, 'parameters.target_dynamics'),
        ('parameters', 'target_dynamics', {'damping': None}, 'parameters.target_dynamics'),
        ('components', 'unknown', 'foo', 'components.unknown'),
    ],
)
def test_invalid_inputs_have_public_paths(section, key, value, path):
    doc = document()
    doc['experiment'].setdefault(section, {})[key] = value
    with pytest.raises(ValueError, match=path.replace('.', r'\.')):
        author().preview(doc)


def test_missing_base_and_ambiguous_variant_rejected():
    doc = document()
    del doc['experiment']['base_case']
    with pytest.raises(ValueError, match='base_case'):
        author().preview(doc)
    doc['experiment']['base_case'] = 'EXP-PREDICTION-COMPOSITION'
    with pytest.raises(ValueError, match='base_variant'):
        author().preview(doc)


def test_absence_false_zero_empty_and_null_keep_owner_semantics():
    doc = document()
    doc['experiment']['parameters']['target_dynamics']['damping'] = 0.0
    doc['experiment']['runtime'] = {'enable_actuation': False}
    doc['experiment']['parameters']['predictor'] = {}
    result = author().compile(doc)
    spec = author().validate(result)
    assert spec['runtime']['enable_actuation'] is False
    assert spec['settling_time'] == 0
    assert author().preview(doc)['components']['target_dynamics']['effective']['damping'] == 0
    invalid = deepcopy(doc)
    invalid['experiment']['components']['adaptation'] = None
    with pytest.raises(ValueError, match='components.adaptation'):
        author().preview(invalid)


def test_inline_owner_document_materializes_without_mutating_inputs(tmp_path):
    from robot_experiments.composition import materialize_resource

    value = {'document': {'controller': {'type': 'jacobian', 'parameters': {'damping': 1.0}}}}
    original = deepcopy(value)
    path = materialize_resource(value, tmp_path, share)
    assert load_yaml(path) == value['document']
    assert value == original


def test_parameter_reset_restores_base_values_and_other_studies(tmp_path):
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    api = author(catalog)
    api.save(document())
    draft = catalog.reset(catalog.draft('EXP-AUTHORED'))
    catalog.revise('EXP-AUTHORED', draft['definition'], draft['overrides'], expected_revision=1)
    spec = catalog.prepare('EXP-AUTHORED')
    run = tmp_path / 'reset'
    run.mkdir()
    materialize_run(spec['variants'][0], 42, run, share, spec['runtime'])
    target = load_yaml(run / 'inputs/target.yaml')['target']['dynamics']['parameters']
    assert target['damping'] == 0.5
    baseline = author().preview(
        {
            'schema_version': 1,
            'experiment': {'id': 'EXP-UNCHANGED', 'name': 'Unchanged', 'base_case': 'EXP-NOMINAL'},
        }
    )
    assert baseline['components']['target_dynamics']['effective']['damping'] == 0.5


def test_incompatible_controller_task_is_rejected_by_owner():
    doc = document()
    doc['experiment']['components']['task'] = 'distance_pointing'
    with pytest.raises(ValueError, match='experiment.components.task.*incompatible'):
        author().preview(doc)


def test_unchanged_simple_path_has_identical_scientific_and_physical_conditions(tmp_path):
    catalog = ExperimentCatalog(share=share)
    api = author(catalog)
    value = {
        'schema_version': 1,
        'experiment': {'id': 'EXP-EQUIVALENT', 'name': 'Equivalent', 'base_case': 'EXP-NOMINAL'},
    }
    specs = [catalog.prepare('EXP-NOMINAL'), api.validate(api.compile(value))]
    results = []
    for index, spec in enumerate(specs):
        directory = tmp_path / str(index)
        directory.mkdir()
        results.append(materialize_run(spec['variants'][0], 42, directory, share, spec['runtime']))
    assert results[0]['comparison_contract'] == results[1]['comparison_contract']
    assert results[0]['scientific_conditions'] == results[1]['scientific_conditions']
    assert results[0]['plan']['identities'] == results[1]['plan']['identities']


def test_scenario_robot_environment_and_sensing_use_existing_preflight():
    value = {
        'schema_version': 1,
        'experiment': {
            'id': 'EXP-RESOURCE',
            'name': 'Existing robot scenario',
            'base_case': 'EXP-NOMINAL',
            'scenario': 'reactive_manipulation_wrist15',
            'resources': {
                'robot': {'description': 'robot_wrist15', 'deployment': 'gazebo_robot_wrist15'},
                'environment': 'environment_manipulation',
                'sensing': {
                    'measurements': {
                        'package': 'robot_simulation',
                        'path': 'config/measurements.yaml',
                    }
                },
            },
        },
    }
    api = author()
    spec = api.validate(api.compile(value))
    description = load_yaml(spec['variants'][0]['plan']['description_path'])
    assert description['robot']['manipulator']['joints']['wrist']['upper'] == pytest.approx(
        0.2617993877991494
    )


def test_discovery_and_cli_use_installed_backend(tmp_path, capsys):
    from robot_experiments.cli import main

    choices = author().discover()
    assert 'constant_velocity' in [d['id'] for d in choices['components']['predictor']]
    assert 'coordinated_turn' not in [d['id'] for d in choices['components']['predictor']]
    path = tmp_path / 'simple.yaml'
    path.write_text(yaml.safe_dump(document()))
    assert main(['author', 'validate', str(path)], share=share) == 0
    assert '"valid": true' in capsys.readouterr().out
    assert main(['author', 'preview', str(path)], share=share) == 0
    assert 'EXP-AUTHORED' in capsys.readouterr().out
    assert main(['author', 'validate'], share=share) == 2


def test_reselecting_same_components_preserves_scenario_parameters():
    api = author()
    value = {
        'schema_version': 1,
        'experiment': {
            'id': 'EXP-SAME',
            'name': 'Same selection',
            'base_case': 'EXP-NOMINAL',
            'scenario': 'reactive_tracking_oscillatory_aggressive',
        },
    }
    a = api.validate(api.compile(value))['variants'][0]['plan']
    deployment = load_yaml(a['profile']['simulation_config'])
    target = load_yaml(deployment['target']['config'])
    value['experiment']['components'] = {
        'controller': a['control_parameters']['controller_type'],
        'target_dynamics': target['target']['dynamics']['type'],
    }
    b = api.validate(api.compile(value))['variants'][0]['plan']
    deployment_b = load_yaml(b['profile']['simulation_config'])
    assert load_yaml(deployment_b['target']['config']) == target
    assert b['strategy'] == a['strategy']


def test_child_parameter_edits_merge_inherited_overrides_and_changed_slots_clear_old_parameters(
    tmp_path,
):
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    api = author(catalog)
    base = document()
    base['experiment']['parameters']['target_dynamics']['a_max'] = 0.25
    api.save(base)
    child = {
        'schema_version': 1,
        'experiment': {
            'id': 'EXP-CHILD',
            'name': 'Child',
            'base_case': 'EXP-AUTHORED',
            'parameters': {'target_dynamics': {'damping': 0.31}},
        },
    }
    result = api.compile(child)
    values = result['overrides']['target']['target']['dynamics']['parameters']
    assert values == {'a_max': 0.25, 'damping': 0.31}
    child['experiment']['components'] = {'target_dynamics': 'fight_flight'}
    changed = api.compile(child)
    assert 'a_max' not in changed['overrides']['target']['target']['dynamics']['parameters']
    api.validate(changed)


def test_empty_runtime_preserves_base_but_false_disables_actuation():
    value = {
        'schema_version': 1,
        'experiment': {
            'id': 'EXP-RUNTIME',
            'name': 'Runtime',
            'base_case': 'EXP-NOMINAL',
            'runtime': {},
        },
    }
    api = author()
    assert api.compile(value)['definition']['experiment']['runtime']['enable_actuation'] is True
    value['experiment']['runtime']['enable_actuation'] = False
    assert api.compile(value)['definition']['experiment']['runtime']['enable_actuation'] is False


def test_empty_parameters_for_disabled_components_keep_none_semantics():
    value = document()
    value['experiment']['parameters'].update({'adaptation': {}, 'fusion': {}, 'predictor': {}})
    api = author()
    spec = api.validate(api.compile(value))
    assert spec['variants'][0]['plan']['selection']['adaptation_type'] == 'none'
    assert spec['variants'][0]['plan']['selection']['fusion_type'] == 'none'
