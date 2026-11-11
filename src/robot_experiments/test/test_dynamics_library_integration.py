"""Library pins enter the actual Author/Catalog pipeline as full frozen D3 data."""

from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest

from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.configuration import materialize_run
from robot_configuration.yaml_loader import load_yaml
from robot_simulation.dynamics.equation_model import EquationDynamics

SRC = Path(__file__).parents[2]
share = lambda package: SRC / package


def api(tmp_path):
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    return ExperimentBackend(tmp_path / 'workspace', catalog=catalog)


def test_two_user_models_saved_experiment_and_library_edits_are_independent(tmp_path):
    backend = api(tmp_path)
    templates = backend.dynamics.templates()
    first = backend.dynamics.create('DYN-SPIRAL', 'User spiral', templates['spiral']['definition'])
    backend.dynamics.create(
        'DYN-GENERAL', 'User standalone', templates['blank_state_space']['definition']
    )
    draft = backend.new_draft('EXP-LIBRARY', 'Frozen library example', 'EXP-NOMINAL')
    selected = backend.draft_with_dynamics(draft, 'DYN-SPIRAL', revision=1)
    assert 'parameters' not in draft['experiment']
    assert backend.validate_experiment(selected)['valid']
    saved = backend.save_experiment(selected)
    assert 'DYN-SPIRAL' in saved['definition']['experiment']['description']
    spec = backend.catalog.prepare('EXP-LIBRARY', 1)
    directory = tmp_path / 'prepared'
    directory.mkdir()
    materialize_run(spec['variants'][0], 42, directory, share, spec['runtime'])
    frozen = load_yaml(directory / 'inputs/target.yaml')['target']['dynamics']['parameters'][
        'model'
    ]
    assert frozen == first['model']
    revised = deepcopy(templates['spiral']['definition'])
    revised['base']['parameters']['stiffness']['value'] = 0.7
    backend.dynamics.revise('DYN-SPIRAL', revised, expected_revision=1)
    backend.dynamics.rename(
        'DYN-SPIRAL', 'Renamed', expected_revision=2, expected_name='User spiral'
    )
    assert backend.catalog.get('EXP-LIBRARY', 1) == saved
    assert (
        load_yaml(directory / 'inputs/target.yaml')['target']['dynamics']['parameters']['model']
        == frozen
    )
    assert (
        backend.catalog.prepare('EXP-LIBRARY', 1)['experiment_identity']['definition_sha256']
        == saved['sha256']
    )
    # Future library/compiler edits cannot reinterpret the saved canonical pin.
    assert backend.dynamics.resolve('DYN-SPIRAL', 1)['model'] == frozen
    with pytest.raises(ValueError):
        backend.draft_with_dynamics(draft, 'DYN-SPIRAL', revision=None)
    selected['experiment']['parameters']['target_dynamics']['model'][
        'id'
    ] = 'edited_after_selection'
    assert not backend.validate_experiment(selected)['valid']


def test_composed_selection_can_freeze_source_mapping_and_reject_advanced_rewrite(tmp_path):
    backend = api(tmp_path)
    templates = backend.dynamics.templates()
    source = deepcopy(templates['spiral']['definition'])
    force = deepcopy(templates['saturated_fight_flight']['definition']['base'])
    force['id'] = 'robot_response'
    force['parameters']['damping']['value'] = 0.0
    source['interactions'] = [force]
    entry = backend.dynamics.create('DYN-COMBINED', 'Combined', source)
    draft = backend.draft_with_dynamics(
        backend.new_draft('EXP-COMBINED', 'Combined', 'EXP-NOMINAL'), 'DYN-COMBINED', revision=1
    )
    assert draft['experiment']['dynamics_source']['symbol_map'] == entry['symbol_map']
    assert backend.validate_experiment(draft)['valid']
    changed = deepcopy(entry['model'])
    changed['id'] = 'hidden_rewrite'
    draft['experiment']['advanced'] = {
        'overrides': {'target': {'target': {'dynamics': {'parameters': {'model': changed}}}}}
    }
    assert not backend.validate_experiment(draft)['valid']


def test_library_only_model_and_custom_path_have_honest_deployment_capabilities(tmp_path):
    backend = api(tmp_path)
    definition = backend.dynamics.templates()['blank_state_space']['definition']
    definition['state'][0].update(shape=[8], initial=[0.0] * 8)
    definition['equations']['q'] = '[0,0,0,0,0,0,0,0]*rate'
    entry = backend.dynamics.create('DYN-EIGHT', 'Eight', definition)
    assert EquationDynamics(model=entry['model']).state_dimension == 8
    assert not backend.dynamics.capabilities('DYN-EIGHT', 1)['deployable']
    draft = backend.draft_with_dynamics(
        backend.new_draft('EXP-EIGHT', 'Eight', 'EXP-NOMINAL'), 'DYN-EIGHT', revision=1
    )
    assert not backend.validate_experiment(draft)['valid']
    custom = tmp_path / 'other' / 'math.yaml'
    other = ExperimentBackend(
        tmp_path / 'second', catalog=backend.catalog, dynamics_library_path=custom
    )
    assert other.dynamics.path == custom.resolve() and not custom.exists()
