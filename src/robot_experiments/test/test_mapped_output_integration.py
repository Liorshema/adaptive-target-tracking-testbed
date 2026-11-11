"""Saved mapped dynamics require compatible fields in actual frozen experiment preflight."""

from pathlib import Path

from robot_configuration.yaml_loader import load_yaml

from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.configuration import materialize_run


def test_mapped_library_pin_requires_explicit_no_field_and_materializes_full_state(tmp_path):
    """Exercise actual Author, Catalog and materialization with one frozen library pin."""
    src = Path(__file__).parents[2]

    def share(package):
        return src / package

    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    backend = ExperimentBackend(tmp_path / 'workspace', catalog=catalog)
    model = {
        'id': 'mapped_motion',
        'type': 'equation_state_space',
        'schema_version': 1,
        'frame': 'world',
        'state': [
            {'name': n, 'shape': [], 'unit': 'm', 'initial': v}
            for n, v in zip(['x', 'y', 'z'], [1.0, 0.0, 1.0])
        ],
        'parameters': {'omega': {'shape': [], 'unit': '1/s', 'value': 0.5}},
        'equations': {'x': '-omega*y', 'y': 'omega*x', 'z': '0'},
        'outputs': {'schema_version': 1, 'frame': 'world', 'position': '[x,y,z]'},
    }
    row = {'id': 'DYN-MAPPED', 'definition': model}
    original = backend.dynamics.create(row['id'], 'Circular Motion', row['definition'])
    spec = backend.draft_with_dynamics(
        backend.new_draft('EXP-MAPPED', 'Mapped', 'EXP-NOMINAL'), row['id'], revision=1
    )
    invalid = backend.validate_experiment(spec)
    assert not invalid['valid'] and 'environment' in str(invalid['errors'])
    options = backend.list_environment_options()
    assert any(o['id'] == 'environment_none' and o['active_fields'] == [] for o in options)
    spec['experiment']['resources'] = {'environment': 'environment_none'}
    check = backend.validate_experiment(spec)
    assert check['valid'], check
    saved = backend.save_experiment(spec)
    plan = catalog.prepare('EXP-MAPPED', 1)
    directory = tmp_path / 'materialized'
    directory.mkdir()
    materialize_run(plan['variants'][0], 42, directory, share, plan['runtime'])
    target = load_yaml(directory / 'inputs/target.yaml')
    environment = load_yaml(directory / 'inputs/environment.yaml')
    assert target['target']['dynamics']['parameters']['model'] == original['model']
    assert environment['environment']['acceleration_fields'] == []
    assert len(original['model']['state']) == 3
    assert catalog.get('EXP-MAPPED', 1) == saved
