"""Small persistent mathematical library; no Runs or package defaults are edited."""

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
from pathlib import Path

import pytest

from robot_experiments.dynamics_library import DynamicsLibrary
from robot_experiments.catalog import CatalogConflict


def model():
    return {
        'id': 'user_decay',
        'type': 'equation_state_space',
        'schema_version': 1,
        'frame': 'world',
        'state': [{'name': 'q', 'shape': [], 'unit': '1', 'initial': 1.0}],
        'parameters': {'rate': {'value': 0.2, 'unit': '1/s', 'shape': []}},
        'equations': {'q': '-rate*q'},
    }


def test_two_models_immutable_revisions_and_pinned_resolution(tmp_path):
    path = tmp_path / 'dynamics_library.yaml'
    library = DynamicsLibrary(path)
    assert library.list() == [] and not path.exists()
    a = library.create('DYN-FIRST', 'First', model(), created_by='test engineer')
    b = library.create('DYN-SECOND', 'Second', model())
    assert len(library.list()) == 2 and a['sha256'] == b['sha256']
    changed = model()
    changed['parameters']['rate']['value'] = 0.3
    new = library.revise('DYN-FIRST', changed, expected_revision=1)
    assert new['revision'] == 2 and new['sha256'] != a['sha256']
    assert library.get('DYN-FIRST', 1) == a
    assert library.resolve('DYN-FIRST', 1)['model'] == a['model']
    assert library.revisions('DYN-FIRST')[0]['sha256'] == a['sha256']
    assert library.capabilities('DYN-FIRST', 1)['deployable'] is False
    with pytest.raises(ValueError):
        library.resolve('DYN-FIRST', None)
    with pytest.raises(CatalogConflict):
        library.revise('DYN-FIRST', model(), expected_revision=1)
    with pytest.raises(CatalogConflict):
        library.create('DYN-FIRST', 'Duplicate', model())
    reloaded = DynamicsLibrary(path)
    assert reloaded.get('DYN-FIRST', 1) == a
    detached = reloaded.get('DYN-FIRST', 1)
    detached['model']['id'] = 'changed'
    assert reloaded.get('DYN-FIRST', 1) == a
    old_hash = library.get('DYN-FIRST', 2)['sha256']
    library.rename('DYN-FIRST', 'Readable name', expected_revision=2, expected_name='First')
    assert library.list()[0]['name'] == 'Readable name'
    assert library.get('DYN-FIRST', 2)['sha256'] == old_hash
    with pytest.raises(CatalogConflict):
        library.rename('DYN-FIRST', 'Other', expected_revision=2, expected_name='First')


def test_concurrent_edits_reject_stale_writer(tmp_path):
    library = DynamicsLibrary(tmp_path / 'library.yaml')
    library.create('DYN-ONE', 'One', model())

    def write(value):
        definition = model()
        definition['parameters']['rate']['value'] = value
        try:
            return library.revise('DYN-ONE', definition, expected_revision=1)
        except CatalogConflict:
            return None

    with ThreadPoolExecutor(max_workers=2) as executor:
        results = list(executor.map(write, [0.3, 0.4]))
    assert sum(r is not None for r in results) == 1
    assert len(library.revisions('DYN-ONE')) == 2


def test_failed_atomic_write_preserves_last_revision_and_cleans_temporary(tmp_path, monkeypatch):
    from robot_configuration import yaml_store

    library = DynamicsLibrary(tmp_path / 'library.yaml')
    library.create('DYN-ONE', 'One', model())
    before = library.path.read_bytes()

    def fail(*args):
        raise OSError('injected replacement failure')

    monkeypatch.setattr(yaml_store.os, 'replace', fail)
    with pytest.raises(OSError):
        library.revise('DYN-ONE', model(), expected_revision=1)
    assert library.path.read_bytes() == before
    assert not list(tmp_path.glob('.library.yaml*'))


@pytest.mark.parametrize(
    'mutation',
    [
        lambda m: m['equations'].update(q='missing'),
        lambda m: m.update(noise=True),
        lambda m: m['parameters']['rate'].update(value=float('nan')),
    ],
)
def test_invalid_drafts_never_persist(mutation, tmp_path):
    library = DynamicsLibrary(tmp_path / 'library.yaml')
    m = model()
    mutation(m)
    assert not library.validate(m)['valid']
    with pytest.raises(ValueError):
        library.create('DYN-INVALID', 'Bad', m)
    assert not library.path.exists()


def test_hash_corruption_and_owner_default_path_are_rejected(tmp_path):
    import yaml
    from robot_configuration.resources import package_config_path
    from robot_simulation.dynamics import equation_model

    library = DynamicsLibrary(tmp_path / 'library.yaml')
    library.create('DYN-ONE', 'One', model())
    data = yaml.safe_load(library.path.read_text())
    data['models']['DYN-ONE']['revisions'][0]['model']['id'] = 'tampered'
    library.path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='hash'):
        library.get('DYN-ONE', 1)
    protected = package_config_path(equation_model.__file__, 'robot_simulation', 'dynamics.yaml')
    with pytest.raises(ValueError):
        DynamicsLibrary(protected)


def test_composed_serialization_preserves_derivative_and_source_mapping(tmp_path):
    import numpy as np
    from robot_simulation.dynamics.equation_model import EquationDynamics

    library = DynamicsLibrary(tmp_path / 'library.yaml')
    definition = library.templates()['spiral']['definition']
    before = library.validate(definition)
    first = library.create('DYN-SPIRAL', 'Spiral', definition)
    reloaded = DynamicsLibrary(library.path).resolve('DYN-SPIRAL', 1)
    assert reloaded['model'] == before['model']
    assert reloaded['source']['symbol_map'] == before['symbol_map']
    assert first['definition'] == definition
    x = [1.0, 0.2, 0.7, 0.1, -0.3, 0.2]
    a = EquationDynamics(model=before['model']).derivative(x, [], 0.0, {})
    b = EquationDynamics(model=reloaded['model']).derivative(x, [], 0.0, {})
    np.testing.assert_array_equal(a, b)


def test_source_aliases_detach_and_nonserializable_drafts_fail_cleanly(tmp_path):
    import numpy as np

    library = DynamicsLibrary(tmp_path / 'library.yaml')
    m = model()
    m['parameters']['second'] = m['parameters']['rate']
    record = library.create('DYN-ALIASES', 'Detached parameters', m)
    record['definition']['parameters']['rate']['value'] = 9.0
    assert record['definition']['parameters']['second']['value'] == 0.2
    bad = model()
    bad['state'][0]['initial'] = np.array(1.0)
    assert not library.validate(bad)['valid']


def test_serialization_cannot_silently_rename_nonstring_symbols(tmp_path):
    library = DynamicsLibrary(tmp_path / 'library.yaml')
    m = model()
    m['parameters'] = {True: {'value': 0.2, 'unit': '1/s', 'shape': []}}
    m['equations']['q'] = '-true*q'
    assert not library.validate(m)['valid']


def test_revision_and_label_change_share_atomic_validation_and_cas(tmp_path):
    library = DynamicsLibrary(tmp_path / 'library.yaml')
    first = library.create('DYN-LABEL', 'First', model())
    library.rename('DYN-LABEL', 'Concurrent label', expected_revision=1, expected_name='First')
    before = library.path.read_bytes()
    with pytest.raises(CatalogConflict, match='label changed'):
        library.revise('DYN-LABEL', model(), expected_revision=1,
                       name='My label', expected_name='First')
    assert library.path.read_bytes() == before
    invalid = model()
    invalid['equations']['q'] = 'missing'
    with pytest.raises(ValueError):
        library.revise('DYN-LABEL', invalid, expected_revision=1,
                       name='My label', expected_name='Concurrent label')
    assert library.path.read_bytes() == before
    changed = model()
    changed['parameters']['rate']['value'] = .3
    second = library.revise('DYN-LABEL', changed, expected_revision=1,
                            name='My label', expected_name='Concurrent label')
    assert second['revision'] == 2
    assert library.list()[0]['name'] == 'My label'
    assert library.get('DYN-LABEL', 1) == first
