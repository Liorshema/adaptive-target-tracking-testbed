"""Scenario projection preserves the actual immutable Catalog and legacy variants."""

from copy import deepcopy
from pathlib import Path

import pytest

from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import CatalogConflict, ExperimentCatalog


@pytest.fixture
def backend(tmp_path):
    src = Path(__file__).parents[2]
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', lambda name: src / name)
    catalog.initialize()
    return ExperimentBackend(tmp_path / 'workspace', catalog=catalog)


def test_legacy_variants_are_individually_selectable_and_lazily_frozen(backend):
    before = deepcopy(backend.catalog._read())
    scenarios = backend.list_scenarios()
    assert backend.catalog._read() == before  # Discovery has no persistence side effect.
    choices = [r for r in scenarios if r.get('source_id') == 'EXP-PREDICTION']
    assert {r['variant'] for r in choices} == {'imm', 'cv'}
    assert len({r['id'] for r in choices}) == 2
    for choice in choices:
        record = backend.ensure_scenario(choice['id'])
        assert [v['name'] for v in record['definition']['experiment']['variants']] == [choice['variant']]
        original = before['experiments']['EXP-PREDICTION']['revisions'][-1]
        expected = deepcopy(original['definition'])
        expected['experiment']['variants'] = [v for v in expected['experiment']['variants']
                                              if v['name'] == choice['variant']]
        assert record['definition'] == expected
        assert backend.ensure_scenario(choice['id']) == record
    after = backend.catalog._read()
    assert all(after['experiments'][key] == value for key, value in before['experiments'].items())
    assert backend.list_experiments()  # Historical API retains its full original studies.
    assert {r['id'] for r in backend.list_scenarios()} == {r['id'] for r in scenarios}


def test_schema_two_legacy_scenario_keeps_assignments_and_remains_preparable(backend):
    choice = next(r for r in backend.list_scenarios()
                  if r.get('source_id') == 'EXP-SENSOR-NOISE' and r['variant'] == 'higher_noise')
    record = backend.ensure_scenario(choice['id'])
    assert record['definition']['schema_version'] == 2
    assert record['definition']['experiment']['variants'][0]['parameters']
    prepared = backend.catalog.prepare(record['id'], record['revision'])
    assert [v['name'] for v in prepared['variants']] == ['higher_noise']


def test_scenario_management_uses_identity_and_deleted_projection_cannot_resurrect(backend):
    choice = next(r for r in backend.list_scenarios() if r.get('source_id') == 'EXP-PREDICTION')
    record = backend.ensure_scenario(choice['id'])
    row = next(r for r in backend.list_scenarios() if r['id'] == record['id'])
    backend.rename_experiment(row['id'], 'circle IMM', expected_revision=row['revision'],
                              expected_name=row['name'])
    with pytest.raises(CatalogConflict):
        backend.rename_experiment(row['id'], 'stale', expected_revision=row['revision'],
                                  expected_name=row['name'])
    backend.archive_experiment(row['id'], expected_revision=row['revision'])
    assert not any(r['id'] == row['id'] for r in backend.list_scenarios())
    assert backend.catalog.get(row['id']) == record
    assert backend.catalog.get('EXP-PREDICTION')['definition']['experiment']['variants']


def test_automatic_names_are_short_collision_safe_and_never_retarget_identity(backend):
    first = backend.create_session()
    second = backend.create_session()
    assert first['metadata']['title'] == 'session'
    assert second['metadata']['title'] == 'session 2'
    assert first['session_id'] != second['session_id']
    assert backend.unique_display_name('circle', ['circle', 'circle 2']) == 'circle 3'
