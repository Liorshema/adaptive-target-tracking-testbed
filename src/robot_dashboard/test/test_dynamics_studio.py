"""Studio forms delegate mathematics, revisions and freezing to existing owners."""

from copy import deepcopy
from pathlib import Path

import pytest

from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog, CatalogConflict


@pytest.fixture
def backend(tmp_path):
    src = Path(__file__).resolve().parents[2]
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share=lambda p: src / p)
    catalog.initialize()
    return ExperimentBackend(tmp_path / 'data', catalog=catalog)


def test_structured_forms_roundtrip_all_templates_without_changing_semantics(backend):
    from robot_dashboard.dynamics_forms import editor_data, read_editor

    for entry in backend.dynamics.templates().values():
        source = entry['definition']
        count = 1 + len(source.get('interactions', []))
        for index in range(count):
            fields, tables = editor_data(source, index)
            assert read_editor(source, index, fields, tables) == source


def test_structured_edits_keep_constraints_and_reject_duplicate_rows(backend):
    from robot_dashboard.dynamics_forms import editor_data, read_editor

    source = backend.dynamics.templates()['spiral']['definition']
    before = deepcopy(source)
    fields, tables = editor_data(source, 0)
    row = next(r for r in tables['parameters'] if r['name'] == 'damping')
    row['value'] = '0.23'
    fields['expression'] += '+[0,0,0]'
    edited = read_editor(source, 0, fields, tables)
    assert edited['base']['parameters']['damping'] == {
        'value': 0.23,
        'unit': '1/s',
        'shape': [],
        'minimum': 0.0,
    }
    assert source == before
    tables['parameters'].append(deepcopy(row))
    with pytest.raises(ValueError, match='Duplicate.*damping'):
        read_editor(source, 0, fields, tables)


def test_interaction_drag_is_explicit_and_draft_only(backend):
    from robot_dashboard.dynamics_forms import add_interaction

    source = backend.dynamics.templates()['spiral']['definition']
    before = deepcopy(source)
    combined = add_interaction(
        source, backend.dynamics.templates(), 'saturated_fight_flight', zero_drag=True
    )
    assert combined['interactions'][0]['parameters']['damping']['value'] == 0.0
    assert combined['base']['parameters']['damping']['value'] == 0.1
    assert source == before
    compiled = backend.dynamics.validate(combined)
    assert compiled['valid'], compiled
    assert len(compiled['symbol_map']) == 2
    assert (
        len(
            add_interaction(combined, backend.dynamics.templates(), 'custom', zero_drag=False)[
                'interactions'
            ]
        )
        == 2
    )


def test_edit_old_revision_cannot_overwrite_new_and_rename_keeps_hash(backend):
    from robot_dashboard.dynamics_forms import load_model, save_model

    source = backend.dynamics.templates()['spiral']['definition']
    first = backend.dynamics.create('DYN-ONE', 'One', source)
    old = load_model(backend, 'DYN-ONE', 1)
    source['base']['parameters']['damping']['value'] = 0.25
    backend.dynamics.revise('DYN-ONE', source, expected_revision=1)
    with pytest.raises(CatalogConflict):
        save_model(backend, old, 'DYN-ONE', 'One', old['definition'])
    assert backend.dynamics.get('DYN-ONE', 1) == first
    latest = load_model(backend, 'DYN-ONE', 2)
    backend.dynamics.rename(
        'DYN-ONE',
        'Renamed',
        expected_revision=latest['expected_revision'],
        expected_name=latest['name'],
    )
    assert backend.dynamics.get('DYN-ONE', 1)['sha256'] == first['sha256']


def test_structured_general_keeps_all_states_and_fixed_inputs(backend):
    from robot_dashboard.dynamics_forms import editor_data, read_editor, preview_inputs

    source = backend.dynamics.templates()['blank_state_space']['definition']
    source['control'] = [{'name': 'u', 'shape': [], 'unit': '1/s', 'frame': 'world'}]
    source['context'] = {'offset': {'shape': [], 'unit': '1/s', 'frame': 'world'}}
    source['equations']['q'] = 'u+offset'
    fields, tables = editor_data(source, 0)
    assert read_editor(source, 0, fields, tables) == source
    checked = backend.dynamics.validate(source)
    assert checked['valid'] and not checked['capabilities']['deployable']
    rows = preview_inputs(checked['model'])
    assert {r['name'] for r in rows} == {'u', 'offset'}
    assert all(r['fixed'] == '' for r in rows)  # no silent generic input assumptions


def test_studio_and_pinned_source_are_mounted_in_real_dash(backend):
    from robot_dashboard.app import create_app

    app = create_app(backend)
    client = app.server.test_client()
    data = client.get('/_dash-layout').get_data(as_text=True)
    for name in [
        'dynamics-studio',
        'dyn-editor',
        'dyn-preview',
        'dyn-confirm',
        'dyn-math',
        'dyn-library-events',
    ]:
        assert name in data, name
    assert any('dyn-state.data' in key for key in app.callback_map)


def test_library_selection_preserves_pin_during_unrelated_form_edits(backend):
    from robot_dashboard.forms import assemble_draft

    source = backend.dynamics.templates()['spiral']['definition']
    backend.dynamics.create('DYN-PINNED', 'Pinned', source)
    spec = backend.draft_with_dynamics(
        backend.new_draft('EXP-UI-PIN', 'Pinned', 'EXP-NOMINAL'), 'DYN-PINNED', revision=1
    )
    edited = assemble_draft(spec, [], [], [], [], [])
    assert edited['experiment']['dynamics_source'] == spec['experiment']['dynamics_source']
    assert (
        edited['experiment']['parameters']['target_dynamics']['model']
        == spec['experiment']['parameters']['target_dynamics']['model']
    )


def test_corrupt_library_keeps_other_workspaces_available_and_reports_error(backend):
    from robot_dashboard.app import create_app

    backend.dynamics.path.parent.mkdir(parents=True, exist_ok=True)
    backend.dynamics.path.write_text('schema_version: 999\nmodels: {}\ngeneration: 0\n')
    app = create_app(backend)
    response = app.server.test_client().get('/_dash-layout')
    assert response.status_code == 200
    data = response.get_data(as_text=True)
    assert 'Library unavailable' in data
    assert 'Experiment Lab' in data and 'Results & History' in data


def studio_action(app, state, trigger, *, identifier='DYN-HTTP', name='HTTP motion', edit=None):
    """Call the real Dash action endpoint with the mounted structured-form payload."""
    from robot_dashboard.dynamics_forms import editor_data

    fields, tables = editor_data(state['definition'], state.get('selected', 0))
    if edit:
        fields.update(edit)
    key, callback = next(
        (k, v) for k, v in app.callback_map.items() if k.startswith('..dyn-state.data')
    )
    state = {**state, 'new_id': identifier}
    inputs = [None]*10 + [state.get('id'), None, list(fields.values()), list(tables.values()), 0, name]
    states = [
        [{'type':'dyn-field','key':k} for k in fields],
        [{'type':'dyn-table','key':k} for k in tables],
        'spiral', 'saturated_fight_flight', ['zero'], .2, .1, [], state, 0]
    response = app.server.test_client().post(
        '/_dash-update-component',
        json={
            'output': key,
            'outputs': [
                {'id': o.component_id, 'property': o.component_property}
                for o in callback['output']
            ],
            'inputs': [{**m, 'value': v} for m, v in zip(callback['inputs'], inputs)],
            'state': [{**m, 'value': v} for m, v in zip(callback['state'], states)],
            'changedPropIds': [trigger + '.n_clicks'],
        },
    )
    assert response.status_code == 200, response.get_data(as_text=True)
    return response.get_json()['response']['dyn-state']['data']


def test_http_studio_save_conflict_and_invalid_equation_keep_saved_revisions(backend):
    from robot_dashboard.app import create_app
    from robot_dashboard.dynamics_forms import load_model

    app = create_app(backend)
    state = {
        'definition': backend.dynamics.templates()['spiral']['definition'],
        'id': None,
        'name': 'HTTP motion',
        'selected': 0,
        'dirty': False,
        'stage': 'Template',
        'expected_revision': None,
    }
    saved = studio_action(app, state, 'dyn-save')
    assert saved['stage'] == 'Saved'
    first = backend.dynamics.get('DYN-HTTP', 1)
    invalid = studio_action(app, saved, 'dyn-save', edit={'expression': '__import__("os")'})
    assert invalid['stage'] == 'Invalid'
    assert backend.dynamics.get('DYN-HTTP', 1) == first
    assert len(backend.dynamics.revisions('DYN-HTTP')) == 1
    backend.dynamics.revise('DYN-HTTP', first['definition'], expected_revision=1)
    conflict = studio_action(app, saved, 'dyn-save')
    assert conflict['stage'] == 'Invalid' and 'reload' in conflict['message']
    assert conflict['definition'] == saved['definition']
    latest = load_model(backend, 'DYN-HTTP', 2)
    renamed = studio_action(app, latest, 'dyn-save', name='New label')
    assert not renamed['dirty']
    assert backend.dynamics.get('DYN-HTTP', 2)['sha256'] == first['sha256']


def test_pinned_library_card_uses_existing_role_grouping_and_never_raw_model_editor(backend):
    from robot_dashboard.app import component_cards
    from robot_dashboard.forms import begin_draft

    state = begin_draft(backend, 'EXP-NOMINAL', 3, 'imm', 'create', 'EXP-PIN-CARD', 'Pinned')
    state['inherited_components'] = deepcopy(state['components'])
    backend.dynamics.create(
        'DYN-CARD', 'Card', backend.dynamics.templates()['spiral']['definition']
    )
    state['spec'] = backend.draft_with_dynamics(state['spec'], 'DYN-CARD', revision=1)
    cards = component_cards(backend, state)
    target = next(
        c for c in cards if c.to_plotly_json()['props'].get('data-role') == 'target_dynamics'
    )
    assert 'DYN-CARD' in str(target)
    assert 'dyn-field' not in str(target)


def test_save_keeps_selected_interaction_and_its_editor_identity(backend):
    from robot_dashboard.dynamics_forms import add_interaction, save_model

    definition = add_interaction(
        backend.dynamics.templates()['spiral']['definition'],
        backend.dynamics.templates(),
        'saturated_fight_flight',
        zero_drag=True,
    )
    state = {'id': None, 'name': 'Combined', 'selected': 1, 'definition': definition}
    saved = save_model(backend, state, 'DYN-SELECTED', 'Combined', definition)
    assert saved['selected'] == 1
    assert not saved['dirty']


def test_open_library_requires_an_explicit_selected_revision(backend):
    from robot_dashboard.dynamics_forms import load_model

    backend.dynamics.create(
        'DYN-EXPLICIT', 'Explicit', backend.dynamics.templates()['spiral']['definition']
    )
    with pytest.raises(ValueError, match='explicit|Select'):
        load_model(backend, 'DYN-EXPLICIT', None)


def test_revision_save_does_not_silently_discard_a_display_name_edit(backend):
    from robot_dashboard.dynamics_forms import load_model, save_model
    source=backend.dynamics.templates()['spiral']['definition']
    backend.dynamics.create('DYN-LABEL', 'Original', source)
    state=load_model(backend, 'DYN-LABEL', 1)
    saved=save_model(backend, state, 'DYN-LABEL', 'Edited label', source)
    assert saved['name']=='Edited label'
    assert saved['saved_hash']==state['saved_hash']
    assert len(backend.dynamics.revisions('DYN-LABEL')) == 1


def test_browser_numeric_serialization_cannot_change_the_pinned_model_hash(backend):
    import json
    from robot_dashboard.dynamics_forms import restore_library_model
    from robot_experiments.dynamics_library import content_hash
    backend.dynamics.create('DYN-NUMERIC', 'Numeric', backend.dynamics.templates()['spiral']['definition'])
    spec=backend.draft_with_dynamics(backend.new_draft('EXP-NUMERIC', 'Numeric', 'EXP-NOMINAL'), 'DYN-NUMERIC', revision=1)
    model=spec['experiment']['parameters']['target_dynamics']['model']
    snapshot=json.dumps(model, allow_nan=False)
    def browser_roundtrip(value):
        if isinstance(value, float) and value.is_integer(): return int(value)
        if isinstance(value, list): return [browser_roundtrip(v) for v in value]
        if isinstance(value, dict): return {k:browser_roundtrip(v) for k,v in value.items()}
        return value
    received=browser_roundtrip(spec)
    assert content_hash(received['experiment']['parameters']['target_dynamics']['model']) != content_hash(model)
    restored=restore_library_model(received, snapshot)
    assert content_hash(restored['experiment']['parameters']['target_dynamics']['model']) == content_hash(model)
    checked=backend.validate_experiment(restored)
    assert checked['valid'], checked
    wrong=deepcopy(received)
    first=next(iter(wrong['experiment']['parameters']['target_dynamics']['model']['parameters']))
    wrong['experiment']['parameters']['target_dynamics']['model']['parameters'][first]['value']=True
    with pytest.raises(ValueError, match='changed'):
        restore_library_model(wrong, snapshot)
    wrong['experiment']['parameters']['target_dynamics']['model']['parameters'][first]['value']=123.4
    with pytest.raises(ValueError, match='changed'):
        restore_library_model(wrong, snapshot)


def test_source_selection_preserves_registered_edits_and_restores_them_after_library(backend):
    from robot_dashboard.dynamics_forms import select_target_source
    spec=backend.new_draft('EXP-SOURCE', 'Source', 'EXP-NOMINAL')
    spec['experiment']['components']={'target_dynamics':'fight_flight'}
    spec['experiment']['parameters']={'target_dynamics':{'damping':.27},'predictor':{'horizon':3.0}}
    state={'spec':spec}
    registered=select_target_source(backend,state,'registered')
    assert registered['spec']==spec
    backend.dynamics.create('DYN-SOURCE','Source',backend.dynamics.templates()['spiral']['definition'])
    pinned=select_target_source(backend,state,'library','DYN-SOURCE',1)
    assert pinned['spec']['experiment']['dynamics_source']['revision']==1
    restored=select_target_source(backend,pinned,'registered')
    assert restored['spec']==spec
    assert state=={'spec':spec}
