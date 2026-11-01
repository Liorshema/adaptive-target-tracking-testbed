"""Exercise the UI against real public experiment services, not a second engine."""

import importlib.util
import subprocess
import sys
from copy import deepcopy
from pathlib import Path

import pytest

from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import CatalogConflict
from robot_experiments.catalog import ExperimentCatalog

SRC = Path(__file__).resolve().parents[2]


@pytest.fixture
def backend(tmp_path):
    """Use the existing Catalog owner in an isolated writable directory."""
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share=lambda p: SRC / p)
    catalog.initialize()
    return ExperimentBackend(tmp_path / 'state', catalog=catalog)


def test_builder_helpers_exist():
    """The optional UI package is discoverable independently."""
    assert importlib.util.find_spec('robot_dashboard') is not None


def test_draft_revision_is_pinned_and_source_unchanged(backend):
    """Historical bases are pinned and revisions remain immutable."""
    from robot_dashboard.forms import begin_draft

    source = deepcopy(backend.catalog.get('EXP-NOMINAL'))
    draft = begin_draft(
        backend, 'EXP-NOMINAL', source['revision'], 'imm', 'create', 'EXP-UI', 'UI study'
    )
    assert draft['expected_revision'] is None
    assert draft['spec']['experiment']['base_revision'] == source['revision']
    assert backend.catalog.get('EXP-NOMINAL') == source
    edited = begin_draft(backend, 'EXP-NOMINAL', source['revision'], 'imm', 'revise')
    assert edited['expected_revision'] == source['revision']
    with pytest.raises(ValueError, match='latest'):
        begin_draft(backend, 'EXP-NOMINAL', 1, 'imm', 'revise')
    with pytest.raises(ValueError, match='schema-1'):
        begin_draft(
            backend, 'EXP-SENSOR-NOISE', 3, 'baseline', 'create', 'EXP-UNSUPPORTED', 'Unsupported'
        )


def test_schema_controls_and_truthful_provenance(backend):
    """Owner schema metadata and uncertain ancestry stay explicit."""
    from robot_dashboard.forms import parameter_rows

    draft = backend.new_draft('EXP-UI', 'UI study', 'EXP-NOMINAL')
    draft['experiment']['parameters'] = {'target_dynamics': {'damping': 0.23}}
    detail = backend.preview_experiment(draft)['components']['target_dynamics']
    row = next(r for r in parameter_rows(detail) if r['key'] == 'damping')
    assert row['kind'] == 'number'
    assert row['unit'] == 's^-1' and row['schema']['minimum'] == 0.0
    assert row['default'] == 0.5 and row['value'] == 0.23
    assert row['source'] == 'Explicit experiment override'
    inherited = next(r for r in parameter_rows(detail) if r['key'] == 'a_max')
    assert 'unknown' in inherited['source'].lower()
    assert inherited['value'] == 0.15


def test_form_edit_parse_reset_validate_save_and_conflict(backend):
    """Detached UI edits delegate validation and persistence to Author."""
    from robot_dashboard.forms import assemble_draft
    from robot_dashboard.forms import begin_draft

    state = begin_draft(backend, 'EXP-NOMINAL', 3, 'imm', 'create', 'EXP-UI', 'UI')
    spec = assemble_draft(
        state['spec'],
        [],
        [],
        [0.23],
        [{'role': 'target_dynamics', 'key': 'damping', 'kind': 'number'}],
        [['override']],
    )
    assert 'parameters' not in state['spec']['experiment']
    assert (
        backend.preview_experiment(spec)['components']['target_dynamics']['effective']['damping']
        == 0.23
    )
    r1 = backend.save_experiment(spec)
    assert r1['revision'] == 1
    changed = deepcopy(spec)
    changed['experiment']['parameters']['target_dynamics']['damping'] = 0.31
    assert backend.save_experiment(changed, expected_revision=1)['revision'] == 2
    with pytest.raises(CatalogConflict):
        backend.save_experiment(spec, expected_revision=1)
    assert backend.catalog.get('EXP-UI', 1) == r1
    invalid = deepcopy(changed)
    invalid['experiment']['parameters']['target_dynamics']['damping'] = -0.1
    assert not backend.validate_experiment(invalid)['valid']
    with pytest.raises(ValueError):
        backend.save_experiment(invalid, expected_revision=2)
    assert len(backend.list_revisions('EXP-UI')) == 2
    assert 'parameters' not in backend.reset_draft(changed)['experiment']


def test_typed_values_are_not_silently_coerced_or_lost():
    """Malformed mathematical inputs fail without silently losing other edits."""
    from robot_dashboard.forms import assemble_draft

    spec = {'schema_version': 1, 'experiment': {'parameters': {'other': {'kept': 1}}}}
    field = {'role': 'predictor', 'key': 'transition_matrix', 'kind': 'array'}
    value = assemble_draft(spec, [], [], ['[[0.9, 0.1], [0.2, 0.8]]'], [field], [['override']])
    assert value['experiment']['parameters']['predictor']['transition_matrix'][0] == [0.9, 0.1]
    assert value['experiment']['parameters']['other'] == {'kept': 1}
    for bad in ['[broken', '[NaN]']:
        with pytest.raises(ValueError, match='predictor.transition_matrix'):
            assemble_draft(spec, [], [], [bad], [field], [['override']])
    inherited = assemble_draft(value, [], [], ['ignored'], [field], [[]])
    assert 'predictor' not in inherited['experiment']['parameters']


def test_model_change_discards_only_its_local_parameters():
    """Switching a model preserves unrelated explicit parameters."""
    from robot_dashboard.forms import assemble_draft

    spec = {
        'schema_version': 1,
        'experiment': {
            'components': {'predictor': 'imm'},
            'parameters': {
                'predictor': {'likelihood_variance': 0.4},
                'target_dynamics': {'damping': 0.23},
            },
        },
    }
    result = assemble_draft(spec, ['constant_velocity'], [{'role': 'predictor'}], [], [], [])
    assert result['experiment']['parameters'] == {'target_dynamics': {'damping': 0.23}}
    assert spec['experiment']['components']['predictor'] == 'imm'


def test_dash_page_and_capability_are_real(backend):
    """Flask serves real Dash assets and truthful deployment capabilities."""
    from robot_dashboard.app import create_app

    app = create_app(backend)
    client = app.server.test_client()
    assert client.get('/').status_code == 200
    layout = client.get('/_dash-layout')
    assert layout.status_code == 200
    text = layout.get_data(as_text=True)
    assert 'Experiment Lab' in text and 'Simulation Setup' in text
    assert 'EXP-NOMINAL' in text and 'Open Simulation' in text
    assert 'Unscored preview' in text and 'close-simulation' in text
    assert client.get('/assets/dashboard.css').status_code == 200
    # Interactive preview and scored execution keep distinct authoritative owners.
    assert any('simulation-state.data' in key for key in app.callback_map)


def test_headless_backend_does_not_import_dashboard_dependency():
    """Headless imports tolerate a missing or broken optional UI."""
    script = """
import sys
class NoDash:
    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] in {'dash', 'plotly', 'robot_dashboard'}:
            raise ImportError('UI unavailable')
sys.meta_path.insert(0, NoDash())
from robot_experiments.backend import ExperimentBackend
capabilities = ExperimentBackend('/tmp/no-ui').gazebo_setup_capabilities()
assert capabilities['interactive_setup_supported'] is True
"""
    subprocess.run([sys.executable, '-c', script], check=True)


def _action(app, state, trigger, value=0.23, *, duration=6.0, seeds='[42]'):
    key, callback = next(
        (k, v) for k, v in app.callback_map.items() if k.startswith('..builder-state.data')
    )
    field = {'role': 'target_dynamics', 'key': 'damping', 'kind': 'number'}
    inputs = [None] * 5 + [[], [value], [['override']], duration, seeds]
    states = [[], [field], 'EXP-NOMINAL', 3, 'imm', 'create', 'EXP-HTTP', 'HTTP', state]
    response = app.server.test_client().post(
        '/_dash-update-component',
        json={
            'output': key,
            'outputs': [
                {'id': o.component_id, 'property': o.component_property}
                for o in callback['output']
            ],
            'inputs': [
                {**meta, 'value': value} for meta, value in zip(callback['inputs'], inputs)
            ],
            'state': [{**meta, 'value': value} for meta, value in zip(callback['state'], states)],
            'changedPropIds': [trigger + '.n_clicks'],
        },
    )
    assert response.status_code == 200, response.get_data(as_text=True)
    return response.get_json()['response']['builder-state']['data']


def test_http_save_rejects_edits_after_validation_and_preserves_draft(backend):
    """The real Dash endpoint compares live form fields with validated content."""
    from robot_dashboard.app import create_app
    from robot_dashboard.forms import begin_draft

    app = create_app(backend)
    state = begin_draft(backend, 'EXP-NOMINAL', 3, 'imm', 'create', 'EXP-HTTP', 'HTTP')
    state['inherited_components'] = deepcopy(state['components'])
    validated = _action(app, state, 'validate-draft')
    assert validated['stage'] == 'Validated'
    stale = _action(app, validated, 'save-draft', value=0.31)
    assert stale['stage'] == 'Invalid' and 'Validate the current form' in stale['message']
    assert stale['spec']['experiment']['parameters']['target_dynamics']['damping'] == 0.31
    assert all(r['id'] != 'EXP-HTTP' for r in backend.list_experiments())
    checked = _action(app, stale, 'validate-draft', value=0.31)
    saved = _action(app, checked, 'save-draft', value=0.31)
    assert saved['stage'] == 'Saved' and saved['expected_revision'] == 1
    original = backend.catalog.get('EXP-HTTP', 1)
    # A second client moves the Catalog forward after this client's validation.
    checked = _action(app, saved, 'validate-draft', value=0.34)
    backend.save_experiment(saved['spec'], expected_revision=1)
    conflicted = _action(app, checked, 'save-draft', value=0.34)
    assert conflicted['stage'] == 'Invalid'
    assert 'reload' in conflicted['message']
    assert conflicted['spec']['experiment']['parameters']['target_dynamics']['damping'] == 0.34
    assert backend.catalog.get('EXP-HTTP', 1) == original
    assert len(backend.list_revisions('EXP-HTTP')) == 2


def _tree(component):
    """Inspect actual serialized Dash controls without browser implementation mocks."""
    if isinstance(component, (list, tuple)):
        for child in component:
            yield from _tree(child)
    elif hasattr(component, 'to_plotly_json'):
        yield component
        yield from _tree(getattr(component, 'children', None))


def test_component_redraw_retains_unpreviewed_parameters_and_switches(backend):
    """Rendering after a model switch must retain another role's detached edits."""
    from robot_dashboard.app import component_cards
    from robot_dashboard.forms import begin_draft

    state = begin_draft(backend, 'EXP-NOMINAL', 3, 'imm', 'create', 'EXP-UI', 'UI')
    state['inherited_components'] = deepcopy(state['components'])
    state['spec']['experiment']['parameters'] = {'target_dynamics': {'damping': 0.23}}
    controls = list(_tree(component_cards(backend, state)))
    value = next(
        c
        for c in controls
        if getattr(c, 'id', None)
        == {'type': 'param', 'role': 'target_dynamics', 'key': 'damping', 'kind': 'number'}
    )
    switch = next(
        c
        for c in controls
        if getattr(c, 'id', None)
        == {'type': 'override', 'role': 'target_dynamics', 'key': 'damping', 'kind': 'number'}
    )
    assert value.value == 0.23
    assert switch.value == ['override']
    assert not value.disabled


def test_successful_save_refreshes_selected_revision_choices(backend):
    """A saved experiment remains editable without switching selection or refreshing."""
    from robot_dashboard.app import create_app
    from robot_dashboard.app import experiment_options

    app = create_app(backend)
    key, callback = next(
        (k, v) for k, v in app.callback_map.items() if k.startswith('..source-revision.options')
    )
    assert {'id': 'source-experiment', 'property': 'options'} in callback['inputs']
    draft = backend.new_draft(
        'EXP-NOMINAL', 'Nominal', 'EXP-NOMINAL', base_revision=3, base_variant='imm'
    )
    backend.save_experiment(draft, expected_revision=3)
    response = app.server.test_client().post(
        '/_dash-update-component',
        json={
            'output': key,
            'outputs': [
                {'id': o.component_id, 'property': o.component_property}
                for o in callback['output']
            ],
            'inputs': [
                {'id': 'source-experiment', 'property': 'value', 'value': 'EXP-NOMINAL'},
                {
                    'id': 'source-experiment',
                    'property': 'options',
                    'value': experiment_options(backend),
                },
            ],
            'state': [],
            'changedPropIds': ['source-experiment.options'],
        },
    )
    assert response.status_code == 200
    revisions = response.get_json()['response']['source-revision']
    assert revisions['value'] == 4
    assert [o['value'] for o in revisions['options']] == [4, 3, 2, 1]


def test_duration_seeds_use_author_revision_and_preserve_history(backend):
    """Execution fields become immutable Author revisions, never launch overrides."""
    from robot_dashboard.forms import apply_execution
    from robot_dashboard.forms import begin_draft

    state = begin_draft(backend, 'EXP-NOMINAL', 3, 'imm', 'create', 'EXP-DURATION', 'Duration')
    original = deepcopy(backend.catalog.get('EXP-NOMINAL', 3))
    spec = apply_execution(state['spec'], 60.0, '[42, 43]')
    preview = backend.preview_experiment(spec)
    assert preview['effective']['experiment']['duration'] == 60.0
    assert preview['effective']['experiment']['seeds'] == [42, 43]
    first = backend.save_experiment(spec)
    second = backend.save_experiment(apply_execution(spec, 30.0, '[42]'), expected_revision=1)
    assert first['revision'] == 1 and second['revision'] == 2
    assert backend.catalog.get('EXP-DURATION', 1) == first
    assert backend.catalog.get('EXP-NOMINAL', 3) == original
    reset = backend.reset_draft(spec)
    assert backend.preview_experiment(reset)['effective']['experiment']['duration'] == 6.0
    for duration, seeds in [(float('nan'), '[42]'), (60, 'broken'), (60, '[true]')]:
        with pytest.raises(ValueError):
            apply_execution(spec, duration, seeds)
    assert not backend.validate_experiment(apply_execution(spec, 0, '[42]'))['valid']


def test_two_primary_tabs_keep_all_workflow_controls_mounted(backend):
    """Switching workspaces must not destroy draft, job or Session controls."""
    from robot_dashboard.app import create_app

    app = create_app(backend)
    controls = list(_tree(app.layout()))
    ids = [c.id for c in controls if hasattr(c, 'id') and isinstance(c.id, str)]
    for name in [
        'workspace-tab',
        'experiment-lab',
        'results-history',
        'execution-duration',
        'execution-seeds',
        'start-experiment',
        'cancel-experiment',
        'current-session',
        'results-run',
        'session-comparison',
        'execution-state',
        'component-cards',
    ]:
        assert ids.count(name) == 1, name
    tabs = next(c for c in controls if getattr(c, 'id', None) == 'workspace-tab')
    assert [o['label'] for o in tabs.options] == ['Experiment Lab', 'Results & History']


def test_duration_edit_invalidates_validation_and_saves_exact_revision(backend):
    """Live duration and seed changes cannot reuse a previous validation approval."""
    from robot_dashboard.app import create_app
    from robot_dashboard.forms import begin_draft

    app = create_app(backend)
    state = begin_draft(backend, 'EXP-NOMINAL', 3, 'imm', 'create', 'EXP-HTTP', 'HTTP')
    state['inherited_components'] = deepcopy(state['components'])
    validated = _action(app, state, 'validate-draft', duration=60, seeds='[42, 43]')
    assert validated['stage'] == 'Validated'
    stale = _action(app, validated, 'save-draft', duration=30, seeds='[42]')
    assert stale['stage'] == 'Invalid'
    assert not any(r['id'] == 'EXP-HTTP' for r in backend.list_experiments())
    checked = _action(app, stale, 'validate-draft', duration=30, seeds='[42]')
    saved = _action(app, checked, 'save-draft', duration=30, seeds='[42]')
    assert saved['stage'] == 'Saved'
    saved_definition = backend.catalog.get('EXP-HTTP', 1)['definition']['experiment']
    assert saved_definition['duration'] == 30 and saved_definition['seeds'] == [42]


def test_scenario_labels_use_catalog_names_without_internal_ids(backend):
    """Display names remain metadata-backed while dropdown values retain identity."""
    from robot_dashboard.app import experiment_options

    options = experiment_options(backend)
    original = {r['id']: r['name'] for r in backend.list_experiments()}
    assert all(o['label'] == original[o['value']] for o in options)


def test_revision_and_provenance_are_available_under_advanced(backend):
    """Internal revision selection stays functional without dominating the main Lab."""
    from robot_dashboard.app import create_app

    controls = list(_tree(create_app(backend).layout()))
    advanced = [c for c in controls if getattr(c, 'className', '') == 'scenario-advanced']
    assert len(advanced) == 1
    ids = {
        getattr(c, 'id', '') for c in _tree(advanced[0])
        if isinstance(getattr(c, 'id', ''), str)
    }
    assert {'source-revision', 'saved-preview'} <= ids
    assert not getattr(advanced[0], 'open', False)
