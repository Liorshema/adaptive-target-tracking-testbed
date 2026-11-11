"""The simplified Lab retains real authoring and identity-safe Scenario management."""

from pathlib import Path
import json

import pytest

from robot_dashboard.app import create_app
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog


@pytest.fixture
def app(tmp_path):
    src = Path(__file__).parents[2]
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', lambda package: src / package)
    catalog.initialize()
    return create_app(ExperimentBackend(tmp_path / 'workspace', catalog=catalog))


def test_primary_layout_is_scenario_only_and_keeps_required_controls(app):
    layout = json.dumps(app.server.test_client().get('/_dash-layout').get_json(), ensure_ascii=False,
                        separators=(',', ':'))
    for identifier in ('source-variant', 'simulation-setup', 'open-simulation',
                       'close-simulation', 'effective-inspector', 'comparison-variable',
                       'ranking-metric', 'ranking-scope'):
        assert f'"id":"{identifier}"' not in layout
    for text in ('Simulation Setup', 'Advanced Comparison Setup', 'Advanced Configuration / Diagnostics'):
        assert text not in layout
    for identifier in ('new-scenario', 'duplicate-scenario', 'rename-scenario', 'delete-scenario',
                       'start-experiment', 'cancel-experiment', 'actuation-opt-in', 'run-gui'):
        assert f'"id":"{identifier}"' in layout
    assert 'Experiment Comparison · RMSE' in layout
    comparison_callback = next(value for key, value in app.callback_map.items()
                               if key.startswith('..session-comparison.children'))
    assert [meta['id'] for meta in comparison_callback['inputs'] if isinstance(meta['id'], str) and not meta['id'].startswith('{')] == [
        'current-session', 'execution-events', 'refresh-sessions', 'management-events']
    assert [output.component_id for output in comparison_callback['output']] == [
        'session-comparison', 'results-run', 'results-run']


def test_rename_resolves_selected_identity_even_before_revision_callback_arrives(app):
    key, callback = next((key, value) for key, value in app.callback_map.items()
                         if 'scenario-delete-target.data' in key)
    inputs = [1, None, None, None, None, None]
    states = ['EXP-NOMINAL', None, None, None, None, 0]
    result = app.server.test_client().post('/_dash-update-component', json={
        'output': key,
        'outputs': [{'id': o.component_id, 'property': o.component_property}
                    for o in callback['output']],
        'inputs': [{**m, 'value': v} for m, v in zip(callback['inputs'], inputs)],
        'state': [{**m, 'value': v} for m, v in zip(callback['state'], states)],
        'changedPropIds': ['rename-scenario.n_clicks'],
    })
    assert result.status_code == 200, result.get_data(as_text=True)
    response = result.get_json()['response']
    assert response['scenario-rename-dialog']['hidden'] is False
    assert response['scenario-rename-target']['data']['id'] == 'EXP-NOMINAL'


def test_new_scenario_returns_name_and_technical_identity_atomically(app):
    key, callback = next((key, value) for key, value in app.callback_map.items()
                         if any(m['id'] == 'new-scenario' for m in value.get('inputs', [])))
    result = app.server.test_client().post('/_dash-update-component', json={
        'output': key,
        'outputs': [{'id': o.component_id, 'property': o.component_property}
                    for o in callback['output']],
        'inputs': [{**m, 'value': v} for m, v in zip(callback['inputs'], [1, None])],
        'state': [{**m, 'value': None} for m in callback['state']],
        'changedPropIds': ['new-scenario.n_clicks'],
    })
    assert result.status_code == 200, result.get_data(as_text=True)
    response = result.get_json()['response']
    assert response['new-name']['value'] == 'scenario'
    assert response['new-id']['value'].startswith('EXP-SCENARIO')


def test_busy_presentation_preserves_explicit_saving_feedback(app):
    key, callback = next((key, value) for key, value in app.callback_map.items()
                         if key.startswith('..draft-identity.children'))
    state = {'stage': 'Validated', 'spec': {'experiment': {'name': 'scenario'}}}
    result = app.server.test_client().post('/_dash-update-component', json={
        'output': key,
        'outputs': [{'id': o.component_id, 'property': o.component_property}
                    for o in callback['output']],
        'inputs': [{**m, 'value': v} for m, v in zip(callback['inputs'], [state, None, True])],
        'state': [], 'changedPropIds': ['editor-busy.data'],
    })
    assert result.status_code == 200
    # The Save click's clientside Saving… must survive unrelated busy feedback.
    response = result.get_json()['response']
    assert 'children' not in response.get('save-draft', {})
    assert response['save-draft']['disabled'] is True
