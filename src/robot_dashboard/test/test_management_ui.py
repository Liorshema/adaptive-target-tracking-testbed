"""Real owner metadata behind explicit, selection-bound dashboard confirmations."""

from pathlib import Path

import pytest

from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog


@pytest.fixture
def backend(tmp_path):
    """Keep destructive management tests isolated from retained scientific records."""
    source = Path(__file__).resolve().parents[2]
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share=lambda p: source / p)
    catalog.initialize()
    return ExperimentBackend(tmp_path / 'state', catalog=catalog)


def test_new_name_generates_id_and_collisions_never_select_existing(backend):
    """Keep draft identity creation independent of existing Catalog selections."""
    from robot_dashboard.management import draft_identity

    assert draft_identity(backend, 'create', 'Dynamics Comparison', [], None)[:2] == (
        'EXP-DYNAMICS-COMPARISON',
        True,
    )
    bad = draft_identity(backend, 'create', 'Nominal', [], None)
    assert bad[0] is None and bad[-1] is True and bad[2]
    custom = draft_identity(backend, 'create', 'Nominal', ['custom'], 'EXP-MY-NEW')
    assert custom[0] == 'EXP-MY-NEW' and custom[-1] is False
    assert draft_identity(backend, 'revise', 'Changed', [], 'EXP-NOMINAL')[0] == 'EXP-NOMINAL'


def test_delete_preview_uses_owner_scope_and_selection_token(backend):
    """Bind the real preview to the selected Session and its unmodified metadata."""
    from robot_dashboard.management import ManagementActions

    actions = ManagementActions(backend)
    session = backend.create_session({'title': 'Recorded session'})['session_id']
    pending, message = actions.confirmation('session', {'id': session})
    assert pending['run_count'] == 0 and pending['shared_run_count'] == 0
    assert 'Recorded session' in message and '0 Runs' in message
    assert backend.get_session(session)
    with pytest.raises(ValueError, match='selection'):
        actions.remove('session', pending, 'different-selection')
    target = backend.get_session(session)
    backend.rename_session(session, 'Other client', expected_token=target['token'])
    with pytest.raises(ValueError):
        actions.remove('session', pending, session)
    assert backend.get_session(session)['session']['metadata']['title'] == 'Other client'


def test_new_flow_rejects_reserved_identity_but_revision_flow_keeps_saved_id(backend):
    """Protect existing immutable identities while allowing explicit revisions."""
    from robot_dashboard.forms import begin_draft

    with pytest.raises(ValueError, match='exists|reserved|retired'):
        begin_draft(backend, 'EXP-NOMINAL', 3, 'imm', 'create', 'EXP-NOMINAL', 'Other')
    state = begin_draft(backend, 'EXP-NOMINAL', 3, 'imm', 'revise', 'EXP-IGNORED', 'Ignored')
    assert state['spec']['experiment']['id'] == 'EXP-NOMINAL'


def test_management_controls_use_compact_dash_dialogs(backend):
    """Mount working Dash overlays and remove native browser confirmations."""
    from robot_dashboard.app import create_app

    text = create_app(backend).server.test_client().get('/_dash-layout').get_data(as_text=True)
    for name in [
        'rename-session',
        'delete-session',
        'rename-scenario',
        'delete-scenario',
        'session-delete-dialog',
        'scenario-delete-dialog',
        'scenario-rename-dialog',
        'session-rename-dialog',
        'custom-id',
        'new-id',
        'management-notice',
    ]:
        assert name in text
    assert 'ConfirmDialog' not in text and 'archive-scenario' not in text
    assert '"role":"dialog"' in text and 'aria-modal' in text
    assert 'Rename needs a separate' not in text


def _post(app, prefix, inputs, states, trigger):
    """Exercise the actual Dash HTTP endpoint, including browser State snapshots."""
    key, meta = next((k, v) for k, v in app.callback_map.items() if k.startswith(prefix))
    response = app.server.test_client().post(
        '/_dash-update-component',
        json={
            'output': key,
            'outputs': [
                {'id': o.component_id, 'property': o.component_property}
                for o in meta['output']
            ],
            'inputs': [{**m, 'value': inputs.get(m['id'])} for m in meta['inputs']],
            'state': [{**m, 'value': states.get(m['id'])} for m in meta['state']],
            'changedPropIds': [trigger],
        },
    )
    assert response.status_code == 200, response.get_data(as_text=True)
    return response.get_json()['response']


def test_http_session_delete_requires_confirmation_and_clears_run(backend):
    """Require confirmation, allow cancellation and clear stale Session and Run state."""
    from robot_dashboard.app import create_app

    app = create_app(backend)
    session = backend.create_session({'title': 'Explicit delete'})['session_id']
    states = {
        'current-session': session,
        'session-management-target': backend.get_session(session),
        'management-events': 0,
    }
    response = _post(
        app,
        '..session-management-feedback.children',
        {'delete-session': 1},
        states,
        'delete-session.n_clicks',
    )
    assert response['session-delete-dialog']['hidden'] is False
    assert 'Explicit delete' in str(response['session-delete-content']['children'])
    assert backend.get_session(session)
    states['session-delete-target'] = response['session-delete-target']['data']
    _post(
        app,
        '..session-management-feedback.children',
        {'session-delete-cancel': 1},
        states,
        'session-delete-cancel.n_clicks',
    )
    assert backend.get_session(session)
    response = _post(
        app,
        '..session-management-feedback.children',
        {'session-delete-confirm': 1},
        states,
        'session-delete-confirm.n_clicks',
    )
    assert response['current-session']['value'] is None
    assert response['results-run']['value'] is None
    assert response['session-delete-dialog']['hidden'] is True
    assert response['management-events']['data'] == 1
    assert 'deleted' in str(response['management-notice']['children']).lower()
    with pytest.raises(KeyError):
        backend.get_session(session)


def test_http_scenario_delete_clears_builder_and_uses_real_counts(backend):
    """Delete the real Catalog record after showing the calculated impact."""
    from robot_dashboard.app import create_app

    app = create_app(backend)
    old = backend.catalog.get('EXP-NOMINAL')
    states = {'source-experiment': old['id'], 'source-revision': 3, 'management-events': 0}
    response = _post(
        app,
        '..scenario-management-feedback.children',
        {'delete-scenario': 1},
        states,
        'delete-scenario.n_clicks',
    )
    pending = response['scenario-delete-target']['data']
    assert pending['run_count'] == 0 and pending['session_count'] == 0
    assert '0 Runs' in str(response['scenario-delete-content']['children'])
    assert '0 Sessions' in str(response['scenario-delete-content']['children'])
    states['scenario-delete-target'] = pending
    _post(
        app,
        '..scenario-management-feedback.children',
        {'scenario-delete-cancel': 1},
        states,
        'scenario-delete-cancel.n_clicks',
    )
    assert any(row['id'] == old['id'] for row in backend.list_experiments())
    response = _post(
        app,
        '..scenario-management-feedback.children',
        {'scenario-delete-confirm': 1},
        states,
        'scenario-delete-confirm.n_clicks',
    )
    assert response['source-experiment']['value'] is None
    assert response['builder-state']['data'] is None
    assert response['results-run']['value'] is None
    assert old['id'] not in {row['id'] for row in backend.list_experiments()}
    with pytest.raises(KeyError):
        backend.catalog.get(old['id'])


def test_http_scenario_rename_preserves_identity_and_immutable_revision(backend):
    """Persist only the display alias and refresh options for the same selected ID."""
    from robot_dashboard.app import create_app

    app = create_app(backend)
    original = backend.catalog.get('EXP-NOMINAL')
    states = {
        'source-experiment': original['id'],
        'source-revision': 3,
        'management-events': 0,
    }
    response = _post(
        app,
        '..scenario-management-feedback.children',
        {'rename-scenario': 1},
        states,
        'rename-scenario.n_clicks',
    )
    assert response['scenario-rename-dialog']['hidden'] is False
    states.update(
        {
            'scenario-rename-target': response['scenario-rename-target']['data'],
            'scenario-rename-title': 'Dynamics Comparison',
        }
    )
    response = _post(
        app,
        '..scenario-management-feedback.children',
        {'scenario-rename-confirm': 1},
        states,
        'scenario-rename-confirm.n_clicks',
    )
    assert response['source-experiment']['value'] == original['id']
    assert backend.catalog.get(original['id']) == original
    assert (
        next(row for row in backend.list_experiments() if row['id'] == original['id'])['name']
        == 'Dynamics Comparison'
    )
    assert any(
        option['label'].startswith('Dynamics Comparison')
        for option in response['source-experiment']['options']
    )


def test_http_stale_rename_and_delete_show_errors_without_clearing_selection(backend):
    """Reject concurrent alias edits and changed selections without false success."""
    from robot_dashboard.app import create_app

    app = create_app(backend)
    states = {'source-experiment': 'EXP-NOMINAL', 'source-revision': 3, 'management-events': 0}
    response = _post(
        app,
        '..scenario-management-feedback.children',
        {'rename-scenario': 1},
        states,
        'rename-scenario.n_clicks',
    )
    pending = response['scenario-rename-target']['data']
    backend.rename_experiment(
        'EXP-NOMINAL', 'Concurrent rename', expected_revision=3, expected_name=pending['name']
    )
    states.update({'scenario-rename-target': pending, 'scenario-rename-title': 'Stale rename'})
    response = _post(
        app,
        '..scenario-management-feedback.children',
        {'scenario-rename-confirm': 1},
        states,
        'scenario-rename-confirm.n_clicks',
    )
    assert 'source-experiment' not in response and 'results-run' not in response
    assert response['management-notice']['children']
    assert (
        next(row for row in backend.list_experiments() if row['id'] == 'EXP-NOMINAL')['name']
        == 'Concurrent rename'
    )
    response = _post(
        app,
        '..scenario-management-feedback.children',
        {'delete-scenario': 1},
        states,
        'delete-scenario.n_clicks',
    )
    states['scenario-delete-target'] = response['scenario-delete-target']['data']
    states['source-experiment'] = 'EXP-PERFECT'
    response = _post(
        app,
        '..scenario-management-feedback.children',
        {'scenario-delete-confirm': 1},
        states,
        'scenario-delete-confirm.n_clicks',
    )
    assert 'results-run' not in response
    assert any(row['id'] == 'EXP-NOMINAL' for row in backend.list_experiments())


def _retained_run(backend, experiment='EXP-NOMINAL'):
    """Disposable Runner-shaped evidence, entirely inside this test's managed root."""
    import uuid
    from robot_experiments.reporting import write_json

    record = backend.catalog.get(experiment)
    identity = {
        'id': experiment,
        'revision': record['revision'],
        'definition_sha256': record['sha256'],
    }
    run_id = str(uuid.uuid4())
    study = backend.directory / 'runs' / run_id
    run = study / 'runs' / 'variant_seed42'
    run.mkdir(parents=True)
    write_json(study / 'experiment_identity.json', identity)
    write_json(
        run / 'status.json',
        {
            'schema_version': 1,
            'run_id': run_id,
            'experiment_identity': identity,
            'execution_status': 'completed',
            'acceptance': True,
            'cleanup': {'returncode': 0, 'survivors': []},
        },
    )
    write_json(
        run / 'metadata.json',
        {
            'schema_version': 1,
            'run_id': run_id,
            'experiment_identity': identity,
            'variant': 'variant',
            'seed': 42,
        },
    )
    write_json(run / 'metrics.json', {'schema_version': 1, 'available': False})
    (run / 'telemetry').mkdir()
    (run / 'telemetry' / 'clock.jsonl').write_bytes(b'{"time":1.0}\n')
    return run_id, run


def _bytes(root):
    return {
        str(path.relative_to(root)): path.read_bytes()
        for path in root.rglob('*')
        if path.is_file()
    }


def test_http_session_delete_reports_and_preserves_shared_evidence(backend):
    """Delete only exclusively referenced Runs and preserve shared bytes and metadata."""
    from robot_dashboard.app import create_app

    own, own_path = _retained_run(backend)
    shared, shared_path = _retained_run(backend)
    first = backend.create_session({'title': 'Delete this Session'})['session_id']
    other = backend.create_session({'title': 'Keep this Session'})['session_id']
    for session, run in [(first, own), (first, shared), (other, shared)]:
        backend.add_run_to_session(session, run)
    preserved = _bytes(shared_path)
    other_before = backend.get_session(other)
    app = create_app(backend)
    states = {
        'current-session': first,
        'session-management-target': backend.get_session(first),
        'management-events': 0,
    }
    response = _post(
        app,
        '..session-management-feedback.children',
        {'delete-session': 1},
        states,
        'delete-session.n_clicks',
    )
    pending = response['session-delete-target']['data']
    assert pending['run_count'] == 1 and pending['shared_run_count'] == 1
    assert '1 shared Runs will be preserved' in response['session-delete-content']['children']
    states['session-delete-target'] = pending
    response = _post(
        app,
        '..session-management-feedback.children',
        {'session-delete-confirm': 1},
        states,
        'session-delete-confirm.n_clicks',
    )
    assert response['results-run']['value'] is None and not own_path.exists()
    assert _bytes(shared_path) == preserved
    assert backend.get_session(other) == other_before


def test_http_scenario_delete_reports_two_impacted_sessions_and_keeps_other_runs(backend):
    """Cascade across real Sessions while preserving unrelated scientific evidence."""
    from robot_dashboard.app import create_app

    target, target_path = _retained_run(backend)
    unrelated, unrelated_path = _retained_run(backend, 'EXP-PREDICTION')
    first = backend.create_session({'title': 'Mixed Session'})['session_id']
    second = backend.create_session({'title': 'Also references target'})['session_id']
    for session, run in [(first, target), (first, unrelated), (second, target)]:
        backend.add_run_to_session(session, run)
    preserved = _bytes(unrelated_path)
    app = create_app(backend)
    states = {'source-experiment': 'EXP-NOMINAL', 'source-revision': 3, 'management-events': 0}
    response = _post(
        app,
        '..scenario-management-feedback.children',
        {'delete-scenario': 1},
        states,
        'delete-scenario.n_clicks',
    )
    pending = response['scenario-delete-target']['data']
    assert pending['run_count'] == 1 and pending['session_count'] == 2
    assert '2 Sessions will be updated' in response['scenario-delete-content']['children']
    states['scenario-delete-target'] = pending
    response = _post(
        app,
        '..scenario-management-feedback.children',
        {'scenario-delete-confirm': 1},
        states,
        'scenario-delete-confirm.n_clicks',
    )
    assert '2 Sessions updated' in response['management-notice']['children']
    assert not target_path.exists() and _bytes(unrelated_path) == preserved
    assert [ref['run_id'] for ref in backend.get_session(first)['session']['runs']] == [
        unrelated
    ]
    assert backend.get_session(second)['session']['runs'] == []


def test_http_confirm_rechecks_new_active_job_without_claiming_success(backend):
    """Revalidate Jobs after preview and preserve selection when ownership becomes busy."""
    import uuid
    from robot_dashboard.app import create_app
    from robot_experiments.reporting import write_json

    selected = backend.create_session({'title': 'Became busy'})['session_id']
    app = create_app(backend)
    states = {
        'current-session': selected,
        'session-management-target': backend.get_session(selected),
        'management-events': 0,
    }
    response = _post(
        app,
        '..session-management-feedback.children',
        {'delete-session': 1},
        states,
        'delete-session.n_clicks',
    )
    states['session-delete-target'] = response['session-delete-target']['data']
    job_id = str(uuid.uuid4())
    job = backend.jobs.directory / job_id
    job.mkdir(parents=True)
    write_json(
        job / 'request.json',
        {'job_id': job_id, 'session_id': selected, 'record': {'id': 'EXP-NOMINAL'}},
    )
    write_json(job / 'status.json', {'job_id': job_id, 'state': 'running'})
    response = _post(
        app,
        '..session-management-feedback.children',
        {'session-delete-confirm': 1},
        states,
        'session-delete-confirm.n_clicks',
    )
    assert backend.get_session(selected)
    assert 'management-events' not in response and 'current-session' not in response
    assert 'results-run' not in response
    assert response['management-notice']['children']
    assert 'deleted.' not in str(response['management-notice']['children']).lower()


def test_incomplete_cleanup_never_reports_success(backend, monkeypatch):
    """Treat incomplete backend cleanup as a failure instead of notifying success."""
    from robot_dashboard.app import create_app

    selected = backend.create_session({'title': 'Incomplete cleanup'})['session_id']
    app = create_app(backend)
    states = {
        'current-session': selected,
        'session-management-target': backend.get_session(selected),
        'management-events': 0,
    }
    response = _post(
        app,
        '..session-management-feedback.children',
        {'delete-session': 1},
        states,
        'delete-session.n_clicks',
    )
    states['session-delete-target'] = response['session-delete-target']['data']
    monkeypatch.setattr(backend, 'delete_resources', lambda *a, **k: {'completed': False})
    response = _post(
        app,
        '..session-management-feedback.children',
        {'session-delete-confirm': 1},
        states,
        'session-delete-confirm.n_clicks',
    )
    assert 'management-events' not in response
    assert 'incomplete' in str(response['management-notice']['children']).lower()
    assert backend.get_session(selected)


def test_http_session_rename_uses_compact_modal_and_preserves_run_selection(backend):
    """Rename only after modal confirmation and retain the selected Run."""
    from robot_dashboard.app import create_app

    selected = backend.create_session({'title': 'Original title', 'notes': 'Keep notes'})[
        'session_id'
    ]
    app = create_app(backend)
    states = {
        'current-session': selected,
        'session-management-target': backend.get_session(selected),
        'management-events': 0,
    }
    response = _post(
        app,
        '..session-management-feedback.children',
        {'rename-session': 1},
        states,
        'rename-session.n_clicks',
    )
    assert response['session-rename-dialog']['hidden'] is False
    assert response['session-rename-title']['value'] == 'Original title'
    assert backend.get_session(selected)['session']['metadata']['title'] == 'Original title'
    states.update(
        {
            'session-rename-target': response['session-rename-target']['data'],
            'session-rename-title': 'Updated title',
        }
    )
    response = _post(
        app,
        '..session-management-feedback.children',
        {'session-rename-confirm': 1},
        states,
        'session-rename-confirm.n_clicks',
    )
    assert response['session-rename-dialog']['hidden'] is True
    assert 'results-run' not in response and 'current-session' not in response
    assert backend.get_session(selected)['session']['metadata'] == {
        'title': 'Updated title',
        'notes': 'Keep notes',
    }


def test_management_refresh_drops_deleted_cached_job_runs_and_keeps_run_unselected(backend):
    """Deleting a Scenario cannot resurrect Run options from cached execution state."""
    from robot_dashboard.app import create_app

    removed_id, removed = _retained_run(backend)
    kept_id, _ = _retained_run(backend, 'EXP-PREDICTION')
    sid = backend.create_session({'title': 'Mixed references'})['session_id']
    backend.add_run_to_session(sid, removed_id)
    backend.add_run_to_session(sid, kept_id)
    pending = backend.preview_deletion('scenario', 'EXP-NOMINAL')
    backend.delete_resources('scenario', 'EXP-NOMINAL', expected_token=pending['token'])
    app = create_app(backend)
    response = _post(
        app,
        '..session-comparison.children',
        {'current-session': sid, 'management-events': 1},
        {
            'results-run': None,
            'execution-state': {
                'job': {'runs': [{'run_id': removed_id, 'directory': str(removed)}]}
            },
        },
        'management-events.data',
    )
    assert response['results-run']['value'] is None
    assert [o['value'] for o in response['results-run']['options']] == [kept_id]


def test_scenario_alias_rename_survives_revision_selector_finishing_its_refresh(backend):
    """The alias belongs to the Experiment ID, independent of viewed historical revision."""
    from robot_dashboard.app import create_app

    app = create_app(backend)
    states = {'source-experiment': 'EXP-NOMINAL', 'source-revision': 1, 'management-events': 0}
    opened = _post(
        app,
        '..scenario-management-feedback.children',
        {'rename-scenario': 1},
        states,
        'rename-scenario.n_clicks',
    )
    states.update(
        {
            'scenario-rename-target': opened['scenario-rename-target']['data'],
            'source-revision': 3,
            'scenario-rename-title': 'Current display alias',
        }
    )
    result = _post(
        app,
        '..scenario-management-feedback.children',
        {'scenario-rename-confirm': 1},
        states,
        'scenario-rename-confirm.n_clicks',
    )
    assert result['management-notice']['children'] == 'Scenario renamed.'
    assert (
        next(r for r in backend.list_experiments() if r['id'] == 'EXP-NOMINAL')['name']
        == 'Current display alias'
    )
