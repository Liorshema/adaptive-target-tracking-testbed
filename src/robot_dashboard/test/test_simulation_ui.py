"""Thin preview actions; lifecycle and process ownership stay in ExperimentBackend."""

import importlib.util


def test_simulation_ui_extension_exists():
    """The UI lifecycle has a separate thin-client responsibility."""
    assert importlib.util.find_spec('robot_dashboard.simulation') is not None


def test_preview_callbacks_do_not_open_on_refresh_and_keep_errors_truthful(tmp_path, monkeypatch):
    """Only an explicit valid Open action calls the lifecycle owner."""
    from pathlib import Path
    from robot_dashboard.app import create_app
    from robot_experiments.backend import ExperimentBackend
    from robot_experiments.catalog import ExperimentCatalog

    src = Path(__file__).resolve().parents[2]
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share=lambda p: src / p)
    catalog.initialize()
    backend = ExperimentBackend(tmp_path / 'state', catalog=catalog)
    current = {'state': 'closed', 'job_id': None, 'cleanup_complete': True}
    calls = []
    monkeypatch.setattr(backend, 'get_simulation_status', lambda: current)

    def open_(identifier, revision, *, variant):
        calls.append((identifier, revision, variant))
        return {'state': 'opening', 'job_id': 'real-owner-reference', 'cleanup_complete': False}

    monkeypatch.setattr(backend, 'open_simulation', open_)
    app = create_app(backend)
    key, meta = next((k, v) for k, v in app.callback_map.items() if k == 'simulation-state.data')

    def call(trigger, clicks, snapshot):
        r = app.server.test_client().post(
            '/_dash-update-component',
            json={
                'output': key,
                'outputs': {'id': 'simulation-state', 'property': 'data'},
                'inputs': [
                    {**m, 'value': v} for m, v in zip(meta['inputs'], [clicks, None, 0, None])
                ],
                'state': [
                    {**m, 'value': v}
                    for m, v in zip(meta['state'], [None, 'EXP-NOMINAL', 3, 'imm', snapshot])
                ],
                'changedPropIds': [trigger],
            },
        )
        assert r.status_code == 200, r.get_data(as_text=True)
        return r.get_json()['response']['simulation-state']['data']

    saved = {'experiment_id': 'EXP-NOMINAL', 'revision': 3}
    assert call('simulation-poll.n_intervals', None, saved)['state'] == 'closed' and not calls
    assert 'action_error' in call('open-simulation.n_clicks', 1, None) and not calls
    assert call('open-simulation.n_clicks', 1, saved)['state'] == 'opening'
    assert calls == [('EXP-NOMINAL', 3, 'imm')]
