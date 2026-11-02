"""Controlled UI behavior tests; only the real backend owns workers and Sessions."""

import importlib.util
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy

import pytest


class ControlledBackend:
    """Fake only action/status transport; no numerical samples or scientific results."""

    directory = '/tmp/ui-actions'

    def __init__(self):
        """Initialize controlled transport state without starting processes."""
        self.starts = []
        self.jobs = {}
        self.associations = {'session-a': [], 'session-b': []}
        self.cancels = []
        self.timeout = False
        self.gui_requests = []
        self.session_requests = []

    def get_simulation_status(self):
        """Return no preview unless a handoff test explicitly supplies one."""
        return {'state': 'closed', 'job_id': None, 'cleanup_complete': True}

    def list_sessions(self, **kwargs):
        """Return controlled Session identities."""
        return [{'session_id': s} for s in self.associations]

    def start_run(
        self,
        identifier,
        revision,
        *,
        output,
        enable_actuation=False,
        gui=False,
        session_id=None,
    ):
        """Record the requested immutable identity and explicit actuation flag."""
        self.starts.append((identifier, revision, output, enable_actuation))
        self.gui_requests.append(gui)
        self.session_requests.append(session_id)
        job = {
            'job_id': f'job-{len(self.starts)}',
            'state': 'queued',
            'run_ids': [],
            'runs': [],
            'stop_requested': False,
        }
        self.jobs[job['job_id']] = job
        return deepcopy(job)

    def get_job_status(self, job_id):
        """Return detached current Job transport state."""
        return deepcopy(self.jobs[job_id])

    def get_run_status(self, run_id):
        """Return an unscored attempt state without invented acceptance."""
        return {
            'run_id': run_id,
            'execution_status': 'preparing',
            'acceptance': None,
            'cleanup': None,
        }

    def add_run_to_session(self, session_id, run_id):
        """Model only idempotent reference association."""
        if run_id not in self.associations[session_id]:
            self.associations[session_id].append(run_id)

    def cancel_job(self, job_id, **kwargs):
        """Model the owner cancellation/timeout interface."""
        self.cancels.append(('job', job_id))
        self.jobs[job_id]['stop_requested'] = True
        if self.timeout:
            raise TimeoutError('owner cleanup pending')
        self.jobs[job_id]['state'] = 'cancelled'
        return self.get_job_status(job_id)

    def cancel_run(self, run_id, **kwargs):
        """Model cancellation scope for a known owned Run."""
        self.cancels.append(('run', run_id))
        job = next(j for j in self.jobs.values() if run_id in j['run_ids'])
        job['stop_requested'] = True
        job['state'] = 'cancelled'
        return deepcopy(job)


def test_execution_module_is_present():
    """The execution extension has a focused UI responsibility."""
    assert importlib.util.find_spec('robot_dashboard.execution') is not None


def test_start_is_explicit_pinned_default_off_and_duplicate_safe():
    """Repeated/concurrent requests cannot start a second worker."""
    from robot_dashboard.execution import ExecutionActions
    from robot_dashboard.execution import new_execution_state

    backend = ControlledBackend()
    actions = ExecutionActions(backend)
    initial = new_execution_state()
    assert not backend.starts
    with ThreadPoolExecutor(max_workers=2) as pool:
        replies = list(
            pool.map(
                lambda _: actions.start(
                    initial, 'EXP-EXISTING', 3, 'session-a', '/tmp/output'
                ),
                range(2),
            )
        )
    assert backend.starts == [('EXP-EXISTING', 3, '/tmp/output', False)]
    assert backend.session_requests == ['session-a']
    assert all(r['job_id'] == 'job-1' for r in replies)
    actions.start(
        replies[-1], 'EXP-OTHER', 2, 'session-b', '/tmp/other', enable_actuation=True
    )
    assert len(backend.starts) == 1
    backend.jobs['job-1']['state'] = 'completed'
    # A replayed original nonce remains a replay even after that job is terminal.
    actions.start(initial, 'EXP-EXISTING', 3, 'session-a', '/tmp/output')
    assert len(backend.starts) == 1
    next_job = actions.start(
        replies[-1], 'EXP-OTHER', 2, 'session-b', '/tmp/other', enable_actuation=True
    )
    assert next_job['job_id'] == 'job-2' and backend.starts[-1][-1] is True


def test_resume_uses_existing_job_and_session_additions_are_idempotent():
    """Polling and refresh never launch; Session identity is frozen at explicit Start."""
    from robot_dashboard.execution import ExecutionActions
    from robot_dashboard.execution import new_execution_state

    backend = ControlledBackend()
    state = ExecutionActions(backend).start(
        new_execution_state(), 'EXP-EXISTING', 3, 'session-a', '/tmp/output'
    )
    backend.jobs['job-1'].update(
        state='running',
        run_ids=['run-1', 'run-2'],
        runs=[
            {'run_id': 'run-1', 'execution_status': 'preparing'},
            {'run_id': 'run-2', 'execution_status': 'preparing'},
        ],
    )
    reloaded_actions = ExecutionActions(backend)
    for _ in range(3):
        state = reloaded_actions.poll(state)
    assert len(backend.starts) == 1
    assert state['job_id'] not in state['job']['run_ids']
    assert backend.associations == {'session-a': ['run-1', 'run-2'], 'session-b': []}
    assert state['run_status']['acceptance'] is None
    assert state['run_status']['cleanup'] is None


def test_cancellation_delegates_and_pending_cleanup_is_not_success():
    """A timeout never becomes a force kill or a fabricated successful cleanup."""
    from robot_dashboard.execution import ExecutionActions
    from robot_dashboard.execution import new_execution_state

    backend = ControlledBackend()
    actions = ExecutionActions(backend)
    state = actions.start(new_execution_state(), 'EXP-EXISTING', 3, 'session-a', '/tmp/output')
    backend.timeout = True
    pending = actions.cancel(state)
    assert pending['job']['stop_requested'] is True
    assert pending['job']['state'] == 'queued'
    assert 'cleanup pending' in pending['notice']
    backend.timeout = False
    backend.jobs['job-1'].update(run_ids=['run-1'], runs=[])
    stopped = actions.cancel(pending, run_id='run-1')
    assert stopped['job']['state'] == 'cancelled'
    assert backend.cancels == [('job', 'job-1'), ('run', 'run-1')]
    with pytest.raises(ValueError, match='current Job'):
        actions.cancel(stopped, run_id='historical-run')


def test_missing_session_and_unknown_cleanup_cannot_launch():
    """Saved revision/Session/explicit boolean inputs and worker-loss states stay guarded."""
    from robot_dashboard.execution import ExecutionActions
    from robot_dashboard.execution import new_execution_state

    backend = ControlledBackend()
    actions = ExecutionActions(backend)
    for session, revision, actuation in [
        (None, 3, False),
        ('session-a', None, False),
        ('session-a', 3, 'true'),
    ]:
        with pytest.raises(ValueError):
            actions.start(
                new_execution_state(),
                'EXP-EXISTING',
                revision,
                session,
                '/tmp/output',
                enable_actuation=actuation,
            )
    assert backend.starts == []
    state = actions.start(new_execution_state(), 'EXP-EXISTING', 3, 'session-a', '/tmp/output')
    backend.jobs['job-1']['state'] = 'worker_lost'
    with pytest.raises(ValueError, match='cleanup'):
        actions.start(state, 'EXP-EXISTING', 3, 'session-a', '/tmp/output')


def _callback(app, match, values, states, trigger):
    callback_key, meta = next((k, v) for k, v in app.callback_map.items() if match(k, v))
    outputs = meta['output']
    if isinstance(outputs, list):
        outputs = [{'id': o.component_id, 'property': o.component_property} for o in outputs]
    else:
        outputs = {'id': outputs.component_id, 'property': outputs.component_property}
    states = [*states, *([None] * (len(meta['state']) - len(states)))]
    response = app.server.test_client().post(
        '/_dash-update-component',
        json={
            'output': callback_key,
            'outputs': outputs,
            'inputs': [{**m, 'value': v} for m, v in zip(meta['inputs'], values)],
            'state': [{**m, 'value': v} for m, v in zip(meta['state'], states)],
            'changedPropIds': [trigger],
        },
    )
    assert response.status_code == 200, response.get_data(as_text=True)
    return response.get_json()['response']


@pytest.fixture
def callback_app(tmp_path, monkeypatch):
    """Use a real Catalog/Session and fake only worker/status transport."""
    from pathlib import Path
    from robot_dashboard.app import create_app
    from robot_experiments.backend import ExperimentBackend
    from robot_experiments.catalog import ExperimentCatalog

    src = Path(__file__).resolve().parents[2]
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share=lambda name: src / name)
    catalog.initialize()
    api = ExperimentBackend(tmp_path / 'state', catalog=catalog)
    session = api.create_session({'title': 'UI transport'})['session_id']
    fake = ControlledBackend()
    monkeypatch.setattr(api, 'start_run', fake.start_run)
    monkeypatch.setattr(api, 'get_job_status', fake.get_job_status)
    monkeypatch.setattr(api, 'get_run_status', fake.get_run_status)
    monkeypatch.setattr(api, 'cancel_job', fake.cancel_job)
    return create_app(api), session, fake


@pytest.mark.parametrize('opt_in,expected', [([], False), (['actuate'], True)])
def test_start_callback_opt_in_is_explicit_and_resets_for_next_launch(
    callback_app, opt_in, expected
):
    """Each successful Start clears the checkbox without changing the recorded request."""
    from robot_dashboard.execution import new_execution_state

    app, session, fake = callback_app
    response = _callback(
        app,
        lambda k, m: any(i['id'] == 'start-experiment' for i in m['inputs']),
        [1, None, None, 0],
        [new_execution_state(), 'EXP-NOMINAL', 3, session, '/tmp/output', opt_in, 'job', None],
        'start-experiment.n_clicks',
    )
    assert fake.starts[-1][-1] is expected
    assert response['actuation-opt-in']['value'] == []
    assert response['execution-state']['data']['launch']['enable_actuation'] is expected


def test_invalid_cancel_does_not_suspend_active_monitoring(callback_app):
    """Reject an earlier Session Run as a cancellation target while observing the live Job."""
    from robot_dashboard.execution import new_execution_state

    app, session, fake = callback_app

    def match(k, metadata):
        return any(i['id'] == 'start-experiment' for i in metadata['inputs'])

    started = _callback(
        app,
        match,
        [1, None, None, 0],
        [new_execution_state(), 'EXP-NOMINAL', 3, session, '/tmp/output', [], 'job', None],
        'start-experiment.n_clicks',
    )['execution-state']['data']
    response = _callback(
        app,
        match,
        [1, 1, None, 0],
        [started, 'EXP-NOMINAL', 3, session, '/tmp/output', [], 'run', 'historical-run'],
        'cancel-experiment.n_clicks',
    )['execution-state']['data']
    assert 'current Job' in response['notice']
    assert not response.get('read_error')
    present = _callback(
        app,
        lambda k, m: k.startswith('..execution-status.children'),
        [response, 'EXP-NOMINAL', 3, session, {'experiment_id': 'EXP-NOMINAL', 'revision': 3}],
        [[response.get('launch_session'), response.get('version', 0)]],
        'execution-state.data',
    )
    assert present['execution-poll']['disabled'] is False
    assert 'execution-events' not in present


def test_gui_option_is_explicit_frozen_and_cleared_after_start(callback_app):
    """The explicit GUI choice is frozen and resets without opting into actuation."""
    from robot_dashboard.execution import new_execution_state

    app, session, fake = callback_app
    result = _callback(
        app,
        lambda k, m: any(i['id'] == 'start-experiment' for i in m['inputs']),
        [1, None, None, 0],
        [
            new_execution_state(),
            'EXP-NOMINAL',
            3,
            session,
            '/tmp/output',
            [],
            'job',
            None,
            ['gui'],
        ],
        'start-experiment.n_clicks',
    )
    assert fake.gui_requests == [True] and fake.starts[0][-1] is False
    assert result['execution-state']['data']['launch']['gui'] is True
    assert result['run-gui']['value'] == []


def test_play_hands_paused_preview_to_backend_and_preserves_opt_in():
    """Play requests a visible scored handoff; no local process control or motion."""
    from robot_dashboard.execution import ExecutionActions, new_execution_state

    backend = ControlledBackend()
    backend.get_simulation_status = lambda: {'state': 'paused', 'job_id': 'preview-owner'}
    received = []

    def start(identifier, revision, **options):
        received.append(options)
        return {'job_id': 'scored-owner', 'state': 'queued', 'run_ids': []}

    backend.start_run = start
    for actuate in (False, True):
        actions = ExecutionActions(backend)
        state = actions.start(
            new_execution_state(),
            'EXP-EXISTING',
            3,
            'session-a',
            '/tmp/output',
            enable_actuation=actuate,
        )
        assert received[-1] == {
            'output': '/tmp/output',
            'session_id': 'session-a',
            'enable_actuation': actuate,
            'gui': True,
            'preview_id': 'preview-owner',
        }
        assert 'handoff' in state['notice'].lower()
