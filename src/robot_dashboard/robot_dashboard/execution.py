"""Explicit UI actions and lightweight monitoring over existing backend jobs/Sessions."""

import json
import uuid
from copy import deepcopy
from pathlib import Path
from threading import Lock

from dash import ALL
from dash import Input
from dash import Output
from dash import State
from dash import ctx
from dash import dcc
from dash import html
from dash import no_update

from .results import comparison_view
from .results import header_view
from .results import results_view
from .results import scenario_label
from .results import scenario_names

ACTIVE = {'queued', 'running'}
TERMINAL = {'completed', 'failed', 'cancelled'}


def new_execution_state():
    """Browser-session references survive refresh; no execution starts at construction."""
    return {
        'client_id': str(uuid.uuid4()),
        'request_key': str(uuid.uuid4()),
        'job_id': None,
        'launch_session': None,
        'attached': [],
        'version': 0,
    }


class ExecutionActions:
    """Guard repeated UI clicks; backend workers retain all execution/process ownership."""

    def __init__(self, backend):
        """Keep only transient launch references for UI click deduplication."""
        self.backend = backend
        self._lock = Lock()
        self._requests = {}
        self._latest = {}

    def _current(self, state):
        state = deepcopy(state or new_execution_state())
        latest = self._latest.get(state['client_id'])
        if latest and latest['job_id'] != state.get('job_id'):
            state.update(latest, attached=[], job=None, run_status=None)
        return state

    def start(
        self,
        state,
        identifier,
        revision,
        session_id,
        output,
        *,
        enable_actuation=False,
        gui=False,
    ):
        """Start explicitly through the backend's saved-revision preflight."""
        if (
            not isinstance(identifier, str)
            or type(revision) is not int
            or revision < 1
            or type(enable_actuation) is not bool
            or type(gui) is not bool
        ):
            raise ValueError(
                'Select a saved experiment/revision and an explicit boolean actuation opt-in'
            )
        if not session_id or session_id not in {
            s['session_id'] for s in self.backend.list_sessions(limit=1000)
        }:
            raise ValueError('Create or select an existing current Session before starting')
        with self._lock:
            token = (state['client_id'], state['request_key'])
            current = self._current(state)
            if token in self._requests:
                return self.poll(current)
            if current.get('job_id'):
                existing = self.backend.get_job_status(current['job_id'])
                if existing['state'] in ACTIVE:
                    current['notice'] = 'Existing Job is active; no duplicate launch was made.'
                    return self._observe(current, existing)
                if existing['state'] not in TERMINAL:
                    raise ValueError(
                        'Job state/cleanup is unresolved; inspect existing evidence before '
                        'launching'
                    )
            preview = self.backend.get_simulation_status()
            preview_id = preview.get('job_id') if preview.get('state') == 'paused' else None
            if preview_id:
                gui = True
            job = self.backend.start_run(
                identifier,
                revision,
                output=output,
                session_id=session_id,
                enable_actuation=enable_actuation,
                gui=gui,
                **({'preview_id': preview_id} if preview_id else {}),
            )
            reference = {
                'job_id': job['job_id'],
                'launch_session': session_id,
                'request_key': str(uuid.uuid4()),
                'launch': {
                    'experiment_id': identifier,
                    'revision': revision,
                    'enable_actuation': enable_actuation,
                    'gui': gui,
                    'output': str(output),
                },
            }
            self._requests[token] = job['job_id']
            self._latest[state['client_id']] = reference
            current.update(
                reference,
                attached=[],
                run_status=None,
                notice=(
                    'Paused preview closed; fresh scored GUI requested through Runner (handoff). '
                    'The saved duration starts after readiness and settling.'
                    if preview_id
                    else 'Job started through the existing backend.'
                ),
            )
            return self._observe(current, job)

    def _observe(self, state, job):
        attached = list(state.get('attached', []))
        errors = []
        for run_id in job.get('run_ids', []):
            if run_id not in attached:
                try:
                    self.backend.add_run_to_session(state['launch_session'], run_id)
                    attached.append(run_id)
                except (ValueError, KeyError, OSError) as error:
                    errors.append(str(error))
        run_status = None
        if job.get('run_ids'):
            try:
                run_status = self.backend.get_run_status(job['run_ids'][-1])
            except (ValueError, KeyError, OSError) as error:
                errors.append(str(error))
        signature = json.dumps(
            [job.get('state'), job.get('runs'), attached, run_status, errors], sort_keys=True
        )
        state['version'] = state.get('version', 0) + (signature != state.get('signature'))
        state.pop('read_error', None)
        state.update(
            job=job,
            attached=attached,
            run_status=run_status,
            signature=signature,
            association_errors=errors,
        )
        return state

    def poll(self, state):
        """Resume an existing Job and attach newly proven UUIDs, never create a worker."""
        state = self._current(state)
        if not state.get('job_id'):
            return state
        return self._observe(state, self.backend.get_job_status(state['job_id']))

    def cancel(self, state, *, run_id=None):
        """Delegate cancellation, including native timeout and cleanup uncertainty."""
        state = self._current(state)
        if not state.get('job_id'):
            raise ValueError('No current Job to cancel')
        job = self.backend.get_job_status(state['job_id'])
        if run_id and run_id not in job.get('run_ids', []):
            raise ValueError('Selected Run is not part of the current Job')
        try:
            job = (
                self.backend.cancel_run(run_id)
                if run_id
                else self.backend.cancel_job(state['job_id'])
            )
            state['notice'] = (
                'Cancellation returned the recorded owner status; inspect cleanup below.'
            )
        except TimeoutError as error:
            state['notice'] = str(error)
            job = self.backend.get_job_status(state['job_id'])
        return self._observe(state, job)


def execution_layout(backend):
    """Return named, mounted workspace sections using the existing callback IDs."""
    return {
        'state': [
            dcc.Store(
                id='execution-state', data=new_execution_state(), storage_type='session'
            ),
            dcc.Store(id='execution-events', data=0),
            dcc.Store(id='row-highlight'),
            dcc.Interval(id='execution-poll', interval=2000, disabled=True, n_intervals=0),
        ],
        'session_controls': html.Div(
            [
                html.Small('CURRENT SESSION'),
                dcc.Dropdown(
                    id='current-session',
                    options=[],
                    placeholder='Select a Session',
                    persistence=True,
                    persistence_type='local',
                    clearable=False,
                ),
                html.Details(
                    [
                        html.Summary('⋯'),
                        html.Div(
                            [
                                dcc.Input(
                                    id='session-title',
                                    type='text',
                                    placeholder='Session title',
                                ),
                                html.Button(
                                    'Create Session', id='create-session', className='button'
                                ),
                                html.Button(
                                    'Refresh Sessions',
                                    id='refresh-sessions',
                                    className='button',
                                ),
                                html.Hr(),
                                html.Button(
                                    'Rename Session', id='rename-session', className='button'
                                ),
                                html.Button(
                                    'Delete Session',
                                    id='delete-session',
                                    className='button danger',
                                ),
                                html.Div(id='session-management-feedback', role='status'),
                                html.Div(id='session-feedback', role='status'),
                            ],
                            className='session-popover',
                        ),
                    ],
                    title='Create or manage Sessions',
                ),
            ],
            className='session-controls',
        ),
        'monitor': html.Section(
            [
                html.H2('Run Experiment'),
                html.P(id='run-selection', className='context-label'),
                dcc.Checklist(
                    id='actuation-opt-in',
                    options=[
                        {'label': 'Enable physical actuation (opt-in)', 'value': 'actuate'}
                    ],
                    value=[],
                ),
                dcc.Checklist(
                    id='run-gui',
                    options=[
                        {
                            'label': 'Show Gazebo GUI',
                            'value': 'gui',
                            'disabled': not backend.gazebo_setup_capabilities()[
                                'gui_available'
                            ],
                        }
                    ],
                    value=[],
                    className='run-gui',
                ),
                html.Div(
                    [
                        html.Button(
                            '▶ Play',
                            id='start-experiment',
                            className='button primary',
                            disabled=True,
                        ),
                        html.Button(
                            '■ Stop / Cancel',
                            id='cancel-experiment',
                            className='button danger',
                            disabled=True,
                        ),
                        html.Button(
                            '↻',
                            id='refresh-execution',
                            className='button',
                            title='Refresh status',
                        ),
                    ],
                    className='action-group',
                ),
                html.Details(
                    [
                        html.Summary('Advanced execution'),
                        html.Label('Cancellation scope'),
                        dcc.Dropdown(
                            id='cancel-scope',
                            options=[
                                {'label': 'Entire current Job', 'value': 'job'},
                                {'label': 'Selected Run and its Job', 'value': 'run'},
                            ],
                            value='job',
                            clearable=False,
                        ),
                        html.P(
                            'Stop cancels the owning Job, including remaining seeds and variants. '
                            'Actuation defaults OFF.',
                            className='hint',
                        ),
                        html.Label('Output location'),
                        dcc.Input(
                            id='run-output',
                            type='text',
                            value=str(Path(backend.directory) / 'runs'),
                        ),
                    ],
                    className='output-details',
                ),
                html.Div(id='execution-feedback', role='status', **{'aria-live': 'polite'}),
                html.Div(id='execution-status'),
            ],
            id='run-monitor',
            className='panel execution-panel',
        ),
        'session': html.Section(
            [
                html.Div(
                    [
                        html.H2('Experiment Comparison · RMSE'),
                    ],
                    className='panel-heading',
                ),
                html.Div(id='session-comparison'),
            ],
            id='session-workspace',
            className='panel session-panel',
        ),
        'results': html.Section(
            [
                html.Div(
                    [
                        dcc.Dropdown(
                            id='results-run',
                            options=[],
                            placeholder='Choose a recorded Run',
                            persistence=True,
                            persistence_type='session',
                            clearable=False,
                        ),
                        html.Details(
                            [
                                html.Summary('Advanced paging'),
                                html.Label('Sample offset', htmlFor='result-offset'),
                                dcc.Input(
                                    id='result-offset',
                                    type='number',
                                    min=0,
                                    step=1000,
                                    value=0,
                                ),
                            ],
                            className='paging-details',
                        ),
                        html.Button('↻ Refresh', id='refresh-results', className='button'),
                    ],
                    className='results-controls',
                ),
                dcc.Loading(
                    html.Div(id='recorded-result-body'),
                    type='circle',
                    parent_className='result-loading',
                ),
            ],
            id='recorded-results',
            className='results-panel',
        ),
    }


def _job_view(state):
    """Keep genuine execution state visible and ownership evidence in diagnostics."""
    job = state.get('job') or {}
    if not state.get('job_id'):
        return html.P(
            'Ready when a saved configuration and Session are selected.', className='muted'
        )
    latest = state.get('run_status') or {}
    phase = job.get('state', 'Status not loaded').capitalize()
    error = job.get('error') or latest.get('error')
    code = (
        error.get('code', 'Execution error') if isinstance(error, dict) else 'Execution error'
    )
    return html.Div(
        [
            html.Div(
                [
                    html.Span(
                        phase, className='status-chip ' + str(job.get('state', 'unknown'))
                    ),
                    html.Span(
                        'Stop requested · awaiting cleanup'
                        if job.get('stop_requested')
                        else ''
                    ),
                ],
                className='job-summary',
            ),
            html.P(code, className='notice error') if error else None,
            html.Details(
                [
                    html.Summary('Execution / safety diagnostics'),
                    html.Pre(
                        json.dumps(
                            {
                                'job_id': state['job_id'],
                                'launch_session': state.get('launch_session'),
                                'frozen_launch': state.get('launch'),
                                'attempt': latest,
                                'job': job,
                                'association_errors': state.get('association_errors', []),
                            },
                            indent=2,
                        ),
                        className='json-preview',
                    ),
                ],
                className='job-details',
            ),
        ]
    )


def register_execution(app, backend):
    """Register bounded polling and explicit UI actions; no startup launches."""
    actions = ExecutionActions(backend)

    @app.callback(
        Output('current-session', 'options'),
        Output('current-session', 'value'),
        Output('session-feedback', 'children'),
        Input('create-session', 'n_clicks'),
        Input('refresh-sessions', 'n_clicks'),
        Input('management-events', 'data'),
        State('session-title', 'value'),
        State('current-session', 'value'),
    )
    def sessions(_create, _refresh, _management, title, selected):
        try:
            if ctx.triggered_id == 'create-session':
                created = backend.create_session({'title': title.strip()} if title and title.strip() else None)
                selected = created['session_id']
            rows = backend.list_sessions(limit=1000)
            options = [
                {
                    'label': r['metadata'].get('title', 'Untitled Session'),
                    'value': r['session_id'],
                }
                for r in rows
            ]
            known = {r['value'] for r in options}
            return (
                options,
                (
                    selected
                    if selected in known
                    else (
                        None
                        if ctx.triggered_id == 'management-events'
                        else (options[-1]['value'] if options else None)
                    )
                ),
                '',
            )
        except (ValueError, KeyError, OSError) as error:
            return no_update, no_update, html.Div(str(error), className='notice error')

    @app.callback(
        Output('execution-state', 'data'),
        Output('actuation-opt-in', 'value'),
        Output('run-gui', 'value'),
        Input('start-experiment', 'n_clicks'),
        Input('cancel-experiment', 'n_clicks'),
        Input('refresh-execution', 'n_clicks'),
        Input('execution-poll', 'n_intervals'),
        State('execution-state', 'data'),
        State('source-experiment', 'value'),
        State('source-revision', 'value'),
        State('current-session', 'value'),
        State('run-output', 'value'),
        State('actuation-opt-in', 'value'),
        State('cancel-scope', 'value'),
        State('results-run', 'value'),
        State('run-gui', 'value'),
        prevent_initial_call=True,
        running=[
            (Output('start-experiment', 'className'), 'button primary busy', 'button primary'),
            (Output('cancel-experiment', 'className'), 'button danger busy', 'button danger'),
        ],
    )
    def operate(
        _start,
        _stop,
        _refresh,
        _poll,
        state,
        identifier,
        revision,
        session_id,
        output,
        opt_in,
        cancel_scope,
        selected_run,
        gui_flags,
    ):
        state = state or new_execution_state()
        try:
            if ctx.triggered_id == 'start-experiment':
                if not isinstance(opt_in or [], list) or not isinstance(gui_flags or [], list):
                    raise ValueError('Actuation requires the explicit launch checkbox')
                started = actions.start(
                    state,
                    identifier,
                    revision,
                    session_id,
                    output,
                    enable_actuation='actuate' in (opt_in or []),
                    gui='gui' in (gui_flags or []),
                )
                return started, [], []
            if ctx.triggered_id == 'cancel-experiment':
                if cancel_scope == 'run' and not selected_run:
                    raise ValueError('Choose a Run from the current Job to cancel')
                return (
                    actions.cancel(
                        state, run_id=selected_run if cancel_scope == 'run' else None
                    ),
                    no_update,
                    no_update,
                )
            return actions.poll(state), no_update, no_update
        except (ValueError, TypeError, KeyError, OSError, TimeoutError) as error:
            state = deepcopy(state)
            state['notice'] = str(error)
            state['read_error'] = ctx.triggered_id in ('execution-poll', 'refresh-execution')
            return state, no_update, no_update

    @app.callback(
        Output('execution-status', 'children'),
        Output('execution-feedback', 'children'),
        Output('execution-poll', 'disabled'),
        Output('execution-events', 'data'),
        Output('start-experiment', 'disabled'),
        Output('cancel-experiment', 'disabled'),
        Output('run-selection', 'children'),
        Input('execution-state', 'data'),
        Input('source-experiment', 'value'),
        Input('source-revision', 'value'),
        Input('current-session', 'value'),
        Input('selected-snapshot', 'data'),
        State('execution-events', 'data'),
    )
    def present(state, identifier, revision, session_id, snapshot, previous_event):
        state = state or {}
        job = state.get('job') or {}
        active = bool(state.get('job_id')) and job.get('state') not in TERMINAL
        valid = bool(
            snapshot
            and snapshot.get('experiment_id') == identifier
            and snapshot.get('revision') == revision
        )
        event = [state.get('launch_session'), state.get('version', 0)]
        return (
            _job_view(state),
            html.P(state.get('notice', ''), className='notice neutral'),
            not (state.get('job_id') and job.get('state') in (ACTIVE | {None}))
            or bool(state.get('read_error')),
            event if event != previous_event else no_update,
            active or not valid or not session_id,
            not state.get('job_id') or job.get('state') not in ACTIVE,
            ('Saved configuration ready' if valid else 'Select a saved configuration.'),
        )

    @app.callback(
        Output('session-comparison', 'children'),
        Output('results-run', 'options'),
        Output('results-run', 'value'),
        Input('current-session', 'value'),
        Input('execution-events', 'data'),
        Input('refresh-sessions', 'n_clicks'),
        Input({'type': 'session-run', 'run_id': ALL}, 'n_clicks'),
        Input('management-events', 'data'),
        State('results-run', 'value'),
        State('execution-state', 'data'),
    )
    def compare(session_id, _events, _refresh, _rows, _management, selected, state):
        rows, view = [], html.P(
            'Create or select a persistent Session.', className='empty-state'
        )
        try:
            if (
                isinstance(ctx.triggered_id, dict)
                and ctx.triggered_id.get('type') == 'session-run'
            ):
                # Dynamic insertion has no clicks: only an explicit row action may select.
                if isinstance(ctx.triggered_id.get('run_id'), str) and any(_rows or []):
                    selected = ctx.triggered_id['run_id']
            if session_id:
                comparison = backend.get_session_comparison(session_id)
                rows = comparison['runs']
                view = comparison_view(
                    comparison, selected_run=selected, names=scenario_names(backend, rows)
                )
            known = {r['run_id']: r for r in rows}
            for row in ((state or {}).get('job') or {}).get('runs', []):
                if row.get('run_id'):
                    try:
                        backend.get_run_status(row['run_id'])
                    except (KeyError, ValueError, OSError):
                        continue
                    known[row['run_id']] = row
            names = scenario_names(backend, known.values())
            options = [
                {
                    'label': f'{scenario_label(r, names)} · Run {index + 1}',
                    'value': rid,
                }
                for index, (rid, r) in enumerate(known.items())
            ]
            return (
                view,
                options,
                (
                    selected
                    if selected in known
                    else (
                        None
                        if 'management-events.data' in ctx.triggered_prop_ids
                        else (options[-1]['value'] if options else None)
                    )
                ),
            )
        except (ValueError, TypeError, KeyError, OSError) as error:
            return html.Div(str(error), className='notice error'), no_update, no_update

    @app.callback(
        Output('recorded-result-body', 'children'),
        Output('run-context', 'children'),
        Input('results-run', 'value'),
        Input('refresh-results', 'n_clicks'),
        Input('execution-events', 'data'),
        State('result-offset', 'value'),
    )
    def show_results(run_id, _refresh, _events, offset):
        try:
            if offset is None or int(offset) != offset or offset < 0:
                raise ValueError('Scored sample offset must be a nonnegative integer')
            result = backend.get_run_results(run_id) if run_id else None
            return results_view(
                backend, run_id, offset=int(offset), result=result
            ), header_view(result, names=scenario_names(backend))
        except (ValueError, TypeError, KeyError, OSError) as error:
            return (
                html.Div('Results unavailable: ' + str(error), className='notice error'),
                html.Span('Recorded Run unavailable', className='status-text'),
            )

    app.clientside_callback(
        """function(id) {
            requestAnimationFrame(function() {
                document.querySelectorAll('[data-run-id]').forEach(function(row) {
                    row.classList.toggle('selected-run', row.dataset.runId === id);
                });
            });
            return window.dash_clientside.no_update;
        }""",
        Output('row-highlight', 'data'),
        Input('results-run', 'value'),
        Input('session-comparison', 'children'),
    )
