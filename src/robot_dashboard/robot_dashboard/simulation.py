"""Thin unscored-preview controls over the existing experiment backend owner."""

import json

from dash import Input, Output, State, ctx, dcc, html

ACTIVE = {'opening', 'paused', 'running', 'closing', 'worker_lost'}


def simulation_layout():
    """Mount controls once so configuration redraws cannot replay Open clicks."""
    return html.Div(
        [
            dcc.Store(id='simulation-state'),
            dcc.Interval(id='simulation-poll', interval=2000, disabled=True),
            html.Div(
                [
                    html.Button(
                        'Open Simulation',
                        id='open-simulation',
                        disabled=True,
                        className='button',
                    ),
                    html.Button(
                        'Close',
                        id='close-simulation',
                        disabled=True,
                        className='button danger',
                    ),
                ],
                className='action-group',
            ),
            html.P(
                'Unscored preview · paused · actuation OFF',
                className='hint',
            ),
            html.Div(id='simulation-status', role='status', **{'aria-live': 'polite'}),
        ],
        className='simulation-lifecycle',
    )


def register_simulation(app, backend):
    """Observe persistent owner status on load; launch and close only on explicit clicks."""

    @app.callback(
        Output('simulation-state', 'data'),
        Input('open-simulation', 'n_clicks'),
        Input('close-simulation', 'n_clicks'),
        Input('simulation-poll', 'n_intervals'),
        Input('refresh-execution', 'n_clicks'),
        State('simulation-state', 'data'),
        State('source-experiment', 'value'),
        State('source-revision', 'value'),
        State('source-variant', 'value'),
        State('selected-snapshot', 'data'),
        running=[
            (Output('open-simulation', 'className'), 'button busy', 'button'),
            (Output('close-simulation', 'className'), 'button danger busy', 'button danger'),
        ],
    )
    def operate(
        opened, closed, _poll, _refresh, state, identifier, revision, variant, snapshot
    ):
        try:
            if ctx.triggered_id == 'open-simulation' and opened:
                if (
                    not snapshot
                    or snapshot.get('experiment_id') != identifier
                    or snapshot.get('revision') != revision
                ):
                    raise ValueError('Select and validate a saved revision before Open')
                return backend.open_simulation(identifier, revision, variant=variant)
            if ctx.triggered_id == 'close-simulation' and closed:
                current = backend.get_simulation_status()
                if current.get('job_id'):
                    return backend.close_simulation(current['job_id'])
            return backend.get_simulation_status()
        except (ValueError, TypeError, KeyError, OSError, TimeoutError) as error:
            try:
                current = backend.get_simulation_status()
            except (ValueError, KeyError, OSError):
                current = state or {}
            return {**current, 'action_error': str(error)}

    @app.callback(
        Output('simulation-status', 'children'),
        Output('simulation-poll', 'disabled'),
        Output('open-simulation', 'disabled'),
        Output('close-simulation', 'disabled'),
        Input('simulation-state', 'data'),
        Input('source-experiment', 'value'),
        Input('source-revision', 'value'),
        Input('source-variant', 'value'),
        Input('selected-snapshot', 'data'),
    )
    def present(state, identifier, revision, variant, snapshot):
        state = state or {}
        phase = state.get('state', 'checking')
        active = phase in ACTIVE
        capability = backend.gazebo_setup_capabilities()
        valid = bool(
            snapshot
            and snapshot.get('experiment_id') == identifier
            and snapshot.get('revision') == revision
            and variant
        )
        clean = state.get('cleanup_complete') is True
        identity = state.get('experiment_identity') or {}
        changed = active and (
            identity.get('id') != identifier
            or identity.get('revision') != revision
            or state.get('variant') != variant
        )
        error = state.get('action_error') or state.get('error')
        display_phase = (
            'Error' if state.get('action_error') and not active else phase.capitalize()
        )
        children = [html.Strong(display_phase, className='simulation-phase')]
        if changed:
            children.append(
                html.P('Selection changed. Close before opening the new configuration.')
            )
        if error:
            children.append(html.P(str(error), className='status-error'))
        if not capability['gui_available']:
            children.append(html.P(capability['reason'], className='status-error'))
        if state.get('job_id'):
            children.append(
                html.Details(
                    [
                        html.Summary('Advanced ownership & cleanup'),
                        html.Pre(json.dumps(state, indent=2), className='json-preview'),
                    ]
                )
            )
        return (
            children,
            phase not in {'opening', 'paused', 'running', 'closing'},
            active or not clean or not valid or not capability['gui_available'],
            phase not in {'opening', 'paused', 'running', 'closing'},
        )
