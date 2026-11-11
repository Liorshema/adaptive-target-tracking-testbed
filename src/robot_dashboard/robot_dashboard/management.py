"""Compact, confirmation-bound presentation over existing metadata owners."""

import re

from dash import Input, Output, State, ctx, dcc, html, no_update


def draft_identity(backend, mode, name, flags, current):
    """Generate only new draft IDs; owner collision checks never retarget identity."""
    if mode != 'create':
        return current, True, '', False
    custom = 'custom' in (flags or [])
    try:
        identifier = (
            backend.validate_experiment_id(current)
            if custom
            else backend.propose_experiment_id(name)
        )
        return identifier, not custom, '', False
    except (ValueError, TypeError, KeyError) as error:
        return current if custom else None, not custom, str(error), True


class ManagementActions:
    """Bind each confirmation to the exact authoritative impact the user saw."""

    def __init__(self, backend):
        """Keep the authoritative owner API without a parallel store."""
        self.backend = backend

    def confirmation(self, kind, target):
        """Resolve actual ownership and freeze its concurrency token without writes."""
        if kind not in ('session', 'scenario'):
            raise ValueError('Unknown management action')
        identifier = target.get('id') or target['session']['session_id']
        pending = self.backend.preview_deletion(kind, identifier)
        label = kind.title()
        message = f'Delete {label} “{pending["name"]}”? '
        message += f'{pending["run_count"]} Runs will be deleted. '
        if kind == 'session':
            message += f'{pending["shared_run_count"]} shared Runs will be preserved. '
        else:
            message += f'{pending["session_count"]} Sessions will be updated. '
            message += 'All saved Scenario revisions will be removed. '
        message += 'Associated results and recorded telemetry will be permanently removed.'
        if pending.get('notes'):
            message += ' ' + ' '.join(str(note) for note in pending['notes'])
        return pending, message

    def remove(self, kind, pending, selected):
        """Reject retargeting; the backend revalidates scope and commits deletion."""
        if not pending or pending.get('kind') != kind or pending.get('id') != selected:
            raise ValueError('The selection changed. Open the confirmation again.')
        result = self.backend.delete_resources(kind, selected, expected_token=pending['token'])
        if result.get('completed') is not True:
            raise ValueError('Cleanup is incomplete. Review diagnostics before trying again.')
        return result


def scenario_menu():
    """Keep management secondary to the existing Scenario selector."""
    return html.Div(
        [
            html.Button('New', id='new-scenario', className='button'),
            html.Button('Duplicate', id='duplicate-scenario', className='button'),
            html.Div(
                [
                    html.Button('Rename', id='rename-scenario', className='button'),
                    html.Button(
                        'Delete', id='delete-scenario', className='button danger'
                    ),
                    html.Div(id='scenario-management-feedback', role='status'),
                ],
                className='scenario-management-actions',
            ),
        ],
        className='scenario-management',
        title='Manage selected Scenario',
    )


def _dialog(kind, action):
    prefix = f'{kind}-{action}'
    title = f'{action.title()} {kind.title()}'
    body = (
        [
            html.Label('Display name', htmlFor=f'{prefix}-title'),
            dcc.Input(
                id=f'{prefix}-title',
                type='text',
                maxLength=200,
                placeholder=f'{kind.title()} name',
                debounce=False,
            ),
        ]
        if action == 'rename'
        else [html.P(id=f'{prefix}-content', className='management-impact')]
    )
    return html.Div(
        html.Section(
            [
                html.H2(title, id=f'{prefix}-heading'),
                *body,
                html.Div(
                    [
                        html.Button('Cancel', id=f'{prefix}-cancel', className='button'),
                        html.Button(
                            title,
                            id=f'{prefix}-confirm',
                            className=(
                                'button danger' if action == 'delete' else 'button primary'
                            ),
                        ),
                    ],
                    className='management-dialog-actions',
                ),
            ],
            className='management-dialog',
            role='dialog',
            tabIndex=-1,
            **{'aria-modal': 'true', 'aria-labelledby': f'{prefix}-heading'},
        ),
        id=f'{prefix}-dialog',
        className='management-overlay',
        hidden=True,
    )


def management_state():
    """Transient confirmation snapshots only; existing owners retain persistence."""
    return [
        dcc.Store(id='management-events', data=0),
        dcc.Store(id='session-management-target'),
        *[
            dcc.Store(id=f'{kind}-{action}-target')
            for kind in ('session', 'scenario')
            for action in ('delete', 'rename')
        ],
        *[
            _dialog(kind, action)
            for kind in ('session', 'scenario')
            for action in ('delete', 'rename')
        ],
        html.Div(id='management-notice', role='status', **{'aria-live': 'polite'}),
    ]


def _notice(error):
    """Keep primary feedback short; retain precise owner diagnostics on demand."""
    if isinstance(error, KeyError):
        return 'Selection no longer exists. Refresh and select again.'
    detail = str(error)
    safe = re.sub(r'[a-f0-9]{8}(?:-[a-f0-9]{4}){3}-[a-f0-9]{12}', '(owned record)', detail)
    primary = (
        safe
        if len(safe) <= 180 and '/' not in safe and '\n' not in safe
        else ('This action could not be completed safely. Review diagnostics and try again.')
    )
    return html.Div(
        [
            html.Span(primary),
            html.Details([html.Summary('Advanced diagnostics'), html.Pre(detail)]),
        ],
        className='notice error',
    )


def _success(kind, result):
    count = result.get('deletedRunCount', result.get('run_count', 0))
    text = f'{kind.title()} deleted. {count} Runs deleted.'
    if kind == 'scenario':
        sessions = result.get('affectedSessionCount', result.get('session_count', 0))
        text += f' {sessions} Sessions updated.'
    elif result.get('shared_run_count'):
        text += f' {result["shared_run_count"]} shared Runs preserved.'
    return text


def register_management(app, backend):
    """Explicit confirmation is the sole UI path to destructive owner operations."""
    actions = ManagementActions(backend)

    @app.callback(
        Output('draft-mode', 'value'),
        Output('new-name', 'value'),
        Output('source-experiment', 'value', allow_duplicate=True),
        Output('new-id', 'value', allow_duplicate=True),
        Input('new-scenario', 'n_clicks'),
        Input('duplicate-scenario', 'n_clicks'),
        State('source-experiment', 'value'),
        prevent_initial_call=True,
    )
    def new_scenario(_new, _duplicate, selected):
        rows = backend.list_scenarios()
        source = selected or (rows[0]['id'] if rows else None)
        row = next((r for r in rows if r['id'] == source), None)
        if row is None:
            return no_update, no_update, no_update, no_update
        base = row['name'] if ctx.triggered_id == 'duplicate-scenario' else 'scenario'
        existing = [r['name'] for r in backend.list_scenarios(include_deleted=True)]
        while True:
            name = backend.unique_display_name(base, existing)
            try:
                identifier = backend.propose_experiment_id(name)
                break
            except ValueError:
                existing.append(name)
        return 'create', name, source, identifier

    @app.callback(
        Output('new-id', 'value'),
        Output('new-id', 'disabled'),
        Output('identity-feedback', 'children'),
        Output('load-draft', 'disabled'),
        Input('draft-mode', 'value'),
        Input('new-name', 'value'),
        Input('custom-id', 'value'),
        Input('new-id', 'value'),
        Input('editor-busy', 'data'),
    )
    def identity(mode, name, flags, current, busy):
        identifier, disabled, error, invalid = draft_identity(backend, mode, name, flags, current)
        return identifier, disabled, error, invalid or bool(busy)

    @app.callback(
        Output('session-management-target', 'data'),
        Output('session-title', 'value'),
        Input('current-session', 'value'),
        Input('management-events', 'data'),
    )
    def session_target(selected, _events):
        try:
            target = backend.get_session(selected) if selected else None
            return target, target['session']['metadata'].get('title', '') if target else ''
        except (ValueError, TypeError, KeyError, OSError):
            return None, ''

    @app.callback(
        Output('session-management-feedback', 'children'),
        Output('session-delete-target', 'data'),
        Output('session-delete-dialog', 'hidden'),
        Output('session-delete-content', 'children'),
        Output('session-rename-target', 'data'),
        Output('session-rename-dialog', 'hidden'),
        Output('session-rename-title', 'value'),
        Output('management-events', 'data', allow_duplicate=True),
        Output('current-session', 'value', allow_duplicate=True),
        Output('results-run', 'value', allow_duplicate=True),
        Output('management-notice', 'children', allow_duplicate=True),
        Input('rename-session', 'n_clicks'),
        Input('delete-session', 'n_clicks'),
        Input('session-delete-confirm', 'n_clicks'),
        Input('session-delete-cancel', 'n_clicks'),
        Input('session-rename-confirm', 'n_clicks'),
        Input('session-rename-cancel', 'n_clicks'),
        State('current-session', 'value'),
        State('session-management-target', 'data'),
        State('session-delete-target', 'data'),
        State('session-rename-target', 'data'),
        State('session-rename-title', 'value'),
        State('management-events', 'data'),
        prevent_initial_call=True,
    )
    def session_action(
        _rename,
        _delete,
        _confirm,
        _cancel,
        _save,
        _close,
        selected,
        target,
        pending,
        rename,
        title,
        event,
    ):
        result = [no_update] * 11
        trigger = ctx.triggered_id
        try:
            if trigger in ('session-delete-cancel', 'session-rename-cancel'):
                return '', None, True, '', None, True, no_update, *([no_update] * 4)
            if trigger == 'session-delete-confirm':
                impact = actions.remove('session', pending, selected)
                message = _success('session', impact)
                return (
                    message,
                    None,
                    True,
                    '',
                    None,
                    True,
                    '',
                    (event or 0) + 1,
                    None,
                    None,
                    message,
                )
            if trigger == 'session-rename-confirm':
                if not rename or rename.get('id') != selected:
                    raise ValueError('The selection changed. Open the rename dialog again.')
                backend.rename_session(selected, title, expected_token=rename['token'])
                return (
                    'Session renamed.',
                    None,
                    True,
                    '',
                    None,
                    True,
                    no_update,
                    (event or 0) + 1,
                    no_update,
                    no_update,
                    'Session renamed.',
                )
            if not target or target['session']['session_id'] != selected:
                raise ValueError(
                    'Select a Session and wait for its metadata before managing it.'
                )
            if trigger == 'delete-session' and _delete:
                pending, message = actions.confirmation('session', {'id': selected})
                result[:7] = ['', pending, False, message, None, True, no_update]
            elif trigger == 'rename-session' and _rename:
                rename = {'id': selected, 'token': target['token']}
                name = target['session']['metadata'].get('title', '')
                result[:7] = ['', None, True, '', rename, False, name]
            return tuple(result)
        except (ValueError, TypeError, KeyError, OSError, RuntimeError) as error:
            notice = _notice(error)
            return (
                notice,
                None,
                True,
                '',
                None,
                True,
                no_update,
                no_update,
                no_update,
                no_update,
                notice,
            )

    @app.callback(
        Output('scenario-management-feedback', 'children'),
        Output('scenario-delete-target', 'data'),
        Output('scenario-delete-dialog', 'hidden'),
        Output('scenario-delete-content', 'children'),
        Output('scenario-rename-target', 'data'),
        Output('scenario-rename-dialog', 'hidden'),
        Output('scenario-rename-title', 'value'),
        Output('management-events', 'data', allow_duplicate=True),
        Output('source-experiment', 'options', allow_duplicate=True),
        Output('source-experiment', 'value'),
        Output('builder-state', 'data', allow_duplicate=True),
        Output('component-cards', 'children', allow_duplicate=True),
        Output('execution-fields', 'children', allow_duplicate=True),
        Output('results-run', 'value', allow_duplicate=True),
        Output('management-notice', 'children', allow_duplicate=True),
        Input('rename-scenario', 'n_clicks'),
        Input('delete-scenario', 'n_clicks'),
        Input('scenario-delete-confirm', 'n_clicks'),
        Input('scenario-delete-cancel', 'n_clicks'),
        Input('scenario-rename-confirm', 'n_clicks'),
        Input('scenario-rename-cancel', 'n_clicks'),
        State('source-experiment', 'value'),
        State('source-revision', 'value'),
        State('scenario-delete-target', 'data'),
        State('scenario-rename-target', 'data'),
        State('scenario-rename-title', 'value'),
        State('management-events', 'data'),
        prevent_initial_call=True,
    )
    def scenario_action(
        _rename,
        _delete,
        _confirm,
        _cancel,
        _save,
        _close,
        selected,
        revision,
        pending,
        rename,
        title,
        event,
    ):
        result = [no_update] * 15
        trigger = ctx.triggered_id
        try:
            if trigger in ('scenario-delete-cancel', 'scenario-rename-cancel'):
                return '', None, True, '', None, True, no_update, *([no_update] * 8)
            if trigger == 'scenario-delete-confirm':
                impact = actions.remove('scenario', pending, selected)
                from .app import execution_fields, experiment_options

                message = _success('scenario', impact)
                return (
                    message,
                    None,
                    True,
                    '',
                    None,
                    True,
                    '',
                    (event or 0) + 1,
                    experiment_options(backend),
                    None,
                    None,
                    [],
                    execution_fields(),
                    None,
                    message,
                )
            if trigger == 'scenario-rename-confirm':
                if not rename or rename.get('id') != selected:
                    raise ValueError('The selection changed. Open the rename dialog again.')
                backend.rename_experiment(
                    selected,
                    title,
                    expected_revision=rename['revision'],
                    expected_name=rename['name'],
                )
                from .app import experiment_options

                return (
                    'Scenario renamed.',
                    None,
                    True,
                    '',
                    None,
                    True,
                    no_update,
                    (event or 0) + 1,
                    experiment_options(backend),
                    selected,
                    no_update,
                    no_update,
                    no_update,
                    no_update,
                    'Scenario renamed.',
                )
            row = next((r for r in backend.list_scenarios() if r['id'] == selected), None)
            if row is None:
                raise ValueError('Select a saved Scenario before managing it.')
            # Resolve actual selected identity independently of the revision callback.
            # The dialog freezes fresh authoritative revision/name for confirmation.
            backend.ensure_scenario(selected)
            row = next(r for r in backend.list_scenarios() if r['id'] == selected)
            if trigger == 'delete-scenario' and _delete:
                pending, message = actions.confirmation('scenario', {'id': selected})
                result[:7] = ['', pending, False, message, None, True, no_update]
            elif trigger == 'rename-scenario' and _rename:
                rename = {
                    'id': selected,
                    'name': row['name'],
                    'revision': row['revision'],
                }
                result[:7] = ['', None, True, '', rename, False, row['name']]
            return tuple(result)
        except (ValueError, TypeError, KeyError, OSError, RuntimeError) as error:
            notice = _notice(error)
            return notice, None, True, '', None, True, no_update, *([no_update] * 7), notice
