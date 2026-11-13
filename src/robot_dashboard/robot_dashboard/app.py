"""Dash presentation and browser-local drafts over the public experiment backend."""

import json
import logging
from copy import deepcopy
from pathlib import Path

from dash import ALL
from dash import Dash
from dash import Input
from dash import MATCH
from dash import Output
from dash import State
from dash import ctx
from dash import dcc
from dash import html
from dash import no_update
from dash.exceptions import PreventUpdate

from .dynamics_forms import (restore_library_model, choose_target_model, hydrate_target_pin, target_options)
from .dynamics_studio import register_studio, studio_layout
from .execution import execution_layout
from .execution import register_execution
from .forms import INHERIT
from .forms import apply_execution
from .forms import assemble_draft
from .forms import begin_draft
from .forms import fingerprint
from .forms import parameter_rows
from .management import management_state
from .management import register_management
from .management import scenario_menu
from .results import register_trajectory

LOG = logging.getLogger(__name__)


def readable(name):
    """Use stable registry keys with legible labels, without a parallel name catalog."""
    return name.replace('_', ' ').capitalize()


def formatted(value):
    """Render detached metadata without interpreting scientific values."""
    return json.dumps(value, ensure_ascii=False, allow_nan=False)


def experiment_options(backend):
    """List public Catalog labels without altering persisted experiment names."""
    return [{'label': r['name'], 'value': r['id']} for r in backend.list_scenarios()]


def component_choices(role, options, inherited, selection):
    """Hide unavailable prediction choices, retaining explicit historical references."""
    filtered = role in {'predictor', 'adaptation', 'fusion'}
    result = []
    for option in sorted(options, key=lambda value: value['id'] != inherited):
        identifier = option['id']
        unavailable = not option['deployable'] or option['compatible'] is False
        selected = identifier == selection or (selection == INHERIT and identifier == inherited)
        if filtered and unavailable and not selected:
            continue
        label = option.get('capabilities', {}).get('display_name', readable(identifier))
        result.append({'label': label + (' · unavailable (saved selection)' if unavailable and selected
                                         else ' · unavailable' if unavailable else ''),
                       'value': INHERIT if identifier == inherited and selection != inherited
                       else identifier, 'disabled': unavailable})
    return result


def _bounds(schema):
    bounds = []
    for key, sign in [
        ('minimum', '≥'),
        ('exclusiveMinimum', '>'),
        ('maximum', '≤'),
        ('exclusiveMaximum', '<'),
    ]:
        if key in schema:
            bounds.append(f'{sign} {schema[key]}')
    for key, text in [('minItems', 'min items'), ('maxItems', 'max items')]:
        if key in schema:
            bounds.append(f'{text}: {schema[key]}')
    return ', '.join(bounds) or 'No bounds declared'


def _parameter(role, row):
    key, kind, schema = row['key'], row['kind'], row['schema']
    field = {'role': role, 'key': key, 'kind': kind}
    value = row['value']
    common = {
        'id': {'type': 'param', **field},
        'disabled': not row['explicit'],
        'className': 'parameter-input',
    }
    if not row['editable']:
        control = html.Code(formatted(value))
    elif 'enum' in schema:
        control = dcc.Dropdown(
            options=[{'label': formatted(v), 'value': v} for v in schema['enum']],
            value=value,
            clearable=False,
            **common,
        )
    elif kind == 'boolean':
        control = dcc.Dropdown(
            options=[{'label': 'True', 'value': True}, {'label': 'False', 'value': False}],
            value=value,
            clearable=False,
            **common,
        )
    elif kind in ('number', 'integer'):
        control = dcc.Input(
            type='number',
            value=value,
            step=1 if kind == 'integer' else 'any',
            min=schema.get('minimum', schema.get('exclusiveMinimum')),
            max=schema.get('maximum', schema.get('exclusiveMaximum')),
            **common,
        )
    elif kind == 'string':
        control = dcc.Input(type='text', value=value, **common)
    else:
        control = dcc.Textarea(value=formatted(value), **common)
    return html.Div(
        [
            html.Div(
                [
                    html.Label(
                        readable(key),
                        htmlFor=json.dumps(
                            {'type': 'param', **field}, sort_keys=True, separators=(',', ':')
                        ),
                    ),
                    html.Small(row['unit'], title=f'{kind} · {_bounds(schema)}'),
                ],
                className='parameter-label',
            ),
            html.Div(
                [
                    dcc.Checklist(
                        id={'type': 'override', **field},
                        options=[
                            {
                                'label': 'Override',
                                'value': 'override',
                                'disabled': not row['editable'],
                            }
                        ],
                        value=['override'] if row['explicit'] else [],
                        className='override-switch',
                    ),
                    control,
                ],
                className='parameter-editor',
            ),
            html.Details(
                [
                    html.Summary('Default & bounds'),
                    html.Span(_bounds(schema) + ' · Owner default: '),
                    html.Code(
                        formatted(row['default']) if row['has_default'] else 'not declared'
                    ),
                    html.Span(' · ' + row['source'], className='source-label'),
                ],
                className='parameter-note',
            ),
            (
                html.Details(
                    [html.Summary('Schema details'), html.Pre(formatted(schema))],
                    className='schema-details',
                )
                if kind in ('array', 'object')
                else None
            ),
        ],
        className='parameter-row',
    )


def component_cards(backend, state):
    """Build component controls exclusively from public Author and owner metadata."""
    result = []
    ex = state['spec']['experiment']
    # Roles come from the Author's live discovery. There is no UI registry.
    for role in backend.author.discover()['components']:
        detail = state['components'].get(role)
        if detail is None:
            continue
        pin = ex.get('dynamics_source') if role == 'target_dynamics' else None
        inherited_detail = state['inherited_components'][role]
        inherited = inherited_detail['id']
        # Preview metadata can precede unsaved form edits. Redrawing a different
        # component must not reset these switches/values to that older snapshot.
        detail = deepcopy(detail)
        local = ex.get('parameters', {}).get(role, {})
        for key in detail.get('overrides', {}):
            if key not in local:
                if detail['id'] == inherited and key in inherited_detail['effective']:
                    detail['effective'][key] = deepcopy(inherited_detail['effective'][key])
                detail.setdefault('parameter_sources', {})[key] = {
                    'kind': 'resolved_owner_or_inherited'
                }
        detail['effective'].update(deepcopy(local))
        detail['overrides'] = deepcopy(local)
        for key in local:
            detail.setdefault('parameter_sources', {})[key] = {'kind': 'experiment_override'}
        selection = (
            f"library:{pin['library_id']}:{pin['revision']}"
            if pin
            else ex.get('components', {}).get(role, INHERIT)
        )
        options = backend.list_component_options(role)
        choices = component_choices(role, options, inherited, selection)
        if role == 'target_dynamics':
            choices = target_options(backend, pin, inherited=inherited, selection=selection)
        rows = [] if pin else [r for r in parameter_rows(detail) if r['key'] != 'influences']
        response = (backend.describe_robot_response(detail['effective']['model'],
                    detail['effective'].get('influences'))
                    if role == 'target_dynamics' and detail['id'] == 'equation_state_space'
                    else {'available': False, 'enabled': False,
                          'reason': 'Requires a compatible equation model with explicit acceleration bindings.'})
        historical = []
        if pin:
            try:
                historical = [
                    {
                        'label': f"Version {r['revision']} · {r['sha256'][:12]}",
                        'value': f"library:{pin['library_id']}:{r['revision']}",
                    }
                    for r in reversed(backend.dynamics.revisions(pin['library_id']))
                ]
            except (ValueError, OSError):
                pass
        result.append(
            html.Div(
                [
                    html.Div(
                        [
                            html.Span(readable(role), className='component-role'),
                        ], className='component-heading'
                    ),
                    html.Div([
                            dcc.Dropdown(
                                id={'type': 'component', 'role': role},
                                options=choices,
                                value=selection,
                                clearable=False,
                            ),
                            html.Div([
                                html.Span('External Influences', className='influence-heading'),
                                dcc.Checklist(
                                    id={'type': 'influence', 'role': 'robot_response'},
                                    options=[{'label': 'Robot Response', 'value': 'enabled',
                                              'disabled': not response['available']}],
                                    value=['enabled'] if response['enabled'] else [],
                                ),
                                dcc.Checklist(
                                    id='environmental-disturbance',
                                    options=[{'label': 'Environmental Disturbance',
                                              'value': 'enabled', 'disabled': True}], value=[],
                                ),
                            ], className='external-influences',
                               title=response.get('reason') or 'Frozen experiment influence; base model unchanged.')
                            if role == 'target_dynamics' else None,
                    ], className='component-selector'),
                    html.Details([html.Summary('Parameters'),
                    html.Div(
                        [
                            (
                                html.Div([
                                    html.Label('External acceleration / environment'),
                                    dcc.Dropdown(
                                        id={'type': 'resource', 'role': 'environment'},
                                        options=[
                                            {'label': 'Inherited environment (unchanged)',
                                             'value': INHERIT},
                                            *([{'label': 'Current explicit resource',
                                                'value': '__current__'}]
                                              if isinstance(ex.get('resources', {}).get(
                                                  'environment'), dict) else []),
                                            *[{'label': o['label'], 'value': o['id']}
                                              for o in backend.list_environment_options()],
                                        ],
                                        value=(
                                            ex.get('resources', {}).get('environment', INHERIT)
                                            if not isinstance(ex.get('resources', {}).get(
                                                'environment'), dict) else '__current__'),
                                        clearable=False,
                                    ),
                                ]) if role == 'target_dynamics' else None
                            ),
                            *(
                                [
                                    html.P(
                                        f"Frozen {pin['library_id']} · version {pin['revision']} · {pin['sha256'][:12]}",
                                        className='hint',
                                    ),
                                    html.Details(
                                        [
                                            html.Summary('Advanced · earlier saved version'),
                                            dcc.Dropdown(
                                                id={'type': 'target-history', 'key': 'revision'},
                                                options=historical,
                                                placeholder='Select an exact saved version',
                                            ),
                                        ]
                                    ),
                                ]
                                if pin
                                else [_parameter(role, row) for row in rows]
                            ),
                            (
                                html.P(
                                    'This component has no editable parameters.',
                                    className='muted',
                                )
                                if not rows
                                else None
                            ),
                        ],
                        className='component-body',
                    )], className='component-parameters'),
                ],
                className='component-card',
                **{'data-role': role},
            )
        )
    primary = {
        'robot_estimator',
        'target_estimator',
        'predictor',
        'adaptation',
        'fusion',
        'controller',
        'target_dynamics',
    }
    visible, secondary = [], []
    for card in result:
        (visible if card.to_plotly_json()['props']['data-role'] in primary else secondary).append(
            card
        )
    if secondary:
        visible.append(
            html.Details(
                [html.Summary('Other model settings'), *secondary], className='other-models'
            )
        )
    return visible


def execution_fields(state=None, backend=None):
    """Render execution controls from the Author-resolved study, with no new defaults."""
    study = ((state or {}).get('preview') or {}).get('effective', {}).get('experiment', {})
    return [
        html.Label(
            [
                'Duration (simulation s)',
                dcc.Input(
                    id='execution-duration',
                    type='number',
                    step='any',
                    value=study.get('duration'),
                    disabled=not state,
                    placeholder='Load a draft',
                ),
            ]
        ),
        html.Label(
            [
                'Seeds',
                dcc.Input(
                    id='execution-seeds',
                    type='text',
                    value=json.dumps(study['seeds']) if 'seeds' in study else None,
                    disabled=not state,
                    placeholder='Load a draft',
                ),
            ]
        ),

    ]


def create_app(backend):
    """Create a UI with browser-local state and explicit calls to existing services."""
    app = Dash(
        __name__,
        assets_folder=str(Path(__file__).parent / 'assets'),
        title='Adaptive Tracking · Experiment Lab',
        suppress_callback_exceptions=True,
        update_title='Working…',
    )

    def layout():
        options = experiment_options(backend)
        execution = execution_layout(backend)
        return html.Div(
            [
                dcc.Store(id='builder-state'),
                dcc.Store(id='editor-busy', data=False),
                dcc.Store(id='selected-snapshot'),
                *execution['state'],
                *management_state(),
                html.Header(
                    [
                        html.Div(
                            [
                                html.Div('AT', className='brand-icon'),
                                html.Div(
                                    [
                                        html.Strong('ADAPTIVE TRACKING'),
                                        html.Small('Scientific experiment lab'),
                                    ]
                                ),
                            ],
                            className='brand',
                        ),
                        html.Div('Mobile Manipulator', id='run-context'),
                        execution['session_controls'],
                    ],
                    className='topbar',
                ),
                dcc.RadioItems(
                    id='workspace-tab',
                    options=[
                        {'label': 'Experiment Lab', 'value': 'lab'},
                        {'label': 'Dynamics Studio', 'value': 'dynamics'},
                        {'label': 'Results & History', 'value': 'results'},
                    ],
                    value='lab',
                    className='primary-tabs',
                    persistence=True,
                    persistence_type='session',
                ),
                html.Main(
                    [
                        html.Div(
                            [
                                html.Aside(
                                    [
                                        html.Div('EXPERIMENT LAB', className='eyebrow'),
                                        html.H2('Scenario'),
                                        html.Label(
                                            'Saved scenario', htmlFor='source-experiment'
                                        ),
                                        html.Div(
                                            [
                                                dcc.Dropdown(
                                                    id='source-experiment',
                                                    options=options,
                                                    value=(
                                                        options[0]['value']
                                                        if options
                                                        else None
                                                    ),
                                                    clearable=False,
                                                ),
                                                scenario_menu(),
                                            ],
                                            className='scenario-selector',
                                        ),
                                        html.Div(id='source-summary', className='hint'),
                                        dcc.RadioItems(
                                            id='draft-mode',
                                            options=[
                                                {
                                                    'label': 'Revise selected',
                                                    'value': 'revise',
                                                },
                                                {'label': 'Create new', 'value': 'create'},
                                            ],
                                            value='revise',
                                            className='mode-switch', style={'display': 'none'},
                                        ),
                                        html.Div(
                                            [
                                                html.Label('Display name', htmlFor='new-name'),
                                                dcc.Input(
                                                    id='new-name',
                                                    className='identity-input',
                                                    placeholder='My tracking study',
                                                    type='text',
                                                ),
                                                html.Div(
                                                    id='identity-feedback',
                                                    className='hint',
                                                    role='status',
                                                ),
                                                html.Details(
                                                    [
                                                        html.Summary('Advanced · identifier'),
                                                        dcc.Checklist(
                                                            id='custom-id',
                                                            options=[
                                                                {
                                                                    'label': 'Custom identifier',
                                                                    'value': 'custom',
                                                                }
                                                            ],
                                                            value=[],
                                                        ),
                                                        dcc.Input(
                                                            id='new-id',
                                                            className='identity-input',
                                                            type='text',
                                                            disabled=True,
                                                        ),
                                                        html.Small(
                                                            'Applied when loading a new draft.'
                                                        ),
                                                    ],
                                                    className='identity-advanced',
                                                ),
                                            ],
                                            id='create-fields',
                                            style={'display': 'none'},
                                        ),
                                        html.Button(
                                            'Load configuration',
                                            id='load-draft',
                                            className='button primary full-width',
                                        ),
                                        html.Small(
                                            'Loading replaces unsaved edits.', className='hint'
                                        ),
                                        html.Details(
                                            [
                                                html.Summary('Advanced · revision history'),
                                                html.Label(
                                                    'Saved revision', htmlFor='source-revision'
                                                ),
                                                dcc.Dropdown(
                                                    id='source-revision', clearable=False
                                                ),
                                                html.Details(
                                                    [
                                                        html.Summary(
                                                            'Saved configuration evidence'
                                                        ),
                                                        html.Pre(
                                                            id='saved-preview',
                                                            className='json-preview',
                                                        ),
                                                    ]
                                                ),
                                            ],
                                            className='scenario-advanced',
                                        ),
                                    ],
                                    className='panel source-panel',
                                ),
                                html.Section(
                                    [
                                        html.Div(
                                            [
                                                html.Div(
                                                    [
                                                        html.H2('Models & Parameters'),
                                                        html.P(
                                                            id='draft-identity',
                                                            children='Load a draft to begin.',
                                                            className='context-label',
                                                        ),
                                                    ]
                                                ),
                                                html.Span(
                                                    'No draft',
                                                    id='draft-badge',
                                                    className='badge neutral',
                                                ),
                                            ],
                                            className='panel-heading',
                                        ),
                                        html.Div(
                                            id='feedback',
                                            role='status',
                                            **{'aria-live': 'polite'},
                                            children='Choose a scenario, then load it.',
                                            className='notice neutral',
                                        ),
                                        html.Div(execution_fields(backend=backend), id='execution-fields'),
                                        dcc.Loading(
                                            html.Div(id='component-cards'),
                                            type='circle',
                                            parent_className='component-loading',
                                        ),
                                        html.Div(
                                            [
                                                html.Button(
                                                    'Reset',
                                                    id='reset-draft',
                                                    className='button subtle',
                                                    disabled=True,
                                                ),
                                                html.Div(
                                                    [
                                                        html.Button(
                                                            'Preview',
                                                            id='preview-draft',
                                                            className='button',
                                                            disabled=True,
                                                        ),
                                                        html.Button(
                                                            'Validate',
                                                            id='validate-draft',
                                                            className='button',
                                                            disabled=True,
                                                        ),
                                                        html.Button(
                                                            'Save revision',
                                                            id='save-draft',
                                                            className='button primary',
                                                            disabled=True,
                                                        ),
                                                    ],
                                                    className='action-group',
                                                ),
                                            ],
                                            className='actions',
                                        ),
                                    ],
                                    className='panel builder-panel',
                                ),
                                html.Aside(
                                    [
                                        execution['monitor'],
                                    ],
                                    className='lab-rail',
                                ),
                            ],
                            id='experiment-lab',
                            className='workspace-grid',
                        ),
                        studio_layout(backend),
                        html.Div(
                            [
                                execution['results'],
                                execution['session'],
                            ],
                            id='results-history',
                            className='results-workspace',
                        ),
                    ],
                    id='workspace',
                    className='workspace show-lab',
                ),
            ],
            className='app-shell',
        )

    app.layout = layout

    app.clientside_callback(
        """function(tab) {
            requestAnimationFrame(function() { requestAnimationFrame(function() {
                window.dispatchEvent(new Event('resize'));
            }); });
            return 'workspace show-' + tab;
        }""",
        Output('workspace', 'className'),
        Input('workspace-tab', 'value'),
    )

    @app.callback(
        Output('source-revision', 'options'),
        Output('source-revision', 'value'),
        Input('source-experiment', 'value'),
        Input('source-experiment', 'options'),
    )
    def revisions(identifier, _options):
        if not identifier:
            return [], None
        try:
            backend.ensure_scenario(identifier)
            records = backend.list_revisions(identifier)
            return (
                [
                    {'label': f"r{r['revision']} · {r['created_utc']}", 'value': r['revision']}
                    for r in reversed(records)
                ],
                records[-1]['revision'],
            )
        except (ValueError, KeyError, OSError):
            return [], None

    @app.callback(
        Output('source-summary', 'children'),
        Output('saved-preview', 'children'),
        Output('selected-snapshot', 'data'),
        Input('source-experiment', 'value'),
        Input('source-revision', 'value'),
    )
    def inspect_saved(identifier, revision):
        if not identifier or revision is None:
            return '', '', None
        try:
            effective = backend.explain_experiment(identifier, revision)
            return (
                f'Saved revision {revision}',
                json.dumps(effective, indent=2, ensure_ascii=False),
                {
                    'effective': effective,
                    'identity': next(
                        (
                            r['name']
                            for r in backend.list_experiments()
                            if r['id'] == identifier
                        ),
                        'Saved scenario',
                    ),
                    'experiment_id': identifier,
                    'revision': revision,
                },
            )
        except (ValueError, TypeError, KeyError, OSError) as error:
            return f'Cannot inspect revision: {error}', '', None

    @app.callback(Output('create-fields', 'style'), Input('draft-mode', 'value'))
    def creation_fields(mode):
        return {'display': 'block' if mode == 'create' else 'none'}

    app.clientside_callback(
        "function(n) { return n ? 'Saving…' : window.dash_clientside.no_update; }",
        Output('save-draft', 'children', allow_duplicate=True),
        Input('save-draft', 'n_clicks'),
        prevent_initial_call=True,
    )

    app.clientside_callback(
        "function(v) { return !(v || []).includes('override'); }",
        Output({'type': 'param', 'role': MATCH, 'key': MATCH, 'kind': MATCH}, 'disabled'),
        Input({'type': 'override', 'role': MATCH, 'key': MATCH, 'kind': MATCH}, 'value'),
    )

    @app.callback(
        Output('builder-state', 'data'),
        Output('component-cards', 'children'),
        Output('source-experiment', 'options'),
        Output('execution-fields', 'children'),
        Output('source-experiment', 'value', allow_duplicate=True),
        Output('draft-mode', 'value', allow_duplicate=True),
        Input('load-draft', 'n_clicks'),
        Input('reset-draft', 'n_clicks'),
        Input('preview-draft', 'n_clicks'),
        Input('validate-draft', 'n_clicks'),
        Input('save-draft', 'n_clicks'),
        Input({'type': 'component', 'role': ALL}, 'value'),
        Input({'type': 'param', 'role': ALL, 'key': ALL, 'kind': ALL}, 'value'),
        Input({'type': 'override', 'role': ALL, 'key': ALL, 'kind': ALL}, 'value'),
        Input('execution-duration', 'value'),
        Input('execution-seeds', 'value'),
        Input({'type':'target-history','key':ALL}, 'value'),
        Input({'type': 'resource', 'role': 'environment'}, 'value', allow_optional=True),
        Input({'type': 'influence', 'role': ALL}, 'value'),
        State({'type': 'component', 'role': ALL}, 'id'),
        State({'type': 'param', 'role': ALL, 'key': ALL, 'kind': ALL}, 'id'),
        State('source-experiment', 'value'),
        State('source-revision', 'value'),
        State('draft-mode', 'value'),
        State('new-id', 'value'),
        State('new-name', 'value'),
        State('builder-state', 'data'),
        prevent_initial_call=True,
        running=[(Output('editor-busy', 'data'), True, False)],
    )
    def operate(
        _load,
        _reset,
        _preview,
        _validate,
        _save,
        selections,
        values,
        switches,
        duration,
        seeds,
        historical_selection,
        environment_selection,
        influences,
        component_ids,
        fields,
        source,
        revision,
        mode,
        identifier,
        name,
        state,
    ):
        trigger = ctx.triggered_id
        cards, choices = no_update, no_update
        state = deepcopy(state) if state else None
        save_started = False
        try:
            if trigger == 'load-draft':
                backend.ensure_scenario(source)
                record = backend.catalog.get(source, revision)
                if record['definition']['schema_version'] != 1:
                    return ({'stage': 'Read-only', 'message':
                             'This saved sensor-assignment Scenario is ready for Play. '
                             'Edit its legacy assignments with the experiment CLI.'},
                            [], no_update, execution_fields(), no_update, no_update)
                variants = record['definition']['experiment']['variants']
                state = begin_draft(backend, source, revision, variants[0]['name'], mode, identifier, name)
                state['inherited_components'] = deepcopy(state['components'])
                state = hydrate_target_pin(state, backend)
                if 'dynamics_source' in state['spec']['experiment']:
                    state['dynamics_model_text'] = json.dumps(
                        state['spec']['experiment']['parameters']['target_dynamics']['model'],
                        allow_nan=False,
                    )
                return (state, component_cards(backend, state), no_update, execution_fields(state, backend), no_update, no_update)
            if not state or 'spec' not in state:
                raise PreventUpdate
            if trigger in ('validate-draft', 'preview-draft', 'save-draft'):
                intended = identifier if mode == 'create' else source
                if state['spec']['experiment']['id'] != intended:
                    raise ValueError('Load the selected Scenario before validating or saving.')
            if trigger == 'reset-draft':
                state['spec'] = backend.reset_draft(
                    restore_library_model(state['spec'], state.get('dynamics_model_text'))
                )
                if 'dynamics_source' in state['spec']['experiment']:
                    state['dynamics_model_text'] = json.dumps(
                        state['spec']['experiment']['parameters']['target_dynamics']['model'],
                        allow_nan=False,
                    )
                state['preview'] = backend.preview_experiment(state['spec'])
                state['components'] = state['preview']['components']
                state = hydrate_target_pin(state, backend)
                state.update(
                    stage='Draft',
                    validated_digest=None,
                    message='Unsaved selections and overrides reset to the pinned base.',
                )
                state['form_digest'] = fingerprint(state['spec'])
                return (state, component_cards(backend, state), no_update, execution_fields(state, backend), no_update, no_update)
            spec = assemble_draft(
                state['spec'],
                [v for v,i in zip(selections,component_ids) if i['role']!='target_dynamics'],
                [i for i in component_ids if i['role']!='target_dynamics'], values, fields, switches
            )
            spec = restore_library_model(spec, state.get('dynamics_model_text'))
            spec = apply_execution(spec, duration, seeds)
            if isinstance(trigger, dict) and trigger.get('type') == 'influence':
                if len(influences) != 1 or influences[0] not in ([], ['enabled']):
                    raise ValueError('Robot Response requires explicit checkbox opt-in')
                enabled = influences[0] == ['enabled']
                previous = spec['experiment'].get('parameters', {}).get('target_dynamics', {}).get('influences', [])
                if enabled == bool(previous and previous[0]['enabled']):
                    raise PreventUpdate
                spec = backend.draft_with_robot_response(spec, enabled)
                checked = backend.validate_experiment(spec)
                if not checked['valid']:
                    raise ValueError('Cannot deploy: ' + '\n'.join(checked['errors']))
                preview = backend.preview_experiment(spec)
                state.update(spec=spec, preview=preview, components=preview['components'])
                state = hydrate_target_pin(state, backend)
                state.update(stage='Draft',
                             validated_digest=None, form_digest=fingerprint(state['spec']),
                             message='Unsaved influence change · validate, then save.')
                return (state, component_cards(backend, state), no_update, no_update, no_update, no_update)
            if isinstance(trigger, dict) and (trigger.get('type') in ('target-history','resource') or
                    (trigger.get('type') == 'component' and trigger['role'] == 'target_dynamics')):
                if trigger['type'] == 'resource':
                    previous = spec['experiment'].get('resources', {}).get('environment', INHERIT)
                    if (environment_selection in (None, '__current__')
                            or environment_selection == previous):
                        raise PreventUpdate
                    if environment_selection == INHERIT:
                        spec['experiment'].get('resources', {}).pop('environment', None)
                    else:
                        if environment_selection not in {
                                o['id'] for o in backend.list_environment_options()}:
                            raise ValueError('Select a registered environment resource')
                        spec['experiment'].setdefault('resources', {})[
                            'environment'] = environment_selection
                    candidate = {**state, 'spec': spec}
                else:
                    selected = (next((v for v in historical_selection if v), None)
                                if trigger['type'] == 'target-history' else
                                next((v for v,i in zip(selections,component_ids) if i['role']=='target_dynamics'), None))
                    current = spec['experiment'].get('dynamics_source')
                    previous = (f"library:{current['library_id']}:{current['revision']}" if current else
                                spec['experiment'].get('components',{}).get('target_dynamics',INHERIT))
                    if not selected or selected == previous: raise PreventUpdate
                    candidate = choose_target_model(backend, {**state, 'spec':spec}, selected)
                spec = candidate['spec']
                state = candidate
                checked = backend.validate_experiment(spec)
                if checked['valid']:
                    state['preview'] = backend.preview_experiment(spec)
                    state['components'] = state['preview']['components']
                    state = hydrate_target_pin(state, backend)
                    spec = state['spec']
                else:
                    state['preview'] = None
                    if 'dynamics_source' in spec['experiment']:
                        state['components']['target_dynamics'] = {
                            **state['components']['target_dynamics'], 'id':'equation_state_space',
                            'effective':spec['experiment']['parameters']['target_dynamics']}
                state.update(spec=spec,stage='Draft' if checked['valid'] else 'Invalid',
                    validated_digest=None, form_digest=fingerprint(spec),
                    message=('Selection validated; exact model pin retained. Validate the complete experiment before saving.'
                             if checked['valid'] else 'Selected configuration cannot deploy: '+ '\n'.join(checked['errors'])))
                return (state, component_cards(backend, state), no_update, no_update, no_update, no_update)
            digest = fingerprint(spec)
            if isinstance(trigger, dict) or trigger in (
                'execution-duration',
                'execution-seeds',
            ):
                if digest == state['form_digest']:
                    raise PreventUpdate
                state.update(
                    spec=spec,
                    stage='Draft',
                    validated_digest=None,
                    form_digest=digest,
                    message='Unsaved changes · validate, then save.',
                )
                if isinstance(trigger, dict) and trigger['type'] == 'component':
                    role = trigger['role']
                    selected = spec['experiment'].get('components', {}).get(role, INHERIT)
                    if selected == INHERIT:
                        state['components'][role] = deepcopy(
                            state['inherited_components'][role]
                        )
                    else:
                        metadata = next(
                            o
                            for o in backend.list_component_options(role)
                            if o['id'] == selected
                        )
                        state['components'][role] = {
                            **metadata,
                            'effective': metadata['defaults'],
                            'overrides': {},
                            'pending': True,
                            'selection_source': {'kind': 'experiment_selection'},
                        }
                    cards = component_cards(backend, state)
                return (state, cards, no_update, no_update, no_update, no_update)
            state['spec'], state['form_digest'] = spec, digest
            if trigger == 'save-draft':
                if state['stage'] != 'Validated' or state['validated_digest'] != digest:
                    raise ValueError(
                        'Validate the current form before saving; edits invalidate approval.'
                    )
                save_started = True
                record = backend.save_experiment(
                    spec, expected_revision=state['expected_revision']
                )
                state.update(
                    stage='Saved',
                    expected_revision=record['revision'],
                    saved_revision=record['revision'],
                    validated_digest=None,
                    message='Saved',
                )
                choices = experiment_options(backend)
            elif trigger in ('preview-draft', 'validate-draft'):
                if trigger == 'validate-draft':
                    checked = backend.validate_experiment(spec)
                    if not checked['valid']:
                        raise ValueError('\n'.join(checked['errors']))
                state['preview'] = backend.preview_experiment(spec)
                state['components'] = state['preview']['components']
                state.update(
                    stage='Validated' if trigger == 'validate-draft' else 'Previewed',
                    validated_digest=digest if trigger == 'validate-draft' else None,
                    message=(
                        'Configuration preflight passed. Ready to save.'
                        if trigger == 'validate-draft'
                        else 'Effective preview refreshed. Validate to enable saving.'
                    ),
                )
                cards = component_cards(backend, state)
            return (state, cards, choices, no_update, spec['experiment']['id'] if trigger == 'save-draft' and state.get('stage') == 'Saved' else no_update, 'revise' if trigger == 'save-draft' and state.get('stage') == 'Saved' else no_update)
        except PreventUpdate:
            raise
        except (ValueError, TypeError, KeyError, OSError) as error:
            state = state or {}
            state.update(stage='Save failed' if save_started else 'Invalid',
                         validated_digest=None, message=str(error))
            cards = (component_cards(backend, state)
                     if isinstance(trigger, dict) and trigger.get('type') == 'influence'
                     else no_update)
            return (state, cards, no_update, no_update, no_update, no_update)
        except Exception:
            LOG.exception('Dashboard action failed')
            state = state or {}
            state.update(
                stage='Save failed' if save_started else 'Invalid',
                validated_digest=None,
                message='The action could not complete. Your draft was not saved. '
                'Check the dashboard terminal for details, then retry explicitly.',
            )
            cards = (component_cards(backend, state)
                     if isinstance(trigger, dict) and trigger.get('type') == 'influence'
                     else no_update)
            return (state, cards, no_update, no_update, no_update, no_update)

    @app.callback(
        Output('draft-identity', 'children'),
        Output('draft-badge', 'children'),
        Output('draft-badge', 'className'),
        Output('feedback', 'children'),
        Output('feedback', 'className'),
        Output('reset-draft', 'disabled'),
        Output('preview-draft', 'disabled'),
        Output('validate-draft', 'disabled'),
        Output('save-draft', 'disabled'),
        Output('save-draft', 'children'),
        Input('builder-state', 'data'),
        Input('selected-snapshot', 'data'),
        Input('editor-busy', 'data'),
    )
    def present(state, selected, busy):
        state = state or {}
        spec, stage = state.get('spec'), state.get('stage', 'No draft')
        identity = (
            spec['experiment']['name']
            if spec
            else (selected or {}).get('identity', 'Load a configuration to begin.')
        )
        tone = (
            'success'
            if stage in ('Validated', 'Saved')
            else 'error' if stage in ('Invalid', 'Save failed') else 'neutral'
        )
        feedback = {
            'Draft': 'Unsaved configuration · validate, then save.',
            'Previewed': 'Preview ready · validate before saving.',
            'Validated': 'Valid configuration · ready to save.',
            'Saved': 'Saved',
        }.get(stage, state.get('message', 'Choose a scenario, then load it.'))
        return (
            identity,
            {'Draft': 'Unsaved', 'Validated': 'Valid'}.get(stage, stage),
            'badge ' + tone,
            feedback,
            'notice ' + tone,
            not spec or bool(busy),
            not spec or bool(busy),
            not spec or bool(busy),
            stage != 'Validated' or bool(busy),
            no_update if busy else (
                'Create experiment' if state.get('expected_revision') is None else 'Save revision'
            ),

        )

    register_studio(app, backend)
    register_execution(app, backend)
    register_management(app, backend)
    register_trajectory(app, backend)
    return app
