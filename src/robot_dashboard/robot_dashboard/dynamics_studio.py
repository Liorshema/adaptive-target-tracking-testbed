"""Structured Dynamics Studio presentation over the existing mathematical backend."""

import json
from copy import deepcopy
from uuid import uuid4

from dash import ALL, Input, MATCH, Output, State, ctx, dash_table, dcc, html, no_update
from dash.exceptions import PreventUpdate

import plotly.graph_objects as go

from .dynamics_forms import (
    BOUNDS,
    add_interaction,
    editor_data,
    load_model,
    numeric,
    preview_inputs,
    read_editor,
    save_model,
    target_options,
)
from .mathematics import model_tex, velocity_tex


def library_options(backend):
    """Display current labels without selecting a mutable latest mathematical revision."""
    try:
        return [{'label': r['name'], 'value': r['id']} for r in backend.dynamics.list()]
    except (ValueError, OSError) as error:
        return [
            {
                'label': 'Library unavailable: ' + str(error),
                'value': '__invalid__',
                'disabled': True,
            }
        ]


def table(key, columns, rows, *, removable=True, hidden=()):
    """Compact editable rows; cells contain quantities/expressions, never YAML documents."""
    return html.Div(
        [
            dash_table.DataTable(
                id={'type': 'dyn-table', 'key': key},
                data=rows,
                columns=[
                    {'name': label, 'id': name, 'editable': editable}
                    for name, label, editable in columns
                ],
                editable=True,
                hidden_columns=list(hidden),
                row_deletable=removable,
                style_table={'overflowX': 'auto'},
                style_cell={
                    'backgroundColor': '#0a1926',
                    'color': '#edf4fa',
                    'border': '1px solid #213c50',
                    'fontFamily': 'inherit',
                    'fontSize': 12,
                    'padding': '3px',
                    'lineHeight': '16px',
                    'minWidth': '55px',
                    'maxWidth': '210px',
                    'whiteSpace': 'normal',
                    'textAlign': 'left',
                },
                style_header={'backgroundColor': '#122d40', 'fontWeight': 600, 'color': '#bcd6e9'},
                css=[
                    {'selector': '.show-hide', 'rule': 'display:none'},
                    {
                        'selector': '.dash-spreadsheet input',
                        'rule': 'color:#edf4fa!important;background:#122d40!important;',
                    },
                ],
            ),
            (
                html.Button(
                    {'state': '+ Add state variable', 'parameters': '+ Add parameter'}.get(
                        key, '+ Add row'
                    ),
                    id={'type': 'dyn-add-row', 'key': key},
                    className='button subtle small',
                )
                if removable
                else None
            ),
        ],
        className='dyn-table-wrap',
    )


def field(key, label, value, *, options=None):
    """Render one structured text or finite-choice control."""
    common = {'id': {'type': 'dyn-field', 'key': key}, 'value': value}
    return html.Div(
        [
            html.Label(label),
            (
                dcc.Dropdown(options=options, clearable=False, **common)
                if options
                else dcc.Input(type='text', debounce=True, **common)
            ),
        ],
        className='dyn-field',
    )


def composition_editor(definition, selected=0):
    """Render a detached complete source using the same forms for every model."""
    f, t = editor_data(definition, selected)
    composed = definition['type'] == 'cartesian_composition'
    state_columns = [
        ('name', 'Variable', True),
        ('shape', 'Shape', True),
        ('unit', 'Unit', True),
        ('initial', 'Initial', True),
    ]
    if not composed:
        state_columns.append(('derivative', 'Derivative', True))
    children = [
        html.Div(
            [
                field('model_id', 'Mathematical ID', f['model_id']),
                field('frame', 'Coordinate frame', f['frame']),
            ],
            className='dyn-two',
        ),
        html.H3('Initial state' if composed else 'Ordered states & derivatives'),
        table('state', state_columns, t['state'], removable=not composed),
    ]
    if composed:
        sources = [definition['base'], *definition.get('interactions', [])]
        children += [
            html.Div(
                [
                    html.Div(
                        [
                            html.Label('Edit contribution'),
                            dcc.Dropdown(
                                id='dyn-contribution',
                                clearable=False,
                                options=[
                                    {
                                        'label': ('Base · ' if i == 0 else 'Interaction · ')
                                        + s['id'],
                                        'value': i,
                                    }
                                    for i, s in enumerate(sources)
                                ],
                                value=selected,
                            ),
                        ],
                        className='dyn-contribution-choice',
                    ),
                    html.Button(
                        '↑',
                        id='dyn-up',
                        title='Move interaction earlier',
                        className='button',
                        disabled=selected < 2,
                    ),
                    html.Button(
                        '↓',
                        id='dyn-down',
                        title='Move interaction later',
                        className='button',
                        disabled=selected == 0 or selected == len(sources) - 1,
                    ),
                    html.Button(
                        'Remove',
                        id='dyn-remove',
                        className='button subtle',
                        disabled=selected == 0,
                    ),
                ],
                className='dyn-contribution-bar',
            ),
            html.Div(
                [
                    field('source_id', 'Contribution identity', f['source_id']),
                    field(
                        'enabled',
                        'Include acceleration',
                        f['enabled'],
                        options=[
                            {'label': 'Enabled', 'value': True},
                            {'label': 'Disabled', 'value': False},
                        ],
                    ),
                ],
                className='dyn-two',
            ),
            field('expression', 'Acceleration expression', f['expression']),
            html.Details(
                [
                    html.Summary('Output & frame'),
                    html.Div(
                        [
                            field(
                                'contribution_frame', 'Contribution frame', f['contribution_frame']
                            ),
                            field('output_shape', 'Output shape', f['output_shape']),
                            field('output_unit', 'Output unit', f['output_unit']),
                        ],
                        className='dyn-three',
                    ),
                ]
            ),
        ]
    children += [
        html.H3('Parameters · values & physical units'),
        table(
            'parameters',
            [
                ('name', 'Name', True),
                ('shape', 'Shape', True),
                ('value', 'Value', True),
                ('unit', 'Unit', True),
            ]
            + [(b, label, True) for b, label in zip(BOUNDS, ['≥', '≤', '>', '<'])],
            t['parameters'],
        ),
        (
            html.P(
                (
                    'Drag ownership is explicit. Added interactions use the visible '
                    'damping value below; base drag is unchanged.'
                ),
                className='hint',
            )
            if composed
            else None
        ),
        html.Details(
            [
                html.Summary('Intermediate expressions'),
                table(
                    'intermediates',
                    [('name', 'Name', True), ('expression', 'Expression', True)],
                    t['intermediates'],
                ),
            ]
        ),
        html.Details(
            [
                html.Summary('Declared context' + (' & control inputs' if not composed else '')),
                html.P(
                    (
                        'Declarations are model inputs. Preview assumptions are entered '
                        'separately; live deployment bindings remain owner-validated.'
                    ),
                    className='hint',
                ),
                table(
                    'context',
                    [
                        ('name', 'Name', True),
                        ('shape', 'Shape', True),
                        ('unit', 'Unit', True),
                        ('frame', 'Frame', True),
                    ],
                    t['context'],
                ),
                (
                    table(
                        'control',
                        [
                            ('name', 'Name', True),
                            ('shape', 'Shape', True),
                            ('unit', 'Unit', True),
                            ('frame', 'Frame', True),
                        ],
                        t['control'],
                    )
                    if not composed
                    else None
                ),
            ]
        ),
    ]
    if not composed:
        children += [
            html.Details(
                [
                    html.Summary('Deployment adapter declaration'),
                    field(
                        'deployment',
                        'Existing adapter',
                        f['deployment'],
                        options=[
                            {'label': 'Numerical only', 'value': 'none'},
                            {
                                'label': 'Existing six-state Cartesian target',
                                'value': 'cartesian_target',
                            },
                        ],
                    ),
                    html.Div(
                        [
                            field('position', 'Position variable', f['position']),
                            field('velocity', 'Velocity variable', f['velocity']),
                        ],
                        className='dyn-two',
                    ),
                    html.P(
                        (
                            'Declaring an adapter does not bypass shape, context, frame or full '
                            'Experiment preflight.'
                        ),
                        className='hint',
                    ),
                ]
            )
        ]
    if not composed:
        children += [
            html.Div(
                [
                    dcc.Dropdown(
                        id='dyn-contribution',
                        options=[{'label': 'General model', 'value': 0}],
                        value=0,
                    ),
                    *[html.Button(id=k) for k in ('dyn-remove', 'dyn-up', 'dyn-down')],
                ],
                style={'display': 'none'},
            )
        ]
    children += [
        html.P(
            (
                'Syntax: sin, cos, tanh, sqrt, norm, dot, cross, vectors [a,b,c], '
                'arithmetic. D3 validates units and restricted expressions.'
            ),
            className='hint',
        )
    ]
    return children


def empty_figure(message='Validate and preview a motion model.'):
    """Show an honest empty preview without invented samples."""
    fig = go.Figure()
    fig.update_layout(
        template='plotly_dark',
        paper_bgcolor='#0b1e2d',
        plot_bgcolor='#0b1e2d',
        margin={'l': 35, 'r': 20, 't': 25, 'b': 35},
        font={'color': '#bcd1df', 'size': 12},
    )
    fig.add_annotation(text=message, x=0.5, y=0.5, xref='paper', yref='paper', showarrow=False)
    return fig


def preview_figure(result, view='3d'):
    """Render genuine numerical samples with physical aspect ratio or named state traces."""
    if not result:
        return empty_figure()
    fig = empty_figure('')
    fig.layout.annotations = []
    samples = result['states']
    p = result['position']
    positions = result.get('positions')
    if p or positions:
        x, y, z = (
            list(zip(*positions))
            if positions
            else [[row[i] for row in samples] for i in p['indices']]
        )
        if view == '2d':
            fig.add_trace(
                go.Scatter(
                    x=x, y=y, mode='lines', name='Model position', line={'color': '#4de4a0'}
                )
            )
            fig.update_xaxes(title='x [m]')
            fig.update_yaxes(title='y [m]', scaleanchor='x', scaleratio=1)
        else:
            fig.add_trace(
                go.Scatter3d(
                    x=x,
                    y=y,
                    z=z,
                    mode='lines',
                    name='Model position',
                    line={'color': '#4de4a0', 'width': 4},
                )
            )
            fig.update_layout(
                scene={
                    'aspectmode': 'data',
                    'xaxis_title': 'x [m]',
                    'yaxis_title': 'y [m]',
                    'zaxis_title': 'z [m]',
                    'bgcolor': '#0b1e2d',
                }
            )
    else:
        for state in result['state_series']:
            for j, i in enumerate(state['indices']):
                label = state['name'] + (f'[{j}]' if len(state['indices']) > 1 else '')
                fig.add_trace(
                    go.Scatter(
                        x=result['times'],
                        y=[s[i] for s in samples],
                        mode='lines',
                        name=f"{label} [{state['unit']}]",
                    )
                )
        fig.update_xaxes(title='Preview time [s]')
        fig.update_yaxes(title='State values · units in legend')
    fig.update_layout(uirevision='studio-preview', legend={'orientation': 'h'})
    return fig


def section(title, children, className=''):
    """One scientific panel in the existing dashboard theme."""
    return html.Section([html.H2(title), *children], className='panel studio-panel ' + className)


def editor(definition, selected=0):
    """Render generic tables and preserve editable original contribution sources."""
    f, t = editor_data(definition, selected)
    composed = definition['type'] == 'cartesian_composition'
    state_cols = [
        ('name', 'State', True),
        ('initial', 'Initial Value', True),
        ('derivative', 'Derivative', True),
        ('shape', 'Shape', True),
        ('unit', 'Unit', True),
    ]
    parameter_cols = [
        ('name', 'Symbol', True),
        ('value', 'Value', True),
        ('unit', 'Unit', True),
        ('shape', 'Shape', True),
    ] + [(b, b, True) for b in BOUNDS]
    if composed:
        states = html.P(
            'Composed Cartesian system. Edit original contributions in Advanced; '
            'the complete compiled system is typeset alongside.',
            className='hint',
        )
        parameters = html.P(
            'Contribution parameters and constraints are preserved in Advanced.', className='hint'
        )
        advanced = composition_editor(definition, selected)
    else:
        states = table('state', state_cols, t['state'], hidden=['shape', 'unit'])
        parameters = table(
            'parameters', parameter_cols, t['parameters'], hidden=['shape', *BOUNDS]
        )
        advanced = [
            html.P(
                'Units and shapes are explicit; variable names have no physical semantics.',
                className='hint',
            ),
            dcc.Checklist(
                id='dyn-units',
                options=[
                    {'label': 'Show state units, shapes and parameter bounds', 'value': 'show'}
                ],
                value=[],
            ),
            html.Div(
                [
                    field('model_id', 'Mathematical identity', f['model_id']),
                    field('frame', 'Model frame', f['frame']),
                ],
                className='dyn-two',
            ),
            html.H3('Intermediate expressions'),
            table(
                'intermediates',
                [('name', 'Name', True), ('expression', 'Expression', True)],
                t['intermediates'],
            ),
            html.H3('Declared context · fixed preview inputs must be explicit'),
            table(
                'context',
                [(k, k.title(), True) for k in ('name', 'shape', 'unit', 'frame')],
                t['context'],
            ),
            html.H3('Declared controls'),
            table(
                'control',
                [(k, k.title(), True) for k in ('name', 'shape', 'unit', 'frame')],
                t['control'],
            ),
            field(
                'deployment',
                'Live adapter declaration',
                f['deployment'],
                options=[
                    {'label': 'Numerical preview only', 'value': 'none'},
                    {'label': 'Existing six-state Cartesian target', 'value': 'cartesian_target'},
                ],
            ),
            html.Div(
                [
                    field('position', 'Position state', f['position']),
                    field('velocity', 'Velocity state', f['velocity']),
                ],
                className='dyn-two',
            ),
            html.Div(
                [
                    dcc.Dropdown(
                        id='dyn-contribution',
                        options=[{'label': 'General model', 'value': 0}],
                        value=0,
                    ),
                    *[html.Button(id=k) for k in ('dyn-remove', 'dyn-up', 'dyn-down')],
                ],
                style={'display': 'none'},
            ),
        ]
    return [
        section('State-Space Equations', [states], 'studio-equations'),
        section('Parameters', [parameters], 'studio-parameters'),
        html.Details(
            [
                html.Summary('Physical Output Mapping'),
                field('output_position', 'World Position (x, y, z)', f['output_position']),
                html.Div(
                    [
                        html.Label('Linear Velocity'),
                        html.Div(
                            'Validate or preview to derive velocity.', id='dyn-output-velocity'
                        ),
                    ],
                    className='studio-output-row',
                ),
                html.Small(
                    'Explicit world mapping · velocity derived from the full ODE. '
                    'No implied orientation.',
                    className='hint',
                ),
            ],
            className='panel studio-output',
            open=False,
        ),
        html.Details(
            [
                html.Summary('Advanced · units, inputs, composition & deployment'),
                html.Div(
                    [*advanced, field('output_frame', 'Output frame', f['output_frame'])],
                    className='studio-advanced-body',
                ),
            ],
            className='studio-advanced',
        ),
    ]


def new_state(backend, template='blank_state_space'):
    """Generate stable technical identities; display names never become identifiers."""
    token = uuid4().hex
    definition = deepcopy(backend.dynamics.templates()[template]['definition'])
    definition['id'] = 'model_' + token
    return {
        'definition': definition,
        'id': None,
        'new_id': 'DYN-' + token.upper(),
        'name': 'Untitled model',
        'expected_revision': None,
        'dirty': False,
        'selected': 0,
        'stage': 'Draft',
        'preview': None,
        'checked': None,
    }


def studio_layout(backend):
    """Reference-inspired toolbar and two columns, retaining the application shell."""
    templates = backend.dynamics.templates()
    state = new_state(backend)
    influences = [
        {'label': v['name'], 'value': k}
        for k, v in templates.items()
        if v['definition']['type'] == 'cartesian_composition'
    ] + [{'label': 'Custom expression', 'value': 'custom'}]
    return html.Div(
        [
            dcc.Store(id='dyn-state', data=state),
            dcc.Store(id='dyn-library-events', data=0),
            html.Div(
                html.Section(
                    [
                        html.H2('Discard unsaved dynamics edits?', id='dyn-switch-heading'),
                        html.P(
                            'This replaces the detached draft. Saved versions remain unchanged.'
                        ),
                        html.Div(
                            [
                                html.Button(
                                    'Keep editing', id='dyn-switch-cancel', className='button'
                                ),
                                html.Button(
                                    'Discard edits & open',
                                    id='dyn-switch-confirm',
                                    className='button primary',
                                ),
                            ],
                            className='management-dialog-actions',
                        ),
                    ],
                    className='management-dialog',
                    role='dialog',
                    **{'aria-modal': 'true', 'aria-labelledby': 'dyn-switch-heading'},
                ),
                id='dyn-confirm',
                className='management-overlay',
                hidden=True,
            ),
            html.Div(
                [
                    html.H2('Dynamics Studio'),
                    html.Small(
                        'Define a dynamical system · inspect its mathematics · preview motion'
                    ),
                    html.Span('Draft', id='dyn-badge', className='badge'),
                ],
                className='studio-heading',
            ),
            html.Div(
                [
                    html.Div(
                        [
                            html.Label('Model Name'),
                            dcc.Input(id='dyn-name', value=state['name'], debounce=True),
                        ],
                        className='studio-name',
                    ),
                    html.Div(
                        [
                            html.Label('Saved Models'),
                            dcc.Dropdown(
                                id='dyn-library',
                                options=library_options(backend),
                                clearable=False,
                                placeholder='Choose a saved model',
                            ),
                        ],
                        className='studio-saved',
                    ),
                    html.Button('+ New Model', id='dyn-new', className='button'),
                    html.Button('▷ Preview', id='dyn-preview-button', className='button'),
                    html.Button('✓ Validate & Save', id='dyn-save', className='button primary'),
                ],
                className='panel studio-toolbar',
            ),
            html.Div(
                '',
                id='dyn-feedback',
                className='notice neutral',
                role='status',
            ),
            html.Div(
                [
                    html.Div(
                        editor(state['definition']), id='dyn-editor', className='studio-left'
                    ),
                    html.Div(
                        [
                            section(
                                'Mathematical Representation',
                                [
                                    dcc.Markdown(
                                        id='dyn-math',
                                        mathjax=True,
                                        dangerously_allow_html=False,
                                        className='studio-math',
                                    ),
                                    html.Small(id='dyn-capabilities', className='hint'),
                                ],
                                'studio-math-panel',
                            ),
                            section(
                                'Motion Preview',
                                [
                                    html.Div(
                                        [
                                            dcc.RadioItems(
                                                id='dyn-preview-view',
                                                options=[
                                                    {'label': '3D', 'value': '3d'},
                                                    {'label': '2D XY', 'value': '2d'},
                                                ],
                                                value='3d',
                                                inline=True,
                                            ),
                                            html.Details(
                                                [
                                                    html.Summary('Preview Settings'),
                                                    html.Div(
                                                        [
                                                            html.Button('Close Preview Settings', id='dyn-close-settings', className='button subtle small preview-settings-close'),
                                                            html.Div(
                                                                [
                                                                    html.Label(
                                                                        'Duration [s] · max 60'
                                                                    ),
                                                                    dcc.Input(
                                                                        id='dyn-preview-duration',
                                                                        value=5,
                                                                        type='number',
                                                                        min=0.001,
                                                                        max=60,
                                                                    ),
                                                                ],
                                                                className='dyn-field',
                                                            ),
                                                            html.Div(
                                                                [
                                                                    html.Label(
                                                                        'Step [s] · max 2000 steps'
                                                                    ),
                                                                    dcc.Input(
                                                                        id='dyn-preview-dt',
                                                                        value=0.02,
                                                                        type='number',
                                                                        min=0.0001,
                                                                        max=1,
                                                                    ),
                                                                ],
                                                                className='dyn-field',
                                                            ),
                                                            html.P(
                                                                (
                                                                    'Model-only RK4. '
                                                                    'Environment excluded. '
                                                                    'Fixed inputs are assumptions.'
                                                                ),
                                                                className='hint',
                                                            ),
                                                            dash_table.DataTable(
                                                                id='dyn-assumptions',
                                                                data=[],
                                                                columns=[
                                                                    {
                                                                        'id': k,
                                                                        'name': k.title(),
                                                                        'editable': k == 'fixed',
                                                                    }
                                                                    for k in (
                                                                        'kind',
                                                                        'name',
                                                                        'shape',
                                                                        'unit',
                                                                        'fixed',
                                                                    )
                                                                ],
                                                                editable=True,
                                                                style_cell={
                                                                    'backgroundColor': '#0a1926',
                                                                    'color': '#edf4fa',
                                                                    'fontSize': 12,
                                                                    'border': '1px solid #213c50',
                                                                },
                                                                style_header={
                                                                    'backgroundColor': '#122d40'
                                                                },
                                                            ),
                                                        ],
                                                        className='studio-settings-body',
                                                    ),
                                                ],
                                                id='dyn-preview-settings',
                                                className='studio-settings',
                                            ),
                                        ],
                                        className='studio-preview-toolbar',
                                    ),
                                    dcc.Graph(
                                        id='dyn-preview',
                                        figure=empty_figure(
                                            'Preview the current mathematical draft.'
                                        ),
                                        config={'displaylogo': False},
                                        responsive=True,
                                        className='dyn-preview-graph',
                                    ),
                                    html.Small(
                                        id='dyn-preview-note',
                                        children='Unscored numerical samples only.',
                                    ),
                                ],
                                'studio-preview-panel',
                            ),
                        ],
                        className='studio-right',
                    ),
                ],
                className='studio-grid',
            ),
            html.Details(
                [
                    html.Summary('Library history, templates & compiler details'),
                    html.Div(
                        [
                            html.Label('Optional starting template'),
                            dcc.Dropdown(
                                id='dyn-template',
                                options=[
                                    {'label': v['name'], 'value': k} for k, v in templates.items()
                                ],
                                value='blank_state_space',
                                clearable=False,
                            ),
                            html.Button(
                                'New from template',
                                id='dyn-template-new',
                                className='button small',
                            ),
                            html.Label('Saved history · opening replaces the current draft'),
                            dcc.Dropdown(
                                id='dyn-history', options=[], placeholder='Open an earlier version'
                            ),
                            html.Div(
                                [
                                    html.Label('Add a contribution to a composed model'),
                                    dcc.Dropdown(
                                        id='dyn-influence-template',
                                        options=influences,
                                        value=influences[0]['value'],
                                        clearable=False,
                                    ),
                                    dcc.Checklist(
                                        id='dyn-zero-drag',
                                        options=[
                                            {
                                                'label': (
                                                    'Zero additional drag ' '(base drag unchanged)'
                                                ),
                                                'value': 'zero',
                                            }
                                        ],
                                        value=['zero'],
                                    ),
                                    html.Button(
                                        '＋ Add influence',
                                        id='dyn-add-influence',
                                        className='button',
                                    ),
                                ],
                                id='dyn-influence-tools',
                            ),
                            html.P(
                                (
                                    'Live binding remains owner-preflighted. '
                                    'Orientation/angular velocity mapping is not implemented. '
                                    'External fields remain owned by Experiment Lab.'
                                ),
                                className='hint',
                            ),
                            html.Pre(id='dyn-canonical'),
                        ],
                        className='studio-settings-body',
                    ),
                ],
                className='studio-library-advanced',
            ),
        ],
        id='dynamics-studio',
        className='dynamics-workspace',
    )


def register_studio(app, backend):
    """Thin structured-draft callbacks over the existing mathematical/library owners."""

    app.clientside_callback(
        """function(n) {
            if (!n) return window.dash_clientside.no_update;
            const panel = document.getElementById('dyn-preview-settings');
            if (panel) panel.open = false;
            return false;
        }""",
        Output('dyn-preview-settings', 'open'), Input('dyn-close-settings', 'n_clicks'),
        prevent_initial_call=True,
    )

    @app.callback(
        Output({'type': 'dyn-table', 'key': MATCH}, 'data'),
        Input({'type': 'dyn-add-row', 'key': MATCH}, 'n_clicks'),
        State({'type': 'dyn-table', 'key': MATCH}, 'data'),
        prevent_initial_call=True,
    )
    def add_row(clicks, rows):
        if not clicks:
            raise PreventUpdate
        key = ctx.triggered_id['key']
        defaults = {
            'state': {'name': '', 'initial': '0', 'derivative': '0', 'shape': '[]', 'unit': ''},
            'parameters': {'name': '', 'value': '0', 'unit': '', 'shape': '[]'},
            'context': {'name': '', 'shape': '[]', 'unit': '', 'frame': 'world'},
            'control': {'name': '', 'shape': '[]', 'unit': '', 'frame': 'world'},
        }
        return [*(rows or []), defaults.get(key, {})]

    @app.callback(
        Output({'type': 'dyn-table', 'key': ALL}, 'hidden_columns'),
        Input('dyn-units', 'value'),
        State({'type': 'dyn-table', 'key': ALL}, 'id'),
        prevent_initial_call=True,
    )
    def units(values, ids):
        return [
            (
                []
                if values
                else {'state': ['shape', 'unit'], 'parameters': ['shape', *BOUNDS]}.get(
                    i['key'], []
                )
            )
            for i in ids
        ]

    actions = [
        'dyn-new',
        'dyn-template-new',
        'dyn-save',
        'dyn-preview-button',
        'dyn-add-influence',
        'dyn-remove',
        'dyn-up',
        'dyn-down',
        'dyn-switch-confirm',
        'dyn-switch-cancel',
    ]

    @app.callback(
        Output('dyn-state', 'data'),
        Output('dyn-editor', 'children'),
        Output('dyn-library', 'options'),
        Output('dyn-library', 'value'),
        Output('dyn-name', 'value'),
        Output('dyn-confirm', 'hidden'),
        Output('dyn-assumptions', 'data'),
        Output('dyn-library-events', 'data'),
        *[Input(k, 'n_clicks') for k in actions],
        Input('dyn-library', 'value'),
        Input('dyn-history', 'value'),
        Input({'type': 'dyn-field', 'key': ALL}, 'value'),
        Input({'type': 'dyn-table', 'key': ALL}, 'data'),
        Input('dyn-contribution', 'value'),
        Input('dyn-name', 'value'),
        State({'type': 'dyn-field', 'key': ALL}, 'id'),
        State({'type': 'dyn-table', 'key': ALL}, 'id'),
        State('dyn-template', 'value'),
        State('dyn-influence-template', 'value'),
        State('dyn-zero-drag', 'value'),
        State('dyn-preview-duration', 'value'),
        State('dyn-preview-dt', 'value'),
        State('dyn-assumptions', 'data'),
        State('dyn-state', 'data'),
        State('dyn-library-events', 'data'),
        prevent_initial_call=True,
    )
    def operate(*args):
        model_id, history, field_values, table_values, contribution, name = args[
            len(actions) : len(actions) + 6
        ]
        (
            field_ids,
            table_ids,
            template,
            influence,
            zero_drag,
            duration,
            dt,
            assumptions,
            state,
            events,
        ) = args[len(actions) + 6 :]
        trigger = ctx.triggered_id
        state = deepcopy(state)
        result = [state, no_update, no_update, no_update, no_update, True, no_update, no_update]
        fields = {i['key']: v for i, v in zip(field_ids, field_values)}
        tables = {i['key']: v for i, v in zip(table_ids, table_values)}
        try:
            if trigger in ('dyn-switch-confirm', 'dyn-switch-cancel'):
                request = state.pop('pending', None)
                if trigger == 'dyn-switch-cancel':
                    result[3] = state.get('id')
                    return result
                if not request:
                    raise PreventUpdate
                trigger, template, model_id, history = request
            elif trigger in ('dyn-new', 'dyn-template-new', 'dyn-library', 'dyn-history'):
                if trigger == 'dyn-library' and (not model_id or model_id == state.get('id')):
                    raise PreventUpdate
                if trigger == 'dyn-history' and (not model_id or history is None):
                    raise PreventUpdate
                if state.get('dirty'):
                    state['pending'] = [trigger, template, model_id, history]
                    result[5] = False
                    return result
            if trigger in ('dyn-new', 'dyn-template-new', 'dyn-library', 'dyn-history'):
                if trigger in ('dyn-new', 'dyn-template-new'):
                    state = new_state(
                        backend, template if trigger == 'dyn-template-new' else 'blank_state_space'
                    )
                else:
                    revision = (
                        int(history.split(':')[-1])
                        if trigger == 'dyn-history'
                        else backend.dynamics.revisions(model_id)[-1]['revision']
                    )
                    state = load_model(backend, model_id, revision)
                result[0:2] = [state, editor(state['definition'])]
                result[3:5] = [state.get('id'), state['name']]
                result[6] = []
                return result
            definition = read_editor(state['definition'], state.get('selected', 0), fields, tables)
            changed = definition != state['definition'] or name != state['name']
            if changed:
                state.update(
                    definition=definition,
                    dirty=True,
                    checked=None,
                    preview=None,
                    stage='Draft',
                    message='Unsaved mathematical draft.',
                )
            state.pop('editor_error', None)
            if isinstance(trigger, dict) or trigger == 'dyn-name':
                if not changed:
                    raise PreventUpdate
                return result
            if trigger == 'dyn-contribution':
                if contribution is None or contribution == state.get('selected', 0):
                    raise PreventUpdate
                state['selected'] = contribution
                result[1] = editor(definition, contribution)
                return result
            if trigger in ('dyn-add-influence', 'dyn-remove', 'dyn-up', 'dyn-down'):
                index = state.get('selected', 0)
                if trigger == 'dyn-add-influence':
                    definition = add_interaction(
                        definition,
                        backend.dynamics.templates(),
                        influence,
                        zero_drag='zero' in zero_drag,
                    )
                    index = len(definition['interactions'])
                elif index:
                    items = definition['interactions']
                    if trigger == 'dyn-remove':
                        items.pop(index - 1)
                        index = 0
                    else:
                        other = index + (-1 if trigger == 'dyn-up' else 1)
                        if not 1 <= other <= len(items):
                            raise PreventUpdate
                        items[index - 1], items[other - 1] = items[other - 1], items[index - 1]
                        index = other
                state.update(
                    definition=definition,
                    selected=index,
                    dirty=True,
                    checked=None,
                    preview=None,
                    stage='Draft',
                )
                result[1] = editor(definition, index)
                return result
            checked = backend.dynamics.validate(definition)
            state['checked'] = checked
            if not checked['valid']:
                raise ValueError('\n'.join(checked['errors']))
            expected = preview_inputs(checked['model'])
            for row in expected:
                old = next(
                    (
                        r
                        for r in assumptions
                        if all(r.get(k) == row[k] for k in ('name', 'kind', 'shape', 'unit'))
                    ),
                    None,
                )
                if old:
                    row['fixed'] = old['fixed']
            result[6] = expected
            if trigger == 'dyn-preview-button':
                if any(r['fixed'] in ('', None) for r in expected):
                    raise ValueError(
                        'Enter fixed input assumptions in Preview Settings, then Preview again.'
                    )
                context = {
                    r['name']: numeric(r['fixed'], r['name'])
                    for r in expected
                    if r['kind'] == 'context'
                }
                control = []
                for r in expected:
                    if r['kind'] == 'control':
                        value = numeric(r['fixed'], r['name'])
                        control.extend(value if isinstance(value, list) else [value])
                state['preview'] = backend.dynamics.preview(
                    definition, duration=duration, dt=dt, context=context, control=control
                )
                state.update(
                    stage='Previewed',
                    message='Numerical preview complete · unscored, environment excluded.',
                )
            elif trigger == 'dyn-save':
                state = save_model(
                    backend, state, state.get('id') or state.get('new_id'), name, definition
                )
                state['message'] = (
                    f"Saved {state['name']} · version {state['expected_revision']}. "
                    'Available in Experiment Lab.'
                )
                result[0] = state
                result[2:5] = [library_options(backend), state['id'], state['name']]
                result[7] = (events or 0) + 1
            return result
        except PreventUpdate:
            raise
        except (ValueError, TypeError, KeyError, IndexError, OSError) as error:
            state.update(
                stage='Invalid',
                message=str(error),
                dirty=True,
                preview=None,
                checked=None,
                editor_error=True,
            )
            result[0] = state
            return result

    @app.callback(
        Output('dyn-feedback', 'children'),
        Output('dyn-feedback', 'className'),
        Output('dyn-badge', 'children'),
        Output('dyn-capabilities', 'children'),
        Output('dyn-canonical', 'children'),
        Output('dyn-math', 'children'),
        Output('dyn-output-velocity', 'children'),
        Output('dyn-preview', 'figure'),
        Output('dyn-preview-note', 'children'),
        Output('dyn-influence-tools', 'style'),
        Output('dyn-history', 'options'),
        Input('dyn-state', 'data'),
        Input('dyn-preview-view', 'value'),
    )
    def present(state, view):
        checked = state.get('checked') or {}
        caps = checked.get('capabilities') or {}
        valid = checked.get('valid', False)
        status = 'Mathematical validation pending.'
        if valid:
            status = 'Mathematically valid · ' + (
                'Live adapter structurally supported; Experiment preflight required.'
                if caps.get('deployable')
                else 'Preview only · ' + '; '.join(caps.get('deployment_reasons', []))
            )
            if caps.get('target_output') == 'prescribed_mapping':
                status += ' · Requires explicit no external acceleration in Experiment Lab.'
            if caps.get('unused_parameters'):
                status += ' · Unused parameters: ' + ', '.join(caps['unused_parameters'])
        try:
            if state.get('editor_error'):
                raise ValueError('Correct invalid cells to render the current draft.')
            model = checked.get('model') or state['definition']
            if model['type'] == 'cartesian_composition':
                compiled = backend.dynamics.validate(model)
                if not compiled['valid']:
                    raise ValueError('Composition cannot be typeset until its source is valid.')
                model = compiled['model']
            math = model_tex(model)
            output = caps.get('output_mapping')
            velocity = (
                dcc.Markdown(
                    r'$$\mathbf{v}_{world}=' + velocity_tex(output, model) + '$$',
                    mathjax=True,
                )
                if output
                else 'No verified explicit mapping. Validate or Preview to derive.'
            )
        except (ValueError, KeyError, TypeError, RecursionError) as error:
            math = 'Current draft unavailable: ' + str(error)
            velocity = 'Not verified.'
        preview = state.get('preview')
        history = []
        if state.get('id'):
            try:
                history = [
                    {
                        'label': f"Version {r['revision']} · {r['sha256'][:12]}",
                        'value': f"{state['id']}:{r['revision']}",
                    }
                    for r in reversed(backend.dynamics.revisions(state['id']))
                ]
            except (ValueError, OSError):
                pass
        return (
            state.get('message', ''),
            'notice ' + ('error' if state['stage'] == 'Invalid' else 'neutral'),
            state['stage'] + (' · unsaved' if state['dirty'] else ''),
            status,
            json.dumps(checked, indent=2),
            math,
            velocity,
            preview_figure(preview, view),
            (
                f"{len(preview['times'])} samples · {preview['times'][-1]:g} s · "
                'fixed inputs · environment excluded'
                if preview
                else 'Model-only preview · no Run or physical acceptance.'
            ),
            {} if state['definition']['type'] == 'cartesian_composition' else {'display': 'none'},
            history,
        )

    @app.callback(
        Output({'type': 'component', 'role': 'target_dynamics'}, 'options'),
        Input('dyn-library-events', 'data'),
        State('builder-state', 'data'),
        prevent_initial_call=True,
    )
    def available_models(_events, state):
        if not state or 'spec' not in state:
            raise PreventUpdate
        inherited = state['inherited_components']['target_dynamics']['id']
        ex = state['spec']['experiment']
        return target_options(backend, ex.get('dynamics_source'), inherited=inherited,
                              selection=ex.get('components', {}).get('target_dynamics'))
