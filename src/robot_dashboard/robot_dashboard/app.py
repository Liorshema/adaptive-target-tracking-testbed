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

from .execution import execution_layout
from .execution import register_execution
from .forms import INHERIT
from .forms import apply_comparison, apply_execution
from .forms import assemble_draft
from .forms import begin_draft
from .forms import fingerprint
from .forms import parameter_rows
from .management import management_state
from .management import register_management
from .management import scenario_menu
from .results import register_trajectory
from .simulation import register_simulation
from .simulation import simulation_layout

LOG = logging.getLogger(__name__)


def readable(name):
    """Use stable registry keys with legible labels, without a parallel name catalog."""
    return name.replace('_', ' ').capitalize()


def formatted(value):
    """Render detached metadata without interpreting scientific values."""
    return json.dumps(value, ensure_ascii=False, allow_nan=False)


def experiment_options(backend):
    """List public Catalog labels without altering persisted experiment names."""
    return [{'label': r['name'], 'value': r['id']} for r in backend.list_experiments()]


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
        choices = [{'label': f'Inherited · {readable(inherited)}', 'value': INHERIT}]
        for option in backend.list_component_options(role):
            unavailable = not option['deployable'] or option['compatible'] is False
            suffix = ' · unavailable' if unavailable else ' · preflight required'
            choices.append(
                {
                    'label': readable(option['id']) + suffix,
                    'value': option['id'],
                    'disabled': unavailable,
                }
            )
        selection = ex.get('components', {}).get(role, INHERIT)
        source = detail.get('selection_source', {})
        origin = (
            f"Inherited selection · {source.get('id', ex['base_case'])} "
            f"r{source.get('revision', ex['base_revision'])}"
            if selection == INHERIT
            else 'Explicit component selection'
        )
        rows = parameter_rows(detail)
        result.append(
            html.Details(
                [
                    html.Summary(
                        [
                            html.Span(readable(role), className='component-role'),
                            html.Span(readable(detail['id']), className='component-id'),
                            html.Span('⌄', className='component-count'),
                        ]
                    ),
                    html.Div(
                        [
                            html.Label('Model selection'),
                            dcc.Dropdown(
                                id={'type': 'component', 'role': role},
                                options=choices,
                                value=selection,
                                clearable=False,
                            ),
                            html.Details(
                                [
                                    html.Summary('Selection provenance'),
                                    html.P(origin),
                                    html.P('Validate the complete draft before saving.'),
                                ],
                                className='selection-origin',
                            ),
                            *[_parameter(role, row) for row in rows],
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
                    ),
                ],
                className='component-card',
                open=role == 'target_dynamics',
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
        (
            visible if card.to_plotly_json()['props']['data-role'] in primary else secondary
        ).append(card)
    if secondary:
        visible.append(
            html.Details(
                [html.Summary('Other model settings'), *secondary], className='other-models'
            )
        )
    return visible


def setup_view(effective, capabilities, *, fresh, identity):
    """Report resolved deployment and the backend's explicit unsupported boundary."""
    rows = []
    for run in (effective or {}).get('runs', []):
        robot, scenario = run['robot'], run['scenario']
        resources = run.get('sources', {}).get('resources', {})
        worlds = [
            path
            for path, meta in resources.items()
            if meta.get('sha256') == scenario.get('world_sha256')
        ]
        rows.append(
            html.Div(
                [
                    html.H4(f"{run['variant']} · seed {run['seed']}"),
                    html.Dl(
                        [
                            html.Dt('Physical robot description'),
                            html.Dd(robot['description_source'], className='path'),
                            html.Dt('Entity'),
                            html.Dd(robot['deployment'].get('entity_name', 'not declared')),
                            html.Dt('World'),
                            html.Dd(
                                ', '.join(worlds) or 'See effective configuration',
                                className='path',
                            ),
                            html.Dt('Physics'),
                            html.Dd(formatted(scenario.get('physics', {}))),
                            html.Dt('Saved actuation default (Run opt-in applies)'),
                            html.Dd(
                                str(
                                    run['runtime']['settings'].get(
                                        'enable_actuation', 'not declared'
                                    )
                                )
                            ),
                        ]
                    ),
                    html.Details(
                        [
                            html.Summary('Deployment and holding configuration'),
                            html.Pre(
                                json.dumps(
                                    {
                                        'deployment': robot['deployment'],
                                        'holding': robot['holding'],
                                        'runtime': run['runtime'],
                                    },
                                    indent=2,
                                )
                            ),
                        ]
                    ),
                ],
                className='deployment',
            )
        )
    return [
        html.H2('Simulation Setup'),
        html.Span(
            (
                'Configuration validated'
                if fresh
                else (
                    'Draft needs validation' if effective else 'Select a saved configuration'
                )
            ),
            className='badge success' if fresh else 'badge neutral',
        ),
        html.Details(
            [html.Summary('Advanced deployment'), html.P(identity), *rows],
            className='deployment-details',
        ),
    ]


def execution_fields(state=None, backend=None):
    """Render execution controls from the Author-resolved study, with no new defaults."""
    study = ((state or {}).get('preview') or {}).get('effective', {}).get('experiment', {})
    policy = (state or {}).get('spec', {}).get('experiment', {}).get('comparison', study.get('comparison'))
    options = backend.list_comparison_variables() if backend else []
    variable = (policy or {}).get('variables', [{}])[0]
    selected = (variable.get('path') if policy and policy.get('schema_version') == 2 and len(policy['variables']) == 1
                else '__preserve__' if policy else '__strict__')
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
        html.Details([
            html.Summary('Advanced Comparison Setup'),
            html.Div([
            html.P('Save each configuration as an immutable revision with the same declaration and Seeds. Run each saved revision and add its Runs to the same Session. Comparability is checked from frozen evidence.', className='hint'),
            dcc.Dropdown(id='comparison-variable', clearable=False, value=selected,
                         options=[{'label': 'Strict comparison (no declared differences)', 'value': '__strict__'},
                                  *([{'label': 'Preserve pinned comparison declaration', 'value': '__preserve__'}] if selected == '__preserve__' else []),
                                  *[{'label': o['path'], 'value': o['path']} for o in options]],
                         disabled=not state),
            dcc.Dropdown(id='comparison-models', multi=True,
                         value=variable.get('values', []) if selected.startswith('components.') else [],
                         placeholder='Permitted registered models'),
            dcc.Input(id='comparison-values', type='text',
                      value=json.dumps(variable['values']) if selected.startswith('parameters.') and 'values' in variable else '',
                      placeholder='Optional permitted parameter values (JSON array)'),
            html.P('Only declared fields may differ. Dynamics studies are labeled separately. Incompatible tasks, physics, timing, sensing and safety remain protected.', className='hint'),
            ], className='comparison-setup-popover'),
        ], className='advanced-comparison'),
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
    capabilities = backend.gazebo_setup_capabilities()

    def layout():
        options = experiment_options(backend)
        execution = execution_layout(backend)
        return html.Div(
            [
                dcc.Store(id='builder-state'),
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
                                        html.Label('Condition', htmlFor='source-variant'),
                                        dcc.Dropdown(id='source-variant', clearable=False),
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
                                            className='mode-switch',
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
                                        html.Details(
                                            [
                                                html.Summary(
                                                    'Advanced Configuration / Diagnostics'
                                                ),
                                                html.P(id='preview-context', className='hint'),
                                                html.P(
                                                    'Inherited component selections and owner '
                                                    'defaults are shown '
                                                    'separately above. A resolved value is '
                                                    'labeled unknown when '
                                                    'the backend cannot prove its exact '
                                                    'parameter ancestry.',
                                                    className='hint',
                                                ),
                                                html.Pre(
                                                    id='full-effective',
                                                    className='json-preview',
                                                ),
                                            ],
                                            id='effective-inspector',
                                            className='effective-inspector',
                                        ),
                                    ],
                                    className='panel builder-panel',
                                ),
                                html.Aside(
                                    [
                                        html.Aside(
                                            [
                                                html.Div(
                                                    setup_view(
                                                        None,
                                                        capabilities,
                                                        fresh=False,
                                                        identity=(
                                                            'Select a saved configuration.'
                                                        ),
                                                    ),
                                                    id='simulation-setup',
                                                ),
                                                simulation_layout(),
                                            ],
                                            className='panel setup-panel',
                                        ),
                                        execution['monitor'],
                                    ],
                                    className='lab-rail',
                                ),
                            ],
                            id='experiment-lab',
                            className='workspace-grid',
                        ),
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
        Output('source-variant', 'options'),
        Output('source-variant', 'value'),
        Output('source-summary', 'children'),
        Output('saved-preview', 'children'),
        Output('selected-snapshot', 'data'),
        Input('source-experiment', 'value'),
        Input('source-revision', 'value'),
    )
    def inspect_saved(identifier, revision):
        if not identifier or revision is None:
            return [], None, '', '', None
        try:
            effective = backend.explain_experiment(identifier, revision)
            variants = list(dict.fromkeys(r['variant'] for r in effective['runs']))
            return (
                [{'label': v, 'value': v} for v in variants],
                variants[0],
                f'{len(variants)} saved condition' + ('s' if len(variants) != 1 else ''),
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
            return [], None, f'Cannot inspect revision: {error}', '', None

    @app.callback(Output('create-fields', 'style'), Input('draft-mode', 'value'))
    def creation_fields(mode):
        return {'display': 'block' if mode == 'create' else 'none'}

    app.clientside_callback(
        "function(v) { return !(v || []).includes('override'); }",
        Output({'type': 'param', 'role': MATCH, 'key': MATCH, 'kind': MATCH}, 'disabled'),
        Input({'type': 'override', 'role': MATCH, 'key': MATCH, 'kind': MATCH}, 'value'),
    )

    @app.callback(
        Output('comparison-models', 'options'),
        Output('comparison-models', 'disabled'),
        Output('comparison-values', 'disabled'),
        Input('comparison-variable', 'value'),
    )
    def comparison_choices(path):
        option = next((o for o in backend.list_comparison_variables() if o['path'] == path), {})
        is_model = option.get('kind') == 'model'
        return ([{'label': name, 'value': name} for name in option.get('model_values', [])],
                not is_model, option.get('kind') != 'parameter')

    @app.callback(
        Output('builder-state', 'data'),
        Output('component-cards', 'children'),
        Output('source-experiment', 'options'),
        Output('execution-fields', 'children'),
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
        Input('comparison-variable', 'value'),
        Input('comparison-models', 'value'),
        Input('comparison-values', 'value'),
        State({'type': 'component', 'role': ALL}, 'id'),
        State({'type': 'param', 'role': ALL, 'key': ALL, 'kind': ALL}, 'id'),
        State('source-experiment', 'value'),
        State('source-revision', 'value'),
        State('source-variant', 'value'),
        State('draft-mode', 'value'),
        State('new-id', 'value'),
        State('new-name', 'value'),
        State('builder-state', 'data'),
        prevent_initial_call=True,
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
        comparison_path,
        comparison_models,
        comparison_values,
        component_ids,
        fields,
        source,
        revision,
        variant,
        mode,
        identifier,
        name,
        state,
    ):
        trigger = ctx.triggered_id
        cards, choices = no_update, no_update
        state = deepcopy(state) if state else None
        try:
            if trigger == 'load-draft':
                state = begin_draft(backend, source, revision, variant, mode, identifier, name)
                state['inherited_components'] = deepcopy(state['components'])
                return (
                    state,
                    component_cards(backend, state),
                    no_update,
                    execution_fields(state, backend),
                )
            if not state or 'spec' not in state:
                raise PreventUpdate
            if trigger == 'reset-draft':
                state['spec'] = backend.reset_draft(state['spec'])
                state['preview'] = backend.preview_experiment(state['spec'])
                state['components'] = state['preview']['components']
                state.update(
                    stage='Draft',
                    validated_digest=None,
                    message='Unsaved selections and overrides reset to the pinned base.',
                )
                state['form_digest'] = fingerprint(state['spec'])
                return (
                    state,
                    component_cards(backend, state),
                    no_update,
                    execution_fields(state, backend),
                )
            spec = assemble_draft(
                state['spec'], selections, component_ids, values, fields, switches
            )
            spec = apply_execution(spec, duration, seeds)
            spec = apply_comparison(spec, comparison_path, comparison_models, comparison_values, backend.list_comparison_variables())
            digest = fingerprint(spec)
            if isinstance(trigger, dict) or trigger in (
                'execution-duration',
                'execution-seeds',
                'comparison-variable', 'comparison-models', 'comparison-values',
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
                return state, cards, no_update, no_update
            state['spec'], state['form_digest'] = spec, digest
            if trigger == 'save-draft':
                if state['stage'] != 'Validated' or state['validated_digest'] != digest:
                    raise ValueError(
                        'Validate the current form before saving; edits invalidate approval.'
                    )
                record = backend.save_experiment(
                    spec, expected_revision=state['expected_revision']
                )
                state.update(
                    stage='Saved',
                    expected_revision=record['revision'],
                    saved_revision=record['revision'],
                    validated_digest=None,
                    message='Configuration saved. Play uses the selected saved configuration.',
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
            return state, cards, choices, no_update
        except PreventUpdate:
            raise
        except (ValueError, TypeError, KeyError, OSError) as error:
            state = state or {}
            state.update(stage='Invalid', validated_digest=None, message=str(error))
            return state, no_update, no_update, no_update
        except Exception:
            LOG.exception('Dashboard action failed')
            state = state or {}
            state.update(
                stage='Invalid',
                validated_digest=None,
                message='The action could not complete. Your draft was not saved. '
                'Check the dashboard terminal for details, then retry explicitly.',
            )
            return state, no_update, no_update, no_update

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
        Output('full-effective', 'children'),
        Output('preview-context', 'children'),
        Output('simulation-setup', 'children'),
        Input('builder-state', 'data'),
        Input('selected-snapshot', 'data'),
    )
    def present(state, selected):
        state = state or {}
        spec, stage = state.get('spec'), state.get('stage', 'No draft')
        preview = state.get('preview')
        resolved = preview['effective'] if preview else (selected or {}).get('effective')
        fresh = bool(spec and preview and stage in ('Previewed', 'Validated', 'Saved'))
        identity = (
            spec['experiment']['name']
            if spec
            else (selected or {}).get('identity', 'Load a configuration to begin.')
        )
        tone = (
            'success'
            if stage in ('Validated', 'Saved')
            else 'error' if stage == 'Invalid' else 'neutral'
        )
        context = (
            'Current form · configuration preflight only; no simulation launched.'
            if fresh
            else (
                'Last resolved snapshot; edits are not reflected until Preview or Validate.'
                if preview
                else 'Saved revision snapshot; load a draft to configure it.'
            )
        )
        effective_text = (
            json.dumps(
                (
                    {
                        'components': preview['components'],
                        'provenance': preview['provenance'],
                        'provenance_scope': preview['provenance_scope'],
                        'effective': resolved,
                    }
                    if preview
                    else resolved
                ),
                indent=2,
                ensure_ascii=False,
            )
            if resolved
            else ''
        )
        feedback = {
            'Draft': 'Unsaved configuration · validate, then save.',
            'Previewed': 'Preview ready · validate before saving.',
            'Validated': 'Valid configuration · ready to save.',
            'Saved': 'Saved · Play uses the selected saved configuration.',
        }.get(stage, state.get('message', 'Choose a scenario, then load it.'))
        return (
            identity,
            'Unsaved' if stage == 'Draft' else stage,
            'badge ' + tone,
            feedback,
            'notice ' + tone,
            not spec,
            not spec,
            not spec,
            stage != 'Validated',
            'Create experiment' if state.get('expected_revision') is None else 'Save revision',
            effective_text,
            context,
            setup_view(
                resolved,
                capabilities,
                fresh=fresh or (not spec and bool(resolved)),
                identity=identity + (' · last resolved' if spec and not fresh else ''),
            ),
        )

    register_execution(app, backend)
    register_simulation(app, backend)
    register_management(app, backend)
    register_trajectory(app, backend)
    return app
