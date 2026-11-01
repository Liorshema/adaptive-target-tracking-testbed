"""Present recorded Run evidence without calculating scientific metrics."""

import json
import math

from dash import dcc
from dash import html

import plotly.graph_objects as go

CHARTS = ('tracking_distance', 'pointing_error', 'control_compute_time')


def label(value):
    """Render stable metric keys as readable labels."""
    return value.replace('_', ' ').capitalize()


def display(value):
    """Preserve missing measurements instead of displaying them as zeros."""
    if value is None:
        return 'Unavailable'
    if isinstance(value, float):
        return f'{value:.6g}' if math.isfinite(value) else 'Unavailable (nonfinite)'
    return str(value)


def outcome(value):
    """Keep unknown recorded acceptance separate from accepted and rejected."""
    return 'Accepted' if value is True else 'Rejected' if value is False else 'Unknown'


def metric_rows(result):
    """Return existing metric values and availability, with no recomputation."""
    document = result.get('metrics') or {}
    if document.get('available') is False:
        return []
    return [
        {'metric': name, **detail}
        for name, detail in document.get('metrics', {}).items()
        if isinstance(detail, dict)
    ]


COLORS = {
    'tracking_distance': '#49adff',
    'pointing_error': '#c383ff',
    'control_compute_time': '#f5cf67',
}
KPIS = (
    ('tracking_distance', 'Mean Position Error', 'blue', 'mean'),
    ('pointing_error', 'Mean Orientation Error', 'violet', 'mean'),
    ('target_position_error', 'Estimator Accuracy', 'green', 'rmse'),
    ('prediction_rmse', 'Predictor Accuracy', 'yellow', 'rmse'),
)


def style_figure(figure, title, xaxis, yaxis):
    """Apply a consistent dark engineering presentation without changing samples."""
    figure.update_layout(
        title={'text': title, 'font': {'size': 17}},
        autosize=True,
        margin={'l': 58, 'r': 20, 't': 44, 'b': 46},
        paper_bgcolor='#0B1E2D',
        plot_bgcolor='#0B1E2D',
        font={'color': '#adc1d3', 'family': 'Inter, Segoe UI, sans-serif', 'size': 13},
        xaxis_title=xaxis,
        yaxis_title=yaxis,
        xaxis={'gridcolor': '#173246', 'zerolinecolor': '#234358', 'title_font_size': 14},
        yaxis={'gridcolor': '#173246', 'zerolinecolor': '#234358', 'title_font_size': 14},
        legend={'orientation': 'h', 'y': 1.02, 'x': 0, 'font': {'size': 13}},
    )
    return figure


def recorded_figures(result, page):
    """Plot exact scored samples as markers; no interpolation, alignment or smoothing."""
    metrics = {row['metric']: row for row in metric_rows(result)}
    figures = {}
    for name in CHARTS:
        detail = page.get('series', {}).get(name)
        if not detail or metrics.get(name, {}).get('available') is not True:
            continue
        samples = detail.get('samples', [])
        if not samples or any(
            not isinstance(s, dict) or any(not _finite(s.get(k)) for k in ('time', 'value'))
            for s in samples
        ):
            continue
        figure = go.Figure(
            go.Scatter(
                x=[s['time'] for s in samples],
                y=[s['value'] for s in samples],
                mode='markers',
                marker={'size': 3, 'color': COLORS[name]},
                name={
                    'tracking_distance': 'Radial error |eᵣ|',
                    'pointing_error': 'Pointing angle |eθ|',
                    'control_compute_time': 'Control compute time',
                }[name],
                connectgaps=False,
            )
        )
        style_figure(
            figure,
            '',
            f"{detail.get('time_basis', 'Time basis not recorded')} "
            f"[{detail.get('time_unit', '?')}]",
            {
                'tracking_distance': '|<i>e</i><sub>r</sub>|',
                'pointing_error': '|<i>e</i><sub>θ</sub>|',
                'control_compute_time': 'Control compute time',
            }[name]
            + f" [{detail.get('unit', 'unit not recorded')}]",
        )
        figure.update_layout(
            uirevision=f"{result.get('run_id')}:{name}",
            margin={'l': 64, 'r': 20, 't': 16, 'b': 52},
        )
        figures[name] = figure
    return figures


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _scalar(metric, statistic):
    """Accept genuine finite nonnegative stored errors, including recorded zero."""
    value = metric.get(statistic)
    return value if metric.get('available') is True and _finite(value) and value >= 0 else None


def _mean_symbol(subscript):
    """Render a mean absolute error using native HTML, with no remote math assets."""
    return html.Span(
        ['|', html.I('e'), html.Sub(subscript), '|'],
        className='math-mean',
        style={'textDecoration': 'overline'},
    )


def kpi_cards(result):
    """Present four recorded errors, keeping RMSE separate from the recorded mean."""
    metrics = {r['metric']: r for r in metric_rows(result)}
    cards = []
    for name, title, color, statistic in KPIS:
        metric = metrics.get(name, {})
        if name == 'target_position_error' and _scalar(metric, 'rmse') is None:
            statistic = 'mean'
        value = _scalar(metric, statistic)
        available = value is not None
        symbol = (
            _mean_symbol('r')
            if name == 'tracking_distance'
            else (
                _mean_symbol('θ')
                if name == 'pointing_error'
                else html.Span(
                    'RMSE' if statistic == 'rmse' else 'Mean |e|', className='math-stat'
                )
            )
        )
        caption = {
            'tracking_distance': 'Recorded mean · radial distance-to-reference error',
            'pointing_error': 'Recorded mean · target pointing angle',
            'target_position_error': (
                f'Recorded {statistic.upper() if statistic == "rmse" else statistic}'
                ' · target estimate'
            ),
            'prediction_rmse': 'Recorded RMSE · target prediction',
        }[name]
        cards.append(
            html.Div(
                [
                    html.Div(
                        [html.Span(title), html.Span(symbol, className='kpi-symbol')],
                        className='kpi-label',
                    ),
                    html.Div(
                        [
                            html.Strong(f'{value:.3f}' if available else 'Unavailable'),
                            html.Small(metric.get('unit', '') if available else ''),
                        ],
                        className='kpi-value',
                    ),
                    html.Small(
                        caption if available else 'No valid recorded scalar',
                        className='kpi-caption',
                    ),
                ],
                className='kpi-card ' + color,
                **{'data-metric': name},
            )
        )
    return cards


def scenario_names(backend, runs=()):
    """Use Catalog names, falling back to immutable metadata for external historical IDs."""
    names = {
        item['id']: item['name']
        for item in backend.list_experiments(include_deleted=True)
        if isinstance(item.get('name'), str) and item['name'].strip()
    }
    for run in runs:
        identifier = run.get('experiment_id')
        if not isinstance(identifier, str) or identifier in names or not run.get('run_id'):
            continue
        try:
            name = scenario_label(backend.get_run_results(run['run_id']))
        except (ValueError, TypeError, KeyError, OSError):
            continue
        if name != 'Scenario unavailable':
            names[identifier] = name
    return names


def scenario_label(result, names=None):
    """Prefer authoritative display metadata; never invent model-specific scenario names."""
    identifier = result.get('experiment_id')
    if names and names.get(identifier):
        return names[identifier]
    identity = result.get('experiment_identity') or {}
    configuration = result.get('configuration') or {}
    experiment = configuration.get('experiment') or {}
    if isinstance(experiment.get('experiment'), dict):
        experiment = experiment['experiment']
    for value in (result.get('experiment_name'), identity.get('name'), experiment.get('name')):
        if isinstance(value, str) and value.strip():
            return value.replace('_', ' ')
    return 'Scenario unavailable'


def header_view(result, names=None):
    """Show recorded robot/experiment identity and execution, not inferred success."""
    if not result:
        return html.Span('No recorded Run selected', className='muted')
    robot = (
        result.get('configuration', {}).get('system', {}).get('manifest', {}).get('robot', {})
    )
    return [
        html.Strong(
            robot.get('type', 'Robot identity unavailable').replace('_', ' ').capitalize()
        ),
        html.Span(scenario_label(result, names)),
        html.Span(
            result.get('status', {}).get('execution_status', 'unknown'),
            className='status-chip '
            + result.get('status', {}).get('execution_status', 'unknown'),
        ),
    ]


def trajectory_figure(pages):
    """Show normalized target XY samples and one recorded forecast; never align frames."""
    figure = go.Figure()
    frame = None
    notes = ['Independent raw XY sample pages; no alignment or interpolation.']
    for stream, title, color, field in [
        ('target_truth', 'Truth', '#d5e6f3', 'position'),
        ('target_estimate', 'Estimate', '#49adff', 'mean'),
        ('prediction', 'Prediction', '#4de4a0', 'states'),
    ]:
        records = pages.get(stream, {}).get('records', [])
        if stream == 'prediction':
            records = records[-1:]
        xs, ys, times = [], [], []
        for record in records:
            source = record.get('frame_id')
            if (
                record.get('schema_version') != 1
                or not source
                or record.get('units', {}).get('position') != 'm'
            ):
                continue
            if frame is not None and source != frame:
                notes.append(
                    f'{title} unavailable in shared frame: {source} differs from {frame}.'
                )
                break
            data = record.get('data', {})
            vectors = data.get(field, []) if stream == 'prediction' else [data.get(field, [])]
            epochs = (
                data.get('timestamps', [])
                if stream == 'prediction'
                else [record.get('source_time')]
            )
            # The normalized target-state contract stores XYZ as its leading components.
            for vector, epoch in zip(vectors[:1000], epochs[:1000]):
                if (
                    not isinstance(vector, list)
                    or len(vector) < 3
                    or not all(_finite(v) for v in vector[:3])
                    or not _finite(epoch)
                ):
                    continue
                frame = source
                xs.append(vector[0])
                ys.append(vector[1])
                times.append(epoch)
        if xs:
            figure.add_trace(
                go.Scatter(
                    x=xs,
                    y=ys,
                    customdata=times,
                    mode='markers',
                    marker={'size': 3 if stream != 'prediction' else 4, 'color': color},
                    name=title,
                    hovertemplate=(
                        'X %{x} m<br>Y %{y} m<br>epoch %{customdata} s'
                        '<extra>%{fullData.name}</extra>'
                    ),
                )
            )
        else:
            notes.append(f'{title}: unavailable on this page.')
    if not figure.data:
        return None, 'Unavailable: no compatible recorded target XY telemetry. ' + ' '.join(
            dict.fromkeys(notes)
        )
    style_figure(figure, '', f'X [m] · {frame}', 'Y [m]')
    notes.append(
        'Prediction is one horizon from the last message on this page; '
        'hover shows source/forecast epoch [s].'
    )
    figure.update_layout(
        uirevision='raw-target-xy',
        yaxis_scaleanchor='x',
        yaxis_scaleratio=1,
        margin={'l': 58, 'r': 20, 't': 26, 'b': 42},
    )
    return figure, ' '.join(dict.fromkeys(notes))


def table(headers, rows):
    """Build a readable, horizontally scrollable engineering table."""
    return html.Div(
        html.Table(
            [
                html.Thead(html.Tr([html.Th(h) for h in headers])),
                html.Tbody([html.Tr([html.Td(v) for v in row]) for row in rows]),
            ]
        ),
        className='evidence-table',
    )


def results_view(backend, run_id, *, offset=0, result=None):
    """Present bounded original evidence in the compact results grid."""
    if not run_id:
        return html.P(
            'Select a recorded Run from the current Session.', className='empty-state'
        )
    result = result if result is not None else backend.get_run_results(run_id)
    status = result['status']
    safety = status.get('safety_status') or {}
    document = result.get('metrics') or {}
    rows = metric_rows(result)
    page = backend.read_time_series(run_id, offset=offset, limit=1000)
    figures = recorded_figures(result, page)
    telemetry = {}
    unavailable = []
    for stream in ('target_truth', 'target_estimate', 'prediction', 'control'):
        try:
            telemetry[stream] = backend.read_telemetry(
                run_id, stream, offset=offset, limit=20 if stream == 'control' else 200
            )
        except FileNotFoundError:
            continue
        except (ValueError, TypeError, OSError) as error:
            unavailable.append(f'{stream}: {error}')
    trajectory, note = trajectory_figure(telemetry)

    def plot(figure, title):
        return (
            dcc.Graph(
                figure=figure,
                config={'displaylogo': False, 'responsive': True},
                responsive=True,
                className='result-graph',
                style={'height': '100%', 'minHeight': 0},
            )
            if figure
            else html.Div(
                html.P('Unavailable: no recorded numeric samples on this page.'),
                className='empty-state',
            )
        )

    return html.Div(
        [
            html.Div(kpi_cards(result), className='kpi-grid'),
            html.Section(
                [
                    html.H2('Position Error'),
                    plot(figures.get('tracking_distance'), 'Position Error'),
                ],
                className='panel position-panel',
                title=(
                    'Absolute radial distance-to-reference error; '
                    'not general 3D position error.'
                ),
            ),
            html.Section(
                [
                    html.H2('Orientation Error'),
                    plot(figures.get('pointing_error'), 'Orientation Error'),
                ],
                className='panel orientation-panel',
                title='Recorded target pointing angle; not full 3D attitude error.',
            ),
            html.Section(
                [
                    html.H2('Truth vs Estimate vs Prediction'),
                    plot(trajectory, 'Truth vs Estimate vs Prediction'),
                ],
                className='panel trajectory-panel',
                title=note,
            ),
            html.Div(
                [
                    html.Span('Execution: ' + str(status.get('execution_status', 'unknown'))),
                    html.Span('Run acceptance: ' + outcome(status.get('acceptance'))),
                    html.Span('Recorded safety: ' + outcome(safety.get('accepted'))),
                    (
                        html.Span(
                            status['error'].get('code', 'Recorded execution error'),
                            className='status-error',
                            title=json.dumps(status['error']),
                        )
                        if isinstance(status.get('error'), dict)
                        else None
                    ),
                    html.Details(
                        [
                            html.Summary('Advanced / Diagnostics'),
                            html.Div(
                                [
                                    html.P('Recorded Run ' + run_id),
                                    html.P(note),
                                    html.Details(
                                        [
                                            html.Summary('Control computation time'),
                                            plot(
                                                figures.get('control_compute_time'),
                                                'Control computation time',
                                            ),
                                            html.P(
                                                'Recorded computation time [s]. '
                                                'Base/arm allocation is not scored.'
                                            ),
                                        ]
                                    ),
                                    html.P(
                                        'Recorded evaluation window: '
                                        + json.dumps(document.get('evaluation_window'))
                                    ),
                                    html.P(
                                        'Scored page offset '
                                        + str(offset)
                                        + ': up to 1000 samples/series. '
                                        'Raw trajectory pages: up to 200 messages/stream; '
                                        'independent spans.'
                                    ),
                                    html.P(
                                        json.dumps(status.get('error'))
                                        if status.get('error')
                                        else 'No recorded execution error.'
                                    ),
                                    html.P(
                                        'Unavailable: '
                                        + str(
                                            document.get(
                                                'reason', 'metric/stream gaps remain explicit.'
                                            )
                                        )
                                    ),
                                    *[html.P(message) for message in unavailable],
                                    table(
                                        [
                                            'Recorded metric',
                                            'Availability',
                                            'Unit',
                                            'Mean',
                                            'Max',
                                            'Samples',
                                        ],
                                        [
                                            [
                                                label(r['metric']),
                                                (
                                                    'Available'
                                                    if r.get('available')
                                                    else 'Unavailable'
                                                ),
                                                r.get('unit', 'not recorded'),
                                                (
                                                    display(r.get('mean'))
                                                    if r.get('available')
                                                    else 'Unavailable'
                                                ),
                                                (
                                                    display(r.get('max'))
                                                    if r.get('available')
                                                    else 'Unavailable'
                                                ),
                                                display(r.get('sample_count')),
                                            ]
                                            for r in rows
                                        ],
                                    ),
                                    html.Details(
                                        [
                                            html.Summary(
                                                'Metric contracts, coverage, safety and cleanup'
                                            ),
                                            html.Pre(
                                                json.dumps(
                                                    {
                                                        'metrics': document,
                                                        'safety': safety,
                                                        'cleanup': status.get('cleanup'),
                                                    },
                                                    indent=2,
                                                ),
                                                className='json-preview',
                                            ),
                                        ]
                                    ),
                                    html.Details(
                                        [
                                            html.Summary(
                                                'Control telemetry page · '
                                                'unscored raw observations'
                                            ),
                                            html.Pre(
                                                json.dumps(
                                                    telemetry.get(
                                                        'control', {'available': False}
                                                    ),
                                                    indent=2,
                                                ),
                                                className='json-preview',
                                            ),
                                        ]
                                    ),
                                    html.Details(
                                        [
                                            html.Summary('Original evidence references'),
                                            html.Pre(
                                                json.dumps(result['artifacts'], indent=2),
                                                className='json-preview',
                                            ),
                                        ]
                                    ),
                                ],
                                className='evidence-popover',
                            ),
                        ]
                    ),
                ],
                className='result-statusbar',
            ),
        ],
        className='result-view',
    )


def _rmse(metric, *, angle=False):
    """Read RMSE only; convert declared radians to degrees without recomputing errors."""
    value = _scalar(metric, 'rmse')
    if value is None:
        return None
    unit = metric.get('unit')
    if angle:
        if unit == 'rad':
            return math.degrees(value)
        return value if unit in ('deg', '°') else None
    return value if unit == 'm' else None


def comparison_view(comparison, *, selected_run=None, names=None):
    """Show recorded RMSE and rank only backend-approved, fully evidenced comparisons."""
    contract = comparison.get('comparability') or {}
    runs = comparison.get('runs', [])
    values = []
    for run in runs:
        metrics = {row['metric']: row for row in metric_rows(run)}
        values.append(
            [
                _rmse(metrics.get(name, {}), angle=name == 'pointing_error')
                for name in (
                    'tracking_distance',
                    'pointing_error',
                    'target_position_error',
                    'prediction_rmse',
                )
            ]
        )
    rankable = (
        contract.get('comparable') is True
        and len(runs) >= 2
        and all(row[0] is not None for row in values)
    )
    criterion = (
        'Rank by recorded radial distance-to-reference RMSE (m), lower is better. '
        'Equal recorded values share a rank. Requires backend scientific comparability '
        'and valid radial RMSE for every Run.'
    )
    ranks = (
        {value: 1 + sum(row[0] < value for row in values) for value in {r[0] for r in values}}
        if rankable
        else {}
    )
    reasons = contract.get('reasons') or []
    explanation = '; '.join(str(reason) for reason in reasons)
    if not rankable:
        explanation = 'Rank unavailable. ' + (
            explanation or 'Comparable Runs with valid recorded radial RMSE are required.'
        )
    rows = []
    for run, recorded in zip(runs, values):
        rank = ranks.get(recorded[0])
        rank_class = (
            'rank-good'
            if rank == 1
            else (
                'rank-mid'
                if rank is not None and rank < len(runs)
                else 'rank-low' if rank is not None else 'rank-unavailable'
            )
        )
        rows.append(
            html.Tr(
                [
                    html.Td(
                        html.Button(
                            scenario_label(run, names),
                            id={'type': 'session-run', 'run_id': run['run_id']},
                            n_clicks=0,
                            className='run-link',
                            title='Inspect recorded Run',
                        )
                    ),
                    *[
                        html.Td(f'{value:.3f}' if value is not None else 'Unavailable')
                        for value in recorded
                    ],
                    html.Td(
                        str(rank) if rank is not None else 'Unavailable',
                        className=rank_class,
                        title=criterion if rankable else explanation,
                    ),
                ],
                **{'data-run-id': run['run_id']},
                className='selected-run' if run['run_id'] == selected_run else '',
            )
        )
    headers = [
        'Scenario',
        'R RMSE (m)',
        'Angle RMSE (°)',
        'Estimator RMSE (m)',
        'Predictor RMSE (m)',
        'Rank',
    ]
    return html.Div(
        [
            (
                html.Div(
                    html.Table(
                        [
                            html.Thead(
                                html.Tr(
                                    [
                                        html.Th(
                                            text, title=criterion if text == 'Rank' else text
                                        )
                                        for text in headers
                                    ]
                                )
                            ),
                            html.Tbody(rows),
                        ]
                    ),
                    className='evidence-table session-table',
                )
                if rows
                else html.P('No recorded Runs in this Session.', className='empty-state')
            ),
            html.Details(
                [
                    html.Summary('Comparison details'),
                    html.Div(
                        [
                            html.P(criterion),
                            html.P(explanation) if explanation else None,
                            html.Pre(json.dumps(contract, indent=2), className='json-preview'),
                        ],
                        className='comparison-popover',
                    ),
                ],
                className='comparison-details',
            ),
        ],
        className='comparison-view',
    )
