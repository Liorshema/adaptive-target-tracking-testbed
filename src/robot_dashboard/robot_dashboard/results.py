"""Present recorded Run evidence without calculating scientific metrics."""

import json
import math

from dash import Input, Output, State, ctx, dcc, no_update
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
        xaxis={
            'gridcolor': '#173246',
            'zerolinecolor': '#234358',
            'title_font_size': 14,
        },
        yaxis={
            'gridcolor': '#173246',
            'zerolinecolor': '#234358',
            'title_font_size': 14,
        },
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
    for item in backend.list_scenarios(include_deleted=True):
        origin = item.get('origin') or {}
        parent = item.get('source_id') or origin.get('id')
        if parent:
            names[parent, item['variant']] = item['name']
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
    if names and names.get((identifier, result.get('variant'))):
        return names[identifier, result['variant']]
    if names and names.get(identifier):
        return names[identifier]
    identity = result.get('experiment_identity') or {}
    configuration = result.get('configuration') or {}
    experiment = configuration.get('experiment') or {}
    if isinstance(experiment.get('experiment'), dict):
        experiment = experiment['experiment']
    for value in (
        result.get('experiment_name'),
        identity.get('name'),
        experiment.get('name'),
    ):
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


def _padded_range(values):
    """Fit genuine samples with a small margin, including stationary trajectories."""
    lower, upper = min(values), max(values)
    padding = (upper - lower) * 0.08 if upper > lower else max(abs(lower) * 0.02, 0.05)
    return [lower - padding, upper + padding]


def trajectory_figure(pages, mode='2d', run_id=None):
    """Plot independently recorded XYZ with explicit coverage and forecast provenance."""
    mode = '3d' if mode == '3d' else '2d'
    figure = go.Figure()
    frame = None
    notes = ['Independent recorded XYZ; no alignment or interpolation.']
    pages = pages if isinstance(pages, dict) else {}
    for stream, title, color, field in [
        ('target_truth', 'Truth', '#d5e6f3', 'position'),
        ('target_estimate', 'Estimate', '#49adff', 'mean'),
        ('prediction', 'Prediction', '#4de4a0', 'states'),
    ]:
        page = pages.get(stream)
        records = page.get('records', []) if isinstance(page, dict) else []
        records = records[:TRAJECTORY_POINT_LIMIT] if isinstance(records, list) else []
        if stream == 'prediction':
            records = records[-1:]
        valid, sources, invalid = [], set(), 0
        for record in records:
            source, samples, rejected = _record_samples(record, stream)
            if source:
                sources.add(source)
            valid.extend(samples)
            invalid += rejected
        coverage = page.get('coverage', {}) if isinstance(page, dict) else {}
        sources.update(coverage.get('frames', []))
        # Refuse the whole stream if its page changes frame; never plot a valid-looking prefix.
        if len(sources) > 1 or (frame is not None and sources - {frame}):
            notes.append(
                f'{title} unavailable: incompatible frames {", ".join(sorted(sources))}'
                + (f' (shared frame {frame}).' if frame else '.')
            )
            continue
        if invalid:
            notes.append(
                f'{title}: invalid or missing coordinates, frame, units or epochs omitted.'
            )
        if not valid:
            notes.append(f'{title}: unavailable in the selected recording interval.')
            continue
        frame = next(iter(sources))
        original_count = len(valid)
        valid = _display_samples(valid)
        coordinates = {
            axis: [sample[0][index] for sample in valid]
            for index, axis in enumerate(('x', 'y', 'z'))
        }
        record = valid[-1][2]
        meta = {
            'frame_id': frame,
            'position_unit': 'm',
            'time_unit': 's',
            'source_artifact': page.get('source'),
            'raw_offset': page.get('offset'),
            'coverage': coverage,
            'plotted_count': len(valid),
        }
        if coverage.get('complete_scan') and stream != 'prediction':
            notes.append(
                f'{title}: complete recorded interval scan; '
                f'{len(valid)}/{coverage.get("valid_count", original_count)} samples plotted'
                + (' (sampled).' if coverage.get('sampled') else '.')
            )
        elif not coverage.get('complete_scan'):
            notes.append(f'{title}: partial raw page; completeness unknown.')
        epoch_label = 'source epoch'
        if stream == 'prediction':
            meta.update(
                source_epoch=record['source_time'],
                horizon=record['data'].get('horizon'),
                recorded_count=original_count,
            )
            epoch_label = 'forecast epoch'
            notes.append(
                f'Prediction: one origin at {record["source_time"]} s; '
                f'{len(valid)}/{record["data"].get("horizon", "unknown")} states plotted. '
                f'{coverage.get("message_count", "Unknown number of")} origins recorded '
                'in the selected interval; history completeness before/after recording is unknown.'
            )
        hover = 'X %{x} m<br>Y %{y} m'
        if mode == '3d':
            hover += '<br>Z %{z} m'
        hover += '<br>' + epoch_label + ' %{customdata} s'
        if stream == 'prediction':
            hover += '<br>message source epoch %{meta.source_epoch} s'
        trace = go.Scatter3d if mode == '3d' else go.Scatter
        figure.add_trace(
            trace(
                **{
                    axis: values
                    for axis, values in coordinates.items()
                    if mode == '3d' or axis != 'z'
                },
                customdata=[sample[1] for sample in valid],
                meta=meta,
                mode='markers',
                marker={'size': 3 if stream != 'prediction' else 4, 'color': color},
                name=title,
                hovertemplate=hover + '<extra>%{fullData.name}</extra>',
            )
        )
    if not figure.data:
        return (
            None,
            'Unavailable: no compatible recorded target XYZ telemetry. '
            + ' '.join(dict.fromkeys(notes)),
        )
    style_figure(figure, '', f'X [m] · {frame}', 'Y [m]')
    revision = f'{run_id or "unselected"}:trajectory:{mode}'
    selected = pages.get('prediction', {}).get('coverage', {}).get('selected_index')
    if selected is not None:
        revision += f':forecast-{selected}'
    figure.update_layout(
        uirevision=revision,
        margin={'l': 42, 'r': 8, 't': 20, 'b': 32},
        legend={
            'orientation': 'h',
            'y': 1.0,
            'yanchor': 'bottom',
            'x': 0,
            'font': {'size': 12},
            'itemsizing': 'constant',
        },
    )
    if mode == '3d':
        figure.update_layout(
            margin={'l': 0, 'r': 0, 't': 20, 'b': 0},
            annotations=[
                {
                    'text': f'Frame: {frame} · m',
                    'xref': 'paper',
                    'yref': 'paper',
                    'x': 1,
                    'y': 1,
                    'xanchor': 'right',
                    'showarrow': False,
                    'font': {'size': 13, 'color': '#9bb6ca'},
                }
            ],
            scene={
                'uirevision': revision,
                'aspectmode': 'data',
                'dragmode': 'orbit',
                'bgcolor': '#0B1E2D',
                'camera': {
                    'up': {'x': 0, 'y': 0, 'z': 1},
                    'eye': {'x': 1.75, 'y': 1.75, 'z': 1.35},
                },
                **{
                    axis
                    + 'axis': {
                        'title': f'{axis.upper()} [m]',
                        'range': _padded_range(
                            [v for t in figure.data for v in getattr(t, axis)]
                        ),
                        'gridcolor': '#294256',
                        'zerolinecolor': '#345267',
                        'backgroundcolor': '#0B1E2D',
                        'linecolor': '#345267',
                        'tickfont': {'size': 13},
                        'title_font': {'size': 13},
                    }
                    for axis in ('x', 'y', 'z')
                },
            },
        )
    else:
        figure.update_xaxes(
            range=_padded_range([v for t in figure.data for v in t.x]),
            constrain='domain',
            nticks=3,
            tickangle=0,
            title_font_size=13,
        )
        figure.update_yaxes(
            range=_padded_range([v for t in figure.data for v in t.y]),
            scaleanchor='x',
            scaleratio=1,
            constrain='domain',
            nticks=5,
            title_font_size=13,
        )
    notes.append(
        'Prediction is one selected recorded forecast, never joined to other origins; '
        'hover shows recorded source/forecast epoch [s].'
    )
    return figure, ' '.join(dict.fromkeys(notes))


TRAJECTORY_POINT_LIMIT = 4000


def _display_samples(samples):
    """Bound a recorded horizon without interpolation or losing its actual endpoints."""
    if len(samples) <= TRAJECTORY_POINT_LIMIT:
        return samples
    step = math.ceil((len(samples) - 1) / (TRAJECTORY_POINT_LIMIT - 1))
    kept = samples[::step]
    if (len(samples) - 1) % step:
        kept.append(samples[-1])
    return kept


def _record_samples(record, stream):
    """Validate original XYZ, source epochs and units without transforming any values."""
    if not isinstance(record, dict):
        return None, [], 1
    source, units, data = record.get('frame_id'), record.get('units'), record.get('data')
    if (
        record.get('schema_version') != 1
        or not isinstance(source, str)
        or not source.strip()
        or not isinstance(units, dict)
        or units.get('position') != 'm'
        or units.get('source_time') != 's'
        or not isinstance(data, dict)
        or not _finite(record.get('source_time'))
    ):
        return None, [], 1
    if stream == 'prediction':
        vectors, epochs = data.get('states'), data.get('timestamps')
    else:
        vectors = [data.get('position' if stream == 'target_truth' else 'mean')]
        epochs = [record['source_time']]
    if (
        not isinstance(vectors, list)
        or not isinstance(epochs, list)
        or len(vectors) != len(epochs)
    ):
        return source, [], 1
    valid, invalid = [], 0
    for vector, epoch in zip(vectors, epochs):
        if (
            not isinstance(vector, list)
            or len(vector) < 3
            or not all(_finite(v) for v in vector[:3])
            or not _finite(epoch)
        ):
            invalid += 1
        else:
            valid.append((vector[:3], epoch, record))
    return source, valid, invalid


def read_trajectory_pages(
    backend, run_id, offset=0, *, prediction_index=None, max_points=TRAJECTORY_POINT_LIMIT
):
    """Scan the selected recorded tail in bounded pages; bound display memory, not time."""
    if type(max_points) is not int or not 2 <= max_points <= TRAJECTORY_POINT_LIMIT:
        raise ValueError('trajectory display limit must be an integer in [2, 4000]')
    pages, unavailable = {}, []
    offset = int(offset) if _finite(offset) and offset >= 0 and int(offset) == offset else 0
    if not run_id:
        return pages, unavailable
    for stream in ('target_truth', 'target_estimate', 'prediction'):
        kept, last, stride, total, valid_count, invalid_count = [], None, 1, 0, 0, 0
        frames, source, selected = set(), None, None
        first_epoch = last_epoch = None
        try:
            for page in backend.iter_telemetry(run_id, stream, offset=offset, limit=1000):
                source = page['source']
                for record in page['records']:
                    index = offset + total
                    total += 1
                    if stream == 'prediction':
                        if prediction_index is None or index == prediction_index:
                            selected = (index, record)
                        continue
                    frame, samples, invalid = _record_samples(record, stream)
                    invalid_count += invalid
                    if frame:
                        frames.add(frame)
                    if not samples:
                        continue
                    epoch = samples[0][1]
                    if first_epoch is None:
                        first_epoch = epoch
                    last_epoch = epoch
                    last = (valid_count, record)
                    if valid_count % stride == 0:
                        kept.append(last)
                    valid_count += 1
                    if len(kept) > max_points:
                        stride *= 2
                        kept = [(i, r) for i, r in kept if i % stride == 0]
            if stream == 'prediction':
                records = [selected[1]] if selected else []
                coverage = {
                    'message_count': total,
                    'selected_index': selected[0] if selected else None,
                }
            else:
                if last is not None and (not kept or kept[-1][0] != last[0]):
                    if len(kept) == max_points:
                        kept.pop()
                    kept.append(last)
                records = [r for _, r in kept]
                coverage = {
                    'record_count': total,
                    'valid_count': valid_count,
                    'invalid_count': invalid_count,
                    'frames': sorted(frames),
                    'first_epoch': first_epoch,
                    'last_epoch': last_epoch,
                    'sampled': len(records) < valid_count,
                }
            pages[stream] = {
                'records': records,
                'source': source,
                'offset': offset,
                'coverage': {**coverage, 'complete_scan': True},
            }
        except FileNotFoundError:
            continue
        except (ValueError, TypeError, OSError) as error:
            # A malformed line or failed scan cannot be presented as a complete trace.
            unavailable.append(f'{stream}: {error}')
            pages[stream] = {'records': [], 'coverage': {'complete_scan': False}}
    return pages, unavailable


def trajectory_summary(pages):
    """Visible sample coverage and one forecast's actual origin/recorded horizon."""
    parts, frame = [], None
    for stream, title in [('target_truth', 'Truth'), ('target_estimate', 'Estimate')]:
        page = pages.get(stream, {})
        cov = page.get('coverage', {})
        if not cov.get('complete_scan'):
            parts.append(f'{title} unavailable')
            continue
        count, valid = len(page.get('records', [])), cov.get('valid_count', 0)
        frames = set(cov.get('frames', []))
        if len(frames) > 1 or (frame is not None and frames - {frame}):
            parts.append(f'{title} unavailable (incompatible frames)')
            continue
        if frames:
            frame = next(iter(frames))
        text = f'{title} {count:,}/{valid:,}'
        if cov.get('sampled'):
            text += ' sampled'
        if cov.get('invalid_count'):
            text += f' ({cov["invalid_count"]} invalid omitted)'
        parts.append(text)
    prediction = pages.get('prediction', {})
    records = prediction.get('records', [])
    cov = prediction.get('coverage', {})
    if records and isinstance(records[0], dict):
        record = records[0]
        source, valid, invalid = _record_samples(record, 'prediction')
        if source is not None and frame is not None and source != frame:
            return ' · '.join(parts + ['Prediction unavailable (incompatible frames)'])
        data = record.get('data')
        expected = data.get('horizon') if isinstance(data, dict) else None
        count = len(valid)
        shown = len(_display_samples(valid))
        completeness = (
            'recorded'
            if type(expected) is int and count == expected and not invalid
            else 'partial/unknown'
        )
        span = f'{valid[0][1]:.3f}–{valid[-1][1]:.3f} s' if valid else 'no valid horizon'
        parts.append(
            f't₀ {display(record.get("source_time"))} s · {count}/{display(expected)} '
            f'{completeness} · {span}'
            + (f' · {shown} plotted (sampled)' if shown < count else '')
        )
    else:
        parts.append('Prediction unavailable')
    return ' · '.join(parts)


def trajectory_config(mode='2d'):
    """Keep zoom/reset controls visible and label them through Plotly's native toolbar."""
    return {
        'displaylogo': False,
        'responsive': True,
        'displayModeBar': True,
        'scrollZoom': True,
        'modeBarButtonsToAdd': (
            ['orbitRotation', 'tableRotation', 'resetCameraDefault3d']
            if mode == '3d'
            else ['zoomIn2d', 'zoomOut2d', 'resetScale2d']
        ),
    }


def _trajectory_display(figure, run_id, mode='2d'):
    """Keep the scoped Graph mounted even when this Run has no usable telemetry."""
    if figure is not None:
        return figure
    figure = style_figure(go.Figure(), '', 'X [m]', 'Y [m]')
    figure.update_layout(
        uirevision=f'{run_id or "unselected"}:trajectory:{mode}',
        annotations=[
            {
                'text': 'Unavailable: no compatible recorded trajectory.',
                'xref': 'paper',
                'yref': 'paper',
                'x': 0.5,
                'y': 0.5,
                'showarrow': False,
                'font': {'size': 13},
            }
        ],
        xaxis={'visible': False},
        yaxis={'visible': False},
    )
    return figure


def _forecast_index(value):
    """Convert the visible one-based message index without inventing an origin."""
    return int(value) - 1 if _finite(value) and value >= 1 and int(value) == value else None


def forecast_controls(pages, *, expanded=False):
    """Inspect any independently recorded forecast with a bounded scalar selector."""
    cov = pages.get('prediction', {}).get('coverage', {})
    count = cov.get('message_count', 0)
    offset = pages.get('prediction', {}).get('offset', 0)
    selected = cov.get('selected_index')
    prefix = 'trajectory-expanded' if expanded else 'trajectory'
    return html.Div(
        [
            html.Label(
                'Forecast',
                htmlFor=prefix + '-origin',
                title='Select one independently recorded forecast origin',
            ),
            dcc.Input(
                id=prefix + '-origin',
                type='number',
                min=offset + 1,
                max=offset + count if count else offset + 1,
                step=1,
                value=selected + 1 if selected is not None else None,
                disabled=not count,
                debounce=True,
            ),
            html.Span(
                f'/ {offset + count}', title=f'{count} forecasts in the selected interval'
            ),
            html.Small(trajectory_summary(pages), id=prefix + '-summary'),
        ],
        className='trajectory-coverage',
        title='Full recorded interval from the selected raw offset; '
        'counts are displayed/valid samples. '
        'First/last actual samples are retained when display sampling is necessary.',
    )


def register_trajectory(app, backend):
    """Reuse one bounded recorded-data pipeline for panel, history and expanded views."""

    @app.callback(
        Output('trajectory-graph', 'figure'),
        Output('trajectory-graph', 'config'),
        Output('trajectory-panel-heading', 'title'),
        Output('trajectory-summary', 'children'),
        Input('trajectory-mode', 'value'),
        Input('trajectory-origin', 'value'),
        State('results-run', 'value'),
        State('result-offset', 'value'),
        prevent_initial_call=True,
    )
    def change_trajectory_mode(mode, origin, run_id, offset):
        mode = '3d' if mode == '3d' else '2d'
        pages, errors = read_trajectory_pages(
            backend, run_id, offset, prediction_index=_forecast_index(origin)
        )
        figure, note = trajectory_figure(pages, mode=mode, run_id=run_id)
        return (
            _trajectory_display(figure, run_id, mode),
            trajectory_config(mode),
            note + ' '.join(errors),
            trajectory_summary(pages),
        )

    @app.callback(
        Output('trajectory-overlay', 'hidden'),
        Output('trajectory-expanded-content', 'children'),
        Output('trajectory-expanded-mode', 'value'),
        Output('trajectory-expanded-origin', 'value'),
        Output('trajectory-origin', 'value'),
        Output('trajectory-expanded-summary', 'children'),
        Input('trajectory-expand', 'n_clicks'),
        Input('trajectory-close', 'n_clicks'),
        Input('trajectory-expanded-mode', 'value'),
        Input('trajectory-expanded-origin', 'value'),
        State('results-run', 'value'),
        State('result-offset', 'value'),
        State('trajectory-mode', 'value'),
        State('trajectory-overlay', 'hidden'),
        State('trajectory-origin', 'value'),
        prevent_initial_call=True,
    )
    def expand_trajectory(
        _expand, _close, expanded_mode, expanded_origin, run_id, offset, mode, hidden, origin
    ):
        if ctx.triggered_id == 'trajectory-close':
            return True, None, no_update, no_update, no_update, no_update
        if not run_id or (ctx.triggered_id != 'trajectory-expand' and hidden):
            return (no_update,) * 6
        opening = ctx.triggered_id == 'trajectory-expand'
        mode = mode if opening else expanded_mode
        mode = '3d' if mode == '3d' else '2d'
        selected = origin if opening else expanded_origin
        pages, _ = read_trajectory_pages(
            backend, run_id, offset, prediction_index=_forecast_index(selected)
        )
        figure, _ = trajectory_figure(pages, mode=mode, run_id=run_id)
        graph = dcc.Graph(
            id='trajectory-expanded-graph',
            figure=_trajectory_display(figure, run_id, mode),
            config=trajectory_config(mode),
            responsive=True,
            className='trajectory-expanded-graph',
            style={'height': '100%', 'width': '100%', 'minHeight': 0},
        )
        return (
            False,
            graph,
            mode,
            selected,
            selected if selected != origin else no_update,
            trajectory_summary(pages),
        )


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
    telemetry, unavailable = read_trajectory_pages(backend, run_id, offset)
    try:
        telemetry['control'] = backend.read_telemetry(
            run_id, 'control', offset=offset, limit=20
        )
    except FileNotFoundError:
        pass
    except (ValueError, TypeError, OSError) as error:
        unavailable.append(f'control: {error}')
    trajectory, note = trajectory_figure(telemetry, run_id=run_id)

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
                    html.Div(
                        [
                            html.H2(
                                'Truth vs Estimate vs Prediction',
                                id='trajectory-panel-heading',
                                title=note,
                            ),
                            dcc.RadioItems(
                                id='trajectory-mode',
                                options=[
                                    {'label': '2D', 'value': '2d'},
                                    {'label': '3D', 'value': '3d'},
                                ],
                                value='2d',
                                inline=True,
                                className='trajectory-controls',
                            ),
                            html.Button(
                                '⤢',
                                id='trajectory-expand',
                                n_clicks=0,
                                title='Expand trajectory',
                                className='trajectory-expand',
                                **{'aria-label': 'Expand trajectory'},
                            ),
                        ],
                        className='panel-title trajectory-heading',
                    ),
                    dcc.Graph(
                        id='trajectory-graph',
                        figure=_trajectory_display(trajectory, run_id),
                        config=trajectory_config(),
                        responsive=True,
                        className='result-graph',
                        style={'height': '100%', 'minHeight': 0},
                    ),
                    forecast_controls(telemetry),
                ],
                className='panel trajectory-panel',
                title=note,
            ),
            html.Div(
                html.Section(
                    [
                        html.Div(
                            [
                                html.H2(
                                    'Truth vs Estimate vs Prediction',
                                    id='trajectory-expanded-title',
                                ),
                                dcc.RadioItems(
                                    id='trajectory-expanded-mode',
                                    options=[
                                        {'label': '2D', 'value': '2d'},
                                        {'label': '3D', 'value': '3d'},
                                    ],
                                    value='2d',
                                    inline=True,
                                    className='trajectory-controls',
                                ),
                                html.Button(
                                    '×',
                                    id='trajectory-close',
                                    n_clicks=0,
                                    title='Close expanded trajectory',
                                    className='trajectory-close',
                                    **{'aria-label': 'Close expanded trajectory'},
                                ),
                            ],
                            className='trajectory-overlay-heading',
                        ),
                        html.Div(id='trajectory-expanded-content'),
                        forecast_controls(telemetry, expanded=True),
                    ],
                    className='trajectory-overlay-panel',
                ),
                id='trajectory-overlay',
                className='trajectory-overlay',
                hidden=True,
                role='dialog',
                **{
                    'aria-modal': 'true',
                    'aria-labelledby': 'trajectory-expanded-title',
                },
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
                                        'Trajectories scan the full remaining recorded '
                                        'interval in 1000-message pages, with up to 4000 '
                                        'genuine samples/trace and retained endpoints. '
                                        'Visible counts disclose display sampling. '
                                        'Forecast indices select independent recorded '
                                        'origins, not one continuous predicted path.'
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
                                                'reason',
                                                'metric/stream gaps remain explicit.',
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
    """Show each Run's recorded RMSE in Session order without ranking presentation."""
    rows = []
    for run in comparison.get('runs', []):
        metrics = {row['metric']: row for row in metric_rows(run)}
        recorded = [
            _rmse(metrics.get(name, {}), angle=name == 'pointing_error')
            for name in (
                'tracking_distance',
                'pointing_error',
                'target_position_error',
                'prediction_rmse',
            )
        ]
        label = scenario_label(run, names)
        if run.get('condition_id') and run.get('variant'):
            label += ' · ' + str(run['variant'])
        rows.append(
            html.Tr(
                [
                    html.Td(
                        html.Button(
                            label,
                            id={'type': 'session-run', 'run_id': run['run_id']},
                            n_clicks=0,
                            className='run-link',
                            title='Inspect recorded Run.',
                        )
                    ),
                    *[
                        html.Td(f'{value:.3f}' if value is not None else 'Unavailable')
                        for value in recorded
                    ],
                ],
                **{'data-run-id': run['run_id']},
                className='selected-run' if run['run_id'] == selected_run else '',
            )
        )
    headers = ['Scenario', 'R (m)', 'Angle (°)', 'Estimator (m)', 'Predictor (m)']
    return html.Div(
        [
            html.Div(
                html.Table(
                    [
                        html.Thead(html.Tr([html.Th(text, title=text) for text in headers])),
                        html.Tbody(rows),
                    ]
                ),
                className='evidence-table session-table',
            )
            if rows
            else html.P('No recorded Runs in this Session.', className='empty-state')
        ],
        className='comparison-view',
    )
