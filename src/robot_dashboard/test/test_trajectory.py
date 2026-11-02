"""Trajectory presentation preserves bounded recorded XYZ evidence and frame semantics."""

import os
from copy import deepcopy
from pathlib import Path

from dash import Dash, dcc, html

import pytest

from robot_dashboard import results

from robot_experiments.backend import ExperimentBackend


@pytest.fixture
def trajectory_evidence(tmp_path):
    """Read retained Run evidence through the existing bounded backend API."""
    source = os.environ.get('ROBOT_DASHBOARD_RECORDED_RUN')
    if not source:
        pytest.skip('Set ROBOT_DASHBOARD_RECORDED_RUN')
    path = Path(source).resolve()
    import json

    run_id = json.loads((path / 'status.json').read_text())['run_id']
    roots = [path.parent.parent]
    failed = os.environ.get('ROBOT_DASHBOARD_FAILED_RUN')
    if failed:
        roots.append(Path(failed).resolve().parent.parent)
    backend = ExperimentBackend(tmp_path, result_roots=roots)
    pages = {
        stream: backend.read_telemetry(run_id, stream, offset=0, limit=200)
        for stream in ('target_truth', 'target_estimate', 'prediction')
    }
    return backend, run_id, pages


def test_3d_uses_exact_retained_xyz_and_last_recorded_horizon(trajectory_evidence):
    """Flattening Z or substituting an earlier/synthetic forecast loses genuine evidence."""
    _, run_id, pages = trajectory_evidence
    figure, note = results.trajectory_figure(pages, mode='3d', run_id=run_id)
    assert [trace.type for trace in figure.data] == ['scatter3d'] * 3
    for trace, stream, field in zip(
        figure.data, ('target_truth', 'target_estimate'), ('position', 'mean')
    ):
        records = pages[stream]['records']
        for axis, index in (('x', 0), ('y', 1), ('z', 2)):
            assert list(getattr(trace, axis)) == [r['data'][field][index] for r in records]
        assert list(trace.customdata) == [r['source_time'] for r in records]
    prediction = pages['prediction']['records'][-1]
    trace = figure.data[-1]
    for axis, index in (('x', 0), ('y', 1), ('z', 2)):
        assert list(getattr(trace, axis)) == [v[index] for v in prediction['data']['states']]
    assert list(trace.customdata) == prediction['data']['timestamps']
    assert trace.meta['source_epoch'] == prediction['source_time']
    assert trace.meta['horizon'] == prediction['data']['horizon']
    assert figure.layout.scene.aspectmode == 'data'
    assert figure.layout.scene.zaxis.title.text == 'Z [m]'
    assert figure.layout.scene.xaxis.title.text == 'X [m]'
    assert any(prediction['frame_id'] in a.text for a in figure.layout.annotations)
    assert 'independent' in note.lower() and 'epoch' in note.lower()
    assert 'interpolation' in note.lower()


def test_2d_ranges_fit_recorded_points_and_equal_physical_scaling(trajectory_evidence):
    """Fixed or zero-anchored ranges waste space; an unconstrained aspect distorts paths."""
    _, run_id, pages = trajectory_evidence
    figure, _ = results.trajectory_figure(pages, mode='2d', run_id=run_id)
    assert figure.layout.yaxis.scaleanchor == 'x'
    assert figure.layout.yaxis.scaleratio == 1
    assert figure.layout.xaxis.constrain == 'domain'
    assert figure.layout.yaxis.constrain == 'domain'
    for axis in ('x', 'y'):
        samples = [value for trace in figure.data for value in getattr(trace, axis)]
        lower, upper = getattr(figure.layout, axis + 'axis').range
        assert lower < min(samples) <= max(samples) < upper
        assert upper - lower < 1.3 * (max(samples) - min(samples))
    again, _ = results.trajectory_figure(pages, mode='3d', run_id=run_id)
    other, _ = results.trajectory_figure(pages, mode='2d', run_id='different-run')
    assert (
        len({figure.layout.uirevision, again.layout.uirevision, other.layout.uirevision}) == 3
    )


@pytest.mark.parametrize(
    'bad', [None, [], [1, 2], [1, 2, float('nan')], [1, 2, float('inf')], [True, 2, 3]]
)
def test_missing_or_nonfinite_xyz_never_becomes_a_fabricated_3d_sample(
    trajectory_evidence, bad
):
    """Omit invalid XYZ samples without inventing coordinates."""
    _, run_id, original = trajectory_evidence
    pages = deepcopy(original)
    records = pages['target_truth']['records']
    records[0]['data']['position'] = bad
    figure, note = results.trajectory_figure(pages, mode='3d', run_id=run_id)
    assert list(figure.data[0].z) == [row['data']['position'][2] for row in records[1:]]
    assert 'invalid' in note.lower()


def test_mixed_frames_reject_entire_stream_instead_of_plotting_accepted_prefix(
    trajectory_evidence,
):
    """Reject a mixed-frame stream including its apparently valid prefix."""
    _, run_id, original = trajectory_evidence
    pages = deepcopy(original)
    pages['target_estimate']['records'][1]['frame_id'] = 'different-frame'
    figure, note = results.trajectory_figure(pages, mode='3d', run_id=run_id)
    assert [trace.name for trace in figure.data] == ['Truth', 'Prediction']
    assert 'different-frame' in note


@pytest.mark.parametrize('mutation', ['frame', 'units', 'epochs', 'message'])
def test_incompatible_or_malformed_last_prediction_is_not_replaced_with_earlier_horizon(
    trajectory_evidence, mutation
):
    """Preserve unavailable latest predictions instead of substituting older forecasts."""
    _, run_id, original = trajectory_evidence
    pages = deepcopy(original)
    prediction = pages['prediction']['records'][-1]
    if mutation == 'frame':
        prediction['frame_id'] = ''
    elif mutation == 'units':
        prediction['units']['position'] = 'cm'
    elif mutation == 'epochs':
        prediction['data']['timestamps'] = [1]
    else:
        pages['prediction']['records'][-1] = None
    figure, note = results.trajectory_figure(pages, mode='3d', run_id=run_id)
    assert [trace.name for trace in figure.data] == ['Truth', 'Estimate']
    assert 'Prediction' in note


def _walk(component):
    yield component
    children = getattr(component, 'children', None)
    if isinstance(children, (list, tuple)):
        for child in children:
            yield from _walk(child)
    elif children is not None:
        yield from _walk(children)


def test_missing_data_keeps_graph_and_default_toggle_mounted(trajectory_evidence):
    """Keep mode and graph controls usable when recorded coordinates are unavailable."""
    backend, run_id, _ = trajectory_evidence
    view = results.results_view(backend, run_id, offset=100000)
    components = list(_walk(view))
    graph = next((c for c in components if getattr(c, 'id', None) == 'trajectory-graph'), None)
    toggle = next((c for c in components if getattr(c, 'id', None) == 'trajectory-mode'), None)
    assert isinstance(graph, dcc.Graph)
    assert len(graph.figure.data) == 0
    assert isinstance(toggle, dcc.RadioItems) and toggle.value == '2d'
    assert graph.config['displayModeBar'] is True
    assert 'zoomIn2d' in graph.config['modeBarButtonsToAdd']
    assert 'resetScale2d' in graph.config['modeBarButtonsToAdd']


@pytest.mark.parametrize('offset', [3, 3.0])
def test_mode_callback_scans_bounded_pages_and_resets_run_camera(trajectory_evidence, offset):
    """Reread the selected bounded page through the scoped mode callback."""
    backend, run_id, pages = trajectory_evidence
    assert callable(getattr(results, 'register_trajectory', None))
    app = Dash(__name__)
    app.layout = html.Div(
        [
            dcc.RadioItems(id='trajectory-mode', value='2d'),
            dcc.Dropdown(id='results-run', value=run_id),
            dcc.Input(id='result-offset', value=3),
            dcc.Graph(id='trajectory-graph'),
            html.H2(id='trajectory-panel-heading'),
        ]
    )
    # The real backend remains responsible for reads; log only the requested bounds.
    original_read = backend.iter_telemetry
    reads = []

    def recording_read(rid, stream, *, offset, limit):
        reads.append((rid, stream, offset, limit))
        return original_read(rid, stream, offset=offset, limit=limit)

    backend.iter_telemetry = recording_read
    results.register_trajectory(app, backend)
    key, callback = next(iter(app.callback_map.items()))
    response = app.server.test_client().post(
        '/_dash-update-component',
        json={
            'output': key,
            'outputs': [
                {'id': o.component_id, 'property': o.component_property}
                for o in callback['output']
            ],
            'inputs': [
                {'id': 'trajectory-mode', 'property': 'value', 'value': '3d'},
                {'id': 'trajectory-origin', 'property': 'value', 'value': None},
            ],
            'state': [
                {'id': 'results-run', 'property': 'value', 'value': run_id},
                {'id': 'result-offset', 'property': 'value', 'value': offset},
            ],
            'changedPropIds': ['trajectory-mode.value'],
        },
    )
    assert response.status_code == 200, response.get_data(as_text=True)
    graph = response.get_json()['response']['trajectory-graph']
    assert graph['figure']['data'][0]['type'] == 'scatter3d'
    assert (
        graph['figure']['data'][0]['z'][0]
        == pages['target_truth']['records'][3]['data']['position'][2]
    )
    assert reads == [
        (run_id, stream, 3, 1000)
        for stream in ('target_truth', 'target_estimate', 'prediction')
    ]
    assert graph['config']['displayModeBar'] is True
    assert 'resetCameraDefault3d' in graph['config']['modeBarButtonsToAdd']
    assert results.trajectory_figure({}, mode='3d', run_id='missing')[0] is None


@pytest.mark.parametrize(
    'page',
    [
        None,
        [],
        {'records': None},
        {'records': [None]},
        {'records': [{'units': None, 'data': None}]},
    ],
)
def test_malformed_pages_return_honest_unavailable_without_crashing(page):
    """Return unavailable evidence for malformed raw telemetry pages."""
    figure, note = results.trajectory_figure({'target_truth': page}, mode='3d', run_id='bad')
    assert figure is None
    assert 'Unavailable' in note


def test_trajectory_does_not_expand_oversized_pages_or_fabricate_stationary_span(
    trajectory_evidence,
):
    """Bound raw page consumption and pad stationary axes without adding samples."""
    _, run_id, original = trajectory_evidence
    pages = deepcopy(original)
    records = pages['target_truth']['records']
    records.extend(deepcopy(records[:5]))
    figure, _ = results.trajectory_figure(pages, run_id=run_id)
    assert len(figure.data[0].x) == 205
    assert list(figure.data[0].x) == [r['data']['position'][0] for r in records]
    stationary = deepcopy(original['target_truth']['records'][0])
    stationary['data']['position'] = [2, 3, 4]
    figure, _ = results.trajectory_figure(
        {'target_truth': {'records': [stationary]}}, run_id=run_id
    )
    assert list(figure.data[0].x) == [2] and list(figure.data[0].y) == [3]
    assert figure.layout.xaxis.range[0] < 2 < figure.layout.xaxis.range[1]
    assert figure.layout.yaxis.range[0] < 3 < figure.layout.yaxis.range[1]


def test_scoped_mode_change_after_real_run_switch_clears_previous_coordinates(
    trajectory_evidence,
):
    """Replace plotted coordinates and camera identity after genuine Run switches."""
    import json

    source = os.environ.get('ROBOT_DASHBOARD_FAILED_RUN')
    if not source:
        pytest.skip('Set ROBOT_DASHBOARD_FAILED_RUN')
    backend, good, _ = trajectory_evidence
    failed = json.loads((Path(source) / 'status.json').read_text())['run_id']
    assert backend.get_run_results(failed)['status']['execution_status'] == 'failed'
    app = Dash(__name__)
    app.layout = html.Div()
    results.register_trajectory(app, backend)
    callback = next(iter(app.callback_map.values()))['callback'].__wrapped__
    successful, _, _, _ = callback('3d', None, good, 0)
    switched, _, _, _ = callback('3d', None, failed, 0)
    recorded = [
        r for p in backend.iter_telemetry(failed, 'target_truth') for r in p['records']
    ]
    assert len(successful.data) == 3
    assert list(switched.data[0].z) == [row['data']['position'][2] for row in recorded]
    assert list(switched.data[0].customdata) == [row['source_time'] for row in recorded]
    assert switched.layout.uirevision != successful.layout.uirevision
    unavailable, _, _, _ = callback('3d', None, failed, 100000)
    assert not unavailable.data
    restored, _, _, _ = callback('2d', None, good, 0)
    assert [trace.type for trace in restored.data] == ['scatter'] * 3
    assert restored.layout.uirevision != successful.layout.uirevision
    for run_id in (good, failed):
        view = results.results_view(backend, run_id)
        toggle = next(c for c in _walk(view) if getattr(c, 'id', None) == 'trajectory-mode')
        assert toggle.value == '2d'


def test_expand_controls_preserve_the_four_result_panels(trajectory_evidence):
    """Expansion must be available without adding a permanent result panel."""
    backend, run_id, _ = trajectory_evidence
    components = list(_walk(results.results_view(backend, run_id)))
    expand = next(
        (c for c in components if getattr(c, 'id', None) == 'trajectory-expand'), None
    )
    assert isinstance(expand, html.Button)
    assert expand.title == 'Expand trajectory'
    overlay = next(c for c in components if getattr(c, 'id', None) == 'trajectory-overlay')
    assert overlay.hidden is True
    assert overlay.role == 'dialog'
    assert overlay.to_plotly_json()['props']['aria-modal'] == 'true'
    assert isinstance(
        next(c for c in components if getattr(c, 'id', None) == 'trajectory-close'),
        html.Button,
    )
    assert isinstance(
        next(c for c in components if getattr(c, 'id', None) == 'trajectory-expanded-mode'),
        dcc.RadioItems,
    )
    content = next(
        c for c in components if getattr(c, 'id', None) == 'trajectory-expanded-content'
    )
    assert not content.children
    assert (
        sum(
            'trajectory-panel' in (getattr(c, 'className', '') or '').split()
            for c in components
        )
        == 1
    )


def _expanded_request(
    app,
    *,
    trigger,
    run_id,
    panel_mode='2d',
    expanded_mode='2d',
    hidden=True,
    offset=0,
    origin=None,
    expanded_origin=None,
):
    key, callback = next(
        (key, callback)
        for key, callback in app.callback_map.items()
        if 'trajectory-overlay.hidden' in key
    )
    return app.server.test_client().post(
        '/_dash-update-component',
        json={
            'output': key,
            'outputs': [
                {'id': o.component_id, 'property': o.component_property}
                for o in callback['output']
            ],
            'inputs': [
                {'id': 'trajectory-expand', 'property': 'n_clicks', 'value': 1},
                {'id': 'trajectory-close', 'property': 'n_clicks', 'value': 1},
                {
                    'id': 'trajectory-expanded-mode',
                    'property': 'value',
                    'value': expanded_mode,
                },
                {
                    'id': 'trajectory-expanded-origin',
                    'property': 'value',
                    'value': expanded_origin,
                },
            ],
            'state': [
                {'id': 'results-run', 'property': 'value', 'value': run_id},
                {'id': 'result-offset', 'property': 'value', 'value': offset},
                {'id': 'trajectory-mode', 'property': 'value', 'value': panel_mode},
                {'id': 'trajectory-overlay', 'property': 'hidden', 'value': hidden},
                {'id': 'trajectory-origin', 'property': 'value', 'value': origin},
            ],
            'changedPropIds': [trigger],
        },
    )


@pytest.mark.parametrize('mode', ['2d', '3d'])
def test_expanded_view_uses_selected_bounded_page_and_physical_geometry(
    trajectory_evidence, mode
):
    """Expansion must render the selected genuine page with the panel's physical scale."""
    backend, run_id, pages = trajectory_evidence
    app = Dash(__name__)
    app.layout = results.results_view(backend, run_id)
    results.register_trajectory(app, backend)
    response = _expanded_request(
        app,
        trigger='trajectory-expand.n_clicks',
        run_id=run_id,
        panel_mode=mode,
        offset=3,
    )
    assert response.status_code == 200, response.get_data(as_text=True)
    output = response.get_json()['response']
    assert output['trajectory-overlay']['hidden'] is False
    assert output['trajectory-expanded-mode']['value'] == mode
    graph = output['trajectory-expanded-content']['children']['props']
    assert graph['responsive'] is True and graph['config']['responsive'] is True
    assert graph['config']['scrollZoom'] is True
    figure = graph['figure']
    assert (
        figure['data'][0]['x'][0] == pages['target_truth']['records'][3]['data']['position'][0]
    )
    assert len(figure['data'][0]['x']) == len(
        [
            r
            for p in backend.iter_telemetry(run_id, 'target_truth', offset=3)
            for r in p['records']
        ]
    )
    assert figure['layout']['uirevision'].startswith(f'{run_id}:trajectory:{mode}:forecast-')
    if mode == '3d':
        assert [t['type'] for t in figure['data']] == ['scatter3d'] * 3
        assert figure['layout']['scene']['aspectmode'] == 'data'
        assert (
            figure['data'][0]['z'][0]
            == pages['target_truth']['records'][3]['data']['position'][2]
        )
    else:
        assert figure['layout']['yaxis']['scaleanchor'] == 'x'
        assert figure['layout']['yaxis']['scaleratio'] == 1


def test_expanded_mode_switch_clears_camera_identity_and_close_keeps_selection(
    trajectory_evidence,
):
    """Switching modes resets graph state; Close must not change the selected Run."""
    backend, run_id, _ = trajectory_evidence
    app = Dash(__name__)
    app.layout = results.results_view(backend, run_id)
    results.register_trajectory(app, backend)
    response = _expanded_request(
        app,
        trigger='trajectory-expanded-mode.value',
        run_id=run_id,
        expanded_mode='3d',
        hidden=False,
    )
    assert response.status_code == 200, response.get_data(as_text=True)
    graph = response.get_json()['response']['trajectory-expanded-content']['children']['props']
    assert graph['figure']['layout']['scene']['uirevision'].startswith(
        f'{run_id}:trajectory:3d:forecast-'
    )
    closed = _expanded_request(
        app, trigger='trajectory-close.n_clicks', run_id=run_id, hidden=False
    )
    assert closed.status_code == 200
    output = closed.get_json()['response']
    assert output['trajectory-overlay']['hidden'] is True
    assert output['trajectory-expanded-content']['children'] is None
    assert 'results-run' not in output
    restored = results.results_view(backend, run_id)
    overlay = next(
        c for c in _walk(restored) if getattr(c, 'id', None) == 'trajectory-overlay'
    )
    assert overlay.hidden is True


def test_full_reader_reaches_recorded_tail_and_selects_forecast_history(trajectory_evidence):
    """Read every genuine sample and expose distinct forecast origins."""
    backend, run_id, _ = trajectory_evidence
    import json

    source = Path(os.environ['ROBOT_DASHBOARD_RECORDED_RUN']) / 'telemetry'
    originals = {
        s: [json.loads(line) for line in (source / (s + '.jsonl')).open()]
        for s in ('target_truth', 'target_estimate', 'prediction')
    }
    pages, errors = results.read_trajectory_pages(backend, run_id)
    assert not errors
    figure, note = results.trajectory_figure(pages, run_id=run_id)
    for trace, stream, field in zip(
        figure.data, ('target_truth', 'target_estimate'), ('position', 'mean')
    ):
        assert list(trace.x) == [r['data'][field][0] for r in originals[stream]]
        assert list(trace.customdata) == [r['source_time'] for r in originals[stream]]
    assert len(figure.data[-1].x) == len(originals['prediction'][-1]['data']['states'])
    assert 'complete' in note.lower()
    first, _ = results.read_trajectory_pages(backend, run_id, prediction_index=0)
    initial, _ = results.trajectory_figure(first, mode='3d', run_id=run_id)
    assert initial.data[-1].meta['source_epoch'] == originals['prediction'][0]['source_time']
    assert (
        list(initial.data[-1].customdata) == originals['prediction'][0]['data']['timestamps']
    )


def test_display_sampling_is_bounded_and_retains_recorded_endpoints(trajectory_evidence):
    """Sample display points without losing the first or last recorded point."""
    backend, run_id, _ = trajectory_evidence
    pages, _ = results.read_trajectory_pages(backend, run_id, max_points=16)
    records = pages['target_truth']['records']
    assert len(records) <= 16
    assert pages['target_truth']['coverage']['valid_count'] > 200
    raw = list(backend.iter_telemetry(run_id, 'target_truth', limit=1000))
    assert records[0] == raw[0]['records'][0]
    assert records[-1] == raw[-1]['records'][-1]
    figure, note = results.trajectory_figure(pages, run_id=run_id)
    assert len(figure.data[0].x) <= 16
    assert 'sampled' in note.lower()


def test_forecast_controls_disclose_history_and_partial_recorded_horizons(trajectory_evidence):
    """Label available origins and distinguish incomplete recorded horizons."""
    backend, run_id, _ = trajectory_evidence
    pages, _ = results.read_trajectory_pages(backend, run_id, prediction_index=0)
    controls = list(_walk(results.forecast_controls(pages)))
    selector = next(c for c in controls if getattr(c, 'id', None) == 'trajectory-origin')
    assert selector.min == 1 and selector.max > 100 and selector.value == 1
    assert selector.debounce is True
    prediction = pages['prediction']['records'][0]
    prediction['data']['states'].pop()
    prediction['data']['timestamps'].pop()
    summary = results.trajectory_summary(pages)
    assert 'partial' in summary and '19/20' in summary
    figure, _ = results.trajectory_figure(pages, run_id=run_id)
    assert len(figure.data[-1].x) == 19


def test_frame_mismatch_in_unsampled_tail_rejects_whole_stream(trajectory_evidence):
    """Validate frames even for points omitted by display sampling."""
    backend, run_id, _ = trajectory_evidence
    original = backend.iter_telemetry

    def changed(rid, stream, **kwargs):
        for page in original(rid, stream, **kwargs):
            if stream == 'target_truth':
                page['records'][-2]['frame_id'] = 'other-frame'
            yield page

    backend.iter_telemetry = changed
    pages, _ = results.read_trajectory_pages(backend, run_id, max_points=16)
    figure, note = results.trajectory_figure(pages, run_id=run_id)
    assert 'Truth' not in [trace.name for trace in figure.data]
    assert 'other-frame' in note


def test_partial_scan_never_looks_like_complete_recording(trajectory_evidence):
    """Reject a partial scan instead of labeling its prefix complete."""
    backend, run_id, _ = trajectory_evidence
    original = backend.iter_telemetry

    def damaged(rid, stream, **kwargs):
        yield from original(rid, stream, **kwargs)
        if stream == 'target_truth':
            raise ValueError('bad recorded line')

    backend.iter_telemetry = damaged
    pages, errors = results.read_trajectory_pages(backend, run_id)
    assert errors and not pages['target_truth']['coverage']['complete_scan']
    figure, _ = results.trajectory_figure(pages, run_id=run_id)
    assert 'Truth' not in [trace.name for trace in figure.data]
    assert 'Truth unavailable' in results.trajectory_summary(pages)


def test_expanded_origin_selection_synchronizes_normal_view_without_joining_forecasts(
    trajectory_evidence,
):
    """Keep panel and overlay on the same independent forecast."""
    backend, run_id, raw = trajectory_evidence
    app = Dash(__name__)
    app.layout = results.results_view(backend, run_id)
    results.register_trajectory(app, backend)
    response = _expanded_request(
        app,
        trigger='trajectory-expanded-origin.value',
        run_id=run_id,
        hidden=False,
        origin=10,
        expanded_origin=1,
    )
    assert response.status_code == 200, response.get_data(as_text=True)
    output = response.get_json()['response']
    assert output['trajectory-origin']['value'] == 1
    trace = output['trajectory-expanded-content']['children']['props']['figure']['data'][-1]
    assert trace['meta']['source_epoch'] == raw['prediction']['records'][0]['source_time']
    assert trace['x'] == [s[0] for s in raw['prediction']['records'][0]['data']['states']]


def test_oversized_horizon_discloses_display_sampling_and_retains_endpoints(
    trajectory_evidence,
):
    """Disclose horizon sampling while retaining original forecast endpoints."""
    _, run_id, original = trajectory_evidence
    pages = deepcopy(original)
    record = pages['prediction']['records'][-1]
    data = record['data']
    data['states'] = data['states'] * 250
    data['timestamps'] = [record['source_time'] + i / 10 for i in range(5000)]
    data['horizon'] = 5000
    figure, _ = results.trajectory_figure(pages, run_id=run_id)
    trace = figure.data[-1]
    assert len(trace.x) <= results.TRAJECTORY_POINT_LIMIT
    assert trace.customdata[0] == data['timestamps'][0]
    assert trace.customdata[-1] == data['timestamps'][-1]
    pages['prediction']['records'] = [record]
    assert 'plotted (sampled)' in results.trajectory_summary(pages)


def test_exact_display_budget_keeps_every_available_sample(trajectory_evidence):
    """Avoid unnecessary sampling when all recorded points fit the budget."""
    backend, run_id, _ = trajectory_evidence
    originals = [
        r for p in backend.iter_telemetry(run_id, 'target_truth') for r in p['records']
    ]
    pages, _ = results.read_trajectory_pages(backend, run_id, max_points=len(originals))
    assert pages['target_truth']['records'] == originals
    assert not pages['target_truth']['coverage']['sampled']
