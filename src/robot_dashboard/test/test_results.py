"""Scientific presentation checks read genuine retained Run artifacts without writes."""

import json
import os
from pathlib import Path

import pytest

from robot_experiments.backend import ExperimentBackend


@pytest.fixture
def recorded(tmp_path):
    """Explicit optional evidence locations keep generated results out of source fixtures."""
    good = os.environ.get('ROBOT_DASHBOARD_RECORDED_RUN')
    bad = os.environ.get('ROBOT_DASHBOARD_FAILED_RUN')
    if not good or not bad:
        pytest.skip('Set ROBOT_DASHBOARD_RECORDED_RUN and ROBOT_DASHBOARD_FAILED_RUN')
    paths = [Path(good).resolve(), Path(bad).resolve()]
    api = ExperimentBackend(tmp_path, result_roots=[p.parent.parent for p in paths])
    return api, [(p, json.loads((p / 'status.json').read_text())['run_id']) for p in paths]


def test_public_recorded_series_paging_is_exact_and_missing_is_honest(recorded):
    """Public paging returns stored curves, units and source epochs unchanged."""
    api, ((path, good), (_, bad)) = recorded
    original = json.loads((path / 'time_series.json').read_text())['series']
    page = api.read_time_series(good, offset=4, limit=7)
    assert page['available'] is True
    for name, series in original.items():
        result = page['series'][name]
        assert result['samples'] == series['samples'][4:11]
        assert result['sample_count'] == len(series['samples'])
        assert all(result[k] == v for k, v in series.items() if k != 'samples')
    assert (
        api.read_time_series(good, offset=100000, limit=1)['series']['tracking_distance'][
            'samples'
        ]
        == []
    )
    missing = api.read_time_series(bad)
    assert missing['available'] is False and missing['series'] == {}
    with pytest.raises(ValueError):
        api.read_time_series(good, limit=1001)


def test_plot_coordinates_and_metric_values_are_recorded_not_recomputed(recorded):
    """Charting and tables preserve every stored value and its declared unit/window."""
    from robot_dashboard.results import metric_rows
    from robot_dashboard.results import recorded_figures

    api, ((path, good), _) = recorded
    result = api.get_run_results(good)
    page = api.read_time_series(good, offset=0, limit=1000)
    figures = recorded_figures(result, page)
    assert len(figures) == 3
    for name, figure in figures.items():
        samples = page['series'][name]['samples']
        assert list(figure.data[0].x) == [s['time'] for s in samples]
        assert list(figure.data[0].y) == [s['value'] for s in samples]
        assert figure.data[0].mode == 'markers'
        assert page['series'][name]['unit'] in figure.layout.yaxis.title.text
        assert 'simulation source time' in figure.layout.xaxis.title.text
    rows = {r['metric']: r for r in metric_rows(result)}
    original = json.loads((path / 'metrics.json').read_text())['metrics']
    assert rows['tracking_distance']['mean'] == original['tracking_distance']['mean']
    assert rows['pointing_error']['max'] == original['pointing_error']['max']
    assert result['status']['execution_status'] == 'completed'


def test_readiness_failure_has_unknown_acceptance_and_no_plotted_results(recorded):
    """A real timeout cannot become an accepted execution or synthetic zero series."""
    from robot_dashboard.results import metric_rows
    from robot_dashboard.results import recorded_figures
    from robot_dashboard.results import results_view

    api, (_, (_, bad)) = recorded
    result = api.get_run_results(bad)
    assert result['status']['execution_status'] == 'failed'
    assert result['status']['acceptance'] is None
    assert result['status']['error']['code'] == 'readiness_timeout'
    assert metric_rows(result) == []
    assert recorded_figures(result, api.read_time_series(bad)) == {}
    text = str(results_view(api, bad, offset=0))
    assert 'readiness_timeout' in text and 'Unknown' in text
    assert 'Unavailable' in text


def test_persistent_session_has_references_and_scientific_reasons(recorded):
    """Failed and completed evidence can coexist without fabricated ranking."""
    api, ((_, good), (_, bad)) = recorded
    session = api.create_session({'title': 'Recorded evidence'})
    for rid in [good, bad, good]:
        api.add_run_to_session(session['session_id'], rid)
    reloaded = ExperimentBackend(api.directory, result_roots=api.result_roots)
    comparison = reloaded.get_session_comparison(session['session_id'])
    assert len(comparison['runs']) == 2
    assert comparison['ranking'] is None
    assert comparison['comparability']['comparable'] is False
    assert comparison['comparability']['reasons']
    stored = json.loads((api.directory / 'sessions' / f"{session['session_id']}.json").read_text())
    assert all(set(r) == {'run_id', 'directory'} for r in stored['runs'])


def test_four_kpis_preserve_genuine_values_and_missingness(recorded):
    """KPI display uses recorded means and never invents missing scalar values."""
    from robot_dashboard.results import kpi_cards

    api, ((_, good), (_, bad)) = recorded
    result = api.get_run_results(good)
    cards = kpi_cards(result)
    assert len(cards) == 4
    names = [
        'tracking_distance',
        'pointing_error',
        'target_position_error',
        'prediction_rmse',
    ]
    for card, name in zip(cards, names):
        metric = result['metrics']['metrics'][name]
        assert card.to_plotly_json()['props']['data-metric'] == name
        if metric['available']:
            statistic = 'rmse' if name in ('prediction_rmse', 'target_position_error') else 'mean'
            assert f'{metric[statistic]:.3f}' in str(card)
            assert metric['unit'] in str(card)
    assert all('Unavailable' in str(c) for c in kpi_cards(api.get_run_results(bad)))


def test_trajectory_samples_are_original_and_frame_mismatch_is_rejected(recorded):
    """Raw XY views use unchanged samples and one actual horizon, without alignment."""
    from robot_dashboard.results import trajectory_figure

    api, ((_, good), _) = recorded
    pages = {}
    for name in ['target_truth', 'target_estimate', 'prediction']:
        pages[name] = api.read_telemetry(good, name, offset=0, limit=200)
    figure, note = trajectory_figure(pages)
    assert figure is not None and len(figure.data) == 3
    truth = pages['target_truth']['records']
    assert list(figure.data[0].x) == [r['data']['position'][0] for r in truth]
    assert list(figure.data[0].y) == [r['data']['position'][1] for r in truth]
    prediction = pages['prediction']['records'][-1]['data']['states']
    assert list(figure.data[-1].x) == [s[0] for s in prediction]
    assert all(t.mode == 'markers' for t in figure.data)
    assert 'independent' in note.lower() and 'epoch' in note.lower()
    from copy import deepcopy

    invalid = deepcopy(pages)
    for row in invalid['target_estimate']['records']:
        row['frame_id'] = 'different-frame'
    rejected, why = trajectory_figure(invalid)
    assert len(rejected.data) == 2
    assert 'different-frame' in why
    assert trajectory_figure({})[0] is None


def test_current_session_row_selects_original_run_through_callback(recorded):
    """A row click selects the immutable Run ID, including failed Runs."""
    from robot_dashboard.app import create_app

    api, ((_, good), (_, bad)) = recorded
    session = api.create_session({'title': 'Row selection'})['session_id']
    for rid in (good, bad):
        api.add_run_to_session(session, rid)
    app = create_app(api)
    key, callback = next(
        (k, v)
        for k, v in app.callback_map.items()
        if k.startswith('..session-comparison.children')
    )
    trigger = (
        json.dumps({'type': 'session-run', 'run_id': bad}, sort_keys=True, separators=(',', ':'))
        + '.n_clicks'
    )
    response = app.server.test_client().post(
        '/_dash-update-component',
        json={
            'output': key,
            'outputs': [
                {'id': o.component_id, 'property': o.component_property}
                for o in callback['output']
            ],
            'inputs': [
                {**meta, 'value': v}
                for meta, v in zip(callback['inputs'], [session, 0, None, [0, 1], 0])
            ],
            'state': [{**meta, 'value': v} for meta, v in zip(callback['state'], [good, None])],
            'changedPropIds': [trigger],
        },
    )
    assert response.status_code == 200, response.get_data(as_text=True)
    body = response.get_json()['response']
    assert body['results-run']['value'] == bad
    assert 'selected-run' in str(body['session-comparison']['children'])
    assert api.get_session_comparison(session)['ranking'] is None


def _presentation_result(run_id='run-a', radial_rmse=0.25):
    """Test-only scalar evidence makes differing recorded statistics explicit."""
    return {
        'run_id': run_id, 'experiment_id': 'EXP-A', 'revision': 4,
        'status': {'execution_status': 'completed', 'acceptance': True},
        'configuration': {'experiment': {'name': 'Recorded scenario'}},
        'artifacts': {},
        'metrics': {'metrics': {
            'tracking_distance': {'available': True, 'unit': 'm', 'mean': 0.123456,
                                  'rmse': radial_rmse},
            'pointing_error': {'available': True, 'unit': 'rad', 'mean': 0.456789,
                               'rmse': 0.5235987755982988},
            'target_position_error': {'available': True, 'unit': 'm', 'mean': 0.444444,
                                      'rmse': 0.555555},
            'prediction_rmse': {'available': True, 'unit': 'm', 'mean': 0.666666,
                                'rmse': 0.777777},
        }},
    }


def _table_rows(view):
    table = next(child for child in view.children if getattr(child, 'className', '')
                 == 'evidence-table session-table').children
    return table.children[0].children.children, table.children[1].children


def test_four_kpi_statistics_are_distinct_and_math_is_native():
    """The four cards select exact scalar fields and render native subscripts/means."""
    from robot_dashboard.results import kpi_cards

    cards = kpi_cards(_presentation_result())
    assert len(cards) == 4
    assert [card.to_plotly_json()['props']['data-metric'] for card in cards] == [
        'tracking_distance', 'pointing_error', 'target_position_error', 'prediction_rmse']
    for card, value in zip(cards, ('0.123', '0.457', '0.556', '0.778')):
        assert value in str(card)
    assert 'Mean Position Error' in str(cards[0])
    assert 'radial' in str(cards[0]).lower()
    assert 'pointing' in str(cards[1]).lower()
    assert 'Sub(' in str(cards[0]) and 'overline' in str(cards[0])
    assert 'rmse' in str(cards[2]).lower()
    assert '0.667' not in str(cards[3])


def test_estimator_kpi_fallback_is_mean_and_never_comparison_rmse():
    """A stored mean remains a mean when its distinct RMSE is absent."""
    from robot_dashboard.results import comparison_view, kpi_cards

    result = _presentation_result()
    del result['metrics']['metrics']['target_position_error']['rmse']
    card = kpi_cards(result)[2]
    assert '0.444' in str(card) and 'Recorded mean' in str(card)
    view = comparison_view({'runs': [result], 'comparability': {'comparable': False}})
    _, rows = _table_rows(view)
    assert rows[0].children[3].children == 'Unavailable'


@pytest.mark.parametrize('value', [None, float('nan'), float('inf'), -0.5, True])
def test_invalid_rmse_is_unavailable_without_hiding_other_recorded_errors(value):
    """Invalid RMSE remains explicit while each other recorded error stays readable."""
    from robot_dashboard.results import comparison_view

    view = comparison_view({
        'runs': [_presentation_result(radial_rmse=value), _presentation_result('run-b')],
        'comparability': {'comparable': True},
    })
    _, rows = _table_rows(view)
    assert rows[0].children[1].children == 'Unavailable'
    assert [cell.children for cell in rows[0].children[2:]] == ['30.000', '0.556', '0.778']


def test_comparison_uses_recorded_rmse_converts_angles_and_preserves_selection():
    """Table units, scientific fields and original immutable selection remain correct."""
    from robot_dashboard.results import comparison_view

    result = _presentation_result()
    view = comparison_view({'runs': [result], 'comparability': {'comparable': False}},
                           selected_run='run-a', names={'EXP-A': 'Authoritative title'})
    headers, rows = _table_rows(view)
    assert [head.children for head in headers] == [
        'Scenario', 'R (m)', 'Angle (°)', 'Estimator (m)',
        'Predictor (m)']
    assert [cell.children for cell in rows[0].children[1:]] == [
        '0.250', '30.000', '0.556', '0.778']
    button = rows[0].children[0].children
    assert button.children == 'Authoritative title'
    assert button.id == {'type': 'session-run', 'run_id': 'run-a'}
    assert rows[0].className == 'selected-run'
    assert rows[0].to_plotly_json()['props']['data-run-id'] == 'run-a'


def test_comparison_keeps_recorded_rows_without_ranking_ui_or_instructions():
    """Backend ranking metadata cannot reorder, label or qualify recorded table rows."""
    from robot_dashboard.results import comparison_view

    runs = [_presentation_result('run-a', .9), _presentation_result('run-b', .1)]
    comparison = {
        'runs': runs,
        'comparability': {'comparable': False, 'reasons': ['Choose one compatible comparison scope']},
        'ranking_diagnostics': {'reasons': ['At least two eligible comparable Runs are required']},
        'descriptive_ranking': {
            'eligible': True,
            'criterion': 'Range RMSE; lower is better. Exact ties share rank.',
            'reasons': ['one result is not a winner'],
            'runs': [{'run_id': 'run-b', 'rank': 1}, {'run_id': 'run-a', 'rank': 2}],
            'excluded': {}, 'scope_differences': {},
            'definition': 'Recorded radial RMSE', 'limitations': 'Ranking limitations',
        },
    }
    view = comparison_view(comparison, selected_run='run-a')
    headers, rows = _table_rows(view)
    assert [header.children for header in headers] == [
        'Scenario', 'R (m)', 'Angle (°)', 'Estimator (m)', 'Predictor (m)']
    assert [row.to_plotly_json()['props']['data-run-id'] for row in rows] == ['run-a', 'run-b']
    assert [row.children[1].children for row in rows] == ['0.900', '0.100']
    assert all(len(row.children) == 5 for row in rows)
    assert [row.className for row in rows] == ['selected-run', '']
    for text in ('Rank', 'rank', 'lower is better', 'compatible comparison scope', 'winner',
                 'eligible comparable Runs', 'Comparison details'):
        assert text not in str(view)


def test_comparison_missing_rmse_and_incompatible_units_stay_unavailable():
    """No mean substitution or undeclared unit conversion supplies missing RMSE."""
    from robot_dashboard.results import comparison_view

    result = _presentation_result()
    for name in ('tracking_distance', 'pointing_error', 'prediction_rmse'):
        del result['metrics']['metrics'][name]['rmse']
    result['metrics']['metrics']['target_position_error']['unit'] = 'mm'
    _, rows = _table_rows(comparison_view({
        'runs': [result], 'comparability': {'comparable': True}}))
    assert all(cell.children == 'Unavailable' for cell in rows[0].children[1:])


def test_comparison_accepts_degree_rmse_without_double_conversion():
    """Already recorded degrees retain their magnitude in the degree-labelled column."""
    from robot_dashboard.results import comparison_view

    result = _presentation_result()
    result['metrics']['metrics']['pointing_error'].update(unit='deg', rmse=12.5)
    _, rows = _table_rows(comparison_view({
        'runs': [result], 'comparability': {'comparable': False}}))
    assert rows[0].children[2].children == '12.500'


def test_result_panels_are_grid_siblings_and_compute_is_advanced():
    """Primary panels can share one grid while computation details remain secondary."""
    from robot_dashboard.results import results_view

    class RecordedBackend:
        def read_time_series(self, *args, **kwargs):
            return {'series': {}}

        def read_telemetry(self, *args, **kwargs):
            raise FileNotFoundError

        def iter_telemetry(self, *args, **kwargs):
            """Model a recorded Run without any target telemetry stream."""
            raise FileNotFoundError

    view = results_view(RecordedBackend(), 'run-a', result=_presentation_result())
    assert [child.className for child in view.children] == [
        'kpi-grid', 'panel position-panel', 'panel orientation-panel',
        'panel trajectory-panel', 'trajectory-overlay', 'result-statusbar']
    assert view.children[4].hidden is True
    headings = [view.children[1].children[0], view.children[2].children[0],
                view.children[3].children[0].children[0]]
    assert [heading.children for heading in headings] == [
        'Position Error', 'Orientation Error', 'Truth vs Estimate vs Prediction']
    assert 'Control computation time' not in str(view.children[:-1])
    assert 'Control computation time' in str(view.children[-1])


def test_header_and_scenario_names_use_authoritative_metadata_without_ids():
    """Catalog display names replace internal IDs without inventing scientific names."""
    from robot_dashboard.results import header_view, scenario_label, scenario_names

    class CatalogBackend:
        def list_scenarios(self, *, include_deleted=False):
            return []

        def list_experiments(self, *, include_deleted=False):
            assert include_deleted is True
            return [{'id': 'EXP-A', 'name': 'Nominal IMM tracking'}]

    names = scenario_names(CatalogBackend())
    assert names == {'EXP-A': 'Nominal IMM tracking'}
    result = _presentation_result()
    result['configuration']['system'] = {'manifest': {'robot': {'type': 'mobile_manipulator'}}}
    assert scenario_label(result, names) == 'Nominal IMM tracking'
    text = str(header_view(result, names))
    assert 'Mobile manipulator' in text and 'Nominal IMM tracking' in text
    assert 'run-a' not in text and 'EXP-A' not in text and 'r4' not in text


def test_historical_scenario_names_use_recorded_configuration_when_catalog_id_is_absent(recorded):
    """External historical IDs retain their genuine saved name in compact Session rows."""
    from robot_dashboard.results import scenario_names

    api, ((_, good), _) = recorded
    result = api.get_run_results(good)
    identifier = result['experiment_id']
    assert identifier not in scenario_names(api)
    names = scenario_names(api, runs=[{'experiment_id': identifier, 'run_id': good}])
    assert names[identifier] == result['configuration']['experiment']['name'].replace('_', ' ')


def test_historical_scenario_names_keep_catalog_authority_and_skip_unreadable_evidence():
    """Fallback reads only missing IDs and never turns missing evidence into a name."""
    from robot_dashboard.results import scenario_names

    class CatalogBackend:
        def list_scenarios(self, *, include_deleted=False):
            return []

        def list_experiments(self, *, include_deleted=False):
            return [{'id': 'EXP-KNOWN', 'name': 'Catalog display name'}]

        def get_run_results(self, run_id):
            if run_id == 'unknown-run':
                raise FileNotFoundError('Run evidence unavailable')
            assert run_id == 'historical-run'
            return {'configuration': {'experiment': {'name': 'recorded_historical_name'}}}

    names = scenario_names(CatalogBackend(), runs=[
        {'experiment_id': 'EXP-KNOWN', 'run_id': 'never-read'},
        {'experiment_id': 'EXP-HISTORICAL', 'run_id': 'historical-run'},
        {'experiment_id': 'EXP-UNKNOWN', 'run_id': 'unknown-run'},
    ])
    assert names == {'EXP-KNOWN': 'Catalog display name',
                     'EXP-HISTORICAL': 'recorded historical name'}


def test_comparison_shows_each_recorded_run_instead_of_condition_seed_aggregates():
    """Scientific ranking aggregates do not replace any Run's recorded RMSE."""
    from robot_dashboard.results import comparison_view

    runs = [_presentation_result('run-a', .9), _presentation_result('run-b', 0)]
    condition = {
        'condition_id': 'A', 'run_ids': ['run-a', 'run-b'], 'rank': 1,
        'seed_count': 2, 'variables': {}, 'variant': 'A',
        'metrics': {'tracking_distance': {'mean': .45, 'std': .636, 'unit': 'm'}},
        'per_seed': [{'seed': 42, 'run_id': 'run-a', 'rmse': .9, 'unit': 'm'},
                     {'seed': 43, 'run_id': 'run-b', 'rmse': 0, 'unit': 'm'}],
    }
    _, rows = _table_rows(comparison_view({
        'runs': runs, 'comparability': {'comparable': True},
        'ranking': {'eligible': True, 'conditions': [condition]},
    }, selected_run='run-b'))
    assert len(rows) == 2
    assert [row.children[1].children for row in rows] == ['0.900', '0.000']
    assert [row.to_plotly_json()['props']['data-run-id'] for row in rows] == ['run-a', 'run-b']
    assert rows[1].className == 'selected-run'
    assert all(len(row.children) == 5 for row in rows)
