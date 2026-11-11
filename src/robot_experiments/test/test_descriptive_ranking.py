"""Descriptive Run ranking is independent of immutable qualification verdicts."""

from copy import deepcopy

import pytest

from robot_experiments.comparison import comparison_views


def observed(identifier, value, *, seed=42, state='completed', dynamics='circle'):
    original = {'duration': 6., 'settling_time': 3., 'target_dynamics': dynamics,
                'world': 'same', 'tracking_reference': {'radius': 1.}}
    conditions = {'experiment_setup': {'name': identifier, 'seeds': [42]},
                  'evaluation': {'required_metrics': ['tracking_distance']},
                  'component_selection': {'predictor': 'imm'}}
    view = comparison_views(original, None, conditions)
    metrics = {name: {'available': True, 'unit': unit, 'sample_count': 12,
                      'rmse': score, 'mean': score / 2}
               for name, unit, score in [('tracking_distance', 'm', value),
                                         ('pointing_error', 'rad', 1-value),
                                         ('target_position_error', 'm', value / 2),
                                         ('prediction_rmse', 'm', value * 2)]}
    return {'run_id': identifier, 'seed': seed, 'execution_status': state,
            'acceptance': None, 'comparison': view, 'frozen_consistent': True,
            'metrics': {'schema_version': 1, 'contract': {'definitions': 'same'},
                        'evaluation_window': {'start': 4., 'end': 10., 'unit': 's'},
                        'safety': {'coverage_complete': True, 'accepted': False},
                        'metrics': metrics}}


def rank(rows, metric='tracking_distance', scope=None):
    from robot_experiments.benchmarking import rank_descriptive_runs
    return rank_descriptive_runs(rows, metric, scope=scope)


def test_null_acceptance_is_preserved_and_exact_ties_share_competition_rank():
    rows = [observed('b', .3), observed('a', .3), observed('c', .4)]
    before = deepcopy(rows)
    result = rank(rows)
    assert result['eligible']
    assert [(r['run_id'], r['rank']) for r in result['runs']] == [('a', 1), ('b', 1), ('c', 3)]
    assert result['kind'] == 'descriptive'
    assert rows == before and all(r['acceptance'] is None for r in rows)
    assert rank(rows, 'pointing_error')['runs'][0]['run_id'] == 'c'


def test_descriptive_errors_do_not_claim_missing_physical_qualification():
    rows = [observed('a', .3), observed('b', .4)]
    for row in rows:
        row['metrics']['safety'] = {'coverage_complete': False, 'accepted': None}
    before = deepcopy(rows)
    assert rank(rows)['eligible']
    assert rows == before


def test_missing_failed_single_and_nonfinite_results_have_no_rank():
    a, b = observed('a', .3), observed('b', .4, state='failed')
    result = rank([a, b])
    assert not result['eligible'] and 'b' in result['excluded']
    assert 'two' in ' '.join(result['reasons'])
    for bad in (None, float('nan'), -1, True):
        b = observed('b', .4)
        b['metrics']['metrics']['tracking_distance']['rmse'] = bad
        assert 'b' in rank([a, b])['excluded']


def test_scope_is_explicit_for_incompatible_dynamics_and_only_comparable_cohort_ranks():
    rows = [observed('a', .3), observed('b', .4), observed('other', .2, dynamics='lorenz')]
    unscoped = rank(rows)
    assert not unscoped['eligible'] and len(unscoped['scopes']) == 2
    scope = next(s['id'] for s in unscoped['scopes'] if set(s['run_ids']) == {'a', 'b'})
    chosen = rank(rows, scope=scope)
    assert chosen['eligible'] and {r['run_id'] for r in chosen['runs']} == {'a', 'b'}
    assert 'other' in chosen['excluded']


@pytest.mark.parametrize('change', ['units', 'definitions', 'window', 'seed', 'frozen', 'policy'])
def test_incompatible_or_tampered_evidence_is_never_silently_ranked(change):
    a, b = observed('a', .3), observed('b', .4)
    if change == 'units':
        b['metrics']['metrics']['tracking_distance']['unit'] = 'cm'
    elif change == 'definitions':
        b['metrics']['contract']['definitions'] = 'different'
    elif change == 'window':
        b['metrics']['evaluation_window']['end'] = 11.
    elif change == 'seed':
        b['seed'] = 43
    elif change == 'frozen':
        b['frozen_consistent'] = False
    else:
        b['comparison']['original_contract']['target_dynamics'] = 'tampered'
    assert not rank([a, b])['eligible']


def test_unknown_metric_is_rejected():
    with pytest.raises(ValueError):
        rank([], 'invented_composite')


def test_repeated_immutable_run_is_not_two_independent_results():
    row = observed('same-run', .3)
    result = rank([row, deepcopy(row)])
    assert not result['eligible']
    assert not result['runs']
    assert 'duplicate' in ' '.join(result['excluded']['same-run'])
