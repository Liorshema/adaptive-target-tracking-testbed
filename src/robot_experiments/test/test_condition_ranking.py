"""Condition ranking requires matched recorded evidence, independently of quality."""
from copy import deepcopy

import pytest

from robot_experiments.benchmarking import rank_conditions
from robot_experiments.comparison import comparison_views


def runs():
    """Test-only outcomes; never persisted as scientific Runs."""
    items = []
    for variant, values in [('A', [.2, .4]), ('B', [.4, .6])]:
        for seed, value in zip([42, 43], values):
            view = comparison_views({'duration': 2}, None, {
                'evaluation': {'required_metrics': ['tracking_distance']}})
            items.append({'variant': variant, 'seed': seed, 'run_id': f'{variant}-{seed}',
                          'execution_status': 'completed', 'acceptance': False,
                          'comparison_contract': view['original_hash'], 'comparison': view,
                          'metrics': {'schema_version': 1, 'contract': {'definition': 'test'},
                                      'evaluation_window': {'start': 1, 'end': 3, 'unit': 's'},
                                      'safety': {'coverage_complete': True, 'accepted': True},
                                      'metrics': {'tracking_distance': {
                                          'available': True, 'sample_count': 10, 'unit': 'm',
                                          'mean': 99, 'rmse': value}}}})
    return items


def test_equal_seed_weight_rmse_sample_variability_and_per_seed_provenance():
    result = rank_conditions(runs(), ['A', 'B'], [42, 43])
    assert result['eligible']
    a, b = result['conditions']
    assert (a['rank'], b['rank']) == (1, 2)
    assert a['metrics']['tracking_distance']['mean'] == pytest.approx(.3)
    assert a['metrics']['tracking_distance']['std'] == pytest.approx(.2 / 2 ** .5)
    assert a['metrics']['tracking_distance']['sample_count'] == 2
    assert a['per_seed'][0] == {'seed': 42, 'run_id': 'A-42', 'rmse': .2, 'unit': 'm'}


@pytest.mark.parametrize('problem', ['missing', 'duplicate', 'failed', 'metric', 'unit',
                                    'negative', 'bool', 'unsafe', 'window', 'coverage'])
def test_incomplete_invalid_or_unsafe_evidence_withholds_all_ranks(problem):
    items = runs()
    if problem == 'missing': items.pop()
    elif problem == 'duplicate': items.append(deepcopy(items[0]))
    elif problem == 'failed': items[-1]['execution_status'] = 'failed'
    elif problem == 'metric': del items[-1]['metrics']['metrics']['tracking_distance']['rmse']
    elif problem == 'unit': items[-1]['metrics']['metrics']['tracking_distance']['unit'] = 'cm'
    elif problem == 'negative': items[-1]['metrics']['metrics']['tracking_distance']['rmse'] = -.1
    elif problem == 'bool': items[-1]['metrics']['metrics']['tracking_distance']['rmse'] = True
    elif problem == 'unsafe': items[-1]['metrics']['safety']['accepted'] = False
    elif problem == 'window': items[-1]['metrics']['evaluation_window']['end'] = 2
    elif problem == 'coverage': items[-1]['metrics']['safety']['coverage_complete'] = False
    result = rank_conditions(items, ['A', 'B'], [42, 43])
    assert not result['eligible'] and result['reasons']
    assert all(c['rank'] is None for c in result['conditions'])


def test_ties_share_rank_and_one_seed_variability_is_unknown():
    items = [item for item in runs() if item['seed'] == 42]
    items[1]['metrics']['metrics']['tracking_distance']['rmse'] = .2
    result = rank_conditions(items, ['A', 'B'], [42])
    assert [c['rank'] for c in result['conditions']] == [1, 1]
    assert result['conditions'][0]['metrics']['tracking_distance']['std'] is None


def session_fixture(tmp_path):
    """Freeze synthetic, explicitly test-only evidence through real storage APIs."""
    import json
    from test_backend import backend, recorded_run
    from robot_experiments.reporting import write_json
    from robot_experiments.components import describe_component
    prediction = {'predictors': {name: {'parameters': describe_component('predictor', name)['defaults']}
                                 for name in ('imm', 'constant_velocity')}}
    api = backend(tmp_path, roots=[tmp_path / 'results'])
    session = api.create_session({'title': 'TEST ONLY'})['session_id']
    paths = []
    declaration = {'schema_version': 2, 'domain': 'component_models', 'variables': [
        {'name': 'predictor', 'path': 'components.predictor', 'values': ['imm', 'constant_velocity']}]}
    for model in ('imm', 'constant_velocity'):
        for seed in (42, 43):
            identifier, path = recorded_run(tmp_path / 'results', seed=seed)
            view = comparison_views({'duration': 2, 'robot': 'test-only'}, declaration, {
                'component_selection': {'predictor_type': model},
                'prediction_configuration': prediction,
                'seed_consumers': {'measurements.target_position.random_seed': seed},
                'experiment_setup': {'seeds': [42, 43], 'variants': ['variant']},
                'evaluation': {'required_metrics': ['tracking_distance']}})
            metadata = json.loads((path / 'metadata.json').read_text())
            metadata['seed_consumers'] = view['protected_conditions']['seed_consumers']
            frozen = json.loads((path / 'resolved_configuration.json').read_text())
            frozen.update(comparison=view, comparison_contract=view['original_contract'])
            frozen['owner_snapshot']['seed_consumers'] = metadata['seed_consumers']
            frozen['system']['selection'] = view['protected_conditions']['component_selection']
            write_json(path / 'resolved_configuration.json', frozen)
            write_json(path / 'metadata.json', metadata)
            write_json(path / 'comparison.json', view)
            metrics = json.loads((path / 'metrics.json').read_text())
            metrics['metrics']['tracking_distance']['rmse'] = (.1 if model == 'imm' else .3) + seed / 1000
            metrics['safety']['accepted'] = True
            write_json(path / 'metrics.json', metrics)
            api.add_run_to_session(session, identifier)
            paths.append(path)
    return api, session, paths


def test_session_groups_real_variant_identity_not_uuid(tmp_path):
    api, session, _ = session_fixture(tmp_path)
    result = api.get_session_comparison(session)
    assert result['comparability']['comparable'], result['comparability']
    assert result['ranking']['eligible']
    assert len(result['runs']) == 4 and len(result['ranking']['conditions']) == 2
    assert [c['seed_count'] for c in result['ranking']['conditions']] == [2, 2]
    assert [c['rank'] for c in result['ranking']['conditions']] == [1, 2]


@pytest.mark.parametrize('problem', ['rehash', 'derived', 'missing_seed', 'physics', 'window', 'seed_label'])
def test_session_frozen_proof_and_incomplete_pairs_are_enforced(tmp_path, problem):
    import json
    from robot_experiments.reporting import write_json
    api, session, paths = session_fixture(tmp_path)
    path = paths[-1]
    view = json.loads((path / 'comparison.json').read_text())
    if problem == 'missing_seed':
        (path / 'status.json').unlink()
    elif problem == 'seed_label':
        metadata = json.loads((path / 'metadata.json').read_text())
        metadata['seed'] = 42
        write_json(path / 'metadata.json', metadata)
    elif problem == 'window':
        metrics = json.loads((path / 'metrics.json').read_text())
        metrics['evaluation_window']['end'] = 2
        write_json(path / 'metrics.json', metrics)
    else:
        if problem == 'derived':
            view['invariant_conditions']['component_selection']['predictor_type'] = 'forged'
        else:
            if problem == 'rehash': view['original_contract']['robot'] = 'forged'
            else: view['protected_conditions']['physics'] = {'max_step_size': 123}
            view = comparison_views(view['original_contract'], view['policy'], view['protected_conditions'])
        write_json(path / 'comparison.json', view)
    result = api.get_session_comparison(session)
    assert not result['comparability']['comparable']
    assert result['ranking'] is None


def test_author_duration_and_policy_survive_immutable_revisions(tmp_path):
    from test_backend import backend, share
    from robot_experiments.catalog import ExperimentCatalog
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    api = backend(tmp_path, catalog=catalog)
    draft = api.new_draft('EXP-TEST-COMPARISON', 'TEST ONLY', 'EXP-NOMINAL')
    draft['experiment']['runtime'] = {'enable_actuation': False}
    draft['experiment']['execution'] = {'duration': 2., 'seeds': [42, 43]}
    declaration = {'schema_version': 2, 'domain': 'component_models', 'variables': [
        {'name': 'predictor', 'path': 'components.predictor', 'values': ['imm', 'constant_velocity']}]}
    draft['experiment']['comparison'] = declaration
    first = api.save_experiment(draft)
    changed = api.new_draft(first['id'], 'TEST ONLY', first['id'], base_revision=1)
    assert changed['experiment']['comparison'] == declaration
    changed['experiment']['components'] = {'predictor': 'constant_velocity'}
    second = api.save_experiment(changed, expected_revision=1)
    assert second['revision'] == 2
    assert catalog.get(first['id'], 1) == first
    a, b = [catalog.prepare(first['id'], revision) for revision in (1, 2)]
    assert a['comparison'] == b['comparison'] == declaration
    assert a['duration'] == b['duration'] == 2 and a['seeds'] == b['seeds'] == [42, 43]


def test_rank_rejects_outside_declared_seed_and_recorded_command_safety_failure():
    items = runs()
    outside = deepcopy(items[0])
    outside.update(seed=99, run_id='outside')
    assert not rank_conditions(items + [outside], ['A', 'B'], [42, 43])['eligible']
    items[0]['status'] = {'acceptance_gates': {'watchdog_events': False}}
    result = rank_conditions(items, ['A', 'B'], [42, 43])
    assert not result['eligible'] and 'watchdog_events' in ' '.join(result['reasons'])


def test_malformed_optional_metric_blocks_ranking_without_crashing():
    items = runs()
    for item in items:
        item['metrics']['metrics']['pointing_error'] = None
    result = rank_conditions(items, ['A', 'B'], [42, 43])
    assert not result['eligible']
    assert 'pointing_error' not in result['conditions'][0]['metrics']


@pytest.mark.parametrize('count', [True, float('nan'), '10', 0])
def test_invalid_recorded_observation_count_never_supplies_rank(count):
    items = runs()
    items[0]['metrics']['metrics']['tracking_distance']['sample_count'] = count
    assert not rank_conditions(items, ['A', 'B'], [42, 43])['eligible']
