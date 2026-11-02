"""Matched-seed aggregation without unsafe or statistically unsupported winners."""
from collections import defaultdict
import math
import statistics

from .comparison import comparison_reasons, metric_comparison_reasons, _metric_evidence, metric_document


def rank_conditions(runs, variants, seeds, policy=None, *, reasons=()):
    """Rank matched Conditions using saved radial RMSE, never metric means.

    Callers supply authoritative Condition IDs; UUIDs remain per-seed provenance.
    Poor tracking does not veto comparability. Missing/unsafe physical evidence
    vetoes ranking independently of the experiment's overall acceptance outcome.
    """
    failures = list(reasons) + comparison_reasons(runs, variants, seeds, policy)
    if not seeds or len(set(seeds)) != len(seeds) or len(set(variants)) != len(variants):
        failures.append('unique nonempty declared Seeds and Conditions are required')
    for run in runs:
        if run['variant'] not in variants or run['seed'] not in seeds:
            failures.append(f'{run["variant"]} seed {run["seed"]}: outside declared Condition/seed policy')
    if len(variants) < 2:
        failures.append('at least two distinct experimental Conditions are required')
    for run in runs:
        label = f'{run["variant"]} seed {run["seed"]}'
        if run.get('comparison'):
            failures.extend(f'{label}: {reason}' for reason in _metric_evidence(run, run['comparison']))
        if runs:
            failures.extend(f'{label}: {reason}' for reason in metric_comparison_reasons(runs[0], run))
        safety = metric_document(run).get('safety')
        if not isinstance(safety, dict) or safety.get('accepted') is not True:
            failures.append(f'{label}: physical safety acceptance unavailable or failed')
        gates = run.get('status', {}).get('acceptance_gates', {})
        for name in ('physical_joint_limits', 'invalid_commands', 'watchdog_events',
                     'rejected_commands', 'feedback_rejections'):
            if name in gates and gates[name] is not True:
                failures.append(f'{label}: recorded safety gate {name} unavailable or failed')
        observations = metric_document(run).get('metrics')
        metric = observations.get('tracking_distance', {}) if isinstance(observations, dict) else {}
        metric = metric if isinstance(metric, dict) else {}
        value = metric.get('rmse')
        if (metric.get('available') is not True or metric.get('unit') != 'm'
                or type(value) not in (int, float) or not math.isfinite(value) or value < 0
                or type(metric.get('sample_count')) is not int or metric['sample_count'] <= 0):
            failures.append(f'{label}: valid recorded tracking_distance.rmse [m] unavailable')
    failures = list(dict.fromkeys(failures))
    def radial(run):
        observations = metric_document(run).get('metrics')
        value = observations.get('tracking_distance') if isinstance(observations, dict) else None
        return value if isinstance(value, dict) else {}

    conditions = []
    for variant in variants:
        group = sorted((r for r in runs if r['variant'] == variant), key=lambda r: r['seed'])
        metrics = {}
        if not failures:
            for name in ('tracking_distance', 'pointing_error', 'target_position_error', 'prediction_rmse'):
                observations = [(r.get('metrics') or {}).get('metrics', {}).get(name, {}) for r in group]
                values = [m.get('rmse') for m in observations]
                units = {m.get('unit') for m in observations}
                if (len(units) == 1 and all(m.get('available') is True for m in observations)
                        and all(type(v) in (int, float) and math.isfinite(v) and v >= 0 for v in values)):
                    metrics[name] = {**_aggregate(values, next(iter(units))), 'source_statistic': 'rmse'}
        conditions.append({'condition_id': variant, 'run_ids': [r.get('run_id') for r in group],
                           'seed_count': len(group), 'metrics': metrics, 'rank': None,
                           'per_seed': [{'seed': r['seed'], 'run_id': r.get('run_id'),
                                         'rmse': radial(r).get('rmse'),
                                         'unit': radial(r).get('unit')}
                                        for r in group]})
    if not failures:
        scores = [c['metrics']['tracking_distance']['mean'] for c in conditions]
        for condition, score in zip(conditions, scores):
            condition['rank'] = 1 + sum(other < score for other in scores)
    return {'eligible': not failures, 'reasons': failures, 'conditions': conditions,
            'criterion': 'Equal-weight mean across matched Seeds of recorded radial tracking RMSE [m]; lower is better.',
            'aggregation': 'Sample standard deviation across Seeds; null for n<2. Ties share rank. No statistical significance or universal winner is inferred.',
            'scope': 'target dynamics comparison' if policy and policy.get('domain') == 'target_dynamics' else 'controlled tracking comparison'}


def _aggregate(values, unit):
    """Give each seed's metric equal weight; dispersion is unknown for n=1."""
    return {'unit': unit, 'sample_count': len(values), 'mean': statistics.mean(values),
            'std': statistics.stdev(values) if len(values) > 1 else None,
            'min': min(values), 'max': max(values)}


def summarize(runs, variants, seeds, comparison_policy=None, experiment_name=None):
    """Compare matching physical/time contracts, preserving prediction horizons."""
    reasons = comparison_reasons(runs, variants, seeds, comparison_policy)
    result = {'schema_version': 1, 'comparable': not reasons,
              'incomparability_reasons': reasons, 'seed_count': len(seeds),
              'run_count': len(runs), 'variants': {},
              'aggregation': 'Equal weight per completed run statistic (prediction RMSE; other metric means); sample standard deviation across seeds (null for n<2). No statistical significance or winner is inferred.'}
    for variant in variants:
        group = [run for run in runs if run['variant'] == variant]
        observations = defaultdict(list)
        horizons = defaultdict(list)
        units = {}
        for run in group:
            if run['execution_status'] != 'completed':
                continue
            metrics_document = run.get('metrics') or {}
            for name, metric in metrics_document.get('metrics', {}).items():
                statistic = 'rmse' if name == 'prediction_rmse' else 'mean'
                value = metric.get(statistic)
                if metric.get('available') and isinstance(value, (int, float)) and math.isfinite(value):
                    observations[name].append(float(value))
                    units[name] = metric.get('unit')
                if name == 'prediction_rmse':
                    for horizon, item in metric.get('horizons', {}).items():
                        value = item.get('rmse')
                        if item.get('available') and isinstance(value, (int, float)) and math.isfinite(value):
                            horizons[horizon].append(float(value))
        metrics = {}
        for name, values in observations.items():
            metrics[name] = {**_aggregate(values, units[name]),
                             'source_statistic': 'rmse' if name == 'prediction_rmse' else 'mean'}
        result['variants'][variant] = {
            'run_count': len(group),
            'completed_count': sum(run['execution_status'] == 'completed' for run in group),
            'acceptance_passed_count': sum(run['acceptance'] is True for run in group),
            'unconditional_winner_eligible': not reasons and len(group) == len(seeds) and all(run['acceptance'] is True for run in group),
            'metrics': metrics if not reasons else {},
            'prediction_horizons': {key: _aggregate(values, 'm') for key, values in horizons.items()} if not reasons else {},
            'prediction_horizon_unit': 's',
            'unavailable_reason': 'incomparable runs' if reasons else None}
    result['runs'] = runs
    result['ranking'] = rank_conditions(runs, variants, seeds, comparison_policy)
    result['comparison'] = {
        'schema_version': 1, 'experiment': experiment_name, 'policy': comparison_policy,
        'eligible': not reasons, 'rejection_reasons': reasons,
        'run_index': [{'variant': run['variant'], 'seed': run['seed'],
                       'result_pointer': f'/runs/{index}', 'artifacts': run.get('artifacts'),
                       'execution_status': run['execution_status'],
                       'outcome': ((run.get('status', {}).get('error') or {}).get('code')
                                   or run['execution_status'])}
                      for index, run in enumerate(runs)],
        'trajectory_caveat': ('Matched setup and seed policy do not imply identical reactive '
                              'physical trajectories or identical random draw sequences.'),
    }
    return result
