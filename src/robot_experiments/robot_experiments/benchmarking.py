"""Matched-seed aggregation without unsafe or statistically unsupported winners."""
from collections import defaultdict
import math
import statistics

from .comparison import comparison_reasons


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
