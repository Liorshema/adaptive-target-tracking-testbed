"""Matched-seed aggregation without unsafe or statistically unsupported winners."""
from collections import defaultdict
import math
import statistics


def _aggregate(values, unit):
    """Give each seed's metric equal weight; dispersion is unknown for n=1."""
    return {'unit': unit, 'sample_count': len(values), 'mean': statistics.mean(values),
            'std': statistics.stdev(values) if len(values) > 1 else None,
            'min': min(values), 'max': max(values)}


def summarize(runs, variants, seeds):
    """Compare matching physical/time contracts, preserving prediction horizons."""
    reasons = []
    seen = {}
    for run in runs:
        key = (run['variant'], run['seed'])
        if key in seen:
            reasons.append(f'duplicate run {key}')
        seen[key] = run
    for seed in seeds:
        paired = [seen.get((variant, seed)) for variant in variants]
        if any(run is None for run in paired):
            reasons.append(f'seed {seed}: incomplete variant pair')
            continue
        if any(run['execution_status'] != 'completed' for run in paired):
            reasons.append(f'seed {seed}: interrupted execution')
        if len({run['comparison_contract'] for run in paired}) != 1:
            reasons.append(f'seed {seed}: scenario/physical/measurement-window mismatch')
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
            for name, metric in run.get('metrics', {}).get('metrics', {}).items():
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
    return result
