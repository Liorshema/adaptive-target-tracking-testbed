"""Authorized experimental variables over resolved contracts, without changing metrics."""

import hashlib
import json
import math
import re
from copy import deepcopy
from typing import Any


SENSOR_PATHS = frozenset(
    'localization_sensor.localization_sensor.' + key for key in (
        'white_noise_std', 'calibration_bias_std', 'latency', 'latency_jitter',
        'dropout_probability', 'outlier_probability', 'outlier_offset')) | {
    'measurements.measurements.target_position.noise.covariance'}

METRIC_NAMES = {
    'estimation_error': ['robot_position_error', 'robot_yaw_error',
                         'robot_joint_error', 'target_position_error'],
    'control_compute': ['control_compute_time'],
}


def contract_hash(value: Any) -> str:
    """Keep the original M08 JSON encoding and hash for finite JSON data."""
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def validate_policy(value: Any) -> dict:
    """Reject unsupported declarations before materialization or process startup."""
    if not isinstance(value, dict) or set(value) != {'schema_version', 'domain', 'variables'}:
        raise ValueError('comparison requires schema_version, domain and variables only')
    if type(value['schema_version']) is not int or value['schema_version'] != 1:
        raise ValueError('comparison.schema_version: supported integer 1')
    if value['domain'] != 'sensor_parameters':
        raise ValueError('comparison.domain: supported sensor_parameters only')
    if not isinstance(value['variables'], list) or not value['variables']:
        raise ValueError('comparison.variables: expected nonempty list')
    names, paths = set(), set()
    for variable in value['variables']:
        if (not isinstance(variable, dict) or not {'name', 'path'} <= variable.keys()
                or variable.keys() - {'name', 'path', 'values'}):
            raise ValueError('comparison variable requires name/path and optional values only')
        name, path = variable['name'], variable['path']
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,79}', name):
            raise ValueError('comparison variable name: expected descriptive identifier')
        if not isinstance(path, str) or path not in SENSOR_PATHS:
            raise ValueError(f'comparison path {path!r}: not a permitted sensor leaf; '
                             'protected or unsupported')
        if name in names or path in paths:
            raise ValueError('comparison variables: duplicate name or path')
        names.add(name)
        paths.add(path)
        if 'values' in variable:
            values = variable['values']
            if not isinstance(values, list) or not values:
                raise ValueError(f'comparison variable {name}: values must be a nonempty list')
            try:
                encoded = [json.dumps(v, sort_keys=True, allow_nan=False) for v in values]
            except (ValueError, TypeError) as error:
                raise ValueError(
                    f'comparison variable {name}: values must be finite JSON') from error
            if len(set(encoded)) != len(values):
                raise ValueError(f'comparison variable {name}: duplicate permitted values')
    return deepcopy(value)


def comparison_views(original: dict, policy: dict | None, conditions: dict) -> dict:
    """Keep original evidence; remove only authorized present leaves from a copy."""
    if not isinstance(original, dict) or not isinstance(conditions, dict):
        raise ValueError('comparison requires resolved contract and condition mappings')
    declaration = validate_policy(policy) if policy is not None else None
    invariant, variables = deepcopy(original), {}
    for variable in declaration['variables'] if declaration else []:
        parent = invariant
        keys = variable['path'].split('.')
        for key in keys[:-1]:
            if not isinstance(parent, dict) or key not in parent:
                raise ValueError(
                    f'comparison path {variable["path"]}: absent from materialized contract')
            parent = parent[key]
        if not isinstance(parent, dict) or keys[-1] not in parent:
            raise ValueError(
                f'comparison path {variable["path"]}: absent from materialized contract')
        effective = parent[keys[-1]]
        if isinstance(effective, dict):
            raise ValueError(f'comparison path {variable["path"]}: containers cannot be masked')
        if 'values' in variable and all(
                contract_hash(effective) != contract_hash(v) for v in variable['values']):
            raise ValueError(
                f'comparison variable {variable["name"]}: effective value not permitted')
        variables[variable['name']] = {'path': variable['path'], 'value': deepcopy(effective)}
        del parent[keys[-1]]
    original_hash = contract_hash(original)
    return {'schema_version': 1, 'policy': declaration,
            'original_contract': deepcopy(original), 'original_hash': original_hash,
            'variables': variables, 'invariant_contract': invariant,
            'protected_conditions': deepcopy(conditions),
            'invariant_hash': contract_hash({'contract': invariant, 'conditions': conditions})
            if declaration else original_hash}


def different_paths(first: Any, second: Any, prefix: str = '') -> list[str]:
    """Explain differences; arrays are atomic and never authorize extra masking."""
    if isinstance(first, dict) and isinstance(second, dict):
        paths = []
        for key in sorted(first.keys() | second.keys()):
            path = f'{prefix}.{key}' if prefix else key
            if key not in first or key not in second:
                paths.append(path)
            else:
                paths.extend(different_paths(first[key], second[key], path))
        return paths
    return [] if contract_hash(first) == contract_hash(second) else [prefix]


def _metric_evidence(run: dict, view: dict) -> list[str]:
    """Require available observations, not a favorable tracking outcome."""
    reasons = []
    metrics = run.get('metrics', {})
    if (type(metrics.get('schema_version')) is not int or metrics['schema_version'] != 1
            or not metrics.get('contract')):
        reasons.append('metric definitions unavailable or unsupported')
    required = view['protected_conditions'].get('evaluation', {}).get('required_metrics', [])
    for name in required:
        metric = metrics.get('metrics', {}).get(name, {})
        statistic = metric.get('rmse' if name == 'prediction_rmse' else 'mean')
        if (not metric.get('available') or metric.get('sample_count', 0) <= 0
                or not metric.get('unit') or type(statistic) not in (float, int)
                or not math.isfinite(statistic)):
            reasons.append(f'required metric {name}: missing valid observations')
    if metrics.get('safety', {}).get('coverage_complete') is not True:
        reasons.append('required physical joint observation coverage incomplete')
    window = metrics.get('evaluation_window', {})
    duration = view['original_contract'].get('duration')
    if (window.get('unit') != 's' or type(window.get('start')) not in (int, float)
            or type(window.get('end')) not in (int, float) or duration is None
            or not math.isclose(window['end'] - window['start'], duration,
                                rel_tol=1e-9, abs_tol=1e-9)):
        reasons.append('incomplete or incompatible scoring window')
    return reasons


def scientific_reasons(runs: list[dict], variants: list[str], seeds: list[int],
                       policy: dict, *, configuration_only: bool = False) -> list[str]:
    """Recompute from original evidence; do not trust a supplied masked hash."""
    declaration = validate_policy(policy)
    reasons, views = [], {}
    for run in runs:
        label = f'{run["variant"]} seed {run["seed"]}'
        if run['variant'] not in variants or run['seed'] not in seeds:
            reasons.append(f'{label}: outside declared variants/seed policy')
            continue
        saved = run.get('comparison')
        try:
            if not saved or saved['policy'] != declaration:
                raise ValueError('missing or different comparison declaration')
            view = comparison_views(saved['original_contract'], declaration,
                                    saved['protected_conditions'])
            if (run['comparison_contract'] != view['original_hash']
                    or saved['original_hash'] != view['original_hash']
                    or saved['invariant_hash'] != view['invariant_hash']
                    or saved['invariant_contract'] != view['invariant_contract']
                    or saved['variables'] != view['variables']):
                raise ValueError('comparison metadata/hash does not match original evidence')
            if not configuration_only and not isinstance(run.get('metrics'), dict):
                raise ValueError('metric evidence missing')
            views[(run['variant'], run['seed'])] = view
            if not configuration_only:
                reasons.extend(f'{label}: {reason}' for reason in _metric_evidence(run, view))
        except (ValueError, TypeError, KeyError) as error:
            reasons.append(f'{label}: {error}')
    for seed in seeds:
        paired = [run for run in runs if run['seed'] == seed and run['variant'] in variants]
        if not paired:
            continue
        first = paired[0]
        for run in paired[1:]:
            a, b = views.get((first['variant'], seed)), views.get((run['variant'], seed))
            if a is None or b is None:
                continue
            for field in ['invariant_contract', 'protected_conditions']:
                paths = different_paths(a[field], b[field])
                if paths:
                    reasons.append(f'seed {seed}: {field} mismatch: ' + ', '.join(paths))
            if configuration_only:
                continue
            if first.get('metrics', {}).get('contract') != run.get('metrics', {}).get('contract'):
                reasons.append(f'seed {seed}: metric definitions mismatch')
            first_metrics = first.get('metrics', {}).get('metrics', {})
            for name, metric in run.get('metrics', {}).get('metrics', {}).items():
                if name in first_metrics and metric.get('unit') != first_metrics[name].get('unit'):
                    reasons.append(f'seed {seed}: metric {name} units mismatch')
    return reasons


def comparison_reasons(runs: list[dict], variants: list[str], seeds: list[int],
                       policy: dict | None = None, *,
                       configuration_only: bool = False) -> list[str]:
    """Share existing pair/policy checks; pre-run mode omits runtime evidence gates."""
    reasons, seen = [], {}
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
        if (not configuration_only
                and any(run['execution_status'] != 'completed' for run in paired)):
            reasons.append(f'seed {seed}: interrupted execution')
        if policy is None and len({run['comparison_contract'] for run in paired}) != 1:
            reason = f'seed {seed}: scenario/physical/measurement-window mismatch'
            if configuration_only:
                paths = sorted({path for run in paired[1:] for path in different_paths(
                    paired[0]['comparison']['original_contract'],
                    run['comparison']['original_contract'])})
                reason += ': ' + ', '.join(paths)
            reasons.append(reason)
    if policy is not None:
        scientific = scientific_reasons(runs, variants, seeds, policy,
                                        configuration_only=configuration_only)
        reasons.extend(scientific)
    return reasons
