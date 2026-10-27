"""Validate explicit primitive parameter contracts without ROS dependencies."""

from copy import deepcopy
import math


def validate_parameters(parameters, schema, *, required=(), path='parameters'):
    """Validate and copy parameters, normalizing DOUBLE numbers for ROS declaration."""
    if not isinstance(parameters, dict):
        raise ValueError(f'{path}: expected parameter mapping')
    unknown = set(parameters) - set(schema)
    missing = set(required) - set(parameters)
    if unknown or missing:
        raise ValueError(f'{path}: unknown keys {sorted(unknown)}; missing keys {sorted(missing)}')
    scalar = {'STRING': str, 'BOOL': bool, 'INTEGER': int, 'DOUBLE': (int, float)}
    normalized = deepcopy(parameters)
    for key, value in parameters.items():
        kind = schema[key]
        array = kind.endswith('_ARRAY')
        basic = kind.removesuffix('_ARRAY')
        values = value if array else [value]
        if array and not isinstance(value, list):
            raise ValueError(f'{path}.{key}: expected {kind}, got {value!r}')
        for item in values:
            if (not isinstance(item, scalar[basic]) or
                    (basic != 'BOOL' and isinstance(item, bool)) or
                    (basic in {'DOUBLE', 'INTEGER'} and not math.isfinite(item))):
                raise ValueError(f'{path}.{key}: expected finite {kind}, got {value!r}')
        if basic == 'DOUBLE':
            normalized[key] = [float(item) for item in value] if array else float(value)
    return normalized


def override_parameters(defaults, overrides, *, path='overrides'):
    """
    Copy known parameter leaves; mappings merge and lists replace in full.

    Structural/physical validation remains with the consuming owner factory.
    Selection must use composition, not invent new parameter keys here.
    """
    if not isinstance(defaults, dict) or not isinstance(overrides, dict):
        raise ValueError(f'{path}: expected mapping')
    unknown = set(overrides) - set(defaults)
    if unknown:
        raise ValueError(f'{path}: unknown keys {sorted(unknown)}')
    result = deepcopy(defaults)
    for key, value in overrides.items():
        previous = defaults[key]
        label = f'{path}.{key}'
        if isinstance(previous, dict):
            result[key] = override_parameters(previous, value, path=label)
            continue
        if type(previous) in (int, float):
            valid = type(value) in (int, float) and math.isfinite(value)
            if type(previous) is int:
                valid = valid and type(value) is int
        else:
            valid = type(value) is type(previous)
        if not valid:
            raise ValueError(f'{label}: invalid parameter type/value')
        if isinstance(value, list):
            def finite(item):
                if isinstance(item, list):
                    return all(finite(v) for v in item)
                return type(item) not in (int, float) or math.isfinite(item)
            if not finite(value):
                raise ValueError(f'{label}: nonfinite list values')
        result[key] = deepcopy(value)
    return result
