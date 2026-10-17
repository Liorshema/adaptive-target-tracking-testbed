"""Validate explicit primitive parameter contracts without ROS dependencies."""

import math
from copy import deepcopy


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
