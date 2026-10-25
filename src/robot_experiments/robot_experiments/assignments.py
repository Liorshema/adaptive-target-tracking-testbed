"""Validated assignments into two existing sensor owners; no general YAML mutation."""

import math
from copy import deepcopy
from pathlib import Path

import numpy as np

from robot_simulation.measurements.planar_localization import validate_configuration
from robot_simulation.measurements.synthetic_measurement import SyntheticMeasurementGenerator

import yaml

from .comparison import SENSOR_PATHS, contract_hash, validate_policy


def _numeric(value):
    """Reject coercion of strings/bools/null or nonfinite sensor values."""
    if isinstance(value, list) and value:
        for item in value:
            _numeric(item)
        return
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError('assignment values must be finite numbers or nonempty numeric lists')


def bind_assignments(policy: dict, parameters: dict) -> dict:
    """Bind every name exactly once to the existing permission and permitted-value set."""
    declaration = validate_policy(policy)
    names = {v['name'] for v in declaration['variables']}
    if not isinstance(parameters, dict) or not all(isinstance(k, str) for k in parameters):
        raise ValueError('variant.parameters: expected named assignment mapping')
    if set(parameters) != names:
        raise ValueError(f'assignment names mismatch: missing {sorted(names-set(parameters))}; '
                         f'unknown {sorted(set(parameters)-names)}')
    bound = {}
    for variable in declaration['variables']:
        name = variable['name']
        if 'values' not in variable:
            raise ValueError(f'schema 2 assignment {name} requires explicit permitted values')
        value = parameters[name]
        _numeric(value)
        if all(contract_hash(value) != contract_hash(v) for v in variable['values']):
            raise ValueError(f'assignment {name}: value not permitted by comparison declaration')
        bound[name] = {'path': variable['path'], 'requested_value': deepcopy(value),
                       'declared_values': deepcopy(variable['values'])}
    return bound


def check_parameter_keys(source: str | Path) -> None:
    """Inspect declaration structure for duplicates hidden by YAML last-key-wins loading.

    This schema-2-only check does not resolve or construct configuration values.
    Shared inheritance/loading semantics and schema-1 parsing remain unchanged.
    """
    def field(node, key):
        if not isinstance(node, yaml.MappingNode):
            return None
        fields = {}
        for name, value in node.value:
            if name.tag == 'tag:yaml.org,2002:merge':
                raise ValueError(f'{source}: ambiguous assignment merge keys are unsupported')
            if name.value in fields:
                raise ValueError(f'{source}: duplicate assignment/declaration key {name.value!r}')
            fields[name.value] = value
        return fields.get(key)

    root = yaml.compose(Path(source).read_text(), Loader=yaml.SafeLoader)
    field(root, 'schema_version')
    experiment = field(root, 'experiment')
    variants = field(experiment, 'variants')
    if isinstance(variants, yaml.SequenceNode):
        for variant in variants.value:
            parameters = field(variant, 'parameters')
            if isinstance(parameters, yaml.MappingNode):
                field(parameters, '')
    variables = field(field(experiment, 'comparison'), 'variables')
    if isinstance(variables, yaml.SequenceNode):
        for variable in variables.value:
            field(variable, 'path')


def apply_assignments(bound: dict, localization: dict | None, measurements: dict,
                      owner_sources: dict, parameter_sources: dict) -> dict:
    """Apply only authorized sensor leaves, validate actual owners, retain provenance."""
    records, changed = {}, set()
    target = measurements['measurements']['target_position']
    dimension = len(target['noise']['covariance'])
    for name, assignment in bound.items():
        path = assignment['path']
        if path not in SENSOR_PATHS:
            raise ValueError(f'assignment {name}: unsupported sensor path {path}')
        if path.startswith('localization_sensor.localization_sensor.'):
            if (localization is None
                    or localization['localization_sensor']['enabled'] is not True):
                raise ValueError(
                    f'assignment {name}: localization producer unavailable or disabled')
            parent = localization['localization_sensor']
            key, owner = path.rsplit('.', 1)[1], 'localization_sensor'
        elif path == 'measurements.measurements.target_position.noise.covariance':
            if target['enabled'] is not True:
                raise ValueError(f'assignment {name}: target measurement producer unavailable')
            parent, key, owner = target['noise'], 'covariance', 'measurements'
        else:
            raise ValueError(f'assignment {name}: no supported sensor owner for {path}')
        if key not in parent:
            raise ValueError(f'assignment {name}: absent from selected owner')
        records[name] = {**assignment, 'baseline_value': deepcopy(parent[key]),
                         'owner_source': owner_sources[owner],
                         'source': parameter_sources[name]}
        parent[key] = deepcopy(assignment['requested_value'])
        changed.add(owner)
    if 'localization_sensor' in changed:
        validate_configuration(localization['localization_sensor'])
    if 'measurements' in changed:
        # Same observation constructor as deployment preflight; dimension from actual owner.
        SyntheticMeasurementGenerator(np.eye(dimension), target['noise']['covariance'],
                                      np.random.default_rng(target['random_seed']))
    return records
