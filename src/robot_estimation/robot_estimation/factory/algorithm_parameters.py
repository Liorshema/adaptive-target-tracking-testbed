"""Freeze intrinsic algorithm settings at the existing flat ROS parameter boundary."""

from collections.abc import Mapping
from copy import deepcopy
import json


def encode_algorithm_parameters(algorithm, parameters):
    """Serialize an identity and complete resolved mapping, retaining nested value types."""
    if not isinstance(algorithm, str) or not algorithm or not isinstance(parameters, Mapping):
        raise ValueError('algorithm parameters require an identity and parameter mapping')
    return json.dumps(
        {'algorithm': algorithm, 'parameters': dict(parameters)},
        sort_keys=True, separators=(',', ':'), allow_nan=False,
    )


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'algorithm parameters: duplicate key {key!r}')
        result[key] = value
    return result


def decode_algorithm_parameters(value, algorithm, registry):
    """Validate frozen settings without filling absent leaves from installed defaults."""
    if not isinstance(value, str):
        raise ValueError('algorithm parameters must be an encoded JSON object')
    document = json.loads(value, object_pairs_hook=_unique_object)
    if (
        not isinstance(document, dict)
        or set(document) != {'algorithm', 'parameters'}
        or document['algorithm'] != algorithm
        or not isinstance(document['parameters'], dict)
    ):
        raise ValueError('algorithm parameters: invalid mapping or algorithm identity mismatch')
    # JSON's Python decoder permits nonfinite constants/overflow. Reject them
    # recursively, including containers whose supported schema leaves items open.
    json.dumps(document, allow_nan=False)
    return registry.validate(algorithm, document['parameters'], use_defaults=False)


def freeze_algorithm_parameters(parameters, selections, *, registries=None):
    """Resolve each selected role once; existing frozen settings never consult defaults."""
    if registries is None:
        from robot_estimation.factory.estimator_factory import create_estimation_registries

        registries = create_estimation_registries()
    result = deepcopy(parameters)
    for role, selection in selections.items():
        info = registries[role + '_estimator'].describe(selection)
        algorithm = info['capabilities']['algorithm']
        key = role + '_algorithm_parameters'
        effective = (
            decode_algorithm_parameters(result[key], algorithm, registries['estimator'])
            if key in result
            else info['defaults']['algorithm_parameters']
        )
        result[key] = encode_algorithm_parameters(algorithm, effective)
    return result
