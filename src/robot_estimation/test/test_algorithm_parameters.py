"""Resolved intrinsic settings must survive default changes without a second merge."""

from copy import deepcopy
import json

import pytest
from robot_configuration.components import ComponentRegistry


def registry(defaults=None):
    result = ComponentRegistry()
    result.register(
        'configured', lambda **values: values,
        defaults={'gain': 0.5} if defaults is None else defaults,
        schema={'type': 'object', 'additionalProperties': False,
                'properties': {'gain': {'type': 'number', 'minimum': 0.0, 'maximum': 1.0}}},
    )
    return result


def test_resolved_creation_does_not_fill_current_defaults():
    source = registry()
    assert source.create('configured') == {'gain': 0.5}
    assert source.create('configured', {}, use_defaults=False) == {}
    assert source.create('configured', {'gain': 0.0}, use_defaults=False) == {'gain': 0.0}


def test_transport_validates_frozen_identity_and_uses_no_defaults():
    from robot_estimation.factory.algorithm_parameters import (
        decode_algorithm_parameters, encode_algorithm_parameters,
    )

    source = registry()
    frozen = encode_algorithm_parameters('configured', {'gain': 0.25})
    assert json.loads(frozen) == {'algorithm': 'configured', 'parameters': {'gain': 0.25}}
    assert decode_algorithm_parameters(frozen, 'configured', source) == {'gain': 0.25}
    assert decode_algorithm_parameters(frozen, 'configured', registry({'gain': 0.9})) == {
        'gain': 0.25,
    }
    # Optional schema leaves stay absent, even if a newer default supplies one.
    empty = encode_algorithm_parameters('configured', {})
    assert decode_algorithm_parameters(empty, 'configured', source) == {}
    with pytest.raises(ValueError, match='identity|algorithm'):
        decode_algorithm_parameters(frozen, 'other', source)


@pytest.mark.parametrize('value', [
    'null', '[]', '{}', '{"algorithm":"configured","parameters":[]}',
    '{"algorithm":"configured","parameters":{"gain":NaN}}',
    '{"algorithm":"configured","parameters":{"gain":true}}',
    '{"algorithm":"configured","parameters":{"gain":2.0}}',
    '{"algorithm":"configured","parameters":{"missing":1}}',
    '{"algorithm":"configured","parameters":{"gain":0.1,"gain":0.2}}',
    '{"algorithm":"configured","parameters":{},"extra":0}',
])
def test_bad_transport_rejects(value):
    from robot_estimation.factory.algorithm_parameters import decode_algorithm_parameters

    with pytest.raises((ValueError, TypeError)):
        decode_algorithm_parameters(value, 'configured', registry())


def test_resolution_freezes_each_role_independently_and_preserves_empty_settings():
    from robot_estimation.factory.algorithm_parameters import freeze_algorithm_parameters
    from robot_estimation.factory.estimator_factory import create_estimation_registries

    registries = create_estimation_registries()
    source = {'target_acceleration_noise_std': 0.2}
    before = deepcopy(source)
    resolved = freeze_algorithm_parameters(
        source, {'robot': 'robot_ekf', 'target': 'kalman'}, registries=registries,
    )
    assert source == before
    assert json.loads(resolved['robot_algorithm_parameters']) == {
        'algorithm': 'ekf', 'parameters': {},
    }
    assert json.loads(resolved['target_algorithm_parameters']) == {
        'algorithm': 'kf', 'parameters': {},
    }


def test_transport_retains_allowed_falsy_values_and_detaches_nested_maps():
    from robot_estimation.factory.algorithm_parameters import (
        decode_algorithm_parameters, encode_algorithm_parameters,
    )
    from robot_configuration.components import schema_for

    values = {'zero': 0.0, 'enabled': False, 'label': '', 'items': [], 'nested': {}}
    source = ComponentRegistry()
    source.register('falsy', lambda **values: values,
                    defaults=values, schema=schema_for(values))
    encoded = encode_algorithm_parameters('falsy', values)
    decoded = decode_algorithm_parameters(encoded, 'falsy', source)
    assert decoded == values
    decoded['nested']['changed'] = True
    assert source.describe('falsy')['defaults'] == values
    with pytest.raises(ValueError):
        decode_algorithm_parameters(
            encode_algorithm_parameters('falsy', {**values, 'zero': None}), 'falsy', source,
        )


def test_same_algorithm_role_resolution_is_independent_without_claiming_cross_role_deployment():
    from robot_estimation.factory.algorithm_parameters import (
        encode_algorithm_parameters, freeze_algorithm_parameters,
    )

    algorithms = registry()
    roles = {}
    for role in ('robot', 'target'):
        roles[role + '_estimator'] = ComponentRegistry()
        roles[role + '_estimator'].register(
            'configured', lambda: None,
            defaults={'algorithm_parameters': {'gain': 0.5}},
            capabilities={'algorithm': 'configured'},
        )
    maps = {
        'robot_algorithm_parameters': encode_algorithm_parameters('configured', {'gain': 0.0}),
        'target_algorithm_parameters': encode_algorithm_parameters('configured', {'gain': 0.75}),
    }
    before = deepcopy(maps)
    resolved = freeze_algorithm_parameters(
        maps, {'robot': 'configured', 'target': 'configured'},
        registries={**roles, 'estimator': algorithms},
    )
    assert resolved == before == maps
    assert algorithms.describe('configured')['defaults'] == {'gain': 0.5}


@pytest.mark.parametrize('constant', ['NaN', 'Infinity', '-Infinity', '1e999'])
def test_transport_rejects_nonfinite_values_in_schema_permitted_empty_arrays(constant):
    from robot_configuration.components import schema_for
    from robot_estimation.factory.algorithm_parameters import decode_algorithm_parameters

    algorithms = ComponentRegistry()
    algorithms.register('untyped', lambda items: items, defaults={'items': []},
                        schema=schema_for({'items': []}))
    encoded = '{"algorithm":"untyped","parameters":{"items":[' + constant + ']}}'
    with pytest.raises(ValueError):
        decode_algorithm_parameters(encoded, 'untyped', algorithms)


def test_actual_role_construction_does_not_merge_new_nested_defaults():
    from robot_estimation.factory.algorithm_factory import COMPONENTS, create_algorithm_registry
    from robot_estimation.factory.algorithm_parameters import encode_algorithm_parameters
    from robot_estimation.factory.estimator_factory import (
        create_estimation_registries, create_target_estimator,
    )

    original = next(item for item in COMPONENTS if item.identifier == 'kf')

    def build(options, **dependencies):
        result = original.builder(**dependencies)
        result.options = options
        return result

    algorithms = create_algorithm_registry()
    algorithms.register(
        'nested', build, defaults={'options': {'gain': 0.9, 'new': 9.0}},
        schema={'type': 'object', 'additionalProperties': False, 'required': ['options'],
                'properties': {'options': {
                    'type': 'object', 'additionalProperties': False, 'required': ['gain'],
                    'properties': {'gain': {'type': 'number'}, 'new': {'type': 'number'}},
                }}},
        dependencies=original.dependencies, capabilities=original.capabilities,
    )
    roles = create_estimation_registries(algorithms=algorithms)
    frozen = {'target_algorithm_parameters': encode_algorithm_parameters(
        'nested', {'options': {'gain': 0.25}})}
    actual = create_target_estimator(
        frozen, 'nested', registry=roles['target_estimator'], require_frozen=True)
    assert actual._filter.options == {'gain': 0.25}
    assert roles['estimator'].describe('nested')['defaults']['options'] == {
        'gain': 0.9, 'new': 9.0,
    }
