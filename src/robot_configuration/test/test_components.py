"""Schemas and trusted registrations form one detached component boundary."""
import importlib

import pytest


def api():
    return importlib.import_module('robot_configuration.components')


def registry():
    module = api()
    result = module.ComponentRegistry()
    defaults = {'gain': 2.0, 'axes': [1.0, 0.0]}
    result.register(
        'test', lambda gain, axes: (gain, axes), description='Test only',
        defaults=defaults,
        schema=module.schema_for(
            defaults, units={'gain': 's^-1'}, constraints={'gain': {'minimum': 0.0}}),
        capabilities={'deployment': 'test_only', 'output': 'vector'},
        requirements={'state_layout': 'pv'})
    return result


def test_registered_defaults_and_descriptions_are_detached():
    result = registry()
    description = result.describe('test')
    assert description['schema']['properties']['gain']['unit'] == 's^-1'
    description['defaults']['axes'][0] = 9.0
    built = result.create('test', {'gain': 3.0})
    built[1][0] = 8.0
    assert result.create('test') == (2.0, [1.0, 0.0])
    assert result.describe_all()[0]['id'] == 'test'


@pytest.mark.parametrize('parameters', [
    {'gain': True}, {'gain': float('nan')}, {'gain': -1.0}, {'gain': '2'},
    {'axes': [False]}, {'typo': 1.0},
])
def test_schema_rejects_invalid_component_parameters(parameters):
    with pytest.raises(ValueError, match='test'):
        registry().create('test', parameters)


def test_registration_and_capability_rejection_are_explicit():
    result = registry()
    with pytest.raises(ValueError, match='already'):
        result.register('test', lambda: None)
    with pytest.raises((KeyError, ValueError), match='unknown|Unknown'):
        result.create('arbitrary.module:Class')
    check = result.compatibility('test', {'state_layout': 'other'})
    assert check['compatible'] is False
    assert check['reasons']
    with pytest.raises(ValueError, match='state_layout'):
        result.create('test', context={'state_layout': 'other'})
    assert result.create('test', context={'state_layout': 'pv'})[0] == 2.0


def test_dependencies_are_separate_and_cannot_be_shadowed():
    module = api()
    result = module.ComponentRegistry()
    dependency = object()
    result.register('test', lambda gain, model: (gain, model), defaults={'gain': 2.0},
                    schema=module.schema_for({'gain': 2.0}), dependencies=('model',))
    assert result.create('test', model=dependency) == (2.0, dependency)
    with pytest.raises(ValueError, match='model'):
        result.create('test', {'model': object()}, model=dependency)


def test_owner_resource_lookup_handles_source_and_copied_installation(tmp_path):
    from robot_configuration import resources
    for layout in ['source', 'installed']:
        root = tmp_path / layout
        if layout == 'source':
            module = root / 'demo/demo/factory/components.py'
            config = root / 'demo/config/defaults.yaml'
        else:
            module = root / 'lib/python3.12/site-packages/demo/factory/components.py'
            config = root / 'share/demo/config/defaults.yaml'
        module.parent.mkdir(parents=True)
        module.touch()
        config.parent.mkdir(parents=True)
        config.write_text('value: 1\n')
        assert resources.package_config_path(module, 'demo', 'defaults.yaml') == config


def test_schema_alternatives_reject_booleans_in_vector_or_matrix():
    module = api()
    schema = {'type': 'array', 'items': {'oneOf': [
        {'type': 'number'}, {'type': 'array', 'items': {'type': 'number'}}]}}
    assert module.validate_schema([[1., 0.], [0., 1.]], schema) == [[1., 0.], [0., 1.]]
    with pytest.raises(ValueError):
        module.validate_schema([True, 1.], schema)
