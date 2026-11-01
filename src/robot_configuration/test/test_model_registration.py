"""Trusted declarations must populate only the existing owner registry."""

import pytest
from robot_configuration.components import ComponentRegistry


def declaration(name='test_gain', **edits):
    from robot_configuration.model_registration import ModelRegistration

    values = {
        'identifier': name,
        'builder': lambda gain: gain,
        'schema': {
            'type': 'object',
            'properties': {'gain': {'type': 'number', 'minimum': 0}},
            'required': ['gain'],
            'additionalProperties': False,
        },
        'capabilities': {'deployable': True, 'runtime_validation': 'unit_tested_extension'},
    }
    values.update(edits)
    return ModelRegistration(**values)


def test_defaults_are_owned_by_yaml_and_registries_are_detached():
    from robot_configuration.model_registration import register_models

    d = declaration()
    groups = {'test_gain': {'parameters': {'gain': 2.5}}}
    a, b = ComponentRegistry(), ComponentRegistry()
    register_models(a, [d], groups)
    register_models(b, [d], groups)
    assert a.create('test_gain') == 2.5
    assert a.create('test_gain', {'gain': 0.0}) == 0.0
    assert groups['test_gain']['parameters']['gain'] == 2.5
    assert b.describe('test_gain')['defaults'] == {'gain': 2.5}
    with pytest.raises(ValueError, match='unknown'):
        a.create('test_gain', {'unknown': 1})
    with pytest.raises(ValueError, match='already registered'):
        register_models(a, [d], groups)


@pytest.mark.parametrize(
    'edits',
    [
        {'schema': None},
        {'schema': {}},
        {'schema': {'type': 'object'}},
        {
            'schema': {
                'type': 'object',
                'properties': {'gain': {'type': 'imaginary'}},
                'additionalProperties': False,
            }
        },
        {'capabilities': {}},
        {'capabilities': {'deployable': 'yes', 'runtime_validation': 'x'}},
        {'builder': None},
        {'dependencies': ('task', 'task')},
    ],
)
def test_bad_declarations_fail_clearly(edits):
    from robot_configuration.model_registration import register_models

    with pytest.raises((ValueError, TypeError)):
        register_models(
            ComponentRegistry(),
            [declaration(**edits)],
            {'test_gain': {'parameters': {'gain': 1.0}}},
        )


def test_missing_defaults_and_dependency_failures():
    from robot_configuration.model_registration import register_models

    with pytest.raises(ValueError, match='parameters'):
        register_models(ComponentRegistry(), [declaration()], {})
    d = declaration(builder=lambda gain, task: (gain, task), dependencies=('task',))
    registry = ComponentRegistry()
    register_models(registry, [d], {'test_gain': {'parameters': {'gain': 1.0}}})
    with pytest.raises(TypeError, match='task'):
        registry.create('test_gain')
    assert registry.create('test_gain', task='injected') == (1.0, 'injected')


def test_sorted_trusted_discovery_ignores_unregistered_modules(tmp_path, monkeypatch):
    from robot_configuration.model_registration import discover_models

    package = tmp_path / 'trusted_models'
    package.mkdir()
    (package / '__init__.py').write_text('')
    (package / 'ignored.py').write_text('VALUE = 1\n')
    template = """from robot_configuration.model_registration import ModelRegistration
COMPONENTS = (ModelRegistration(identifier={name!r}, builder=lambda gain: gain,
 schema={{'type':'object','properties':{{'gain':{{'type':'number'}}}},'additionalProperties':False}},
 capabilities={{'deployable':True,'runtime_validation':'unit_tested_extension'}}),)
"""
    for name in ['zeta', 'alpha']:
        (package / (name + '.py')).write_text(template.format(name=name))
    monkeypatch.syspath_prepend(str(tmp_path))
    registry = ComponentRegistry()
    discover_models(
        registry,
        'trusted_models',
        {n: {'parameters': {'gain': i}} for i, n in enumerate(['alpha', 'zeta'])},
    )
    assert registry.names() == ('alpha', 'zeta')
    assert registry.create('zeta') == 1
    assert not (tmp_path / 'executed').exists()


@pytest.mark.parametrize(
    'property_schema',
    [
        {'type': 'number', 'minimun': 0},
        {'type': 'number', 'minimum': 'zero'},
        {'type': 'number', 'minimum': float('nan')},
        {'type': 'array'},
        {'type': 'number', 'default': 2.0},
    ],
)
def test_incomplete_or_conflicting_parameter_metadata_rejected(property_schema):
    with pytest.raises(ValueError, match='schema'):
        declaration(
            schema={
                'type': 'object',
                'properties': {'gain': property_schema},
                'additionalProperties': False,
            }
        )


@pytest.mark.parametrize('values', ['safe', [], {'safe': True}, [1.0, float('nan')]])
def test_invalid_enum_schema_cannot_turn_membership_into_substring_matching(values):
    with pytest.raises(ValueError, match='schema'):
        declaration(schema={'type': 'object', 'additionalProperties': False,
                            'properties': {'gain': {'type': 'number', 'enum': values}}})
