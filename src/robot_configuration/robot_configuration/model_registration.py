"""Trusted owner-local declarations feed the existing ComponentRegistry only."""

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
from dataclasses import field
from importlib import import_module
import math
from numbers import Real
import pkgutil

from .components import validate_schema


def _schema(schema, path='schema'):
    if not isinstance(schema, Mapping):
        raise ValueError(f'{path}: expected parameter schema')
    kinds = {'object', 'array', 'number', 'integer', 'boolean', 'string'}
    allowed = {
        'type',
        'properties',
        'required',
        'additionalProperties',
        'items',
        'minItems',
        'maxItems',
        'minimum',
        'maximum',
        'exclusiveMinimum',
        'exclusiveMaximum',
        'enum',
        'unit',
        'description',
    }
    if set(schema) - allowed:
        raise ValueError(f'{path}: unknown schema fields {sorted(set(schema) - allowed)}')
    if schema.get('type') not in kinds:
        raise ValueError(f'{path}: explicit supported type required')
    if schema['type'] == 'object':
        if (
            not isinstance(schema.get('properties'), Mapping)
            or schema.get('additionalProperties') is not False
        ):
            raise ValueError(
                f'{path}: explicit properties and additionalProperties=false required'
            )
        required = schema.get('required', [])
        if (
            not isinstance(required, (list, tuple))
            or any(not isinstance(k, str) for k in required)
            or len(set(required)) != len(required)
            or set(required) - set(schema['properties'])
        ):
            raise ValueError(f'{path}: invalid required parameters')
        for key, value in schema['properties'].items():
            _schema(value, path + '.' + key)
    if schema['type'] == 'array' and 'items' not in schema:
        raise ValueError(f'{path}: array schema requires explicit item types')
    for key in ('minimum', 'maximum', 'exclusiveMinimum', 'exclusiveMaximum'):
        if key in schema and (
            not isinstance(schema[key], Real)
            or isinstance(schema[key], bool)
            or not math.isfinite(schema[key])
        ):
            raise ValueError(f'{path}: {key} must be a finite numerical schema bound')
    for key in ('minItems', 'maxItems'):
        if key in schema and (type(schema[key]) is not int or schema[key] < 0):
            raise ValueError(f'{path}: {key} must be a nonnegative schema integer')
    if 'enum' in schema:
        values = schema['enum']
        if not isinstance(values, (list, tuple)) or not values:
            raise ValueError(f'{path}: schema enum must be a nonempty sequence')
        member_schema = {k: v for k, v in schema.items() if k != 'enum'}
        for value in values:
            validate_schema(value, member_schema, path + '.enum')
    if 'items' in schema:
        _schema(schema['items'], path + '[]')


@dataclass(frozen=True)
class ModelRegistration:
    """An explicit register() argument bundle; contains no parameter defaults or global table."""

    identifier: str
    builder: object
    schema: dict
    capabilities: dict
    requirements: dict = field(default_factory=dict)
    dependencies: tuple = ()
    description: str = ''

    def __post_init__(self):
        """Reject incomplete declarations before loading owner defaults or building models."""
        if not isinstance(self.identifier, str) or not self.identifier.strip():
            raise ValueError('model identifier must be nonempty')
        if not callable(self.builder):
            raise TypeError('model builder must be callable')
        _schema(self.schema)
        if self.schema['type'] != 'object':
            raise ValueError('model schema must describe a parameter object')
        if (
            not isinstance(self.capabilities, Mapping)
            or type(self.capabilities.get('deployable')) is not bool
            or not isinstance(self.capabilities.get('runtime_validation'), str)
            or not self.capabilities['runtime_validation'].strip()
        ):
            raise ValueError('model capabilities require deployable and runtime_validation')
        if not isinstance(self.requirements, Mapping):
            raise ValueError('model requirements must be a mapping')
        if (
            not isinstance(self.dependencies, (list, tuple))
            or any(not isinstance(k, str) or not k for k in self.dependencies)
            or len(set(self.dependencies)) != len(self.dependencies)
        ):
            raise ValueError('model dependencies must name unique injected arguments')
        for key in ('schema', 'capabilities', 'requirements'):
            object.__setattr__(self, key, deepcopy(dict(getattr(self, key))))
        object.__setattr__(self, 'dependencies', tuple(self.dependencies))


def register_models(registry, declarations, owner_groups):
    """Read authoritative YAML defaults and use the existing registry's validation/storage."""
    if not isinstance(declarations, (tuple, list)) or not isinstance(owner_groups, Mapping):
        raise ValueError(
            'COMPONENTS and owner parameter groups must be explicit sequences/mappings'
        )
    for declaration in declarations:
        if not isinstance(declaration, ModelRegistration):
            raise ValueError('COMPONENTS entries must be ModelRegistration declarations')
        group = owner_groups.get(declaration.identifier)
        if not isinstance(group, Mapping) or not isinstance(group.get('parameters'), Mapping):
            raise ValueError(f'{declaration.identifier}.parameters: missing owner YAML defaults')
        defaults = validate_schema(group['parameters'], declaration.schema, declaration.identifier)
        registry.register(
            declaration.identifier,
            declaration.builder,
            description=declaration.description,
            defaults=defaults,
            schema=declaration.schema,
            capabilities=declaration.capabilities,
            requirements=declaration.requirements,
            dependencies=declaration.dependencies,
        )


def discover_models(registry, namespace, owner_groups):
    """Load sorted trusted owner modules; experiment YAML never supplies import paths."""
    package = import_module(namespace)
    if not hasattr(package, '__path__'):
        raise ValueError('trusted model namespace must be a package')
    for module in sorted(
        pkgutil.iter_modules(package.__path__, namespace + '.'), key=lambda m: m.name
    ):
        selected = import_module(module.name)
        if hasattr(selected, 'COMPONENTS'):
            register_models(registry, selected.COMPONENTS, owner_groups)
