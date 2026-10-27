"""Trusted component registration and a small, explicit parameter-schema subset."""

from collections.abc import Mapping
from copy import deepcopy
import math
from numbers import Integral
from numbers import Real


def schema_for(defaults, *, units=None, constraints=None):
    """Describe known parameter leaves; owners supply units and numerical bounds."""
    units = units or {}
    constraints = constraints or {}

    def infer(value, path):
        if isinstance(value, Mapping):
            result = {
                'type': 'object',
                'additionalProperties': False,
                'properties': {
                    k: infer(v, f'{path}.{k}' if path else k) for k, v in value.items()
                },
                'required': list(value),
            }
        elif isinstance(value, (list, tuple)):
            result = {'type': 'array'}
            if value:
                result['items'] = infer(value[0], path + '[]')
        elif isinstance(value, bool):
            result = {'type': 'boolean'}
        elif isinstance(value, int):
            result = {'type': 'integer'}
        elif isinstance(value, float):
            result = {'type': 'number'}
        elif isinstance(value, str):
            result = {'type': 'string'}
        else:
            raise ValueError(f'{path}: unsupported parameter default type {type(value).__name__}')
        if path in units:
            result['unit'] = units[path]
        result.update(deepcopy(constraints.get(path, {})))
        return result

    return infer(defaults, '')


def validate_schema(value, schema, path='parameters'):
    """
    Validate the supported schema subset, returning a detached value.

    Supports object/properties/required/additionalProperties, array/items and item
    counts, finite number/integer, boolean/string, enum and inclusive/exclusive
    numeric bounds. Domain constructors retain dimensional and coupled constraints.
    This is deliberately not an arbitrary JSON Schema or executable YAML evaluator.
    """
    if 'oneOf' in schema:
        matches = 0
        for option in schema['oneOf']:
            try:
                validate_schema(value, option, path)
                matches += 1
            except ValueError:
                pass
        if matches != 1:
            raise ValueError(f'{path}: expected exactly one permitted parameter representation')
    kind = schema.get('type')
    valid = {
        'object': lambda: isinstance(value, Mapping),
        'array': lambda: isinstance(value, (list, tuple))
        or (hasattr(value, 'ndim') and value.ndim > 0),
        'number': lambda: isinstance(value, Real)
        and not isinstance(value, bool)
        and math.isfinite(value),
        'integer': lambda: isinstance(value, Integral) and not isinstance(value, bool),
        'boolean': lambda: type(value) is bool,
        'string': lambda: isinstance(value, str),
    }
    if kind is not None and (kind not in valid or not valid[kind]()):
        raise ValueError(f'{path}: expected finite {kind}, got {value!r}')
    if 'enum' in schema and value not in schema['enum']:
        raise ValueError(f"{path}: expected one of {schema['enum']}, got {value!r}")
    if kind == 'object':
        properties = schema.get('properties', {})
        missing = set(schema.get('required', ())) - set(value)
        unknown = set(value) - set(properties)
        if missing or (unknown and schema.get('additionalProperties') is False):
            raise ValueError(f'{path}: unknown keys {sorted(unknown)}; missing {sorted(missing)}')
        for key, item in value.items():
            if key in properties:
                validate_schema(item, properties[key], f'{path}.{key}')
    if kind == 'array':
        for key, predicate in [
            ('minItems', lambda n: len(value) >= n),
            ('maxItems', lambda n: len(value) <= n),
        ]:
            if key in schema and not predicate(schema[key]):
                raise ValueError(f'{path}: violates {key}={schema[key]}')
        if 'items' in schema:
            for index, item in enumerate(value):
                validate_schema(item, schema['items'], f'{path}[{index}]')
    for key, predicate in [
        ('minimum', lambda bound: value >= bound),
        ('maximum', lambda bound: value <= bound),
        ('exclusiveMinimum', lambda bound: value > bound),
        ('exclusiveMaximum', lambda bound: value < bound),
    ]:
        if key in schema and not predicate(schema[key]):
            qualifier = (
                '; must be positive'
                if key == 'exclusiveMinimum' and schema[key] == 0
                else '; must be non-negative' if key == 'minimum' and schema[key] == 0 else ''
            )
            raise ValueError(f'{path}: violates {key}={schema[key]}{qualifier}')
    return deepcopy(value)


class ComponentRegistry:
    """One trusted constructor table with detached schema and capability metadata."""

    def __init__(self):
        self._entries = {}

    def register(
        self,
        name,
        builder,
        *,
        description='',
        defaults=None,
        schema=None,
        capabilities=None,
        requirements=None,
        dependencies=(),
    ):
        """Register code explicitly; YAML identifiers never resolve import paths."""
        if not isinstance(name, str) or not name.strip():
            raise ValueError('Component name must be a nonempty string')
        if not callable(builder):
            raise TypeError('Component constructor must be callable')
        if name in self._entries:
            raise ValueError(f'Component {name!r} is already registered')
        defaults = {} if defaults is None else deepcopy(defaults)
        if schema is not None:
            validate_schema(defaults, schema, name)
        self._entries[name] = {
            'builder': builder,
            'dependencies': tuple(dependencies),
            'id': name,
            'description': description,
            'defaults': defaults,
            'schema': deepcopy(schema),
            'capabilities': deepcopy(capabilities or {}),
            'requirements': deepcopy(requirements or {}),
        }

    def _entry(self, name):
        if name not in self._entries:
            raise KeyError(f"Unknown component {name!r}; available: {', '.join(self.names())}")
        return self._entries[name]

    def names(self):
        """Return stable identifiers without exposing constructors."""
        return tuple(sorted(self._entries))

    def describe(self, name):
        """Return detached defaults, schema and non-executable capability metadata."""
        return deepcopy({k: v for k, v in self._entry(name).items() if k != 'builder'})

    def describe_all(self):
        """List registered components in stable order."""
        return [self.describe(name) for name in self.names()]

    def compatibility(self, name, context):
        """Check an explicit adapter context; missing required capabilities reject."""
        reasons = []
        for key, required in self._entry(name)['requirements'].items():
            actual = context.get(key)
            if key not in context:
                reasons.append(f'{key}: required adapter capability missing')
            elif isinstance(required, Mapping):
                try:
                    validate_schema(actual, required, key)
                except (ValueError, TypeError) as error:
                    reasons.append(str(error))
            elif actual != required:
                reasons.append(f'{key}: requires {required!r}, got {actual!r}')
        return {'compatible': not reasons, 'reasons': reasons}

    def validate(self, name, parameters=None, context=None):
        """Validate named parameter overrides without changing registry defaults."""
        entry = self._entry(name)
        if parameters is None:
            parameters = {}
        if not isinstance(parameters, Mapping):
            raise TypeError(f'{name}: parameters must be a mapping')
        if context is not None:
            check = self.compatibility(name, context)
            if not check['compatible']:
                raise ValueError(f"{name}: incompatible: {'; '.join(check['reasons'])}")
        if entry['schema'] is None:
            return {**deepcopy(entry['defaults']), **dict(parameters)}

        def merge(defaults, changes):
            result = deepcopy(defaults)
            for key, value in changes.items():
                result[key] = (
                    merge(result[key], value)
                    if isinstance(result.get(key), Mapping) and isinstance(value, Mapping)
                    else deepcopy(value)
                )
            return result

        merged = merge(entry['defaults'], parameters)
        return validate_schema(merged, entry['schema'], name)

    def create(self, name, parameters=None, *, context=None, **dependencies):
        """Construct validated parameters plus separately declared runtime dependencies."""
        entry = self._entry(name)
        unknown = set(dependencies) - set(entry['dependencies'])
        overlap = set(parameters or {}) & set(dependencies)
        if unknown or overlap:
            raise ValueError(f'{name}: invalid/shadowed dependencies {sorted(unknown | overlap)}')
        values = self.validate(name, parameters, context)
        return entry['builder'](**values, **dependencies)
