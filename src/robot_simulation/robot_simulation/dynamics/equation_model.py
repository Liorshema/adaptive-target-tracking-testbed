"""Canonical named state-space equations behind the existing Dynamics interface."""

import ast
import re
from collections.abc import Mapping
from copy import deepcopy
from numbers import Real

import numpy as np

from robot_simulation.interfaces.disturbance import ContextualAccelerationField
from robot_simulation.interfaces.dynamics import Dynamics

from .equation_expression import ExpressionCompiler, Node, RESERVED, constant, parse_expression
from .equation_units import QuantityType, SECOND, multiply, parse_unit, same

# Public metadata schema; the owner below validates dynamic names and coupled rules.
MODEL_SCHEMA = {
    'type': 'object',
    'additionalProperties': False,
    'required': ['id', 'type', 'schema_version', 'frame', 'state', 'parameters', 'equations'],
    'properties': {
        k: {'type': v}
        for k, v in {
            'id': 'string',
            'type': 'string',
            'schema_version': 'integer',
            'frame': 'string',
            'state': 'array',
            'parameters': 'object',
            'equations': 'object',
            'control': 'array',
            'context': 'object',
            'intermediates': 'object',
            'deployment': 'object',
            'outputs': 'object',
        }.items()
    },
}
PARAMETER_SCHEMA = {
    'type': 'object',
    'additionalProperties': False,
    'required': ['model'],
    'properties': {'model': MODEL_SCHEMA, 'influences': {'type': 'array', 'maxItems': 4,
                                                     'items': {'type': 'object'}}},
}


def _mapping(value, allowed, required, label):
    if not isinstance(value, Mapping) or set(value) - set(allowed) or set(required) - set(value):
        raise ValueError(
            f'{label}: invalid fields; allowed {sorted(allowed)}, required {sorted(required)}'
        )
    return value


def _name(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,63}', value):
        raise ValueError('Names must be ASCII identifiers of at most 64 characters')
    return value


def _shape(value):
    if value == []:
        return ()
    if (
        not isinstance(value, list)
        or len(value) != 1
        or type(value[0]) is not int
        or not 1 <= value[0] <= 64
    ):
        raise ValueError('shape must be [] (scalar) or [1..64] (flat vector)')
    return tuple(value)


def _numeric(value, shape, label):
    array = np.asarray(value)
    if array.dtype.kind not in 'fiu' or array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f'{label}: expected real finite value with shape {shape}')
    return array.astype(float, copy=True)


class EquationDynamics(Dynamics):
    """Prepare once, then evaluate f(t,x,u,c;theta) with finite, typed numerical inputs."""

    @staticmethod
    def prepare_parameters(parameters, defaults, *, use_defaults=True):
        """Treat a supplied full model as atomic; never merge equations from another model."""
        _mapping(
            parameters, {'model', 'influences'}, set() if use_defaults else {'model'}, 'equation parameters'
        )
        values = deepcopy(dict(parameters) if 'model' in parameters else defaults)
        if 'influences' in parameters:
            values['influences'] = deepcopy(parameters['influences'])
        EquationDynamics(**values)
        return values

    def __init__(self, *, model, environment_field=None, influences=None):
        """Validate and compile a detached, complete versioned mathematical definition."""
        from robot_configuration.components import validate_schema

        validate_schema(model, MODEL_SCHEMA, 'model')
        self._base_definition = deepcopy(dict(model))
        self.influences = deepcopy([] if influences is None else influences)
        if self.influences:
            from .derivative_influences import compose_influences
            model = compose_influences(model, self.influences)
        elif not isinstance(self.influences, list):
            raise ValueError('influences must be a list')
        self.requires_tracking_context = any(p['enabled'] for p in self.influences)
        self._definition = deepcopy(dict(model))
        m = self._definition
        if (
            m['type'] != 'equation_state_space'
            or type(m['schema_version']) is not int
            or m['schema_version'] != 1
        ):
            raise ValueError('Expected equation_state_space schema_version 1')
        self.model_id = _name(m['id'])
        self.frame_id = m['frame']
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError('model frame must be explicit and nonempty')
        self.environment_field = environment_field
        self._symbols = {}
        self._states, self._controls, self._contexts = [], [], {}
        self.state_dimension = self.control_dimension = 0
        initial = []

        def declare(name, spec, extra=(), required=()):
            _name(name)
            if name in RESERVED or name in self._symbols:
                raise ValueError(f'Duplicate/reserved symbol: {name}')
            _mapping(spec, {'unit', 'shape', *extra}, {'unit', 'shape', *required}, name)
            if 'frame' in spec and spec['frame'] != self.frame_id:
                raise ValueError(f'{name}: incompatible coordinate frame')
            kind = QuantityType(parse_unit(spec['unit']), _shape(spec['shape']))
            self._symbols[name] = Node(kind, lambda values, key=name: values[key])
            return kind

        if not isinstance(m['state'], list) or not 1 <= len(m['state']) <= 128:
            raise ValueError('state must be an ordered list of 1..128 named quantities')
        for spec in m['state']:
            if not isinstance(spec, Mapping) or 'name' not in spec:
                raise ValueError('State declaration requires name')
            name = spec['name']
            kind = declare(
                name, {k: v for k, v in spec.items() if k != 'name'}, ('initial',), ('initial',)
            )
            value = _numeric(spec['initial'], kind.shape, name + ' initial')
            size = value.size
            self._states.append(
                (name, kind, slice(self.state_dimension, self.state_dimension + size))
            )
            self.state_dimension += size
            initial.extend(value.reshape(-1))
        if self.state_dimension > 1024:
            raise ValueError('Model exceeds 1024 scalar state cost bound')
        self._initial_state = np.asarray(initial)
        controls = m.get('control', [])
        if not isinstance(controls, list) or len(controls) > 128:
            raise ValueError('control must be a bounded ordered list')
        for spec in controls:
            if not isinstance(spec, Mapping) or 'name' not in spec:
                raise ValueError('Control declaration requires name')
            name = spec['name']
            kind = declare(
                name, {k: v for k, v in spec.items() if k != 'name'}, ('frame',), ('frame',)
            )
            size = kind.shape[0] if kind.shape else 1
            self._controls.append(
                (name, kind, slice(self.control_dimension, self.control_dimension + size))
            )
            self.control_dimension += size
        context = m.get('context', {})
        if not isinstance(context, Mapping) or len(context) > 128:
            raise ValueError('context must be a bounded named mapping')
        for name, spec in context.items():
            self._contexts[name] = declare(name, spec, ('frame',), ('frame',))
        parameters = m['parameters']
        if not isinstance(parameters, Mapping) or len(parameters) > 128:
            raise ValueError('parameters must be a bounded named mapping')
        bounds = ('minimum', 'maximum', 'exclusiveMinimum', 'exclusiveMaximum')
        for name, spec in parameters.items():
            kind = declare(name, spec, ('value', *bounds), ('value',))
            value = _numeric(spec['value'], kind.shape, name + ' value')
            for bound in bounds:
                if bound not in spec:
                    continue
                limit = spec[bound]
                if (
                    isinstance(limit, bool)
                    or not isinstance(limit, Real)
                    or not np.isfinite(limit)
                ):
                    raise ValueError(f'{name}: bounds must be finite real scalars')
                valid = {
                    'minimum': value >= limit,
                    'maximum': value <= limit,
                    'exclusiveMinimum': value > limit,
                    'exclusiveMaximum': value < limit,
                }[bound]
                if not np.all(valid):
                    raise ValueError(f'{name}: parameter violates {bound}')
            self._symbols[name] = constant(value, kind)
        self._symbols['t'] = Node(QuantityType(SECOND), lambda values: values['t'])
        intermediate = m.get('intermediates', {})
        if not isinstance(intermediate, Mapping) or len(intermediate) > 128:
            raise ValueError('intermediates must be a bounded named mapping')
        trees = {}
        for name, text in intermediate.items():
            _name(name)
            if name in self._symbols or name in RESERVED:
                raise ValueError(f'Duplicate/reserved intermediate: {name}')
            trees[name] = parse_expression(text)
        visiting = set()
        self._intermediates = []

        def resolve(name):
            if name in self._symbols:
                return self._symbols[name]
            if name not in trees:
                raise ValueError(f'Undeclared symbol: {name}')
            if name in visiting or len(visiting) >= 32:
                raise ValueError('Circular or excessively deep intermediate dependency')
            visiting.add(name)
            node = compiler.compile(trees[name])
            visiting.remove(name)
            self._intermediates.append((name, node))
            self._symbols[name] = (
                constant(node.value, node.kind)
                if node.constant
                else Node(node.kind, lambda values, key=name: values[key])
            )
            return self._symbols[name]

        compiler = ExpressionCompiler(resolve)
        for name in sorted(trees):
            resolve(name)
        equations = m['equations']
        if not isinstance(equations, Mapping) or set(equations) != {
            name for name, _, _ in self._states
        }:
            raise ValueError('Exactly one derivative equation is required for each state')
        self._equations = []
        for name, kind, _ in self._states:
            node = compiler.compile(parse_expression(equations[name]))
            if node.kind.shape != kind.shape:
                raise ValueError(f'{name}: derivative shape differs from state')
            same(node.kind.unit, multiply(kind.unit, SECOND, -1))
            self._equations.append((name, node))
        self.expression_cost = compiler.cost
        self._deployment = m.get('deployment')
        if self._deployment is not None:
            self._validate_cartesian()
        self._position_output = None
        if 'outputs' in m:
            from .position_output import PositionOutput
            self._position_output = PositionOutput(self, m['outputs'])
            self.expression_cost += self._position_output.cost
            if self.expression_cost > 4096:
                raise ValueError('Model and outputs exceed total expression cost limit')
        self._validate_environment()

    @property
    def base_definition(self):
        """The original immutable Library model, before experiment influences."""
        return deepcopy(self._base_definition)

    @property
    def uses_mapped_output(self):
        """Declare prescribed physical output without imposing a Cartesian state layout."""
        return self._deployment is None and self._position_output is not None

    def _validate_environment(self):
        if self._deployment is not None or self.environment_field is None:
            return
        from robot_simulation.disturbances.composite_field import CompositeAccelerationField
        if (self.uses_mapped_output
                and isinstance(self.environment_field, CompositeAccelerationField)
                and not self.environment_field.fields):
            return
        if self.uses_mapped_output:
            raise ValueError(
                'Prescribed mapped dynamics do not support active environment acceleration. '
                'Select an explicit no-field experiment.resources.environment '
                '(environment.acceleration_fields: []). Inherited fields are not ignored.')
        raise ValueError('Environment acceleration requires explicit Cartesian deployment adapter '
                         'or a prescribed physical output mapping with an empty environment')

    @property
    def definition(self):
        """Return detached effective equations/schema/values for reproducible evidence."""
        return deepcopy(self._definition)

    @property
    def initial_state(self):
        """Standalone default; explicit integration states may override initial conditions."""
        return self._initial_state.copy()

    @property
    def capabilities(self):
        """Expose structural support separately from live deployment and numerical stability."""
        try:
            self.validate_deployment()
            reasons = []
        except ValueError as error:
            reasons = [str(error)]
        return {
            'mathematically_valid': True,
            'state_dimension': self.state_dimension,
            'control_dimension': self.control_dimension,
            'numerical_contract': 'Dynamics/RK4; finite domains checked at each stage',
            'deployable': not reasons,
            'deployment_reasons': reasons,
            'output_mapping': self.output_mapping,
            'target_output': 'prescribed_mapping' if self.uses_mapped_output else 'cartesian',
            'environment_coupling': ('none; empty field list required' if self.uses_mapped_output
                                     else 'Cartesian acceleration once'),
            'initial_state_source': ('model declaration' if self.uses_mapped_output
                                     else 'deployment position/velocity'),
            'unused_parameters': self.unused_parameters,
        }

    @property
    def unused_parameters(self):
        """Find actual equation/output dependencies, without implied parameter meaning."""
        m=self._definition
        texts=list(m['equations'].values())
        if 'outputs' in m:
            texts.append(m['outputs']['position'])
        used=set()
        while texts:
            names={n.id for n in ast.walk(parse_expression(texts.pop())) if isinstance(n,ast.Name)}
            for name in names-used:
                if name in m.get('intermediates',{}):
                    texts.append(m['intermediates'][name])
            used.update(names)
        return sorted(set(m['parameters'])-used)

    def _validate_cartesian(self):
        d = _mapping(
            self._deployment,
            {'adapter', 'position', 'velocity'},
            {'adapter', 'position', 'velocity'},
            'deployment',
        )
        if d['adapter'] != 'cartesian_target':
            raise ValueError('Unsupported deployment adapter')
        if self.state_dimension != 6 or len(self._states) != 2:
            raise ValueError('Cartesian deployment requires exactly six states in two 3-vectors')
        p, v = self._states
        if (
            p[0] != d['position']
            or v[0] != d['velocity']
            or p[1] != QuantityType(parse_unit('m'), (3,))
            or v[1] != QuantityType(parse_unit('m/s'), (3,))
        ):
            raise ValueError('Cartesian deployment requires ordered position [m], velocity [m/s]')
        derivative = parse_expression(self._definition['equations'][p[0]])
        if not isinstance(derivative, ast.Name) or derivative.id != v[0]:
            raise ValueError('Cartesian deployment requires position derivative exactly velocity')

    def validate_deployment(self, reference_frame=None):
        """Validate mapped or legacy Cartesian output against actual live bindings."""
        self._validate_environment()
        if not self.uses_mapped_output:
            if self._deployment is None:
                raise ValueError(
                    'Model requires a validated physical output mapping '
                    'or Cartesian adapter for deployment')
            self._validate_cartesian()
            if self._position_output is not None:
                position = parse_expression(self._position_output.metadata['position'])
                if not isinstance(position, ast.Name) or position.id != self._deployment['position']:
                    raise ValueError('Explicit output must match the existing Cartesian adapter position')
        if reference_frame is not None and reference_frame != self.frame_id:
            raise ValueError('Deployment and model frames differ')
        if self.control_dimension:
            raise ValueError('Live target deployment provides no control input vector')
        supported = {'tracking_position_world': QuantityType(parse_unit('m'), (3,))}
        for name, kind in self._contexts.items():
            if supported.get(name) != kind:
                raise ValueError(f'Unsupported live deployment context: {name}')

    def _values(self, state, control, time, context):
        """Bind validated numerical inputs and existing intermediate expressions."""
        state = _numeric(state, (self.state_dimension,), 'state')
        control = _numeric(control, (self.control_dimension,), 'control')
        if isinstance(time, bool) or not isinstance(time, Real) or not np.isfinite(time):
            raise ValueError('time must be finite real simulation seconds')
        if not isinstance(context, Mapping):
            raise ValueError('context must be a mapping')
        if 'frame_id' in context and context['frame_id'] != self.frame_id:
            raise ValueError('Runtime context frame differs from model frame')
        values = {'t': float(time)}
        for name, kind, indices in self._states:
            values[name] = state[indices].reshape(kind.shape)
        for name, kind, indices in self._controls:
            values[name] = control[indices].reshape(kind.shape)
        for name, kind in self._contexts.items():
            if name not in context:
                raise ValueError(f'Missing required context: {name}')
            values[name] = _numeric(context[name], kind.shape, 'context ' + name)
        for name, node in self._intermediates:
            values[name] = node(values)
        return values

    @property
    def output_mapping(self):
        """Return explicit validated mapping diagnostics, without implied state semantics."""
        return deepcopy(self._position_output.metadata) if self._position_output else None

    def physical_output(self, state, control, time, context):
        """Evaluate explicit mathematical position and verified total velocity."""
        if self._position_output is None:
            raise ValueError('No explicit physical output mapping')
        return self._position_output.evaluate(self._values(state, control, time, context))

    def derivative(self, state, control, time, context):
        """Evaluate the same full-state D3 derivative; external acceleration added once."""
        values = self._values(state, control, time, context)
        result = np.concatenate([node(values).reshape(-1) for _, node in self._equations])
        if self.environment_field is not None and self._deployment is not None:
            field = self.environment_field
            external = (
                field.acceleration_with_context(state[:3], time, context.get('physical_base'))
                if isinstance(field, ContextualAccelerationField)
                else field.acceleration(state[:3], time)
            )
            result[3:] += _numeric(external, (3,), 'environment acceleration')
        if result.shape != (self.state_dimension,) or not np.isfinite(result).all():
            raise ValueError('Derivative must be a finite vector matching the full state')
        return result
