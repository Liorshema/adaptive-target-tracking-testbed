"""Explicit position and analytic total-derivative output through the D3 compiler."""

import ast
from copy import deepcopy

import numpy as np

from .equation_expression import ExpressionCompiler, Node, constant, parse_expression
from .equation_units import ANGLE, QuantityType, SECOND, multiply, parse_unit


def _op(operator, a, b):
    return ast.BinOp(left=deepcopy(a), op=operator(), right=deepcopy(b))


def _call(name, *values):
    return ast.Call(func=ast.Name(id=name, ctx=ast.Load()), args=list(values), keywords=[])


def _zero(node):
    return (
        isinstance(node, ast.Constant)
        and node.value == 0
        or (isinstance(node, ast.List) and all(_zero(v) for v in node.elts))
    )


class PositionOutput:
    """Compile h and Dh*f + h_t; no new parser, numerical evaluator or solver."""

    def __init__(self, owner, declaration):
        """Validate the declared output and compile its bounded total derivative."""
        from .equation_model import _mapping

        _mapping(
            declaration,
            {'schema_version', 'frame', 'position'},
            {'schema_version', 'frame', 'position'},
            'outputs',
        )
        if type(declaration['schema_version']) is not int or declaration['schema_version'] != 1:
            raise ValueError('outputs: expected schema_version 1')
        if declaration['frame'] != owner.frame_id or declaration['frame'] != 'world':
            raise ValueError('Position output requires explicit matching world frame')
        symbols = dict(owner._symbols)
        intermediates = owner.definition.get('intermediates', {})
        parameters = owner.definition['parameters']
        state_rates = {}
        rate_symbols = {}
        unit_symbols = {}
        for index, ((name, kind, _), (_, equation)) in enumerate(
            zip(owner._states, owner._equations)
        ):
            generated = f'outputRate{index}'
            while generated in symbols:
                generated += 'x'
            symbols[generated] = Node(
                QuantityType(multiply(kind.unit, SECOND, -1), kind.shape),
                lambda values, equation=equation: equation(values),
            )
            state_rates[name] = ast.Name(id=generated, ctx=ast.Load())
            rate_symbols[generated] = name

        def resolve(name):
            if name not in symbols:
                raise ValueError(f'Undeclared output symbol: {name}')
            return symbols[name]

        compiler = ExpressionCompiler(resolve)
        tree = parse_expression(declaration['position'])
        self.position = compiler.compile(tree)
        if self.position.kind != QuantityType(parse_unit('m'), (3,)):
            raise ValueError('World position output must have shape [3] and unit m')

        def zero_like(node):
            shape = ExpressionCompiler(resolve).compile(node).kind.shape
            return (
                ast.List(elts=[ast.Constant(0) for _ in range(shape[0])], ctx=ast.Load())
                if shape
                else ast.Constant(0)
            )

        def add(a, b, original):
            if _zero(a) and _zero(b):
                return zero_like(original)
            return b if _zero(a) else a if _zero(b) else _op(ast.Add, a, b)

        def product(a, da, b, db, original, function=None):
            make = (
                (lambda x, y: _call(function, x, y))
                if function
                else (lambda x, y: _op(ast.Mult, x, y))
            )
            left = zero_like(original) if _zero(da) else make(da, b)
            right = zero_like(original) if _zero(db) else make(a, db)
            return add(left, right, original)

        derivative_work = 0

        def derive(node, depth=0):
            nonlocal derivative_work
            derivative_work += 1
            if derivative_work > 4096:
                raise ValueError('Output derivative exceeds expansion work budget')
            if depth > 32:
                raise ValueError('Output derivative exceeds dependency depth bound')

            def d(child):
                return derive(child, depth + 1)

            if isinstance(node, ast.Constant):
                return ast.Constant(0)
            if isinstance(node, ast.Name):
                if node.id in state_rates:
                    return state_rates[node.id]
                if node.id == 't':
                    return ast.Constant(1)
                if node.id in parameters or node.id == 'pi':
                    return zero_like(node)
                if node.id in intermediates:
                    return d(parse_expression(intermediates[node.id]))
                raise ValueError(
                    'Position mapping may depend only on state, time and fixed parameters; '
                    f'unsupported input {node.id}'
                )
            if isinstance(node, ast.List):
                return ast.List(elts=[d(v) for v in node.elts], ctx=ast.Load())
            if isinstance(node, ast.Subscript):
                value = d(node.value)
                return ast.Subscript(value=value, slice=deepcopy(node.slice), ctx=ast.Load())
            if isinstance(node, ast.UnaryOp):
                return ast.UnaryOp(op=deepcopy(node.op), operand=d(node.operand))
            if isinstance(node, ast.BinOp):
                a, b = node.left, node.right
                da, db = d(a), d(b)
                if isinstance(node.op, (ast.Add, ast.Sub)):
                    return add(
                        da,
                        (
                            db
                            if isinstance(node.op, ast.Add)
                            else ast.UnaryOp(op=ast.USub(), operand=db)
                        ),
                        node,
                    )
                if isinstance(node.op, ast.Mult):
                    return product(a, da, b, db, node)
                if isinstance(node.op, ast.Div):
                    if _zero(db):
                        return zero_like(node) if _zero(da) else _op(ast.Div, da, b)
                    numerator = add(
                        zero_like(node) if _zero(da) else _op(ast.Mult, da, b),
                        ast.UnaryOp(op=ast.USub(), operand=_op(ast.Mult, a, db)),
                        node,
                    )
                    return _op(ast.Div, numerator, _op(ast.Pow, b, ast.Constant(2)))
                if isinstance(node.op, ast.Pow):
                    exponent = ExpressionCompiler(resolve).compile(b)
                    if not exponent.constant:
                        raise ValueError(
                            'Automatic output velocity does not support variable exponents'
                        )
                    if _zero(da) or float(exponent.value) == 0:
                        return zero_like(node)
                    if float(exponent.value) == 1:
                        return da
                    return _op(
                        ast.Mult,
                        _op(ast.Mult, b, _op(ast.Pow, a, _op(ast.Sub, b, ast.Constant(1)))),
                        da,
                    )
            if isinstance(node, ast.Call):
                name = node.func.id
                args = node.args
                derivatives = [d(v) for v in args]
                if all(_zero(v) for v in derivatives):
                    return zero_like(node)
                a, da = args[0], derivatives[0]
                if name in ('dot', 'cross'):
                    return product(a, da, args[1], derivatives[1], node, name)
                factor = {
                    'sin': lambda: _call('cos', a),
                    'cos': lambda: ast.UnaryOp(op=ast.USub(), operand=_call('sin', a)),
                    'tanh': lambda: _op(
                        ast.Sub, ast.Constant(1), _op(ast.Pow, _call('tanh', a), ast.Constant(2))
                    ),
                    'exp': lambda: _call('exp', a),
                }
                if name in factor:
                    if (
                        name in ('sin', 'cos')
                        and ExpressionCompiler(resolve).compile(a).kind.unit == ANGLE
                    ):
                        # D3 keeps rad as a dimension. d sin(theta)/dt is
                        # cos(theta) * theta_dot / (1 rad), with no unit relaxation.
                        symbol = 'outputRadianUnit'
                        while symbol in symbols:
                            symbol += 'x'
                        symbols[symbol] = constant(1, QuantityType(ANGLE))
                        unit_symbols[symbol] = {'value': 1, 'unit': 'rad'}
                        da = _op(ast.Div, da, ast.Name(id=symbol, ctx=ast.Load()))
                    return _op(ast.Mult, factor[name](), da)
                if name == 'sqrt':
                    return _op(ast.Div, da, _op(ast.Mult, ast.Constant(2), _call('sqrt', a)))
                if name == 'norm':
                    return _op(ast.Div, _call('dot', a, da), _call('norm', a))
            raise ValueError('Unsupported analytic position-mapping derivative')

        derived = ast.unparse(ast.fix_missing_locations(derive(tree)))
        self.velocity = compiler.compile(parse_expression(derived))
        if self.velocity.kind != QuantityType(parse_unit('m/s'), (3,)) and not (
            self.velocity.kind.shape == (3,) and self.velocity.kind.unit is None
        ):
            raise ValueError('Derived world velocity must have shape [3] and unit m/s')
        self.cost = compiler.cost
        self.metadata = {
            'schema_version': 1,
            'frame': 'world',
            'position': declaration['position'],
            'velocity': derived,
            'velocity_verified': True,
            'rate_symbols': rate_symbols,
            'unit_symbols': unit_symbols,
        }

    def evaluate(self, values):
        """Use compiled D3 nodes and their finite numerical-domain checks."""
        p, v = self.position(values), self.velocity(values)
        if not np.isfinite(p).all() or not np.isfinite(v).all():
            raise ValueError('Nonfinite physical output')
        return p.copy(), v.copy()
