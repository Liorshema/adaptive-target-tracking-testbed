"""Compile a bounded mathematical AST to allowlisted numerical callables only."""

import ast
import math
from dataclasses import dataclass
from fractions import Fraction

import numpy as np

from .equation_units import ANGLE, ONE, QuantityType, multiply, power, same

FUNCTIONS = frozenset({'sin', 'cos', 'tanh', 'exp', 'sqrt', 'norm', 'dot', 'cross'})
RESERVED = FUNCTIONS | {'t', 'pi'}
MAX_NODES = 256
MAX_DEPTH = 32
MAX_VECTOR = 64


def parse_expression(text):
    """Reject oversized syntax before constructing a bounded executable tree."""
    if not isinstance(text, str) or not text.strip() or len(text) > 2048:
        raise ValueError('Expression must be nonempty and at most 2048 characters')
    try:
        tree = ast.parse(text, mode='eval').body
    except (SyntaxError, ValueError, RecursionError) as error:
        raise ValueError('Invalid mathematical expression') from error
    if sum(1 for _ in ast.walk(tree)) > MAX_NODES:
        raise ValueError('Expression exceeds the 256-node cost bound')
    return tree


def checked(function, *args):
    """Turn invalid real arithmetic into an explicit error; never clip or regularize."""
    try:
        with np.errstate(divide='raise', invalid='raise', over='raise', under='ignore'):
            value = np.asarray(function(*args), dtype=float)
        if not np.isfinite(value).all():
            raise ValueError('Nonfinite equation result')
        return value
    except (ArithmeticError, TypeError) as error:
        raise ValueError(f'Invalid equation numerical domain: {error}') from error


@dataclass(frozen=True)
class Node:
    """A typed compiled expression; constants are folded once at preparation."""

    kind: QuantityType
    function: object
    constant: bool = False
    value: object = None

    def __call__(self, values):
        """Evaluate a compiled node without reparsing its source."""
        return self.value if self.constant else self.function(values)


def constant(value, kind=None):
    """Construct finite float constants, avoiding arbitrary-precision integer math."""
    value = np.asarray(value, dtype=float)
    if not np.isfinite(value).all():
        raise ValueError('Constant must be finite')
    if kind is None:
        kind = QuantityType(None if np.all(value == 0) else ONE, value.shape)
    value.flags.writeable = False
    return Node(kind, None, True, value)


def operation(kind, function, children):
    """Fold constant operations or bind an allowlisted numerical operation."""
    if all(c.constant for c in children):
        return constant(checked(function, *(c.value for c in children)), kind)
    return Node(kind, lambda env: checked(function, *(c(env) for c in children)))


class ExpressionCompiler:
    """Resolve symbols through the model owner; never execute Python source."""

    def __init__(self, resolve):
        """Use the model's declared-symbol resolver and one total cost counter."""
        self.resolve = resolve
        self.cost = 0

    def compile(self, tree, depth=0):  # noqa: A003 - mathematical compiler API.
        """Type-check and compile only the explicitly supported AST forms."""
        self.cost += 1
        if depth > MAX_DEPTH or self.cost > 4096:
            raise ValueError('Model exceeds expression depth/total cost limits')

        def child(tree):
            return self.compile(tree, depth + 1)

        if isinstance(tree, ast.Constant):
            if type(tree.value) not in (float, int) or not math.isfinite(tree.value):
                raise ValueError('Only finite real numeric literals are supported')
            if abs(tree.value) > 1e100:
                raise ValueError('Numeric literal exceeds supported magnitude')
            return constant(tree.value)
        if isinstance(tree, ast.Name):
            if tree.id == 'pi':
                return constant(math.pi)
            return self.resolve(tree.id)
        if isinstance(tree, ast.List):
            if not 1 <= len(tree.elts) <= MAX_VECTOR:
                raise ValueError('Vector construction needs 1..64 scalar entries')
            children = [child(t) for t in tree.elts]
            unit = children[0].kind.unit
            for c in children:
                if c.kind.shape:
                    raise ValueError('Nested vectors are unsupported')
                unit = same(unit, c.kind.unit)
            return operation(
                QuantityType(unit, (len(children),)), lambda *v: np.array(v), children
            )
        if isinstance(tree, ast.UnaryOp) and isinstance(tree.op, (ast.UAdd, ast.USub)):
            a = child(tree.operand)
            return operation(
                a.kind, np.positive if isinstance(tree.op, ast.UAdd) else np.negative, [a]
            )
        if isinstance(tree, ast.Subscript):
            a = child(tree.value)
            i = tree.slice
            if (
                not isinstance(i, ast.Constant)
                or type(i.value) is not int
                or not a.kind.shape
                or not 0 <= i.value < a.kind.shape[0]
            ):
                raise ValueError('Vector access requires an in-range constant nonnegative index')
            return operation(QuantityType(a.kind.unit), lambda v: v[i.value], [a])
        if isinstance(tree, ast.BinOp):
            a, b = child(tree.left), child(tree.right)
            x, y = a.kind, b.kind
            if isinstance(tree.op, (ast.Add, ast.Sub)):
                if x.shape != y.shape:
                    raise ValueError('Addition/subtraction shapes must match')
                kind = QuantityType(same(x.unit, y.unit), x.shape)
                return operation(
                    kind, np.add if isinstance(tree.op, ast.Add) else np.subtract, [a, b]
                )
            if isinstance(tree.op, (ast.Mult, ast.Div)):
                if x.shape and y.shape and x.shape != y.shape:
                    raise ValueError('Vector multiplication/division shapes must match')
                div = isinstance(tree.op, ast.Div)
                if div and b.constant and np.any(b.value == 0):
                    raise ValueError('Division by a constant zero denominator')
                kind = QuantityType(multiply(x.unit, y.unit, -1 if div else 1), x.shape or y.shape)
                return operation(kind, np.divide if div else np.multiply, [a, b])
            if isinstance(tree.op, ast.Pow):
                if y.shape or y.unit not in (ONE, None):
                    raise ValueError('Exponent must be dimensionless scalar')
                if x.unit not in (ONE, None) and not b.constant:
                    raise ValueError('Unitful powers require a constant exponent')
                exponent = float(b.value) if b.constant else 1.0
                if b.constant and abs(exponent) > 16:
                    raise ValueError('Constant exponent must be within [-16,16]')
                if x.unit not in (ONE, None) and exponent * 2 != int(exponent * 2):
                    raise ValueError('Unitful constant power must be a half-integer')

                def raise_power(base, exp):
                    if abs(float(exp)) > 16:
                        raise ValueError('Exponent exceeds numerical cost/domain bound')
                    return np.power(base, exp)

                unit = power(x.unit, Fraction(exponent))
                if x.unit is None and (not b.constant or exponent <= 0):
                    # 0**0 is one: polymorphic zero must never license a nonzero
                    # dimensionless result as an arbitrary physical quantity.
                    unit = ONE
                return operation(QuantityType(unit, x.shape), raise_power, [a, b])
            raise ValueError('Unsupported mathematical operator')
        if isinstance(tree, ast.Call):
            if (
                not isinstance(tree.func, ast.Name)
                or tree.func.id not in FUNCTIONS
                or tree.keywords
            ):
                raise ValueError('Only allowlisted mathematical functions are supported')
            name = tree.func.id
            count = 2 if name in ('dot', 'cross') else 1
            if len(tree.args) != count:
                raise ValueError(f'{name} requires {count} arguments')
            args = [child(t) for t in tree.args]
            a = args[0].kind
            if name in ('sin', 'cos', 'tanh', 'exp'):
                allowed = (ONE, None, ANGLE) if name in ('sin', 'cos') else (ONE, None)
                if a.unit not in allowed:
                    raise ValueError(
                        f'{name} requires dimensionless input (sin/cos also accept rad)'
                    )
                return operation(QuantityType(ONE, a.shape), getattr(np, name), args)
            if name == 'sqrt':
                return operation(
                    QuantityType(power(a.unit, Fraction(1, 2)), a.shape), np.sqrt, args
                )
            if name == 'norm':
                if not a.shape:
                    raise ValueError('norm requires a vector')
                return operation(QuantityType(a.unit), np.linalg.norm, args)
            b = args[1].kind
            if not a.shape or a.shape != b.shape or (name == 'cross' and a.shape != (3,)):
                raise ValueError(f'{name} requires compatible vector dimensions')
            return operation(
                QuantityType(multiply(a.unit, b.unit), a.shape if name == 'cross' else ()),
                getattr(np, name),
                args,
            )
        raise ValueError(f'Unsupported expression syntax: {type(tree).__name__}')
