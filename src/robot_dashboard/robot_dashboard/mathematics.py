"""Safe typesetting of the existing restricted D3 AST, with no numerical execution."""

import ast
import math
import re

from robot_simulation.dynamics.equation_expression import FUNCTIONS, parse_expression

GREEK = frozenset(
    (
        'alpha beta gamma delta epsilon zeta eta theta iota kappa lambda mu '
        'nu xi pi rho sigma tau upsilon phi chi psi omega'
    ).split()
)


def symbol_tex(name, vectors=()):
    """Render mathematical identifiers only; raw TeX never enters the output."""
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,63}', name):
        raise ValueError('Mathematical display needs a valid D3 identifier')
    base, *suffix = name.split('_')
    text = '\\' + base if base in GREEK else base if len(base) == 1 else '\\mathrm{' + base + '}'
    if suffix:
        text += '_{\\mathrm{' + r'\_'.join(suffix) + '}}'
    return '\\mathbf{' + text + '}' if name in vectors else text


def _tex(node, vectors, bindings, inline_vectors):
    def child(value):
        return _tex(value, vectors, bindings, inline_vectors)

    if isinstance(node, ast.Name):
        return bindings.get(node.id, symbol_tex(node.id, vectors))
    if (
        isinstance(node, ast.Constant)
        and type(node.value) in (int, float)
        and math.isfinite(node.value)
    ):
        return str(node.value)
    if isinstance(node, ast.List):
        if inline_vectors:
            return r'\left[' + ', '.join(child(n) for n in node.elts) + r'\right]'
        return r'\begin{bmatrix}' + r'\\'.join(child(n) for n in node.elts) + r'\end{bmatrix}'
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
        return (
            ('-' if isinstance(node.op, ast.USub) else '+')
            + r'\left('
            + child(node.operand)
            + r'\right)'
        )
    if (
        isinstance(node, ast.Subscript)
        and isinstance(node.slice, ast.Constant)
        and type(node.slice.value) is int
        and node.slice.value >= 0
    ):
        return child(node.value) + '_{' + str(node.slice.value) + '}'
    if isinstance(node, ast.BinOp):
        a, b = child(node.left), child(node.right)
        if isinstance(node.op, ast.Div):
            return r'\frac{' + a + '}{' + b + '}'
        if isinstance(node.op, ast.Pow):
            return r'\left(' + a + r'\right)^{' + b + '}'
        operators = {ast.Add: '+', ast.Sub: '-', ast.Mult: r'\mathbin{*}'}
        if type(node.op) in operators:
            return r'\left(' + a + ' ' + operators[type(node.op)] + ' ' + b + r'\right)'
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id in FUNCTIONS
        and not node.keywords
    ):
        name = node.func.id
        if len(node.args) != (2 if name in ('dot', 'cross') else 1):
            raise ValueError('Invalid function arity in mathematical display')
        args = [child(v) for v in node.args]
        if name == 'sqrt':
            return r'\sqrt{' + args[0] + '}'
        if name == 'norm':
            return r'\left\lVert ' + args[0] + r'\right\rVert'
        if name in ('dot', 'cross'):
            return (
                r'\left('
                + args[0]
                + (r'\cdot ' if name == 'dot' else r'\times ')
                + args[1]
                + r'\right)'
            )
        return '\\operatorname{' + name + r'}\left(' + args[0] + r'\right)'
    raise ValueError('Unsupported syntax in mathematical display')


def expression_tex(expression, vectors=(), *, bindings=None, inline_vectors=False):
    """Format a bounded owner-parsed expression without interpreting user TeX."""
    return _tex(parse_expression(expression), vectors, bindings or {}, inline_vectors)


def model_tex(model):
    """Display actual ordered scalar/vector states, nonlinear derivatives and initials."""
    vectors = {s['name'] for s in model['state'] if s['shape']}
    vectors.update(k for k, v in model.get('parameters', {}).items() if v['shape'])
    names = [symbol_tex(s['name'], vectors) for s in model['state']]
    derivatives = [expression_tex(model['equations'][s['name']], vectors) for s in model['state']]
    initials = [expression_tex(repr(s['initial']), vectors) for s in model['state']]

    def matrix(values):
        return r'\begin{bmatrix}' + r'\\[3pt]'.join(values) + r'\end{bmatrix}'

    return (
        '$$\\mathbf{s}='
        + matrix(names)
        + r'\qquad\dot{\mathbf{s}}='
        + matrix(derivatives)
        + r'\qquad\mathbf{s}(0)='
        + matrix(initials)
        + '$$'
    )


def velocity_tex(mapping, model):
    """Show verified derivative bindings as state rates, retaining typed unit constants."""
    vectors = {s['name'] for s in model['state'] if s['shape']}
    bindings = {
        generated: r'\dot{' + symbol_tex(state, vectors) + '}'
        for generated, state in mapping.get('rate_symbols', {}).items()
    }
    for generated, quantity in mapping.get('unit_symbols', {}).items():
        if quantity != {'value': 1, 'unit': 'rad'}:
            raise ValueError('Unknown derived output unit binding')
        bindings[generated] = r'(1\,\mathrm{rad})'
    return expression_tex(mapping['velocity'], bindings=bindings, inline_vectors=True)
