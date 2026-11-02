"""Small SI dimensional algebra for equation validation, without unit conversion."""

import re
from dataclasses import dataclass
from fractions import Fraction

# rad is retained as an angle dimension; trig accepts rad or dimensionless input.
BASES = ('m', 's', 'kg', 'rad')
ONE = (Fraction(0),) * len(BASES)
SECOND = (Fraction(0), Fraction(1), Fraction(0), Fraction(0))
ANGLE = (Fraction(0), Fraction(0), Fraction(0), Fraction(1))


def multiply(a, b, sign=1):
    """Combine dimensions; None denotes an exact, unit-polymorphic literal zero."""
    if a is None or b is None:
        return None
    return tuple(x + sign * y for x, y in zip(a, b))


def power(unit, exponent):
    """Raise a unit to a rational exponent (sqrt introduces halves)."""
    return None if unit is None else tuple(x * exponent for x in unit)


def same(a, b):
    """Require equal dimensions, allowing only exact literal zero to adopt units."""
    if a is not None and b is not None and a != b:
        raise ValueError('Incompatible physical units')
    return b if a is None else a


def parse_unit(text):
    """Read products/divisions of m, s, kg, rad with bounded integer exponents."""
    if not isinstance(text, str) or not text or len(text) > 80:
        raise ValueError('unit must be a supported SI unit string')
    tokens = re.split(r'([*/])', text)
    result, sign = ONE, 1
    for i, token in enumerate(tokens):
        if i % 2:
            sign = -1 if token == '/' else 1
            continue
        match = re.fullmatch(r'(1|m|s|kg|rad)(?:\^(-?\d{1,2}))?', token)
        if not match:
            raise ValueError(f'Unsupported unit {text!r}; use m, s, kg, rad, 1, *, /, ^')
        base, exponent = match.groups()
        exponent = int(exponent or 1)
        if abs(exponent) > 16:
            raise ValueError('Unit exponent exceeds the supported bound')
        if base != '1':
            dims = tuple(Fraction(exponent) if b == base else Fraction(0) for b in BASES)
            result = multiply(result, dims, sign)
    return result


@dataclass(frozen=True)
class QuantityType:
    """A scalar () or flat vector shape and its statically checked dimensions."""

    unit: tuple | None
    shape: tuple = ()
