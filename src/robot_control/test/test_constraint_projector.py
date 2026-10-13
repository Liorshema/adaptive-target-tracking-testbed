
"""Unit tests for generic box-constraint projection."""

import numpy as np
import pytest

from robot_control.interfaces.control_data import ControlMode, ControlOutput
from robot_control.safety.constraint_projector import ConstraintProjector


def make_output(command, mode=ControlMode.VELOCITY):
    """Create a controller output for testing."""
    return ControlOutput(
        command=np.asarray(command, dtype=float),
        mode=mode,
    )


def test_single_constraint():
    """Project a command onto one box constraint."""
    output = make_output([3.0, -3.0, 0.5])

    bounds = [
        (
            np.array([-2.0, -2.0, -2.0]),
            np.array([2.0, 2.0, 2.0]),
        ),
    ]

    result = ConstraintProjector().project(output, bounds)

    np.testing.assert_allclose(
        result.command,
        [2.0, -2.0, 0.5],
    )


def test_multiple_constraints():
    """Intersect several independent box constraints."""
    output = make_output([3.0, -2.0, 0.5])

    bounds = [
        (np.full(3, -2.0), np.full(3, 2.0)),
        (np.full(3, -0.1), np.full(3, 0.1)),
        (
            np.array([-2.0, -2.0, -0.05]),
            np.array([2.0, 2.0, 0.05]),
        ),
    ]

    result = ConstraintProjector().project(output, bounds)

    np.testing.assert_allclose(
        result.command,
        [0.1, -0.1, 0.05],
    )


def test_command_inside_constraints():
    """Preserve a command that is already admissible."""
    output = make_output([0.2, -0.3])

    bounds = [
        (np.array([-1.0, -1.0]), np.array([1.0, 1.0])),
    ]

    result = ConstraintProjector().project(output, bounds)

    np.testing.assert_array_equal(
        result.command,
        output.command,
    )


@pytest.mark.parametrize("dof", [1, 2, 4, 6, 8])
def test_variable_dimensions(dof):
    """Support arbitrary generalized velocity dimensions."""
    output = make_output(np.full(dof, 5.0))

    bounds = [
        (np.full(dof, -1.0), np.full(dof, 1.0)),
    ]

    result = ConstraintProjector().project(output, bounds)

    assert result.command.shape == (dof,)

    np.testing.assert_allclose(
        result.command,
        np.ones(dof),
    )


def test_constraint_order_independence():
    """Changing constraint order must not change projection."""
    output = make_output([2.0, -2.0])

    bounds = [
        (np.array([-1.0, -1.0]), np.array([1.0, 1.0])),
        (np.array([-0.5, -0.2]), np.array([0.8, 0.5])),
    ]

    projector = ConstraintProjector()

    result_a = projector.project(output, bounds)
    result_b = projector.project(output, reversed(bounds))

    np.testing.assert_allclose(
        result_a.command,
        result_b.command,
    )


def test_infeasible_constraints():
    """Reject an empty intersection of admissible bounds."""
    output = make_output([0.0])

    bounds = [
        (np.array([-2.0]), np.array([-1.0])),
        (np.array([1.0]), np.array([2.0])),
    ]

    with pytest.raises(ValueError, match="Infeasible"):
        ConstraintProjector().project(output, bounds)


def test_invalid_bound_order():
    """Reject a constraint whose lower bound exceeds its upper."""
    output = make_output([0.0])

    bounds = [
        (np.array([2.0]), np.array([1.0])),
    ]

    with pytest.raises(ValueError):
        ConstraintProjector().project(output, bounds)


def test_dimension_mismatch():
    """Reject bounds incompatible with command dimensions."""
    output = make_output([0.0, 0.0, 0.0])

    bounds = [
        (np.zeros(2), np.ones(2)),
    ]

    with pytest.raises(ValueError):
        ConstraintProjector().project(output, bounds)


def test_nonfinite_bounds():
    """Reject NaN and infinite constraint bounds."""
    output = make_output([0.0, 0.0])

    bounds = [
        (
            np.array([-1.0, np.nan]),
            np.array([1.0, 1.0]),
        ),
    ]

    with pytest.raises(ValueError):
        ConstraintProjector().project(output, bounds)


def test_empty_constraints():
    """Prevent unconstrained passage through the safety layer."""
    output = make_output([1.0, 2.0])

    with pytest.raises(ValueError, match="At least one"):
        ConstraintProjector().project(output, [])


def test_unsupported_control_mode():
    """Reject commands outside the velocity-control contract."""
    output = make_output(
        [1.0],
        mode=ControlMode.EFFORT,
    )

    bounds = [
        (np.array([-2.0]), np.array([2.0])),
    ]

    with pytest.raises(ValueError):
        ConstraintProjector().project(output, bounds)


def test_input_command_unchanged():
    """Projection must not mutate the controller output."""
    output = make_output([5.0, -5.0])

    bounds = [
        (np.array([-1.0, -1.0]), np.array([1.0, 1.0])),
    ]

    result = ConstraintProjector().project(output, bounds)

    np.testing.assert_allclose(
        output.command,
        [5.0, -5.0],
    )

    np.testing.assert_allclose(
        result.command,
        [1.0, -1.0],
    )


def test_generator_of_constraints():
    """Accept lazily generated constraints, not only lists."""
    output = make_output([3.0, -3.0])

    def generate_bounds():
        yield (
            np.array([-2.0, -2.0]),
            np.array([2.0, 2.0]),
        )
        yield (
            np.array([-1.0, -0.5]),
            np.array([1.0, 0.5]),
        )

    result = ConstraintProjector().project(
        output,
        generate_bounds(),
    )

    np.testing.assert_allclose(
        result.command,
        [1.0, -0.5],
    )


def test_projection_is_idempotent():
    """Projecting an admissible result again changes nothing."""
    output = make_output([3.0, -3.0])

    bounds = [
        (np.array([-1.0, -1.0]), np.array([1.0, 1.0])),
    ]

    projector = ConstraintProjector()

    first = projector.project(output, bounds)
    second = projector.project(first, bounds)

    np.testing.assert_array_equal(
        first.command,
        second.command,
    )
