
"""Unit tests for joint-position-aware velocity limiting."""

import numpy as np
import pytest
from robot_models.constraints.joint_limits import JointLimits

from robot_control.interfaces.control_data import ControlMode, ControlOutput
from robot_control.safety.joint_position_limiter import JointPositionLimiter


def make_limiter(dof=2, base_dof=2):
    """Construct a limiter with variable robot dimensions."""
    return JointPositionLimiter(
        joint_limits=JointLimits(
            lower=-np.ones(dof),
            upper=np.ones(dof),
        ),
        joint_velocity_indices=np.arange(
            base_dof, base_dof + dof
        ),
    )


def make_output(values):
    """Create a generalized velocity command."""
    return ControlOutput(
        command=np.asarray(values, dtype=float),
        mode=ControlMode.VELOCITY,
    )


def test_joint_velocity_clipping():
    """Prevent predicted joint-position violations."""
    limiter = make_limiter()

    result = limiter.limit(
        output=make_output([0.5, 0.2, 2.0, -2.0]),
        joint_positions=np.array([0.95, -0.95]),
        dt=0.1,
    )

    np.testing.assert_allclose(
        result.command,
        [0.5, 0.2, 0.5, -0.5],
        atol=1e-12,
    )


def test_command_within_limits_is_unchanged():
    """Keep admissible commands unchanged."""
    limiter = make_limiter()

    command = np.array([0.2, 0.1, 0.3, -0.4])

    result = limiter.limit(
        make_output(command),
        joint_positions=np.zeros(2),
        dt=0.1,
    )

    np.testing.assert_allclose(result.command, command)


@pytest.mark.parametrize("dof", [1, 2, 4, 7])
def test_variable_joint_dimensions(dof):
    """Support manipulators with different joint counts."""
    limiter = make_limiter(dof=dof)

    command = np.concatenate(
        (np.zeros(2), np.full(dof, 10.0))
    )

    result = limiter.limit(
        make_output(command),
        joint_positions=np.full(dof, 0.9),
        dt=0.1,
    )

    assert result.command.shape == (dof + 2,)

    np.testing.assert_allclose(
        result.command[2:],
        np.ones(dof),
        atol=1e-12,
    )


def test_motion_away_from_limit():
    """Allow motion away from an upper joint limit."""
    limiter = make_limiter(dof=1)

    result = limiter.limit(
        make_output([0.0, 0.0, -2.0]),
        joint_positions=np.array([1.0]),
        dt=0.1,
    )

    np.testing.assert_allclose(result.command[-1], -2.0)


def test_joint_at_upper_limit():
    """Block positive motion at the upper limit."""
    limiter = make_limiter(dof=1)

    result = limiter.limit(
        make_output([0.0, 0.0, 1.0]),
        joint_positions=np.array([1.0]),
        dt=0.1,
    )

    np.testing.assert_allclose(result.command[-1], 0.0)


def test_joint_at_lower_limit():
    """Block negative motion at the lower limit."""
    limiter = make_limiter(dof=1)

    result = limiter.limit(
        make_output([0.0, 0.0, -1.0]),
        joint_positions=np.array([-1.0]),
        dt=0.1,
    )

    np.testing.assert_allclose(result.command[-1], 0.0)


def test_joint_outside_limits():
    """Reject states that already violate position limits."""
    limiter = make_limiter(dof=1)

    with pytest.raises(ValueError):
        limiter.limit(
            make_output([0.0, 0.0, -1.0]),
            joint_positions=np.array([1.5]),
            dt=0.1,
        )


@pytest.mark.parametrize("dt", [0.0, -0.1, np.nan, np.inf])
def test_invalid_dt(dt):
    """Reject invalid control time intervals."""
    limiter = make_limiter()

    with pytest.raises(ValueError):
        limiter.limit(
            make_output([0.0, 0.0, 1.0, 1.0]),
            joint_positions=np.zeros(2),
            dt=dt,
        )


def test_invalid_joint_indices():
    """Reject duplicate joint velocity indices."""
    with pytest.raises(ValueError):
        JointPositionLimiter(
            joint_limits=JointLimits(
                lower=-np.ones(2),
                upper=np.ones(2),
            ),
            joint_velocity_indices=np.array([2, 2]),
        )


def test_joint_index_out_of_range():
    """Reject joint indices beyond command dimensions."""
    limiter = JointPositionLimiter(
        joint_limits=JointLimits(
            lower=np.array([-1.0]),
            upper=np.array([1.0]),
        ),
        joint_velocity_indices=np.array([5]),
    )

    with pytest.raises(ValueError):
        limiter.limit(
            make_output([0.0, 0.0, 1.0]),
            joint_positions=np.array([0.0]),
            dt=0.1,
        )


def test_invalid_joint_position_dimension():
    """Reject mismatched joint state dimensions."""
    limiter = make_limiter(dof=2)

    with pytest.raises(ValueError):
        limiter.limit(
            make_output([0.0, 0.0, 1.0, 1.0]),
            joint_positions=np.zeros(3),
            dt=0.1,
        )


def test_unsupported_command_mode():
    """Reject non-velocity command modes."""
    limiter = make_limiter()

    output = ControlOutput(
        command=np.zeros(4),
        mode=ControlMode.EFFORT,
    )

    with pytest.raises(ValueError):
        limiter.limit(
            output,
            joint_positions=np.zeros(2),
            dt=0.1,
        )


def test_original_command_unchanged():
    """Do not modify the controller's original command."""
    limiter = make_limiter()

    original = make_output([0.0, 0.0, 5.0, -5.0])

    limiter.limit(
        original,
        joint_positions=np.array([0.9, -0.9]),
        dt=0.1,
    )

    np.testing.assert_allclose(
        original.command,
        [0.0, 0.0, 5.0, -5.0],
    )
