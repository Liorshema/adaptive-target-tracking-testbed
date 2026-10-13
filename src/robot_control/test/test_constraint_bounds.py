
"""Unit tests for model-based velocity-bound conversion."""

import numpy as np
import pytest
from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.joint_limits import JointLimits
from robot_models.constraints.velocity_limits import VelocityLimits

from robot_control.interfaces.control_data import ControlMode, ControlOutput
from robot_control.safety.constraint_bounds import (
    acceleration_bounds,
    joint_position_bounds,
    velocity_bounds,
)
from robot_control.safety.constraint_projector import ConstraintProjector


def make_velocity_limits(dof=4):
    """Create symmetric whole-body velocity limits."""
    return VelocityLimits(
        lower=np.full(dof, -2.0),
        upper=np.full(dof, 2.0),
    )


def make_acceleration_limits(dof=4):
    """Create symmetric acceleration limits."""
    return AccelerationLimits(
        lower=np.full(dof, -1.0),
        upper=np.full(dof, 1.0),
    )


def make_joint_limits(dof=2):
    """Create symmetric joint position limits."""
    return JointLimits(
        lower=-np.ones(dof),
        upper=np.ones(dof),
    )


def test_velocity_bounds():
    """Return independent copies of physical velocity limits."""
    limits = make_velocity_limits()

    lower, upper = velocity_bounds(limits)

    np.testing.assert_array_equal(lower, -2.0 * np.ones(4))
    np.testing.assert_array_equal(upper, 2.0 * np.ones(4))

    lower[0] = 100.0
    assert limits.lower[0] == -2.0


def test_acceleration_bounds():
    """Convert acceleration limits using the previous velocity."""
    lower, upper = acceleration_bounds(
        limits=make_acceleration_limits(),
        previous_velocity=np.array([0.0, 1.0, -1.0, 0.5]),
        dt=0.2,
    )

    np.testing.assert_allclose(
        lower, [-0.2, 0.8, -1.2, 0.3]
    )
    np.testing.assert_allclose(
        upper, [0.2, 1.2, -0.8, 0.7]
    )


def test_joint_position_bounds():
    """Constrain joint rates while preserving base velocity bounds."""
    lower, upper = joint_position_bounds(
        joint_limits=make_joint_limits(),
        joint_positions=np.array([0.95, -0.95]),
        joint_velocity_indices=np.array([2, 3]),
        velocity_limits=make_velocity_limits(),
        dt=0.1,
    )

    np.testing.assert_allclose(
        lower, [-2.0, -2.0, -2.0, -0.5]
    )
    np.testing.assert_allclose(
        upper, [2.0, 2.0, 0.5, 2.0]
    )


@pytest.mark.parametrize("dof", [1, 2, 4, 7])
def test_variable_joint_dimensions(dof):
    """Support arbitrary joint counts."""
    base_dof = 2

    lower, upper = joint_position_bounds(
        joint_limits=make_joint_limits(dof),
        joint_positions=np.full(dof, 0.95),
        joint_velocity_indices=np.arange(
            base_dof, base_dof + dof
        ),
        velocity_limits=make_velocity_limits(
            base_dof + dof
        ),
        dt=0.1,
    )

    assert lower.shape == (base_dof + dof,)
    assert upper.shape == (base_dof + dof,)

    np.testing.assert_allclose(
        upper[base_dof:],
        np.full(dof, 0.5),
    )


def test_noncontiguous_joint_indices():
    """Support joints at noncontiguous command indices."""
    lower, upper = joint_position_bounds(
        joint_limits=make_joint_limits(),
        joint_positions=np.array([0.95, -0.95]),
        joint_velocity_indices=np.array([1, 3]),
        velocity_limits=make_velocity_limits(),
        dt=0.1,
    )

    np.testing.assert_allclose(
        upper[1], 0.5
    )
    np.testing.assert_allclose(
        lower[3], -0.5
    )


def test_joint_state_outside_limits():
    """Reject invalid current joint positions."""
    with pytest.raises(ValueError):
        joint_position_bounds(
            joint_limits=make_joint_limits(),
            joint_positions=np.array([1.5, 0.0]),
            joint_velocity_indices=np.array([2, 3]),
            velocity_limits=make_velocity_limits(),
            dt=0.1,
        )


def test_duplicate_joint_indices():
    """Reject duplicate generalized velocity indices."""
    with pytest.raises(ValueError):
        joint_position_bounds(
            joint_limits=make_joint_limits(),
            joint_positions=np.zeros(2),
            joint_velocity_indices=np.array([2, 2]),
            velocity_limits=make_velocity_limits(),
            dt=0.1,
        )


def test_invalid_acceleration_dt():
    """Reject nonpositive time steps."""
    with pytest.raises(ValueError):
        acceleration_bounds(
            limits=make_acceleration_limits(),
            previous_velocity=np.zeros(4),
            dt=0.0,
        )


def test_acceleration_dimension_mismatch():
    """Reject dimensions inconsistent with acceleration limits."""
    with pytest.raises(ValueError):
        acceleration_bounds(
            limits=make_acceleration_limits(),
            previous_velocity=np.zeros(3),
            dt=0.1,
        )


def test_joint_dimension_mismatch():
    """Reject incorrect joint position dimensions."""
    with pytest.raises(ValueError):
        joint_position_bounds(
            joint_limits=make_joint_limits(),
            joint_positions=np.zeros(3),
            joint_velocity_indices=np.array([2, 3]),
            velocity_limits=make_velocity_limits(),
            dt=0.1,
        )


def test_joint_position_velocity_conflict():
    """Preserve an empty interval for the projector to reject."""
    velocity = VelocityLimits(
        lower=np.array([0.5]),
        upper=np.array([2.0]),
    )

    joints = JointLimits(
        lower=np.array([-1.0]),
        upper=np.array([1.0]),
    )

    lower, upper = joint_position_bounds(
        joint_limits=joints,
        joint_positions=np.array([1.0]),
        joint_velocity_indices=np.array([0]),
        velocity_limits=velocity,
        dt=0.1,
    )

    assert lower[0] > upper[0]

    output = ControlOutput(
        command=np.array([1.0]),
        mode=ControlMode.VELOCITY,
    )

    with pytest.raises(ValueError, match="Infeasible"):
        ConstraintProjector().project(
            output,
            [(lower, upper)],
        )


def test_all_constraints_together():
    """Verify projected commands satisfy all physical limits."""
    velocity = make_velocity_limits()
    acceleration = make_acceleration_limits()
    joints = make_joint_limits()

    previous = np.zeros(4)
    positions = np.array([0.95, -0.95])
    dt = 0.1

    bounds = [
        velocity_bounds(velocity),
        acceleration_bounds(
            acceleration, previous, dt
        ),
        joint_position_bounds(
            joints,
            positions,
            np.array([2, 3]),
            velocity,
            dt,
        ),
    ]

    output = ControlOutput(
        command=np.array([3.0, -3.0, 2.0, -2.0]),
        mode=ControlMode.VELOCITY,
    )

    result = ConstraintProjector().project(output, bounds)

    np.testing.assert_allclose(
        result.command,
        [0.1, -0.1, 0.1, -0.1],
        atol=1e-12,
    )

    assert velocity.contains(result.command)
    assert acceleration.contains(
        result.command, previous, dt
    )
    assert joints.contains(
        positions + dt * result.command[2:]
    )


def test_joint_bound_more_restrictive_than_acceleration():
    """Joint proximity may dominate an acceleration constraint."""
    velocity = make_velocity_limits(dof=1)
    acceleration = AccelerationLimits(
        lower=np.array([-10.0]),
        upper=np.array([10.0]),
    )
    joints = make_joint_limits(dof=1)

    positions = np.array([0.99])
    previous = np.array([0.0])
    dt = 0.1

    bounds = [
        velocity_bounds(velocity),
        acceleration_bounds(
            acceleration, previous, dt
        ),
        joint_position_bounds(
            joints,
            positions,
            np.array([0]),
            velocity,
            dt,
        ),
    ]

    output = ControlOutput(
        command=np.array([2.0]),
        mode=ControlMode.VELOCITY,
    )

    result = ConstraintProjector().project(output, bounds)

    np.testing.assert_allclose(
        result.command,
        [0.1],
        atol=1e-12,
    )
