
"""Tests for unified generalized velocity command safety."""

import numpy as np
import pytest
from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.joint_limits import JointLimits
from robot_models.constraints.velocity_limits import VelocityLimits

from robot_control.interfaces.control_data import ControlMode, ControlOutput
from robot_control.safety.safe_control_pipeline import SafeControlPipeline


@pytest.fixture
def velocity_limits():
    """Provide three-dimensional generalized velocity bounds."""
    return VelocityLimits(
        lower=np.array([-1.0, -1.0, -1.0]),
        upper=np.array([1.0, 1.0, 1.0]),
    )


@pytest.fixture
def acceleration_limits():
    """Provide symmetric generalized acceleration bounds."""
    return AccelerationLimits(
        lower=np.array([-2.0, -2.0, -2.0]),
        upper=np.array([2.0, 2.0, 2.0]),
    )


@pytest.fixture
def joint_limits():
    """Constrain the last two generalized velocity components."""
    return JointLimits(
        lower=np.array([-1.0, -1.0]),
        upper=np.array([1.0, 1.0]),
    )


def velocity_output(command):
    """Construct a velocity-mode control output."""
    return ControlOutput(
        command=np.asarray(command, dtype=np.float64),
        mode=ControlMode.VELOCITY,
    )


def test_command_inside_velocity_limits_is_unchanged(velocity_limits):
    """Preserve a command that already satisfies velocity limits."""
    pipeline = SafeControlPipeline(velocity_limits)

    output = pipeline.apply(
        velocity_output([0.2, -0.5, 0.8])
    )

    np.testing.assert_allclose(
        output.command,
        [0.2, -0.5, 0.8],
    )


def test_velocity_limits_clip_command(velocity_limits):
    """Project an excessive command onto velocity bounds."""
    pipeline = SafeControlPipeline(velocity_limits)

    output = pipeline.apply(
        velocity_output([2.0, -3.0, 0.5])
    )

    np.testing.assert_allclose(
        output.command,
        [1.0, -1.0, 0.5],
    )


def test_acceleration_limits_clip_change(
    velocity_limits, acceleration_limits
):
    """Bound the change in velocity over a control interval."""
    pipeline = SafeControlPipeline(
        velocity_limits=velocity_limits,
        acceleration_limits=acceleration_limits,
    )

    output = pipeline.apply(
        velocity_output([1.0, -1.0, 0.5]),
        previous_velocity=np.zeros(3),
        dt=0.1,
    )

    np.testing.assert_allclose(
        output.command,
        [0.2, -0.2, 0.2],
    )


def test_acceleration_limits_use_previous_velocity(
    velocity_limits, acceleration_limits
):
    """Center acceleration bounds around the previous velocity."""
    pipeline = SafeControlPipeline(
        velocity_limits,
        acceleration_limits=acceleration_limits,
    )

    output = pipeline.apply(
        velocity_output([1.0, -1.0, 1.0]),
        previous_velocity=np.array([0.5, -0.5, 0.0]),
        dt=0.1,
    )

    np.testing.assert_allclose(
        output.command,
        [0.7, -0.7, 0.2],
    )


def test_joint_position_limits(
    velocity_limits, joint_limits
):
    """Keep predicted next-step joint positions within bounds."""
    pipeline = SafeControlPipeline(
        velocity_limits=velocity_limits,
        joint_limits=joint_limits,
        joint_velocity_indices=np.array([1, 2]),
    )

    output = pipeline.apply(
        velocity_output([0.8, 1.0, -1.0]),
        joint_positions=np.array([0.95, -0.95]),
        dt=0.1,
    )

    np.testing.assert_allclose(
        output.command,
        [0.8, 0.5, -0.5],
    )


def test_joint_indices_are_configurable(
    velocity_limits, joint_limits
):
    """Apply joint bounds to externally selected command indices."""
    pipeline = SafeControlPipeline(
        velocity_limits,
        joint_limits=joint_limits,
        joint_velocity_indices=np.array([2, 0]),
    )

    output = pipeline.apply(
        velocity_output([-1.0, 0.4, 1.0]),
        joint_positions=np.array([0.95, -0.95]),
        dt=0.1,
    )

    np.testing.assert_allclose(
        output.command,
        [-0.5, 0.4, 0.5],
    )


def test_all_constraints_intersect(
    velocity_limits, acceleration_limits, joint_limits
):
    """Project once onto velocity, acceleration, and joint bounds."""
    pipeline = SafeControlPipeline(
        velocity_limits=velocity_limits,
        acceleration_limits=acceleration_limits,
        joint_limits=joint_limits,
        joint_velocity_indices=np.array([1, 2]),
    )

    output = pipeline.apply(
        velocity_output([2.0, 1.0, -1.0]),
        previous_velocity=np.zeros(3),
        joint_positions=np.array([0.99, -0.99]),
        dt=0.1,
    )

    # Velocity:     [-1, 1]
    # Acceleration: [-0.2, 0.2]
    # Joint 1:      velocity <= 0.1
    # Joint 2:      velocity >= -0.1
    np.testing.assert_allclose(
        output.command,
        [0.2, 0.1, -0.1],
        atol=1e-10,
    )


def test_infeasible_constraints_raise(
    velocity_limits, acceleration_limits, joint_limits
):
    """Reject empty intersections instead of violating constraints."""
    pipeline = SafeControlPipeline(
        velocity_limits=velocity_limits,
        acceleration_limits=acceleration_limits,
        joint_limits=joint_limits,
        joint_velocity_indices=np.array([1, 2]),
    )

    # Joint 1 is at its upper bound, requiring velocity <= 0.
    # Previous velocity and acceleration limits require >= 0.8.
    with pytest.raises(ValueError, match="Infeasible"):
        pipeline.apply(
            velocity_output([0.0, 1.0, 0.0]),
            previous_velocity=np.array([0.0, 1.0, 0.0]),
            joint_positions=np.array([1.0, 0.0]),
            dt=0.1,
        )


def test_missing_acceleration_context(
    velocity_limits, acceleration_limits
):
    """Require previous velocity and dt for acceleration constraints."""
    pipeline = SafeControlPipeline(
        velocity_limits,
        acceleration_limits=acceleration_limits,
    )

    with pytest.raises(ValueError, match="previous_velocity and dt"):
        pipeline.apply(
            velocity_output([0.1, 0.2, 0.3])
        )


def test_missing_joint_context(velocity_limits, joint_limits):
    """Require joint positions and dt for joint-position constraints."""
    pipeline = SafeControlPipeline(
        velocity_limits,
        joint_limits=joint_limits,
        joint_velocity_indices=np.array([1, 2]),
    )

    with pytest.raises(ValueError, match="joint_positions and dt"):
        pipeline.apply(
            velocity_output([0.1, 0.2, 0.3])
        )


def test_reject_invalid_command_dimension(velocity_limits):
    """Reject commands incompatible with configured limits."""
    pipeline = SafeControlPipeline(velocity_limits)

    with pytest.raises(ValueError, match="dimensions"):
        pipeline.apply(
            velocity_output([0.1, 0.2])
        )


def test_reject_non_velocity_mode(velocity_limits):
    """Reject unsupported control modes."""
    pipeline = SafeControlPipeline(velocity_limits)

    output = ControlOutput(
        command=np.array([0.1, 0.2, 0.3]),
        mode=ControlMode.ACCELERATION,
    )

    with pytest.raises(ValueError, match="velocity mode"):
        pipeline.apply(output)


def test_original_command_is_not_modified(velocity_limits):
    """Avoid mutating the original controller output."""
    pipeline = SafeControlPipeline(velocity_limits)

    original = velocity_output([2.0, -2.0, 0.5])
    result = pipeline.apply(original)

    np.testing.assert_allclose(
        original.command,
        [2.0, -2.0, 0.5],
    )

    np.testing.assert_allclose(
        result.command,
        [1.0, -1.0, 0.5],
    )

    assert result is not original


def test_reject_joint_limits_without_indices(
    velocity_limits, joint_limits
):
    """Require joint limits and indices to be configured together."""
    with pytest.raises(ValueError, match="supplied together"):
        SafeControlPipeline(
            velocity_limits,
            joint_limits=joint_limits,
        )


def test_reject_duplicate_joint_indices(
    velocity_limits, joint_limits
):
    """Reject ambiguous joint mappings."""
    with pytest.raises(ValueError, match="valid and unique"):
        SafeControlPipeline(
            velocity_limits,
            joint_limits=joint_limits,
            joint_velocity_indices=np.array([1, 1]),
        )


def test_reject_acceleration_dimension_mismatch(
    velocity_limits
):
    """Reject acceleration limits with incompatible dimensions."""
    invalid_acceleration = AccelerationLimits(
        lower=np.array([-1.0, -1.0]),
        upper=np.array([1.0, 1.0]),
    )

    with pytest.raises(ValueError, match="dimension"):
        SafeControlPipeline(
            velocity_limits,
            acceleration_limits=invalid_acceleration,
        )


@pytest.mark.parametrize("dt", [0.0, -0.1, np.nan])
def test_reject_invalid_dt(
    velocity_limits, acceleration_limits, dt
):
    """Reject invalid control intervals."""
    pipeline = SafeControlPipeline(
        velocity_limits,
        acceleration_limits=acceleration_limits,
    )

    with pytest.raises(ValueError, match="dt"):
        pipeline.apply(
            velocity_output([0.0, 0.0, 0.0]),
            previous_velocity=np.zeros(3),
            dt=dt,
        )
