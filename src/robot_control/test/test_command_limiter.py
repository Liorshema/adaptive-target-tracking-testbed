
"""Unit tests for generalized velocity command limiting."""

import numpy as np
import pytest
from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.velocity_limits import VelocityLimits

from robot_control.interfaces.control_data import ControlMode, ControlOutput
from robot_control.safety.command_limiter import CommandLimiter


def make_limiter(
    dof=3,
    velocity_bound=2.0,
    acceleration_bound=None,
):
    """Create limits for an arbitrary number of velocity DOFs."""
    velocity_limits = VelocityLimits(
        lower=np.full(dof, -velocity_bound),
        upper=np.full(dof, velocity_bound),
    )

    acceleration_limits = None

    if acceleration_bound is not None:
        acceleration_limits = AccelerationLimits(
            lower=np.full(dof, -acceleration_bound),
            upper=np.full(dof, acceleration_bound),
        )

    return CommandLimiter(
        velocity_limits=velocity_limits,
        acceleration_limits=acceleration_limits,
    )


def make_output(command):
    """Create a generalized velocity command."""
    return ControlOutput(
        command=np.asarray(command, dtype=float),
        mode=ControlMode.VELOCITY,
    )


def test_command_within_limits():
    """Commands already within bounds must remain unchanged."""
    limiter = make_limiter()

    requested = np.array([0.5, -1.0, 1.5])
    result = limiter.limit(make_output(requested))

    np.testing.assert_allclose(result.command, requested)


def test_velocity_clipping():
    """Limit each velocity component independently."""
    limiter = make_limiter()

    result = limiter.limit(
        make_output([3.0, -4.0, 0.5])
    )

    np.testing.assert_allclose(
        result.command,
        [2.0, -2.0, 0.5],
    )


def test_acceleration_clipping():
    """Limit velocity changes over a control interval."""
    limiter = make_limiter(acceleration_bound=1.0)

    result = limiter.limit(
        output=make_output([3.0, -2.0, 0.5]),
        previous_velocity=np.zeros(3),
        dt=0.1,
    )

    np.testing.assert_allclose(
        result.command,
        [0.1, -0.1, 0.1],
        atol=1e-12,
    )


def test_acceleration_around_nonzero_velocity():
    """Use the previous velocity as the acceleration reference."""
    limiter = make_limiter(acceleration_bound=1.0)

    result = limiter.limit(
        output=make_output([2.0, -2.0, 0.0]),
        previous_velocity=np.array([1.0, -1.0, 0.5]),
        dt=0.2,
    )

    np.testing.assert_allclose(
        result.command,
        [1.2, -1.2, 0.3],
        atol=1e-12,
    )


@pytest.mark.parametrize("dof", [1, 2, 4, 6, 8])
def test_arbitrary_velocity_dimensions(dof):
    """Support varying generalized velocity dimensions."""
    limiter = make_limiter(dof=dof)

    result = limiter.limit(
        make_output(np.full(dof, 5.0))
    )

    assert result.command.shape == (dof,)

    np.testing.assert_allclose(
        result.command,
        np.full(dof, 2.0),
    )


def test_asymmetric_velocity_limits():
    """Support independent lower and upper limits."""
    limiter = CommandLimiter(
        velocity_limits=VelocityLimits(
            lower=np.array([-1.0, -2.0]),
            upper=np.array([3.0, 0.5]),
        )
    )

    result = limiter.limit(
        make_output([-5.0, 4.0])
    )

    np.testing.assert_allclose(
        result.command,
        [-1.0, 0.5],
    )


def test_asymmetric_acceleration_limits():
    """Support different acceleration and braking bounds."""
    limiter = CommandLimiter(
        velocity_limits=VelocityLimits(
            lower=np.array([-5.0]),
            upper=np.array([5.0]),
        ),
        acceleration_limits=AccelerationLimits(
            lower=np.array([-2.0]),
            upper=np.array([1.0]),
        ),
    )

    accelerate = limiter.limit(
        make_output([4.0]),
        previous_velocity=np.array([0.0]),
        dt=0.1,
    )

    brake = limiter.limit(
        make_output([-4.0]),
        previous_velocity=np.array([0.0]),
        dt=0.1,
    )

    np.testing.assert_allclose(
        accelerate.command, [0.1]
    )
    np.testing.assert_allclose(
        brake.command, [-0.2]
    )


def test_infeasible_constraints():
    """Reject contradictory velocity and acceleration bounds."""
    limiter = make_limiter(
        dof=1,
        velocity_bound=1.0,
        acceleration_bound=0.5,
    )

    with pytest.raises(ValueError, match="infeasible"):
        limiter.limit(
            output=make_output([0.0]),
            previous_velocity=np.array([3.0]),
            dt=0.1,
        )


def test_missing_acceleration_inputs():
    """Require previous velocity and dt when acceleration is limited."""
    limiter = make_limiter(acceleration_bound=1.0)

    with pytest.raises(ValueError):
        limiter.limit(make_output([1.0, 0.0, 0.0]))


@pytest.mark.parametrize("dt", [0.0, -0.1, np.nan, np.inf])
def test_invalid_dt(dt):
    """Reject invalid control intervals."""
    limiter = make_limiter(acceleration_bound=1.0)

    with pytest.raises(ValueError):
        limiter.limit(
            output=make_output([1.0, 0.0, 0.0]),
            previous_velocity=np.zeros(3),
            dt=dt,
        )


def test_command_dimension_mismatch():
    """Reject commands that do not match the configured limits."""
    limiter = make_limiter(dof=3)

    with pytest.raises(ValueError):
        limiter.limit(make_output([1.0, 2.0]))


def test_previous_velocity_dimension_mismatch():
    """Reject incompatible previous velocity dimensions."""
    limiter = make_limiter(acceleration_bound=1.0)

    with pytest.raises(ValueError):
        limiter.limit(
            output=make_output([1.0, 0.0, 0.0]),
            previous_velocity=np.zeros(2),
            dt=0.1,
        )


def test_unsupported_control_mode():
    """Reject actuation modes not supported by this limiter."""
    limiter = make_limiter()

    output = ControlOutput(
        command=np.zeros(3),
        mode=ControlMode.EFFORT,
    )

    with pytest.raises(ValueError):
        limiter.limit(output)


def test_input_command_is_not_modified():
    """Keep the original controller output unchanged."""
    limiter = make_limiter()

    output = make_output([5.0, -5.0, 0.0])

    result = limiter.limit(output)

    np.testing.assert_allclose(
        output.command,
        [5.0, -5.0, 0.0],
    )

    np.testing.assert_allclose(
        result.command,
        [2.0, -2.0, 0.0],
    )
