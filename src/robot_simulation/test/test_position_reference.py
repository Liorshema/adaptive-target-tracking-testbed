"""Verify measured initialization, time integration and anti-windup."""

import numpy as np

import pytest

from robot_simulation.actuation.position_reference import PositionReference


def test_integration_is_bounded_by_measured_position_and_physical_limits():
    """Repeated stalled commands cannot move the reference beyond its lead bound."""
    reference = PositionReference([-1., -1.], [1., 1.], [1., 2.], .08, .1)
    reference.initialize([.99, 0.])
    for _ in range(100):
        result = reference.step([1., 2.], [.99, 0.], .05)
    assert np.allclose(result, [1., .08])
    reference.initialize([.2, -.3])
    assert np.allclose(reference.step([.5, -1.], [.2, -.3], .02), [.21, -.32])


@pytest.mark.parametrize('dt', [0., -.1, .2, float('nan')])
def test_invalid_interval_does_not_advance_reference(dt):
    """Reject pauses, backward jumps and gaps without mutating the reference."""
    reference = PositionReference([-1.], [1.], [1.], .08, .1)
    reference.initialize([.3])
    with pytest.raises(ValueError):
        reference.step([.5], [.3], dt)
    assert np.allclose(reference.position, [.3])


def test_missing_initialization_or_unsafe_velocity_is_rejected():
    """Require a measured initial configuration and physically safe commands."""
    reference = PositionReference([-1.], [1.], [1.], .08, .1)
    with pytest.raises(ValueError):
        reference.step([.1], [0.], .05)
    reference.initialize([0.])
    with pytest.raises(ValueError):
        reference.step([2.], [0.], .05)
    with pytest.raises(ValueError):
        reference.initialize([float('nan')])


def test_lead_clipping_cannot_create_a_reference_speed_discontinuity():
    """Delayed displaced feedback must not instantaneously project the reference."""
    reference = PositionReference([-1.], [1.], [1.], .08, .1)
    reference.initialize([0.])
    with pytest.raises(ValueError, match='discontinuity'):
        reference.step([0.], [.5], .02)
    np.testing.assert_array_equal(reference.position, [0.])


@pytest.mark.parametrize('position,velocity,expected', [(1., -1., .98), (-1., 1., -.98)])
def test_command_reversal_at_a_limit_is_inward_and_continuous(position, velocity, expected):
    """Legal boundary reversals must not stick or jump."""
    reference = PositionReference([-1.], [1.], [1.], .08, .1)
    reference.initialize([position])
    np.testing.assert_allclose(reference.step([velocity], [position], .02), [expected])
