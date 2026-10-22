"""Analytical fixtures independent of ROS, recording cadence and robot layout."""

import math

import numpy as np

import pytest

from robot_experiments.metrics.joint_diagnostics import (
    body_to_world, causal_indices, named_joint_series, sampled_motion,
)


def test_irregular_time_integral_and_gap_exclusion():
    """Irregular time integral and gap exclusion."""
    result = sampled_motion([0., 1., 3., 10.], [0., 2., 4., 100.],
                            [2., 2., 2., 10.], 0., 12., 2.)
    assert result['observed_duration_s'] == 3.
    assert result['velocity_rms_observed_rad_s'] == 2.
    assert result['full_window_normalized_observed_rms_rad_s'] == 1.
    assert result['coverage_fraction'] == .25
    assert result['observed_travel_excluding_gaps_rad'] == 4.
    assert result['sampled_adjacent_travel_rad'] == 100.
    assert result['largest_gap_including_edges_s'] == 7.


def test_trapezoid_squared_velocity_not_equal_sample_average():
    """Trapezoid squared velocity not equal sample average."""
    result = sampled_motion([0., 1., 3.], [0., 0., 0.], [0., 2., 0.], 0., 3., 2.)
    assert result['velocity_squared_integral_rad2_per_s'] == 6.
    assert result['velocity_rms_observed_rad_s'] == pytest.approx(math.sqrt(2))


def test_tolerance_and_left_held_violation_duration():
    """Tolerance and left held violation duration."""
    result = sampled_motion([0., 1., 2., 10.], [0., -.002, 0., 0.],
                            [1., 1.0005, 1.1, 2.], 0., 10., 1.,
                            speed_limit=1., position_limits=(0., 1.))
    assert result['speed_limit']['violating_samples'] == 2
    assert result['speed_limit']['observed_violation_duration_s'] == 0.
    assert result['speed_limit']['max_excess_above_description_rad_s'] == 1.
    assert result['position_limits']['observed_violation_duration_s'] == 1.


def test_names_are_resolved_for_each_reordered_sample():
    """Names are resolved for each reordered sample."""
    records = [
        {'source_time': 1., 'data': {
            'names': ['wrist', 'wheel'], 'position': [1., 20.], 'velocity': [3., 40.]}},
        {'source_time': 2., 'data': {
            'names': ['wheel', 'wrist'], 'position': [21., 2.], 'velocity': [41., 4.]}},
    ]
    t, q, v, rejected = named_joint_series(records, 'wrist')
    np.testing.assert_equal(t, [1., 2.])
    np.testing.assert_equal(q, [1., 2.])
    np.testing.assert_equal(v, [3., 4.])
    assert rejected == 0


def test_causal_alignment_never_uses_future_and_expires_gaps():
    """Causal alignment never uses future and expires gaps."""
    np.testing.assert_equal(causal_indices([1., 2., 10.], [.9, 1., 1.5, 2.1, 9., 10.], .2),
                            [-1, 0, -1, 1, -1, 2])
    np.testing.assert_equal(causal_indices([], [1.], .2), [-1])


def test_empty_window_is_unavailable_not_zero_motion():
    """Empty window is unavailable not zero motion."""
    result = sampled_motion([], [], [], 0., 3., .1)
    assert not result['available']
    assert 'velocity_rms_observed_rad_s' not in result
    assert result['observed_duration_s'] == 0.


def test_rotation_distinguishes_world_and_body_velocity():
    """Rotation distinguishes world and body velocity."""
    rotated = body_to_world([0., 0., math.sqrt(.5), math.sqrt(.5)], [2., 0., 0.])
    np.testing.assert_allclose(rotated, [0., 2., 0.], atol=1e-14)


@pytest.mark.parametrize('times', [[0., 0.], [1., 0.]])
def test_duplicate_or_reset_epoch_must_not_be_integrated(times):
    """Duplicate or reset epoch must not be integrated."""
    with pytest.raises(ValueError):
        sampled_motion(times, [0., 0.], [0., 0.], 0., 2., 1.)


def test_named_series_refuses_to_merge_reset_epochs():
    """Named series refuses to merge reset epochs."""
    records = [{'source_time': stamp, 'data': {'names': ['joint'],
                'position': [0.], 'velocity': [0.]}} for stamp in [5., 1., 6.]]
    with pytest.raises(ValueError, match='split clock epochs'):
        named_joint_series(records, 'joint')


def test_nonfinite_feedback_is_not_integrated_or_zero_filled():
    """Nonfinite feedback is not integrated or zero filled."""
    with pytest.raises(ValueError, match='nonfinite'):
        sampled_motion([0., 1.], [0., 0.], [0., float('nan')], 0., 2., 1.)
