"""Analytical contracts for source-time target diagnostics, without ROS."""

import numpy as np

import pytest

from robot_experiments.metrics.target_diagnostics import (
    bounded_interpolate, bounded_previous, common_grid_roughness,
    error_metrics, replay_constant_velocity, sampling_summary, turn_lags,
)


def test_vector_rmse_and_bias_are_not_axis_averages():
    """A 3-4-5 error has norm RMSE five and signed y bias four."""
    result = error_metrics([[3., 4., 0.], [-3., 4., 0.]])
    assert result['rmse_norm_m'] == pytest.approx(5.)
    assert result['rmse_axis_m'] == pytest.approx([3., 4., 0.])
    assert result['bias_axis_m'] == pytest.approx([0., 4., 0.])


def test_interpolation_requires_brackets_and_respects_gaps():
    """Retain exact endpoints without interpolating across a dropout."""
    values = bounded_interpolate([0., 1., 4.], [[0.], [2.], [8.]],
                                 [-.1, 0., .5, 2., 4., 4.1], max_gap=1.1)
    assert values[[1, 2, 4], 0] == pytest.approx([0., 1., 8.])
    assert np.isnan(values[[0, 3, 5], 0]).all()


@pytest.mark.parametrize('times', [[0., 0.], [1., 0.], [0., float('nan')]])
def test_interpolation_rejects_noncausal_or_nonfinite_epochs(times):
    """Ambiguous epochs must be split before numerical alignment."""
    with pytest.raises(ValueError):
        bounded_interpolate(times, [[0.], [1.]], [0.], max_gap=1.)


def test_common_grid_roughness_matches_constant_acceleration():
    """Quadratic motion has the expected acceleration on the common grid."""
    times = np.arange(0., 2.01, .1)
    position = np.column_stack([times ** 2, np.zeros_like(times), times * 3.])
    result = common_grid_roughness(times, [position], step=.1, max_gap=.11)
    assert result['series'][0]['acceleration_rms_m_s2'] == pytest.approx(2.)
    assert result['series'][0]['acceleration_rms_axis_m_s2'] == pytest.approx([2., 0., 0.])


def test_roughness_does_not_connect_missing_data():
    """Only triples wholly supported by observed epochs contribute."""
    times = np.array([0., .1, .2, 1., 1.1, 1.2])
    position = np.column_stack([times, times * 0., times * 0.])
    result = common_grid_roughness(times, [position], step=.1, max_gap=.11)
    assert result['valid_triples'] == 2
    assert result['series'][0]['acceleration_rms_m_s2'] == pytest.approx(0., abs=1e-12)


def test_first_kalman_update_has_known_innovation_gain_and_joseph_covariance():
    """The first observation corrects the initial prior without prediction."""
    replay = replay_constant_velocity(
        [1.], [[2., 0., 0.]], np.zeros(6), np.eye(6), [1., 1., 1.], 0.)
    assert replay['innovation'][0] == pytest.approx([2., 0., 0.])
    assert replay['mean'][0] == pytest.approx([1., 0., 0., 0., 0., 0.])
    assert np.diag(replay['covariance'][0]) == pytest.approx([.5, .5, .5, 1., 1., 1.])
    assert replay['gain'][0, 0, 0] == pytest.approx(.5)
    assert replay['nis'][0] == pytest.approx(2.)


def test_second_kalman_update_predicts_before_innovation():
    """A two-observation fixture determines the position and velocity gains."""
    replay = replay_constant_velocity(
        [1., 2.], [[2., 0., 0.], [3., 0., 0.]], np.zeros(6), np.eye(6),
        [1., 1., 1.], 0.)
    assert replay['innovation'][1] == pytest.approx([2., 0., 0.])
    assert replay['gain'][1, 0, 0] == pytest.approx(.6)
    assert replay['gain'][1, 3, 0] == pytest.approx(.4)
    assert replay['mean'][1] == pytest.approx([2.2, 0., 0., .8, 0., 0.])


def test_replay_process_noise_uses_deployed_dt_fourth_contract():
    """The recorded model uses discrete held-acceleration noise."""
    replay = replay_constant_velocity(
        [0., 2.], np.zeros((2, 3)), np.zeros(6), np.zeros((6, 6)),
        [1., 1., 1.], 1.)
    assert replay['innovation_covariance'][1, 0, 0] == pytest.approx(5.)
    assert replay['gain'][1, 3, 0] == pytest.approx(.8)


@pytest.mark.parametrize('variance,sigma', [([0., 1., 1.], .1), ([1., 1., 1.], -1.)])
def test_replay_rejects_invalid_noise(variance, sigma):
    """Nonpositive measurement noise and negative process noise are invalid."""
    with pytest.raises(ValueError):
        replay_constant_velocity([0.], [[0., 0., 0.]], np.zeros(6),
                                 np.eye(6), variance, sigma)


def test_sampling_reports_duplicate_backwards_and_true_gaps():
    """Fault counts retain acquisition order before unique gap statistics."""
    result = sampling_summary([0., .1, .1, .05, 1.], gap_threshold=.2)
    assert result['duplicates'] == 1
    assert result['backward_steps'] == 1
    assert result['gaps_over_threshold'] == 1
    assert result['largest_gap_s'] == pytest.approx(.9)


def test_bounded_previous_never_uses_future_samples_or_bridges_stale_commands():
    """Commands require a causal and fresh source epoch."""
    indices = bounded_previous([1., 2., 3.], [.9, 1.1, 1.9, 2., 3.5], max_age=.2)
    assert indices.tolist() == [-1, 0, -1, 1, -1]


def test_local_turn_shift_recovers_known_delay():
    """An analytic sinusoid delayed by 200 ms recovers that local shift."""
    times = np.arange(0., 5.01, .05)
    truth = np.column_stack([np.cos(times * np.pi), times * 0., times * 0.])
    estimate = np.column_stack([np.cos((times - .2) * np.pi), times * 0., times * 0.])
    events = turn_lags(times, truth, estimate, step=.05, radius=.25, max_delay=.5)
    assert events
    assert all(event['lag_s'] == pytest.approx(.2) for event in events)


@pytest.mark.parametrize('limit', [float('nan'), float('inf'), -float('inf')])
def test_alignment_and_sampling_reject_nonfinite_limits(limit):
    """An infinite or undefined bound must not disable alignment constraints."""
    with pytest.raises(ValueError):
        bounded_interpolate([0., 1.], [[0.], [1.]], [.5], max_gap=limit)
    with pytest.raises(ValueError):
        bounded_previous([0., 1.], [.5], max_age=limit)
    with pytest.raises(ValueError):
        sampling_summary([0., 1.], gap_threshold=limit)


@pytest.mark.parametrize('parameter', ['step', 'radius', 'max_delay', 'minimum_excursion'])
def test_turn_search_rejects_nonfinite_configuration(parameter):
    """Every lag-search scalar must have a finite value."""
    settings = {'step': .05, 'radius': .2, 'max_delay': .5, 'minimum_excursion': .02}
    settings[parameter] = float('nan')
    with pytest.raises(ValueError):
        turn_lags([0., 1.], np.zeros((2, 3)), np.zeros((2, 3)), **settings)


def test_turn_search_requires_a_nonempty_window_on_each_side():
    """Turn comparisons need samples before and after the extremum."""
    with pytest.raises(ValueError):
        turn_lags([0., 1.], np.zeros((2, 3)), np.zeros((2, 3)), step=.1, radius=.01)
