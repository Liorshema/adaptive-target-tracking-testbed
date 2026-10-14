"""Regress online updates whose cadence differs from forecast spacing."""

import numpy as np
import pytest

from robot_prediction.baseline.constant_velocity import ConstantVelocityPredictor
from robot_prediction.baseline.imm import IMMPredictor
from robot_prediction.confidence.performance_monitor import PerformanceMonitor
from robot_prediction.interfaces.prediction_data import PredictionInput, TrajectoryPrediction
from robot_prediction.interfaces.predictor import Predictor
from robot_prediction.online_adaptation.classical.residual_adapter import ResidualAdapter
from robot_prediction.trajectory_prediction import TrajectoryPredictionSystem


def accelerating_state(timestamp):
    """Evaluate independent one-dimensional constant-acceleration motion."""
    return np.array([
        timestamp + 0.5 * timestamp ** 2, 0.0, 1.0,
        1.0 + timestamp, 0.0, 0.0,
    ])


def make_imm():
    """Create the actual online motion-model mixture."""
    return IMMPredictor(
        turn_rate_epsilon=1.0e-6,
        likelihood_variance=1.0e-3,
    )


def predict(predictor, state, timestamp, horizon=3):
    """Forecast with spacing independent of observation cadence."""
    return predictor.predict(PredictionInput(
        state=state, timestamp=timestamp, dt=0.1, horizon=horizon,
    ))


@pytest.mark.parametrize('observation_dt', [0.03, 0.15])
def test_imm_compares_constant_velocity_at_observation_time(observation_dt):
    """An exact CV observation has zero residual between forecast samples."""
    predictor = make_imm()
    origin = np.array([0.0, 0.0, 1.0, 2.0, -1.0, 0.5])
    predict(predictor, origin, timestamp=4.0)
    observed = origin.copy()
    observed[:3] += observation_dt * origin[3:]

    predictor.update(observed, observation_dt=observation_dt)

    np.testing.assert_allclose(predictor.last_residuals, 0.0, atol=1.0e-14)


def test_imm_rewards_acceleration_model_at_faster_observation_cadence():
    """Model evidence must favor CA after learning exact accelerated motion."""
    predictor = make_imm()
    for timestamp in [0.0, 0.02, 0.04]:
        if timestamp:
            predictor.update(
                accelerating_state(timestamp), observation_dt=0.02,
            )
        predict(predictor, accelerating_state(timestamp), timestamp)
    prior = predictor.predicted_probabilities

    posterior = predictor.update(
        accelerating_state(0.06), observation_dt=0.02,
    )

    assert posterior[1] > prior[1]
    assert np.linalg.norm(predictor.last_residuals[1]) < 0.001


def test_timestamp_only_system_update_uses_actual_elapsed_time():
    """An explicit observation stamp also supplies the IMM update interval."""
    predictor = make_imm()
    system = TrajectoryPredictionSystem(
        predictor=predictor,
        performance_monitor=PerformanceMonitor(10, 0.5),
    )
    for timestamp in [0.0, 0.03, 0.06]:
        if timestamp:
            system.update(
                accelerating_state(timestamp),
                observation_timestamp=timestamp,
            )
        predict(system, accelerating_state(timestamp), timestamp)

    np.testing.assert_allclose(predictor.estimated_acceleration, [1.0, 0.0, 0.0])


def test_repeated_observations_score_their_timestamp_not_the_update_interval():
    """An unchanged forecast can score multiple observations at distinct times."""
    predictor = make_imm()
    system = TrajectoryPredictionSystem(predictor, PerformanceMonitor(10, .5))
    initial = np.array([0., 0., 1., 1., 0., 0.])
    predict(system, initial, 0.)
    for timestamp in [.1, .2]:
        observed = initial.copy()
        observed[0] = timestamp
        performance = system.update(observed, observation_dt=.1,
                                    observation_timestamp=timestamp)
        np.testing.assert_allclose(performance.residual, 0., atol=1e-12)
        np.testing.assert_allclose(predictor.last_residuals, 0., atol=1e-12)


def test_imm_skips_model_evidence_after_horizon_but_updates_motion():
    """A late observation cannot be scored against an expired forecast."""
    predictor = make_imm()
    for timestamp in [0.0, 0.02, 0.04]:
        if timestamp:
            predictor.update(
                accelerating_state(timestamp), observation_dt=0.02,
            )
        predict(predictor, accelerating_state(timestamp), timestamp, horizon=1)
    prior = predictor.predicted_probabilities

    posterior = predictor.update(
        accelerating_state(0.2), observation_dt=0.16,
    )

    np.testing.assert_allclose(posterior, prior)
    np.testing.assert_allclose(predictor.estimated_acceleration, [1.0, 0.0, 0.0])


class PositionBiasedPredictor(Predictor):
    """Supply a deterministic forecast calibration error for adaptation."""

    def predict(self, prediction_input):
        """Forecast CV motion with a constant negative position bias."""
        baseline = ConstantVelocityPredictor().predict(prediction_input)
        states = baseline.states.copy()
        states[:, 0] -= 0.1
        return TrajectoryPrediction(states=states, timestamps=baseline.timestamps)


def test_residual_adaptation_retains_baseline_bias_after_correcting_forecast():
    """Successful correction must not erase the bias estimate that produced it."""
    monitor = PerformanceMonitor(10, 0.5)
    system = TrajectoryPredictionSystem(
        predictor=PositionBiasedPredictor(),
        performance_monitor=monitor,
        residual_adapter=ResidualAdapter(1.0),
    )
    for step in range(3):
        timestamp = step * 0.1
        origin = np.array([timestamp, 0.0, 1.0, 1.0, 0.0, 0.0])
        prediction = predict(system, origin, timestamp)
        observed = origin.copy()
        observed[0] += 0.1
        performance = system.update(
            observed, observation_dt=0.1,
            observation_timestamp=timestamp + 0.1,
        )
        if step > 0:
            np.testing.assert_allclose(prediction.states[0], observed, atol=1.0e-14)
            np.testing.assert_allclose(performance.residual, 0.0, atol=1.0e-14)

    np.testing.assert_allclose(monitor.baseline_statistics.mean, [0.1, 0, 0, 0, 0, 0])
    assert performance.confidence > 0.99
