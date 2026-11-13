"""None means estimate position hold, never constant-velocity extrapolation."""

from pathlib import Path

import numpy as np

import pytest

from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.factory.system_factory import PredictionSystemFactory
from robot_prediction.interfaces.prediction_data import PredictionInput


def factory():
    """Use the real registered owner configuration."""
    return PredictionSystemFactory(
        PredictionConfig(Path(__file__).parents[1] / 'config/prediction.yaml')
    )


@pytest.mark.parametrize('d', [1, 2, 3, 5])
def test_none_holds_latest_estimate_with_zero_future_velocity(d):
    """A new estimate updates the hold; forecasts preserve source timestamps."""
    system = factory().create(predictor_type='none', adaptation_type='none', fusion_type='none')
    x = np.r_[np.arange(d) + 1.0, np.full(d, 7.0)]
    out = system.predict(PredictionInput(x, 2.0, 0.1, 5))
    np.testing.assert_array_equal(out.states[:, :d], np.repeat(x[None, :d], 5, axis=0))
    np.testing.assert_array_equal(out.states[:, d:], 0.0)
    np.testing.assert_allclose(out.timestamps, 2.0 + 0.1 * np.arange(1, 6))
    assert out.model_name == 'none' and out.covariance is None
    x[:d] += 3
    system.update(x, observation_timestamp=2.2)
    newer = system.predict(PredictionInput(x, 2.2, 0.1, 2))
    np.testing.assert_array_equal(newer.states[:, :d], np.repeat(x[None, :d], 2, axis=0))


@pytest.mark.parametrize(
    'change',
    [
        {'state': [1.0, np.nan]},
        {'state': [1.0, 2.0, 3.0]},
        {'dt': 0.0},
        {'timestamp': float('inf')},
        {'horizon': True},
        {'horizon': 1.5},
        {'timestamp': 1e300},
    ],
)
def test_none_rejects_invalid_input(change):
    """Keep the standard finite state and future-grid contract."""
    values = {'state': np.array([1.0, 5.0]), 'timestamp': 1.0, 'dt': 0.1, 'horizon': 3, **change}
    with pytest.raises(ValueError):
        factory().create(
            predictor_type='none', adaptation_type='none', fusion_type='none'
        ).predict(PredictionInput(**values))


def test_none_cannot_be_silently_corrected_into_a_motion_forecast():
    """Disabled forecasting requires disabled residual correction components."""
    with pytest.raises(ValueError):
        factory().create(
            predictor_type='none', adaptation_type='residual', fusion_type='weighted_correction'
        )
