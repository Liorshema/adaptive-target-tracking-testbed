import numpy as np

from robot_prediction.baseline.constant_velocity import (
    ConstantVelocityPredictor,
)
from robot_prediction.interfaces.prediction_data import PredictionInput


def test_constant_velocity_prediction():
    predictor = ConstantVelocityPredictor()

    prediction_input = PredictionInput(
        state=np.array([1.0, 2.0, 0.5, -0.2]),
        timestamp=0.0,
        dt=1.0,
        horizon=4,
    )

    prediction = predictor.predict(prediction_input)

    expected_states = np.array([
        [1.5, 1.8, 0.5, -0.2],
        [2.0, 1.6, 0.5, -0.2],
        [2.5, 1.4, 0.5, -0.2],
        [3.0, 1.2, 0.5, -0.2],
    ])

    expected_timestamps = np.array([
        1.0,
        2.0,
        3.0,
        4.0,
    ])

    assert np.allclose(prediction.states, expected_states)
    assert np.allclose(prediction.timestamps, expected_timestamps)
    assert prediction.model_name == 'constant_velocity'
