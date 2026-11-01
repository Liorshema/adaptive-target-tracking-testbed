"""Test-only deterministic forecast with explicitly absent uncertainty."""

import numpy as np
from robot_prediction.baseline.stateless_model import predictor_model


@predictor_model(
    'example_forecast',
    parameters_schema={
        'type': 'object',
        'additionalProperties': False,
        'properties': {'acceleration': {'type': 'number', 'unit': 'm/s^2'}},
        'required': ['acceleration'],
    },
)
def forecast(state, horizons, parameters):
    t = horizons[:, None]
    a = parameters['acceleration']
    return np.concatenate((state[:3] + t * state[3:] + 0.5 * a * t * t, state[3:] + a * t), axis=1)


COMPONENTS = (forecast,)
