"""Test-only fading-memory KF: P minus = lambda A P A' + Q, lambda in [1,4]."""

from robot_configuration.model_registration import ModelRegistration
from robot_estimation.factory.algorithm_factory import COMPONENTS as BUILTINS
from robot_estimation.filters.kalman_filter import KalmanFilter


class FadingKalmanFilter(KalmanFilter):
    """Inflate propagated prior uncertainty without changing process/measurement noise."""

    def __init__(self, fading_factor, **dependencies):
        super().__init__(**dependencies)
        self.fading_factor = fading_factor

    def predict(self, dt, control=None):
        prior = self.state.covariance
        transition = self._state_transition(dt)
        super().predict(dt, control)
        self._state.covariance += (self.fading_factor - 1.0) * transition @ prior @ transition.T
        return self.state


original = next(d for d in BUILTINS if d.identifier == 'kf')
COMPONENTS = (ModelRegistration(
    'example_fading_kf', FadingKalmanFilter,
    {'type': 'object', 'additionalProperties': False,
     'required': ['fading_factor'],
     'properties': {'fading_factor': {'type': 'number', 'minimum': 1.0, 'maximum': 4.0}}},
    {**original.capabilities, 'runtime_validation': 'test-only fading-memory equations'},
    dependencies=original.dependencies,
),)
