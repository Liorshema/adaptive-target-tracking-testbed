"""Estimation-only position hold through the standard Predictor contract."""

from numbers import Integral

import numpy as np

from robot_configuration.model_registration import ModelRegistration

from robot_prediction.interfaces.prediction_data import TrajectoryPrediction
from robot_prediction.interfaces.predictor import Predictor
from robot_prediction.interfaces.state_layout import StateLayout


class PositionHoldPredictor(Predictor):
    """Repeat the latest estimated position; future velocity is identically zero."""

    def predict(self, prediction_input):
        """Produce future samples without extrapolating the estimated velocity."""
        raw = prediction_input.state
        if np.iscomplexobj(raw):
            raise ValueError('Position hold requires a real state')
        state = np.asarray(raw, dtype=float)
        if state.ndim != 1 or not np.all(np.isfinite(state)):
            raise ValueError('Position hold requires a finite state vector')
        layout = StateLayout.from_state_dimension(state.size)
        t, dt, horizon = prediction_input.timestamp, prediction_input.dt, prediction_input.horizon
        if (
            not np.isfinite(t)
            or not np.isfinite(dt)
            or dt <= 0
            or isinstance(horizon, bool)
            or not isinstance(horizon, Integral)
            or horizon <= 0
        ):
            raise ValueError('Position hold requires a finite positive prediction grid')
        with np.errstate(over='ignore', invalid='ignore'):
            times = t + dt * np.arange(1, horizon + 1, dtype=float)
        if not np.all(np.isfinite(times)) or times[0] <= t or np.any(np.diff(times) <= 0):
            raise ValueError('Position hold timestamps must advance finitely')
        states = np.zeros((horizon, state.size))
        states[:, layout.position_slice()] = state[layout.position_slice()]
        return TrajectoryPrediction(states, times, model_name='none')


COMPONENTS = (
    ModelRegistration(
        'none',
        PositionHoldPredictor,
        {'type': 'object', 'properties': {}, 'additionalProperties': False},
        requirements={
            'state_layout': 'position_velocity',
            'adaptation_type': 'none',
            'fusion_type': 'none',
        },
        description='None',
        capabilities={
            'deployable': True,
            'runtime_validation': 'finite_position_hold',
            'display_name': 'None',
            'input_layout': 'position_velocity',
            'output_layout': 'position_velocity',
            'uncertainty': 'none',
            'coordinate_frames': 'same fixed frame as input',
        },
    ),
)
