"""Causal, bounded polynomial EDMD on a fixed identification interval.

Only received estimates train the model. A fitted affine normalization and
polynomial dictionary are frozen with each regularized lifted transition.
Forecasts propagate K repeatedly (without relifting) and decode physical states.
"""

from collections import deque
from itertools import combinations_with_replacement
from math import comb
from numbers import Integral
from time import perf_counter

import numpy as np

from robot_configuration.model_registration import ModelRegistration

from robot_prediction.baseline.constant_velocity import ConstantVelocityPredictor
from robot_prediction.interfaces.prediction_data import PredictionInput, TrajectoryPrediction
from robot_prediction.interfaces.predictor import Predictor
from robot_prediction.interfaces.state_layout import StateLayout


class PolynomialDictionary:
    """Constant, original normalized coordinates and total-degree monomials."""

    def __init__(self, center, scale, degree, max_features):
        """Build a bounded normalized dictionary and exact physical decoder."""
        self.center = np.array(center, dtype=float, copy=True)
        self.scale = np.array(scale, dtype=float, copy=True)
        if (
            self.center.ndim != 1
            or not self.center.size
            or self.scale.shape != self.center.shape
            or not np.all(np.isfinite(self.center))
            or not np.all(np.isfinite(self.scale))
            or np.any(self.scale <= 0)
            or type(degree) is not int
            or not 1 <= degree <= 3
        ):
            raise ValueError('Invalid EDMD dictionary dimensions, scales or degree')
        dimension = self.center.size
        if comb(dimension + degree, degree) > max_features:
            raise ValueError('EDMD observable dictionary exceeds max_features')
        self.terms = [()] + [
            term
            for power in range(1, degree + 1)
            for term in combinations_with_replacement(range(dimension), power)
        ]
        self.decoder = np.zeros((dimension, len(self.terms)))
        self.decoder[:, 0] = self.center
        self.decoder[:, 1:dimension + 1] = np.diag(self.scale)

    def lift(self, states):
        """Lift a physical vector or matrix with state coordinates on rows."""
        if np.iscomplexobj(states):
            raise ValueError('EDMD states must be real')
        states = np.asarray(states, dtype=float)
        if (
            states.ndim not in (1, 2)
            or states.shape[0] != len(self.center)
            or not np.all(np.isfinite(states))
        ):
            raise ValueError('Invalid EDMD physical states')
        try:
            with np.errstate(over='raise', invalid='raise', divide='raise'):
                normalized = (
                    (states - self.center) / self.scale
                    if states.ndim == 1
                    else (states - self.center[:, None]) / self.scale[:, None]
                )
                ones = np.ones(normalized.shape[1:] or ())
                return np.array(
                    [
                        np.prod(normalized[list(term)], axis=0) if term else ones
                        for term in self.terms
                    ]
                )
        except FloatingPointError as error:
            raise ValueError('EDMD observable overflow') from error


def fit_edmd(X, Y, regularization, max_condition):
    """Return ridge EDMD K=YXᵀ(XXᵀ+lambda I)^-1 via SVD, never an inverse."""
    X, Y = np.asarray(X, dtype=float), np.asarray(Y, dtype=float)
    if (
        X.ndim != 2
        or X.shape != Y.shape
        or min(X.shape) == 0
        or not np.all(np.isfinite(X))
        or not np.all(np.isfinite(Y))
        or not np.isfinite(regularization)
        or regularization <= 0
        or not np.isfinite(max_condition)
        or max_condition < 1
    ):
        raise ValueError('Invalid EDMD snapshots or ridge configuration')
    try:
        U, singular, Vt = np.linalg.svd(X, full_matrices=False)
        smallest = singular[-1] if len(singular) == X.shape[0] else 0.0
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            condition = (singular[0] ** 2 + regularization) / (smallest**2 + regularization)
            if not np.isfinite(condition) or condition > max_condition:
                raise ValueError('EDMD regularized Gram matrix exceeds max_condition')
            operator = ((Y @ Vt.T) * (singular / (singular**2 + regularization))) @ U.T
        if not np.all(np.isfinite(operator)):
            raise ValueError('Nonfinite EDMD operator')
        return operator, float(condition)
    except (np.linalg.LinAlgError, FloatingPointError) as error:
        raise ValueError('EDMD numerical fit failed') from error


class KoopmanEDMDPredictor(Predictor):
    """Fit each Run from causal estimates; report CV warm-up explicitly."""

    def __init__(
        self,
        *,
        basis,
        polynomial_degree,
        regularization,
        training_interval,
        history_capacity,
        min_training_samples,
        refit_interval,
        max_condition,
        minimum_scale,
        max_observation_gap,
        max_features,
        max_transition_steps,
        max_spectral_radius,
    ):
        """Configure the causal window, feature family and numerical eligibility policy."""
        for name, value, minimum in [
            ('polynomial_degree', polynomial_degree, 1),
            ('history_capacity', history_capacity, 2),
            ('min_training_samples', min_training_samples, 2),
            ('refit_interval', refit_interval, 1),
            ('max_features', max_features, 2),
            ('max_transition_steps', max_transition_steps, 1),
        ]:
            if type(value) is not int or value < minimum:
                raise ValueError(f'{name} must be an integer >= {minimum}')
        if basis not in ('linear', 'polynomial') or polynomial_degree > 3:
            raise ValueError('EDMD supports linear or polynomial degree 1..3')
        if min_training_samples > history_capacity:
            raise ValueError('history_capacity must cover min_training_samples')
        for name, value in [
            ('regularization', regularization),
            ('training_interval', training_interval),
            ('max_condition', max_condition),
            ('minimum_scale', minimum_scale),
            ('max_observation_gap', max_observation_gap),
            ('max_spectral_radius', max_spectral_radius),
        ]:
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if max_condition < 1 or max_observation_gap < training_interval:
            raise ValueError(
                'max_condition >= 1 and max_observation_gap >= training_interval required'
            )
        self._degree = polynomial_degree if basis == 'polynomial' else 1
        self._regularization, self._interval = regularization, training_interval
        self._capacity, self._minimum, self._refit_interval = (
            history_capacity,
            min_training_samples,
            refit_interval,
        )
        self._max_condition, self._minimum_scale = max_condition, minimum_scale
        self._max_gap, self._max_features = max_observation_gap, max_features
        self._max_transition_steps = max_transition_steps
        self._max_spectral_radius = max_spectral_radius
        self._cv = ConstantVelocityPredictor()
        self.reset()

    def reset(self):
        """Discard all observations and learned state; never share Run history."""
        self._history = deque(maxlen=self._capacity + 1)
        self._last_state = self._last_time = self._origin = None
        self._grid_steps = self._since_fit = self._fits = self._fit_failures = 0
        self._operator = self._dictionary = self._condition = None
        self._spectral_radius = None
        self._status = 'cv_warmup'
        self._fit_error = None
        self._fit_seconds = self._update_seconds = self._predict_seconds = 0.0

    @staticmethod
    def _state(state):
        if np.iscomplexobj(state):
            raise ValueError('EDMD state must be real')
        state = np.array(state, dtype=float, copy=True)
        if state.ndim != 1 or not np.all(np.isfinite(state)):
            raise ValueError('EDMD state must be a finite vector')
        StateLayout.from_state_dimension(state.size)
        return state

    def _seed(self, state, timestamp):
        # Validate dictionary capacity before any observations are accepted.
        if timestamp + self._interval <= timestamp:
            raise ValueError('EDMD training grid cannot advance at this timestamp')
        PolynomialDictionary(
            np.zeros(len(state)), np.ones(len(state)), self._degree, self._max_features
        )
        self._last_state, self._last_time, self._origin = state.copy(), timestamp, timestamp
        self._history.append(state.copy())

    def update_at(self, observed_state, observation_dt, observation_timestamp):
        """Causally interpolate completed fixed-grid states from received brackets."""
        started = perf_counter()
        state = self._state(observed_state)
        timestamp = float(observation_timestamp)
        if not np.isfinite(timestamp) or not np.isfinite(observation_dt) or observation_dt <= 0:
            self.reset()
            raise ValueError('EDMD observation time/dt must advance and be finite')
        if self._last_time is None:
            self._seed(state, timestamp)
        else:
            elapsed = timestamp - self._last_time
            if elapsed <= 0 or not np.isclose(elapsed, observation_dt, rtol=1e-8, atol=1e-10):
                self.reset()
                raise ValueError('EDMD observation time sequence is invalid; history reset')
            if state.shape != self._last_state.shape:
                self.reset()
                raise ValueError('EDMD state dimension changed; history reset')
            if elapsed > self._max_gap:
                self.reset()
                self._seed(state, timestamp)
            else:
                progress = (timestamp - self._origin) / self._interval
                if not np.isfinite(progress):
                    self.reset()
                    raise ValueError('EDMD resampling exceeds transition work budget; reset')
                nearest = np.rint(progress)
                if abs(progress - nearest) <= 8 * np.finfo(float).eps * max(1.0, abs(progress)):
                    progress = nearest
                pending = int(np.floor(progress)) - self._grid_steps
                if pending > self._max_transition_steps:
                    self.reset()
                    raise ValueError('EDMD resampling exceeds transition work budget; reset')
                for _ in range(pending):
                    next_time = self._origin + (self._grid_steps + 1) * self._interval
                    fraction = np.clip((next_time - self._last_time) / elapsed, 0.0, 1.0)
                    self._history.append((1 - fraction) * self._last_state + fraction * state)
                    self._grid_steps += 1
                    self._since_fit += 1
                self._last_state, self._last_time = state, timestamp
                if len(self._history) - 1 >= self._minimum and (
                    self._operator is None
                    and self._status == 'cv_warmup'
                    or self._since_fit >= self._refit_interval
                ):
                    self._fit()
        self._update_seconds = perf_counter() - started

    def _fit(self):
        started = perf_counter()
        states = np.asarray(self._history).T
        center = np.mean(states, axis=1)
        scale = np.maximum(np.std(states, axis=1), self._minimum_scale)
        failure_status = 'cv_conditioning'
        self._spectral_radius = None
        try:
            dictionary = PolynomialDictionary(center, scale, self._degree, self._max_features)
            operator, condition = fit_edmd(
                dictionary.lift(states[:, :-1]),
                dictionary.lift(states[:, 1:]),
                self._regularization,
                self._max_condition,
            )
            radius = float(np.max(np.abs(np.linalg.eigvals(operator))))
            self._spectral_radius = radius
            if not np.isfinite(radius) or radius > self._max_spectral_radius:
                failure_status = 'cv_unstable'
                raise ValueError(f'EDMD spectral radius {radius} exceeds max_spectral_radius')
            self._dictionary, self._operator, self._condition = dictionary, operator, condition
            self._status = 'trained'
            self._fit_error = None
            self._fits += 1
        except (ValueError, np.linalg.LinAlgError) as error:
            self._operator = self._dictionary = self._condition = None
            self._fit_failures += 1
            self._status = failure_status
            self._fit_error = str(error)
        self._since_fit = 0
        self._fit_seconds = perf_counter() - started

    def predict(self, prediction_input: PredictionInput) -> TrajectoryPrediction:
        """Propagate a frozen lifted model, or emit an explicitly labeled warm-up CV."""
        started = perf_counter()
        state = self._state(prediction_input.state)
        t, dt, horizon = prediction_input.timestamp, prediction_input.dt, prediction_input.horizon
        if (
            not np.isfinite(t)
            or not np.isfinite(dt)
            or dt <= 0
            or isinstance(horizon, bool)
            or not isinstance(horizon, Integral)
            or horizon <= 0
        ):
            raise ValueError('EDMD prediction time/grid must be finite and positive')
        if horizon > self._max_transition_steps:
            raise ValueError('EDMD forecast exceeds transition work budget')
        required_steps = (dt / self._interval) * horizon
        if (
            not np.isfinite(required_steps)
            or required_steps > self._max_transition_steps + 1e-8
        ):
            raise ValueError('EDMD forecast exceeds transition work budget')
        if self._last_time is None:
            self._seed(state, float(t))
        if t < self._last_time - 1e-10:
            self.reset()
            raise ValueError('EDMD prediction time precedes future training observations; reset')
        if not np.isclose(t, self._last_time, rtol=0.0, atol=1e-10):
            raise ValueError('EDMD predict requires a timestamped observation update first')
        if state.shape != self._last_state.shape:
            raise ValueError('EDMD prediction state dimension changed')
        times = t + dt * np.arange(1, horizon + 1, dtype=float)
        if not np.all(np.isfinite(times)) or times[0] <= t or np.any(np.diff(times) <= 0):
            raise ValueError('EDMD forecast timestamps must advance finitely')
        if self._operator is None:
            try:
                with np.errstate(over='raise', invalid='raise'):
                    baseline = self._cv.predict(prediction_input)
            except FloatingPointError as error:
                raise ValueError('EDMD CV fallback overflow') from error
            result = TrajectoryPrediction(
                baseline.states, baseline.timestamps, model_name='koopman_edmd/' + self._status
            )
        else:
            steps = dt * np.arange(1, horizon + 1) / self._interval
            nearest = np.rint(steps)
            # Preserve exact K^h on integer grids despite floating division.
            close = np.abs(steps - nearest) <= 8 * np.finfo(float).eps * np.maximum(
                1.0, np.abs(steps)
            )
            steps = np.where(close, nearest, steps)
            maximum = int(np.ceil(steps[-1]))
            z = self._dictionary.lift(state)
            decoded = [self._dictionary.decoder @ z]
            try:
                with np.errstate(over='raise', invalid='raise'):
                    for _ in range(maximum):
                        z = self._operator @ z
                        decoded.append(self._dictionary.decoder @ z)
                    decoded = np.asarray(decoded)
                    lower = np.floor(steps).astype(int)
                    fraction = steps - lower
                    states = (1 - fraction[:, None]) * decoded[lower] + fraction[
                        :, None
                    ] * decoded[np.minimum(lower + 1, maximum)]
                if not np.all(np.isfinite(states)):
                    raise ValueError('Nonfinite EDMD prediction')
            except FloatingPointError as error:
                raise ValueError('EDMD lifted prediction overflow') from error
            result = TrajectoryPrediction(states, times, model_name='koopman_edmd')
        if not np.all(np.isfinite(result.states)):
            raise ValueError('Nonfinite EDMD prediction or fallback')
        self._predict_seconds = perf_counter() - started
        return result

    @property
    def diagnostics(self):
        """Return detached learning state and measured wall durations, not uncertainty."""
        return {
            'status': self._status,
            'training_pairs': max(0, len(self._history) - 1),
            'fit_count': self._fits,
            'fit_failures': self._fit_failures,
            'condition': self._condition,
            'last_observation_time': self._last_time,
            'spectral_radius': self._spectral_radius,
            'fit_error': self._fit_error,
            'last_grid_time': (
                None if self._origin is None else self._origin + self._grid_steps * self._interval
            ),
            'fit_seconds': self._fit_seconds,
            'update_seconds': self._update_seconds,
            'predict_seconds': self._predict_seconds,
        }


_PROPERTIES = {
    'basis': {'type': 'string', 'enum': ['linear', 'polynomial'], 'unit': '1'},
    'polynomial_degree': {'type': 'integer', 'minimum': 1, 'maximum': 3, 'unit': 'degree'},
    'regularization': {
        'type': 'number',
        'exclusiveMinimum': 0.0,
        'unit': 'normalized feature squared',
    },
    'training_interval': {'type': 'number', 'exclusiveMinimum': 0.0, 'unit': 's'},
    'history_capacity': {'type': 'integer', 'minimum': 2, 'unit': 'transition pairs'},
    'min_training_samples': {'type': 'integer', 'minimum': 2, 'unit': 'transition pairs'},
    'refit_interval': {'type': 'integer', 'minimum': 1, 'unit': 'new transition pairs'},
    'max_condition': {'type': 'number', 'minimum': 1.0, 'unit': '1'},
    'minimum_scale': {'type': 'number', 'exclusiveMinimum': 0.0, 'unit': 'state coordinate units'},
    'max_observation_gap': {'type': 'number', 'exclusiveMinimum': 0.0, 'unit': 's'},
    'max_features': {'type': 'integer', 'minimum': 2, 'unit': 'observables'},
    'max_transition_steps': {'type': 'integer', 'minimum': 1, 'unit': 'transition steps per call'},
    'max_spectral_radius': {
        'type': 'number',
        'exclusiveMinimum': 0.0,
        'unit': '1 per training interval',
    },
}
COMPONENTS = (
    ModelRegistration(
        'koopman_edmd',
        KoopmanEDMDPredictor,
        {
            'type': 'object',
            'properties': _PROPERTIES,
            'required': list(_PROPERTIES),
            'additionalProperties': False,
        },
        requirements={'state_layout': 'position_velocity'},
        description='Koopman (EDMD)',
        capabilities={
            'deployable': True,
            'runtime_validation': 'causal_online_edmd',
            'display_name': 'Koopman (EDMD)',
            'input_layout': 'position_velocity',
            'output_layout': 'position_velocity',
            'coordinate_frames': 'same fixed frame as input',
            'uncertainty': 'none',
            'learning': 'bounded causal timestamped updates',
        },
    ),
)
