"""Bounded causal RBF acceleration regression around constant-velocity dynamics."""

from collections import deque
from numbers import Integral
from time import perf_counter

import numpy as np

from robot_configuration.model_registration import ModelRegistration

from robot_prediction.baseline.constant_velocity import ConstantVelocityPredictor
from robot_prediction.interfaces.prediction_data import PredictionInput, TrajectoryPrediction
from robot_prediction.interfaces.predictor import Predictor
from robot_prediction.interfaces.state_layout import StateLayout

from scipy.linalg import cho_solve, solve_triangular
from scipy.spatial.distance import cdist


class RBFRegression:
    """Independent zero-mean output GPs sharing one covariance factorization.

    Inputs are already normalized; outputs and all kernel/noise variances retain
    physical acceleration units. Jitter is a numerical diagonal nugget, distinct
    from observation noise. No matrix inverse or hyperparameter optimization.
    """

    def __init__(
        self, X, Y, length_scale, signal_variance, observation_noise, jitter, max_condition
    ):
        """Configure the regression factorization."""
        self.X, Y = np.array(X, dtype=float, copy=True), np.asarray(Y, dtype=float)
        if (
            self.X.ndim != 2
            or Y.ndim != 2
            or len(self.X) != len(Y)
            or min(self.X.shape) == 0
            or Y.shape[1] == 0
            or not np.all(np.isfinite(self.X))
            or not np.all(np.isfinite(Y))
        ):
            raise ValueError('GP requires finite aligned feature and output matrices')
        for value in (length_scale, signal_variance, observation_noise, jitter, max_condition):
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError('GP kernel and regularization parameters must be positive')
        self.length_scale, self.signal_variance = length_scale, signal_variance
        try:
            with np.errstate(over='raise', invalid='raise', divide='raise'):
                C = self.kernel(self.X, self.X)
                C.flat[:: len(C) + 1] += observation_noise + jitter
                eigenvalues = np.linalg.eigvalsh(C)
                self.condition = float(eigenvalues[-1] / eigenvalues[0])
                if (
                    eigenvalues[0] <= 0
                    or not np.isfinite(self.condition)
                    or (self.condition > max_condition)
                ):
                    raise ValueError('GP covariance exceeds condition ceiling')
                self._L = np.linalg.cholesky(C)
                self._alpha = cho_solve((self._L, True), Y, check_finite=False)
                if not np.all(np.isfinite(self._alpha)):
                    raise ValueError('Nonfinite GP coefficients')
        except (FloatingPointError, np.linalg.LinAlgError) as error:
            raise ValueError('GP covariance factorization failed') from error

    def kernel(self, left, right):
        """Squared-exponential kernel on normalized feature rows."""
        distances = cdist(left / self.length_scale, right / self.length_scale, 'sqeuclidean')
        return self.signal_variance * np.exp(-0.5 * distances)

    def posterior(self, point, *, variance=True):
        """Return acceleration mean and latent variance at a deterministic query.

        Variance excludes new observation noise and is conditional on this fitted
        kernel, finite-difference labels and exact input; it is not trajectory risk.
        """
        point = np.asarray(point, dtype=float)
        if point.shape != (self.X.shape[1],) or not np.all(np.isfinite(point)):
            raise ValueError('GP query must match finite training features')
        try:
            with np.errstate(over='raise', invalid='raise', divide='raise'):
                k = self.kernel(self.X, point[None, :])[:, 0]
                mean = k @ self._alpha
                result = None
                if variance:
                    w = solve_triangular(self._L, k, lower=True, check_finite=False)
                    value = self.signal_variance - w @ w
                    tolerance = 64 * np.finfo(float).eps * self.signal_variance * len(k)
                    if value < -tolerance or not np.isfinite(value):
                        raise ValueError('Invalid GP posterior variance')
                    result = np.full(self._alpha.shape[1], max(0.0, value))
                if not np.all(np.isfinite(mean)):
                    raise ValueError('Nonfinite GP posterior mean')
                return mean, result
        except FloatingPointError as error:
            raise ValueError('GP posterior arithmetic failed') from error


class GaussianProcessPredictor(Predictor):
    """Learn completed consecutive estimate transitions; recursively integrate means."""

    def __init__(
        self,
        *,
        kernel,
        length_scale,
        signal_variance,
        observation_noise,
        jitter,
        history_capacity,
        min_training_samples,
        refit_interval,
        normalize_features,
        minimum_scale,
        max_condition,
        max_observation_gap,
        max_state_dimension,
        max_horizon,
    ):
        """Validate the bounded online learning configuration."""
        if kernel != 'rbf' or type(normalize_features) is not bool:
            raise ValueError('GP requires RBF and boolean feature normalization')
        for name, value, lower, upper in [
            ('history_capacity', history_capacity, 2, 256),
            ('min_training_samples', min_training_samples, 2, history_capacity),
            ('refit_interval', refit_interval, 1, 1000),
            ('max_state_dimension', max_state_dimension, 2, 128),
            ('max_horizon', max_horizon, 1, 1000),
        ]:
            if type(value) is not int or not lower <= value <= upper:
                raise ValueError(f'{name} must be an integer in [{lower}, {upper}]')
        for value in (
            length_scale,
            signal_variance,
            observation_noise,
            jitter,
            minimum_scale,
            max_condition,
            max_observation_gap,
        ):
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError('GP scales, variances and numerical bounds must be positive')
        if max_condition < 1:
            raise ValueError('GP max_condition must be >= 1')
        self._gp_parameters = (
            length_scale,
            signal_variance,
            observation_noise,
            jitter,
            max_condition,
        )
        self._capacity, self._minimum, self._refit = (
            history_capacity,
            min_training_samples,
            refit_interval,
        )
        self._normalize, self._scale_floor = normalize_features, minimum_scale
        self._max_gap, self._max_dimension, self._max_horizon = (
            max_observation_gap,
            max_state_dimension,
            max_horizon,
        )
        self._cv = ConstantVelocityPredictor()
        self.reset()

    def reset(self):
        """Discard all Run-local observations, normalizers and fits."""
        self._history = deque(maxlen=self._capacity)
        self._last_state = self._last_time = self._training_end = None
        self._gp = self._center = self._scale = None
        self._since_fit = self._fits = self._fit_failures = self._prediction_failures = 0
        self._status, self._fit_error = 'cv_warmup', None
        self._fit_seconds = self._update_seconds = self._predict_seconds = 0.0

    def _state(self, state):
        """Validate the bounded online learning configuration."""
        if np.iscomplexobj(state):
            raise ValueError('GP state must be real')
        state = np.array(state, dtype=float, copy=True)
        if state.ndim != 1 or state.size > self._max_dimension or not np.all(np.isfinite(state)):
            raise ValueError('GP state must be finite and within dimension budget')
        StateLayout.from_state_dimension(state.size)
        return state

    def update_at(self, observed_state, observation_dt, observation_timestamp):
        """Commit (x_previous, delta_velocity / actual_dt) only after arrival."""
        started = perf_counter()
        try:
            state = self._state(observed_state)
            timestamp, dt = float(observation_timestamp), float(observation_dt)
            if not np.isfinite(timestamp) or not np.isfinite(dt) or dt <= 0:
                raise ValueError('GP observation time/dt must advance finitely')
            if self._last_time is not None:
                elapsed = timestamp - self._last_time
                if (
                    elapsed <= 0
                    or not np.isclose(elapsed, dt, rtol=1e-8, atol=1e-10)
                    or state.shape != self._last_state.shape
                ):
                    raise ValueError('Invalid GP observation sequence; reset')
                if elapsed > self._max_gap:
                    self.reset()
                else:
                    dimension = state.size // 2
                    with np.errstate(over='raise', invalid='raise', divide='raise'):
                        acceleration = (state[dimension:] - self._last_state[dimension:]) / dt
                    if not np.all(np.isfinite(acceleration)):
                        raise ValueError('Nonfinite GP training acceleration')
                    self._history.append((self._last_state.copy(), acceleration))
                    self._training_end = timestamp
                    self._since_fit += 1
                    if len(self._history) >= self._minimum and (
                        self._status == 'cv_warmup' or self._since_fit >= self._refit
                    ):
                        self._fit()
            self._last_state, self._last_time = state, timestamp
        except (ValueError, TypeError, FloatingPointError) as error:
            self.reset()
            raise ValueError('Invalid GP observation; history reset') from error
        self._update_seconds = perf_counter() - started

    def _fit(self):
        """Validate the bounded online learning configuration."""
        started = perf_counter()
        try:
            X, Y = [np.asarray(values) for values in zip(*self._history)]
            with np.errstate(over='raise', invalid='raise', divide='raise'):
                center = np.mean(X, axis=0) if self._normalize else np.zeros(X.shape[1])
                scale = (
                    np.maximum(np.std(X, axis=0), self._scale_floor)
                    if self._normalize
                    else np.ones(X.shape[1])
                )
                gp = RBFRegression((X - center) / scale, Y, *self._gp_parameters)
            self._gp, self._center, self._scale = gp, center, scale
            self._fits += 1
            self._status, self._fit_error = 'trained', None
        except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
            self._gp = self._center = self._scale = None
            self._fit_failures += 1
            self._status, self._fit_error = 'cv_conditioning', str(error)
        self._since_fit = 0
        self._fit_seconds = perf_counter() - started

    def acceleration_posterior(self, state):
        """Expose latent acceleration mean/variance, never trajectory covariance."""
        state = self._state(state)
        if self._gp is None:
            raise ValueError('No fitted GP acceleration posterior')
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            return self._gp.posterior((state - self._center) / self._scale)

    def predict(self, prediction_input: PredictionInput) -> TrajectoryPrediction:
        """Roll out frozen posterior means; explicitly tag CV warm-up/rejections."""
        started = perf_counter()
        state = self._state(prediction_input.state)
        t, dt, horizon = prediction_input.timestamp, prediction_input.dt, prediction_input.horizon
        if (
            not np.isfinite(t)
            or not np.isfinite(dt)
            or dt <= 0
            or isinstance(horizon, bool)
            or not isinstance(horizon, Integral)
            or not 1 <= horizon <= self._max_horizon
        ):
            raise ValueError('GP prediction grid exceeds finite work budget')
        with np.errstate(over='ignore', invalid='ignore'):
            times = t + dt * np.arange(1, horizon + 1, dtype=float)
        if not np.all(np.isfinite(times)) or times[0] <= t or np.any(np.diff(times) <= 0):
            raise ValueError('GP forecast timestamps must advance finitely')
        if self._last_time is None:
            self._last_state, self._last_time = state.copy(), float(t)
        if t < self._last_time - 1e-10:
            self.reset()
            raise ValueError('GP forecast precedes training observations; reset')
        if not np.isclose(t, self._last_time, rtol=0.0, atol=1e-10) or not np.array_equal(
            state, self._last_state
        ):
            raise ValueError('GP predict requires matching timestamped observation update')
        result = None
        if self._gp is not None:
            try:
                states = []
                d = state.size // 2
                with np.errstate(over='raise', invalid='raise', divide='raise'):
                    for _ in range(horizon):
                        a, _ = self._gp.posterior(
                            (state - self._center) / self._scale, variance=False
                        )
                        state = np.r_[
                            state[:d] + dt * state[d:] + 0.5 * dt**2 * a, state[d:] + dt * a
                        ]
                        if not np.all(np.isfinite(state)):
                            raise ValueError('Nonfinite recursive GP forecast')
                        states.append(state)
                result = TrajectoryPrediction(
                    np.asarray(states), times, model_name='gaussian_process'
                )
            except (ValueError, FloatingPointError, OverflowError):
                self._prediction_failures += 1
                self._status = 'cv_numerical'
                self._gp = None
        if result is None:
            try:
                with np.errstate(over='raise', invalid='raise'):
                    cv = self._cv.predict(prediction_input)
                if not np.all(np.isfinite(cv.states)):
                    raise ValueError('Nonfinite GP CV fallback')
                result = TrajectoryPrediction(
                    cv.states, times, model_name='gaussian_process/' + self._status
                )
            except FloatingPointError as error:
                raise ValueError('GP CV fallback overflow') from error
        self._predict_seconds = perf_counter() - started
        return result

    @property
    def diagnostics(self):
        """Detached bounded-learning counters and wall timings (seconds)."""
        return {
            'status': self._status,
            'training_pairs': len(self._history),
            'fit_count': self._fits,
            'fit_failures': self._fit_failures,
            'prediction_failures': self._prediction_failures,
            'condition': None if self._gp is None else self._gp.condition,
            'last_observation_time': self._last_time,
            'last_training_end': self._training_end,
            'fit_error': self._fit_error,
            'fit_seconds': self._fit_seconds,
            'update_seconds': self._update_seconds,
            'predict_seconds': self._predict_seconds,
        }


_PROPERTIES = {
    'kernel': {'type': 'string', 'enum': ['rbf'], 'unit': '1'},
    'normalize_features': {'type': 'boolean', 'unit': '1'},
}
for _name, _unit in [
    ('length_scale', 'normalized state units'),
    ('signal_variance', 'm^2/s^4'),
    ('observation_noise', 'm^2/s^4'),
    ('jitter', 'm^2/s^4'),
    ('minimum_scale', 'state coordinate units'),
    ('max_observation_gap', 's'),
]:
    _PROPERTIES[_name] = {'type': 'number', 'exclusiveMinimum': 0.0, 'unit': _unit}
_PROPERTIES['max_condition'] = {'type': 'number', 'minimum': 1.0, 'unit': '1'}
for _name, _min, _max, _unit in [
    ('history_capacity', 2, 256, 'transition pairs'),
    ('min_training_samples', 2, 256, 'transition pairs'),
    ('refit_interval', 1, 1000, 'new transition pairs'),
    ('max_state_dimension', 2, 128, 'state coordinates'),
    ('max_horizon', 1, 1000, 'future steps'),
]:
    _PROPERTIES[_name] = {'type': 'integer', 'minimum': _min, 'maximum': _max, 'unit': _unit}

COMPONENTS = (
    ModelRegistration(
        'gaussian_process',
        GaussianProcessPredictor,
        {
            'type': 'object',
            'properties': _PROPERTIES,
            'required': list(_PROPERTIES),
            'additionalProperties': False,
        },
        requirements={'state_layout': 'position_velocity'},
        description='Gaussian Process',
        capabilities={
            'deployable': True,
            'runtime_validation': 'causal_online_gp',
            'display_name': 'Gaussian Process',
            'input_layout': 'position_velocity',
            'output_layout': 'position_velocity',
            'coordinate_frames': 'same fixed frame as input',
            'uncertainty': 'latent acceleration posterior only; no trajectory covariance',
            'learning': 'bounded causal timestamped updates',
        },
    ),
)
