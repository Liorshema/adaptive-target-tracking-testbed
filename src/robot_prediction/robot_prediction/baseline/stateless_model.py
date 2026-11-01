"""
Optional mathematical API layered on the existing Predictor contract.

``@predictor_model(id, parameters_schema=...)`` produces a ModelRegistration,
not an import-time registry mutation. Export declarations in a trusted owner
module's ``COMPONENTS`` tuple and put defaults in prediction.yaml/predictors.

The callback is ``function(state, relative_horizons, parameters)``. State is
``[position, velocity]`` (metres, metres/second); offsets are seconds and equal
``dt * [1, ..., horizon]``. Return ``(horizon, 2 * spatial_dimension)`` states,
or a genuine TrajectoryPrediction on the exact absolute input timestamp grid.
A mapping with states and optional covariance/confidence lets the adapter own
the absolute grid while the function supplies genuine uncertainty.
No interpolation or resampling occurs. Output covariance, when provided, has
shape ``(horizon, state_dimension, state_dimension)``; confidence has shape
``(horizon,)`` and lies in [0, 1]. Missing uncertainty remains absent.

The existing PredictionInput and TrajectoryPrediction have no frame identifiers.
All states must use the same fixed input frame; this adapter cannot transform
frames or validate a frame identifier. Full Predictor implementations remain
the interface for history, learning, multimodal or other advanced algorithms.
"""

from collections.abc import Mapping
from copy import deepcopy
from numbers import Integral, Real
from types import MappingProxyType

import numpy as np

from robot_configuration.components import validate_schema
from robot_configuration.model_registration import ModelRegistration
from robot_prediction.interfaces.prediction_data import PredictionInput, TrajectoryPrediction
from robot_prediction.interfaces.predictor import Predictor


def _readonly(value):
    if isinstance(value, dict):
        return MappingProxyType({key: _readonly(item) for key, item in value.items()})
    if isinstance(value, (tuple, list)):
        return tuple(_readonly(item) for item in value)
    if isinstance(value, np.ndarray):
        result = value.copy()
        result.setflags(write=False)
        return result
    return value


def _finite_array(value, shape, label):
    if np.iscomplexobj(value):
        raise ValueError(f'{label}: expected real array')
    result = np.array(value, dtype=float, copy=True)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f'{label}: expected finite array with shape {shape}')
    return result


def _covariance(value, shape, label):
    result = _finite_array(value, shape, label)
    if not np.allclose(result, np.swapaxes(result, -1, -2), rtol=1.0e-10, atol=1.0e-12):
        raise ValueError(f'{label}: covariance must be symmetric')
    tolerance = 1.0e-12 * max(1.0, float(np.max(np.abs(result))))
    if np.any(np.linalg.eigvalsh(result) < -tolerance):
        raise ValueError(f'{label}: covariance must be positive semidefinite')
    return result


class StatelessPredictor(Predictor):
    """Execute one validated, stateless position/velocity prediction function."""

    def __init__(self, function, identifier, spatial_dimension, parameters_schema, parameters):
        self._function = function
        self._identifier = identifier
        self._dimension = 2 * spatial_dimension
        self._parameters = _readonly(validate_schema(parameters, parameters_schema, identifier))

    def predict(self, prediction_input: PredictionInput) -> TrajectoryPrediction:
        """Validate the exact future grid and package the mathematical result."""
        state = _finite_array(prediction_input.state, (self._dimension,), 'state')
        for label in ['timestamp', 'dt']:
            value = getattr(prediction_input, label)
            if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value):
                raise ValueError(f'{label}: expected finite number')
        if prediction_input.dt <= 0:
            raise ValueError('dt: expected positive number')
        horizon = prediction_input.horizon
        if isinstance(horizon, bool) or not isinstance(horizon, Integral) or horizon <= 0:
            raise ValueError('horizon: expected positive integer')
        if prediction_input.covariance is not None:
            _covariance(
                prediction_input.covariance, (self._dimension, self._dimension), 'input covariance'
            )
        with np.errstate(over='ignore', invalid='ignore'):
            offsets = prediction_input.dt * np.arange(1, horizon + 1, dtype=float)
            timestamps = prediction_input.timestamp + offsets
        if (
            not np.all(np.isfinite(timestamps))
            or timestamps[0] <= prediction_input.timestamp
            or np.any(np.diff(timestamps) <= 0)
        ):
            raise ValueError('timestamps: requested grid must be finite and advance in time')
        state.setflags(write=False)
        offsets.setflags(write=False)
        output = self._function(state, offsets, self._parameters)
        if isinstance(output, Mapping):
            unknown = set(output) - {'states', 'covariance', 'confidence'}
            if unknown or 'states' not in output:
                raise ValueError(f'result: unknown fields {sorted(unknown)} or missing states')
            output = TrajectoryPrediction(
                states=output['states'], timestamps=timestamps,
                covariance=output.get('covariance'), confidence=output.get('confidence'))
        covariance = confidence = None
        model_name = self._identifier
        if isinstance(output, TrajectoryPrediction):
            actual_times = _finite_array(output.timestamps, (horizon,), 'timestamps')
            if not np.array_equal(actual_times, timestamps):
                raise ValueError('timestamps: output must equal the requested absolute grid')
            if output.covariance is not None:
                covariance = _covariance(
                    output.covariance,
                    (horizon, self._dimension, self._dimension),
                    'output covariance',
                )
            if output.confidence is not None:
                confidence = _finite_array(output.confidence, (horizon,), 'confidence')
                if np.any((confidence < 0) | (confidence > 1)):
                    raise ValueError('confidence: values must lie in [0, 1]')
            model_name = output.model_name or model_name
            output = output.states
        states = _finite_array(output, (horizon, self._dimension), 'states')
        return TrajectoryPrediction(
            states=states,
            timestamps=timestamps,
            covariance=covariance,
            confidence=confidence,
            model_name=model_name,
        )


def predictor_model(identifier, *, parameters_schema, spatial_dimension=3):
    """
    Declare a stateless model without defaults or global registration state.

    Parameters are a detached, recursively read-only mapping. The callback never
    receives or propagates input uncertainty implicitly: only explicitly returned
    genuine output covariance/confidence is packaged.
    """
    if (
        isinstance(spatial_dimension, bool)
        or not isinstance(spatial_dimension, Integral)
        or spatial_dimension <= 0
    ):
        raise ValueError('spatial_dimension: expected positive integer')
    if (
        not isinstance(parameters_schema, dict)
        or parameters_schema.get('type') != 'object'
        or not isinstance(parameters_schema.get('properties'), dict)
        or parameters_schema.get('additionalProperties') is not False
    ):
        raise ValueError('parameters_schema: expected closed object schema')
    schema = deepcopy(parameters_schema)

    def decorate(function):
        if not callable(function):
            raise TypeError('predictor function must be callable')

        def build(**parameters):
            return StatelessPredictor(function, identifier, spatial_dimension, schema, parameters)

        return ModelRegistration(
            identifier=identifier,
            builder=build,
            schema=schema,
            requirements={
                'state_layout': 'position_velocity',
                'spatial_dimension': spatial_dimension,
            },
            capabilities={
                'deployable': True,
                'runtime_validation': 'unit_tested_extension',
                'input_layout': 'position_velocity',
                'output_layout': 'position_velocity',
                'spatial_dimension': spatial_dimension,
                'coordinate_frames': 'same fixed frame as input',
                'state_units': {'position': 'm', 'velocity': 'm/s'},
                'time_grid': 'relative dt*[1..horizon]; output timestamp+offsets',
                'uncertainty': 'absent unless explicitly returned',
            },
        )

    return decorate
