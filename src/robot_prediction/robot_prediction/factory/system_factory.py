"""Assemble registered predictor, adaptation and fusion contracts."""

import math

from robot_prediction.evaluation.performance_monitor import PerformanceMonitor
from robot_prediction.factory.component_registry import create_prediction_registries
from robot_prediction.interfaces.correction import CorrectionFusion
from robot_prediction.interfaces.correction import ResidualCorrection
from robot_prediction.interfaces.predictor import Predictor
from robot_prediction.trajectory_prediction import TrajectoryPredictionSystem


class PredictionSystemFactory:
    """Use owner registries; no per-algorithm branch belongs in orchestration."""

    def __init__(self, config, registries=None):
        self._config = config
        self._registries = registries or create_prediction_registries(config)

    def create(
        self, *, predictor_type, adaptation_type, fusion_type, parameters=None, context=None
    ):
        """Construct actual compatible components, preserving monitor/update semantics."""
        settings = self._config.get('prediction')
        context = {
            'state_layout': 'position_velocity',
            'spatial_dimension': 3,
            'deployment': 'trajectory_system',
            **(context or {}),
        }
        parameters = parameters or {}
        if set(parameters) - set(self._registries):
            raise ValueError('Unknown prediction component parameter family')
        selections = {
            'predictor': predictor_type,
            'adaptation': adaptation_type,
            'fusion': fusion_type,
        }
        built = {}
        for family in ['predictor', 'adaptation', 'fusion']:
            registry = self._registries[family]
            name = selections[family]
            try:
                built[family] = registry.create(name, parameters.get(family), context=context)
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f'prediction.{family}={name!r}: {error}') from error
            if family == 'adaptation':
                metadata = registry.describe(name)['capabilities']
                context['correction_representation'] = metadata.get('correction_representation')
        if not isinstance(built['predictor'], Predictor):
            raise TypeError('Registered predictor must implement Predictor')
        for family, contract in [('adaptation', ResidualCorrection), ('fusion', CorrectionFusion)]:
            if built[family] is not None and not isinstance(built[family], contract):
                raise TypeError(f'Registered {family} does not implement {contract.__name__}')
        if (built['adaptation'] is None) != (built['fusion'] is None):
            raise ValueError(
                'prediction.fusion: adaptation and fusion must agree on correction presence'
            )
        monitor = PerformanceMonitor(
            buffer_capacity=max(
                1, math.ceil(self._config.get('history')['window_duration'] / settings['dt'])
            ),
            confidence_scale=self._config.get('confidence')['residual_scale'],
        )
        return TrajectoryPredictionSystem(
            predictor=built['predictor'],
            performance_monitor=monitor,
            residual_adapter=built['adaptation'],
            fusion=built['fusion'],
        )
