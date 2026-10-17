import math

import numpy as np

from robot_prediction.baseline.imm import IMMPredictor
from robot_prediction.baseline.constant_velocity import ConstantVelocityPredictor
from robot_prediction.evaluation.performance_monitor import (
    PerformanceMonitor,
)
from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.online_adaptation.classical.residual_adapter import (
    ResidualAdapter,
)
from robot_prediction.trajectory_prediction import (
    TrajectoryPredictionSystem,
)


class PredictionSystemFactory:
    """Create configured trajectory-prediction systems."""

    def __init__(
        self,
        config: PredictionConfig,
    ):
        self._config = config

    def create(
        self,
        predictor_type=None,
        adaptation_type=None,
        fusion_type=None,
    ) -> TrajectoryPredictionSystem:
        """Create a trajectory-prediction system from configuration."""
        prediction_config = self._config.get(
            'prediction'
        )

        history_config = self._config.get(
            'history'
        )

        confidence_config = self._config.get(
            'confidence'
        )

        adaptation_config = self._config.get(
            'adaptation'
        )
        predictor_type = prediction_config.get('type', 'imm') if predictor_type is None else predictor_type
        adaptation_type = adaptation_config.get('type', 'residual') if adaptation_type is None else adaptation_type
        fusion_type = self._config.get('fusion').get('type', 'weighted_correction') if fusion_type is None else fusion_type
        if predictor_type not in {'imm', 'constant_velocity'}:
            raise ValueError(
                f'prediction.type={predictor_type!r}: supported predictors: imm, constant_velocity; '
                'CA/CT require acceleration/turn-rate input unavailable in the deployed [p,v] contract')
        if adaptation_type not in {'residual', 'none'}:
            raise ValueError(f'prediction.adaptation={adaptation_type!r}: supported: residual, none')
        expected_fusion = 'weighted_correction' if adaptation_type == 'residual' else 'none'
        if fusion_type != expected_fusion:
            raise ValueError(f'prediction.fusion={fusion_type!r}: {adaptation_type} adaptation requires {expected_fusion}')

        dt = float(
            prediction_config['dt']
        )
        horizon = prediction_config['horizon']
        if not np.isfinite(dt) or dt <= 0 or type(horizon) is not int or horizon <= 0:
            raise ValueError('prediction.dt/horizon must be finite positive seconds / positive integer')

        window_duration = float(
            history_config['window_duration']
        )

        buffer_capacity = max(
            1,
            math.ceil(
                window_duration / dt
            ),
        )

        predictors = {
            'imm': lambda: IMMPredictor(
                turn_rate_epsilon=float(self._config.get('coordinated_turn')['turn_rate_epsilon']),
                likelihood_variance=float(self._config.get('imm')['likelihood_variance']),
                model_probabilities=np.asarray(self._config.get('imm')['model_probabilities'], dtype=float),
                transition_matrix=np.asarray(self._config.get('imm')['transition_matrix'], dtype=float)),
            'constant_velocity': ConstantVelocityPredictor,
        }
        predictor = predictors[predictor_type]()

        performance_monitor = PerformanceMonitor(
            buffer_capacity=buffer_capacity,
            confidence_scale=float(
                confidence_config[
                    'residual_scale'
                ]
            ),
        )

        residual_adapter = (ResidualAdapter(
            correction_gain=float(adaptation_config['residual']['correction_gain']))
            if adaptation_type == 'residual' else None)

        return TrajectoryPredictionSystem(
            predictor=predictor,
            performance_monitor=performance_monitor,
            residual_adapter=residual_adapter,
        )
