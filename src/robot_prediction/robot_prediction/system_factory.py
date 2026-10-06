import math

import numpy as np

from robot_prediction.baseline.imm import IMMPredictor
from robot_prediction.confidence.performance_monitor import (
    PerformanceMonitor,
)
from robot_prediction.config_loader import PredictionConfig
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

        imm_config = self._config.get(
            'imm'
        )

        coordinated_turn_config = self._config.get(
            'coordinated_turn'
        )

        adaptation_config = self._config.get(
            'adaptation'
        )

        dt = float(
            prediction_config['dt']
        )

        window_duration = float(
            history_config['window_duration']
        )

        buffer_capacity = max(
            1,
            math.ceil(
                window_duration / dt
            ),
        )

        predictor = IMMPredictor(
            turn_rate_epsilon=float(
                coordinated_turn_config[
                    'turn_rate_epsilon'
                ]
            ),
            likelihood_variance=float(
                imm_config[
                    'likelihood_variance'
                ]
            ),
            model_probabilities=np.asarray(
                imm_config[
                    'model_probabilities'
                ],
                dtype=float,
            ),
            transition_matrix=np.asarray(
                imm_config[
                    'transition_matrix'
                ],
                dtype=float,
            ),
        )

        performance_monitor = PerformanceMonitor(
            buffer_capacity=buffer_capacity,
            confidence_scale=float(
                confidence_config[
                    'residual_scale'
                ]
            ),
        )

        residual_config = adaptation_config[
            'residual'
        ]

        residual_adapter = ResidualAdapter(
            correction_gain=float(
                residual_config[
                    'correction_gain'
                ]
            )
        )

        return TrajectoryPredictionSystem(
            predictor=predictor,
            performance_monitor=performance_monitor,
            residual_adapter=residual_adapter,
        )
