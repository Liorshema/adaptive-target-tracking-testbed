import numpy as np

from robot_prediction.baseline.constant_acceleration import (
    ConstantAccelerationPredictor,
)
from robot_prediction.baseline.constant_velocity import (
    ConstantVelocityPredictor,
)
from robot_prediction.baseline.coordinated_turn import (
    CoordinatedTurnPredictor,
)
from robot_prediction.interfaces.prediction_data import (
    PredictionInput,
    TrajectoryPrediction,
)
from robot_prediction.interfaces.predictor import Predictor
from robot_prediction.interfaces.state_layout import StateLayout
from robot_prediction.residuals.prediction_residual import (
    compute_residual,
)


class IMMPredictor(Predictor):
    """Fuse multiple motion models with adaptive model probabilities."""

    MODEL_NAMES = (
        'cv',
        'ca',
        'ct',
    )

    def __init__(
        self,
        turn_rate_epsilon: float,
        likelihood_variance: float,
        model_probabilities: np.ndarray | None = None,
        transition_matrix: np.ndarray | None = None,
    ):
        if turn_rate_epsilon <= 0.0:
            raise ValueError(
                'turn_rate_epsilon must be positive.'
            )

        if likelihood_variance <= 0.0:
            raise ValueError(
                'likelihood_variance must be positive.'
            )

        self._turn_rate_epsilon = (
            turn_rate_epsilon
        )

        self._likelihood_variance = (
            likelihood_variance
        )

        self._predictors = {
            'cv': ConstantVelocityPredictor(),
            'ca': ConstantAccelerationPredictor(),
            'ct': CoordinatedTurnPredictor(
                turn_rate_epsilon=turn_rate_epsilon
            ),
        }

        model_count = len(
            self.MODEL_NAMES
        )

        if model_probabilities is None:
            model_probabilities = np.full(
                model_count,
                1.0 / model_count,
            )

        self._model_probabilities = (
            self._validate_probabilities(
                model_probabilities
            )
        )

        if transition_matrix is None:
            transition_matrix = (
                self._default_transition_matrix(
                    model_count=model_count
                )
            )

        self._transition_matrix = (
            self._validate_transition_matrix(
                transition_matrix
            )
        )

        self._predicted_probabilities = None
        self._last_model_predictions = None
        self._last_likelihoods = None
        self._last_residuals = None

        self._layout = None
        self._last_dt = None

        self._estimated_acceleration = None
        self._estimated_turn_rate = 0.0

        self._previous_observation = None

    def predict(
        self,
        prediction_input: PredictionInput,
    ) -> TrajectoryPrediction:
        """Predict a fused future trajectory."""
        state = np.asarray(
            prediction_input.state,
            dtype=float,
        )

        self._validate_prediction_input(
            prediction_input=prediction_input,
            state=state,
        )

        layout = (
            StateLayout.from_state_dimension(
                state.size
            )
        )

        self._initialize_motion_estimates(
            spatial_dimension=layout.spatial_dimension
        )

        self._layout = layout
        self._last_dt = prediction_input.dt

        self._predicted_probabilities = (
            self._transition_matrix.T
            @ self._model_probabilities
        )

        predictions = []

        for model_name in self.MODEL_NAMES:
            model_state = self._to_model_state(
                model_name=model_name,
                common_state=state,
                layout=layout,
            )

            model_input = PredictionInput(
                state=model_state,
                timestamp=prediction_input.timestamp,
                dt=prediction_input.dt,
                horizon=prediction_input.horizon,
                covariance=prediction_input.covariance,
            )

            prediction = self._predictors[
                model_name
            ].predict(
                model_input
            )

            predictions.append(
                prediction
            )

        self._validate_model_outputs(
            predictions=predictions,
            expected_dimension=layout.state_dimension,
        )

        self._last_model_predictions = (
            predictions
        )

        stacked_predictions = np.stack(
            [
                prediction.states
                for prediction in predictions
            ],
            axis=0,
        )

        fused_states = np.tensordot(
            self._predicted_probabilities,
            stacked_predictions,
            axes=(0, 0),
        )

        return TrajectoryPrediction(
            states=fused_states,
            timestamps=predictions[0].timestamps.copy(),
            model_name='imm',
        )

    def update(
        self,
        observed_state: np.ndarray,
    ) -> np.ndarray:
        """Update model probabilities and motion estimates."""
        if self._last_model_predictions is None:
            raise RuntimeError(
                'predict() must be called before update().'
            )

        if self._predicted_probabilities is None:
            raise RuntimeError(
                'predicted probabilities are unavailable.'
            )

        if self._layout is None:
            raise RuntimeError(
                'state layout is unavailable.'
            )

        observation = self._validate_observation(
            observed_state
        )

        residuals = []
        likelihoods = []

        for prediction in self._last_model_predictions:
            predicted_state = (
                prediction.states[0]
            )

            residual = compute_residual(
                observed_state=observation,
                predicted_state=predicted_state,
            )

            likelihood = (
                self._gaussian_likelihood(
                    residual
                )
            )

            residuals.append(
                residual
            )

            likelihoods.append(
                likelihood
            )

        likelihoods = np.asarray(
            likelihoods,
            dtype=float,
        )

        weighted_likelihoods = (
            self._predicted_probabilities
            * likelihoods
        )

        normalization = np.sum(
            weighted_likelihoods
        )

        if normalization <= np.finfo(float).eps:
            posterior_probabilities = (
                self._predicted_probabilities.copy()
            )

        else:
            posterior_probabilities = (
                weighted_likelihoods
                / normalization
            )

        self._model_probabilities = (
            posterior_probabilities
        )

        self._last_likelihoods = (
            likelihoods
        )

        self._last_residuals = np.stack(
            residuals,
            axis=0,
        )

        self._update_motion_estimates(
            observation
        )

        self._previous_observation = (
            observation.copy()
        )

        return self._model_probabilities.copy()

    def _update_motion_estimates(
        self,
        observation: np.ndarray,
    ) -> None:
        """Estimate acceleration and planar turn rate from state history."""
        if self._previous_observation is None:
            return

        if self._last_dt is None:
            return

        if self._last_dt <= 0.0:
            return

        layout = self._layout

        current_velocity = observation[
            layout.velocity_slice()
        ]

        previous_velocity = self._previous_observation[
            layout.velocity_slice()
        ]

        self._estimated_acceleration = (
            current_velocity
            - previous_velocity
        ) / self._last_dt

        if layout.spatial_dimension < 2:
            self._estimated_turn_rate = 0.0
            return

        current_planar_velocity = (
            current_velocity[:2]
        )

        previous_planar_velocity = (
            previous_velocity[:2]
        )

        current_speed = np.linalg.norm(
            current_planar_velocity
        )

        previous_speed = np.linalg.norm(
            previous_planar_velocity
        )

        if (
            current_speed < self._turn_rate_epsilon
            or previous_speed < self._turn_rate_epsilon
        ):
            self._estimated_turn_rate = 0.0
            return

        current_heading = np.arctan2(
            current_planar_velocity[1],
            current_planar_velocity[0],
        )

        previous_heading = np.arctan2(
            previous_planar_velocity[1],
            previous_planar_velocity[0],
        )

        heading_difference = (
            current_heading
            - previous_heading
        )

        wrapped_difference = np.arctan2(
            np.sin(
                heading_difference
            ),
            np.cos(
                heading_difference
            ),
        )

        self._estimated_turn_rate = (
            wrapped_difference
            / self._last_dt
        )

    def _to_model_state(
        self,
        model_name: str,
        common_state: np.ndarray,
        layout: StateLayout,
    ) -> np.ndarray:
        """Convert common [position, velocity] state to model state."""
        if model_name == 'cv':
            return common_state.copy()

        if model_name == 'ca':
            return np.concatenate((
                common_state,
                self._estimated_acceleration,
            ))

        if model_name == 'ct':
            return np.concatenate((
                common_state,
                np.array(
                    [
                        self._estimated_turn_rate,
                    ],
                    dtype=float,
                ),
            ))

        raise ValueError(
            f'unknown model: {model_name}'
        )

    def _initialize_motion_estimates(
        self,
        spatial_dimension: int,
    ) -> None:
        """Initialize inferred motion quantities when needed."""
        if self._estimated_acceleration is None:
            self._estimated_acceleration = np.zeros(
                spatial_dimension,
                dtype=float,
            )

            return

        if self._estimated_acceleration.shape != (
            spatial_dimension,
        ):
            raise ValueError(
                'spatial dimension changed during IMM execution.'
            )

    def _gaussian_likelihood(
        self,
        residual: np.ndarray,
    ) -> float:
        """Compute isotropic Gaussian likelihood of a residual."""
        dimension = residual.size

        inverse_covariance = (
            np.eye(
                dimension,
                dtype=float,
            )
            / self._likelihood_variance
        )

        mahalanobis_squared = (
            residual.T
            @ inverse_covariance
            @ residual
        )

        determinant = (
            self._likelihood_variance
            ** dimension
        )

        normalization = np.sqrt(
            (2.0 * np.pi) ** dimension
            * determinant
        )

        likelihood = (
            np.exp(
                -0.5
                * mahalanobis_squared
            )
            / normalization
        )

        return float(
            likelihood
        )

    def _validate_observation(
        self,
        observed_state: np.ndarray,
    ) -> np.ndarray:
        """Validate a common observed [position, velocity] state."""
        observation = np.asarray(
            observed_state,
            dtype=float,
        )

        if observation.ndim != 1:
            raise ValueError(
                'observed_state must be one-dimensional.'
            )

        if self._layout is None:
            raise RuntimeError(
                'state layout is unavailable.'
            )

        if observation.shape != (
            self._layout.state_dimension,
        ):
            raise ValueError(
                'observed_state has incompatible dimension.'
            )

        return observation

    @staticmethod
    def _validate_prediction_input(
        prediction_input: PredictionInput,
        state: np.ndarray,
    ) -> None:
        """Validate common IMM prediction input."""
        if state.ndim != 1:
            raise ValueError(
                'IMM state must be one-dimensional.'
            )

        if state.size == 0:
            raise ValueError(
                'IMM state must not be empty.'
            )

        if state.size % 2 != 0:
            raise ValueError(
                'IMM state must contain '
                '[position, velocity].'
            )

        if prediction_input.dt <= 0.0:
            raise ValueError(
                'dt must be positive.'
            )

        if prediction_input.horizon <= 0:
            raise ValueError(
                'horizon must be positive.'
            )

    @staticmethod
    def _validate_model_outputs(
        predictions: list[TrajectoryPrediction],
        expected_dimension: int,
    ) -> None:
        """Validate that all motion models share a common output."""
        reference_shape = (
            predictions[0].states.shape
        )

        reference_timestamps = (
            predictions[0].timestamps
        )

        if reference_shape[1] != expected_dimension:
            raise ValueError(
                'predictor output has incompatible state dimension.'
            )

        for prediction in predictions[1:]:
            if prediction.states.shape != reference_shape:
                raise ValueError(
                    'all IMM model predictions must '
                    'have the same shape.'
                )

            if not np.allclose(
                prediction.timestamps,
                reference_timestamps,
            ):
                raise ValueError(
                    'all IMM model predictions must '
                    'share timestamps.'
                )

    @classmethod
    def _validate_probabilities(
        cls,
        probabilities: np.ndarray,
    ) -> np.ndarray:
        """Validate model probabilities."""
        probabilities = np.asarray(
            probabilities,
            dtype=float,
        )

        model_count = len(
            cls.MODEL_NAMES
        )

        if probabilities.shape != (
            model_count,
        ):
            raise ValueError(
                'model_probabilities have '
                'incompatible dimension.'
            )

        if np.any(
            probabilities < 0.0
        ):
            raise ValueError(
                'model probabilities must be non-negative.'
            )

        probability_sum = np.sum(
            probabilities
        )

        if probability_sum <= 0.0:
            raise ValueError(
                'model probabilities must have positive sum.'
            )

        return (
            probabilities
            / probability_sum
        )

    @classmethod
    def _validate_transition_matrix(
        cls,
        transition_matrix: np.ndarray,
    ) -> np.ndarray:
        """Validate model transition probabilities."""
        matrix = np.asarray(
            transition_matrix,
            dtype=float,
        )

        model_count = len(
            cls.MODEL_NAMES
        )

        expected_shape = (
            model_count,
            model_count,
        )

        if matrix.shape != expected_shape:
            raise ValueError(
                'transition_matrix has incompatible dimension.'
            )

        if np.any(
            matrix < 0.0
        ):
            raise ValueError(
                'transition probabilities must be non-negative.'
            )

        row_sums = np.sum(
            matrix,
            axis=1,
        )

        if not np.allclose(
            row_sums,
            1.0,
        ):
            raise ValueError(
                'each transition-matrix row must sum to one.'
            )

        return matrix

    @staticmethod
    def _default_transition_matrix(
        model_count: int,
    ) -> np.ndarray:
        """Create a neutral model-persistence transition matrix."""
        if model_count <= 0:
            raise ValueError(
                'model_count must be positive.'
            )

        if model_count == 1:
            return np.ones(
                (1, 1),
                dtype=float,
            )

        stay_probability = 0.90

        switch_probability = (
            1.0 - stay_probability
        ) / (
            model_count - 1
        )

        matrix = np.full(
            (
                model_count,
                model_count,
            ),
            switch_probability,
            dtype=float,
        )

        np.fill_diagonal(
            matrix,
            stay_probability,
        )

        return matrix

    @property
    def model_probabilities(
        self,
    ) -> np.ndarray:
        """Return posterior model probabilities."""
        return self._model_probabilities.copy()

    @property
    def predicted_probabilities(
        self,
    ) -> np.ndarray | None:
        """Return model probabilities before observation update."""
        if self._predicted_probabilities is None:
            return None

        return self._predicted_probabilities.copy()

    @property
    def last_likelihoods(
        self,
    ) -> np.ndarray | None:
        """Return the latest model likelihoods."""
        if self._last_likelihoods is None:
            return None

        return self._last_likelihoods.copy()

    @property
    def last_residuals(
        self,
    ) -> np.ndarray | None:
        """Return the latest model residuals."""
        if self._last_residuals is None:
            return None

        return self._last_residuals.copy()

    @property
    def estimated_acceleration(
        self,
    ) -> np.ndarray | None:
        """Return the acceleration inferred from observation history."""
        if self._estimated_acceleration is None:
            return None

        return self._estimated_acceleration.copy()

    @property
    def estimated_turn_rate(
        self,
    ) -> float:
        """Return the planar turn rate inferred from observation history."""
        return float(
            self._estimated_turn_rate
        )
