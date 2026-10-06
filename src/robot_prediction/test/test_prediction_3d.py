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
from robot_prediction.baseline.imm import IMMPredictor
from robot_prediction.interfaces.prediction_data import PredictionInput
from robot_prediction.interfaces.state_layout import StateLayout


def test_state_layout_infers_spatial_dimension():
    """Infer spatial dimension from common [position, velocity] state."""
    layout = StateLayout.from_state_dimension(
        6
    )

    assert layout.spatial_dimension == 3
    assert layout.state_dimension == 6
    assert layout.acceleration_state_dimension == 9


def test_constant_velocity_predicts_3d_motion():
    """Predict three-dimensional constant-velocity motion."""
    predictor = ConstantVelocityPredictor()

    prediction_input = PredictionInput(
        state=np.array([
            0.0, 0.0, 1.0,
            1.0, 2.0, -0.5,
        ]),
        timestamp=0.0,
        dt=1.0,
        horizon=2,
    )

    prediction = predictor.predict(
        prediction_input
    )

    expected = np.array([
        [
            1.0, 2.0, 0.5,
            1.0, 2.0, -0.5,
        ],
        [
            2.0, 4.0, 0.0,
            1.0, 2.0, -0.5,
        ],
    ])

    assert prediction.states.shape == (
        2,
        6,
    )

    assert np.allclose(
        prediction.states,
        expected,
    )


def test_constant_acceleration_predicts_3d_motion():
    """Predict three-dimensional constant-acceleration motion."""
    predictor = ConstantAccelerationPredictor()

    prediction_input = PredictionInput(
        state=np.array([
            0.0, 0.0, 1.0,
            1.0, 2.0, -0.5,
            0.2, 0.0, 0.1,
        ]),
        timestamp=0.0,
        dt=1.0,
        horizon=1,
    )

    prediction = predictor.predict(
        prediction_input
    )

    expected = np.array([
        [
            1.1, 2.0, 0.55,
            1.2, 2.0, -0.4,
        ]
    ])

    assert prediction.states.shape == (
        1,
        6,
    )

    assert np.allclose(
        prediction.states,
        expected,
    )


def test_coordinated_turn_preserves_vertical_velocity():
    """Turn in the horizontal plane while preserving vertical motion."""
    predictor = CoordinatedTurnPredictor(
        turn_rate_epsilon=1.0e-6
    )

    prediction_input = PredictionInput(
        state=np.array([
            0.0, 0.0, 1.0,
            1.0, 0.0, 0.5,
            0.5,
        ]),
        timestamp=0.0,
        dt=1.0,
        horizon=2,
    )

    prediction = predictor.predict(
        prediction_input
    )

    assert prediction.states.shape == (
        2,
        6,
    )

    assert np.allclose(
        prediction.states[:, 2],
        np.array([
            1.5,
            2.0,
        ]),
    )

    assert np.allclose(
        prediction.states[:, 5],
        np.array([
            0.5,
            0.5,
        ]),
    )


def test_imm_uses_common_3d_state():
    """Use common three-dimensional [position, velocity] input and output."""
    predictor = IMMPredictor(
        turn_rate_epsilon=1.0e-6,
        likelihood_variance=1.0,
    )

    prediction_input = PredictionInput(
        state=np.array([
            0.0, 0.0, 1.0,
            1.0, 0.0, 0.5,
        ]),
        timestamp=0.0,
        dt=0.1,
        horizon=5,
    )

    prediction = predictor.predict(
        prediction_input
    )

    assert prediction.states.shape == (
        5,
        6,
    )

    assert np.isclose(
        np.sum(
            predictor.predicted_probabilities
        ),
        1.0,
    )


def test_imm_estimates_acceleration_from_history():
    """Infer acceleration from consecutive common-state observations."""
    predictor = IMMPredictor(
        turn_rate_epsilon=1.0e-6,
        likelihood_variance=0.05,
    )

    dt = 0.1

    states = [
        np.array([
            0.00, 0.0, 0.0,
            1.00, 0.0, 0.0,
        ]),
        np.array([
            0.11, 0.0, 0.0,
            1.20, 0.0, 0.0,
        ]),
        np.array([
            0.24, 0.0, 0.0,
            1.40, 0.0, 0.0,
        ]),
    ]

    for index in range(
        len(states) - 1
    ):
        predictor.predict(
            PredictionInput(
                state=states[index],
                timestamp=index * dt,
                dt=dt,
                horizon=3,
            )
        )

        predictor.update(
            states[index + 1]
        )

    assert np.allclose(
        predictor.estimated_acceleration,
        np.array([
            2.0,
            0.0,
            0.0,
        ]),
    )


def test_imm_estimates_planar_turn_rate():
    """Infer planar turn rate from velocity-direction changes."""
    predictor = IMMPredictor(
        turn_rate_epsilon=1.0e-6,
        likelihood_variance=0.05,
    )

    dt = 0.1
    turn_rate = 0.5

    state_0 = np.array([
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
    ])

    heading_1 = (
        turn_rate * dt
    )

    state_1 = np.array([
        0.1, 0.0, 1.0,
        np.cos(heading_1),
        np.sin(heading_1),
        0.0,
    ])

    heading_2 = (
        2.0 * turn_rate * dt
    )

    state_2 = np.array([
        0.2, 0.0, 1.0,
        np.cos(heading_2),
        np.sin(heading_2),
        0.0,
    ])

    predictor.predict(
        PredictionInput(
            state=state_0,
            timestamp=0.0,
            dt=dt,
            horizon=3,
        )
    )

    predictor.update(
        state_1
    )

    predictor.predict(
        PredictionInput(
            state=state_1,
            timestamp=dt,
            dt=dt,
            horizon=3,
        )
    )

    predictor.update(
        state_2
    )

    assert np.isclose(
        predictor.estimated_turn_rate,
        turn_rate,
    )
