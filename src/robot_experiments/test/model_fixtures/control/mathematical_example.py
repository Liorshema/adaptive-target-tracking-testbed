"""Test-only Cartesian feedback; independent safety remains authoritative."""

import numpy as np
from robot_control.controllers.feedback_model import velocity_feedback_model


@velocity_feedback_model(
    'example_feedback',
    schema={
        'type': 'object',
        'additionalProperties': False,
        'properties': {'gain': {'type': 'number', 'minimum': 0, 'unit': 's^-1'}},
        'required': ['gain'],
    },
    selection_matrix=np.eye(6)[:3],
    frame_id='odom',
)
def feedback(error, jacobian, target_rate, robot_velocity, parameters):
    return -parameters['gain'] * jacobian.T @ error


COMPONENTS = (feedback,)
