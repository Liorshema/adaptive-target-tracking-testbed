"""Test-only local acceleration law; defaults belong to owner_parameters.yaml."""

from robot_simulation.dynamics.acceleration_model import acceleration_model


@acceleration_model(
    'example_spring',
    dimension=3,
    schema={
        'type': 'object',
        'additionalProperties': False,
        'properties': {
            'stiffness': {'type': 'number', 'minimum': 0, 'unit': 's^-2'},
            'damping': {'type': 'number', 'minimum': 0, 'unit': 's^-1'},
        },
        'required': ['stiffness', 'damping'],
    },
)
def spring(position, velocity, time, parameters, context):
    return -parameters['stiffness'] * position - parameters['damping'] * velocity


COMPONENTS = (spring,)
