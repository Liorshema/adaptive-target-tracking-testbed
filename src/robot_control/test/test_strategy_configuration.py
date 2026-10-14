"""Verify configured task selection, shared geometry, and the legacy registry."""

from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from robot_models.interfaces.model_data import ModelData

from robot_control.factory.project_components import create_project_registries
from robot_control.factory.project_configuration import apply_strategy_configuration
from robot_control.interfaces.control_data import ControlInput, RobotState, TrackingReference
from robot_control.tracking.task_space_error import TaskSpaceError


@pytest.fixture
def shared_configuration():
    """Provide geometry independently of the selected controller strategy."""
    return {
        'controller': {'task_selection': np.eye(6)[:3].tolist()},
        'tracking': {
            'policy': {'parameters': {'radius': 1.3}},
            'pose': {
                'forward_axis_local': [0., 0., 1.],
                'up_axis_local': [1., 0., 0.],
            },
            'lookahead_time': 0.,
        },
    }


def task_inputs():
    """Use an aligned target with known separation and radial velocity."""
    desired = np.eye(4)
    desired[2, 3] = 2.
    data = ModelData(
        np.eye(4), np.zeros(6), desired, np.array([0., 0., .4, 0., 0., 0.]),
        np.eye(6))
    inputs = ControlInput(
        RobotState(np.zeros(6), np.zeros(6), 1., 'world'),
        TrackingReference(np.array([0., 0., 2.])),
        target_state=np.array([0., 0., 2., 0., 0., .4]))
    return inputs, data


@pytest.mark.parametrize('task_type,error,target_rate', [
    ('distance_pointing', [.7, 0., 0.], [.4, 0., 0.]),
    ('cartesian', [0., 0., -2.], [0., 0., -.4]),
])
def test_strategy_selects_registered_task_with_shared_geometry(
        shared_configuration, task_type, error, target_rate):
    """Selected tasks must consume the shared radius, axes, or selection."""
    strategy = {'controller': {
        'type': 'weighted_least_squares', 'task': {'type': task_type},
    }}
    original = deepcopy(shared_configuration)
    configured = apply_strategy_configuration(shared_configuration, strategy)
    task_specification = configured['controller']['task']
    task = create_project_registries()['task'].create(
        task_specification['type'], task_specification['parameters'])
    inputs, data = task_inputs()
    result = task.evaluate(inputs, data)

    np.testing.assert_allclose(result.error, error, atol=1e-12)
    np.testing.assert_allclose(result.target_rate, target_rate, atol=1e-12)
    assert shared_configuration == original
    assert strategy['controller']['task'] == {'type': task_type}


@pytest.mark.parametrize('task_type,parameters', [
    ('distance_pointing', {'radius': 2.}),
    ('distance_pointing', {'forward_axis_local': [1., 0., 0.]}),
    ('cartesian', {'selection_matrix': np.eye(6)[:1].tolist()}),
])
def test_strategy_rejects_duplicate_task_geometry(
        shared_configuration, task_type, parameters):
    """Strategy parameters must not create another source for task geometry."""
    strategy = {'controller': {'task': {
        'type': task_type, 'parameters': parameters,
    }}}

    with pytest.raises(ValueError, match='shared tracking configuration'):
        apply_strategy_configuration(shared_configuration, strategy)


def test_project_registry_preserves_legacy_jacobian_controller():
    """The original registered strategy must retain its analytical DLS command."""
    instance = create_project_registries()['controller'].create('jacobian', {
        'task': TaskSpaceError.position_only(), 'gains': np.eye(3), 'damping': .1,
    })
    inputs, data = task_inputs()
    output = instance.compute(
        inputs, SimpleNamespace(generalized_velocity_dimension=6), data)

    np.testing.assert_allclose(
        output.command, [0., 0., 2.4 / 1.01, 0., 0., 0.], atol=1e-12)
