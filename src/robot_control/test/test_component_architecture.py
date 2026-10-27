"""Canonical controller/task defaults retain deployed parameter contracts."""
from pathlib import Path

import pytest
import yaml


def test_control_families_are_separate_and_legacy_projection_is_detached():
    from robot_control.configuration import load_control_parameters
    path = Path(__file__).parents[1] / 'config/control.yaml'
    document = yaml.safe_load(path.read_text())
    assert {'controllers', 'tasks', 'tracking', 'transport', 'runtime'} <= document.keys()
    params = load_control_parameters(path, 'parameters')['control_node']['ros__parameters']
    assert params['controller_damping'] == 1.0
    assert params['controller_task_gains'] == [1.0, 1.0, 1.0]
    assert params['tracking_forward_axis'] == [1.0, 0.0, 0.0]
    strategy = load_control_parameters(path, 'distance_pointing')
    assert strategy['controller']['parameters']['damping'] == .08
    strategy['controller']['parameters']['gains'][0] = 99.0
    assert load_control_parameters(path, 'distance_pointing')['controller']['parameters']['gains'][0] == 1.0


def test_controllers_and_references_have_discoverable_strict_schemas():
    from robot_control.factory.project_components import create_project_registries
    registries = create_project_registries()
    info = registries['controller'].describe('weighted_least_squares')
    assert info['schema']['properties']['damping']['unit'] == '1'
    assert info['capabilities']['command_layout'] == 'body_base_joint_velocity'
    for parameters in [{'damping': True}, {'damping': float('nan')}, {'typo': 2.0}]:
        with pytest.raises(ValueError):
            registries['controller'].validate('weighted_least_squares', parameters)
    assert registries['policy'].describe('radial')['capabilities']['spatial_dimension'] == 3


@pytest.mark.parametrize('name', ['gains', 'task_weights', 'task_scales', 'motion_weights'])
@pytest.mark.parametrize('values', [[], [-1.0], [0.0], [True]])
def test_weighted_controller_vector_schemas_match_constructor_bounds(name, values):
    from robot_control.factory.project_components import create_project_registries
    with pytest.raises(ValueError):
        create_project_registries()['controller'].validate('weighted_least_squares', {name: values})
