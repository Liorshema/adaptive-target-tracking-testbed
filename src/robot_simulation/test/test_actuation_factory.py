"""Preflight and the ROS adapter share actual actuator startup construction."""
from robot_control.configuration import load_control_parameters

from pathlib import Path
from copy import deepcopy

import numpy as np
import pytest

from robot_configuration.yaml_loader import load_yaml


def inputs():
    src = Path(__file__).resolve().parents[2]
    config = load_yaml(src / 'robot_simulation/config/actuation.yaml')['actuation']
    robot = load_yaml(src / 'robot_simulation/config/gazebo/robot.yaml')['robot']
    physical = load_yaml(src / 'robot_description/config/robot/robot.yaml')['robot']['manipulator']['joints']
    holding = load_yaml(src / 'robot_simulation/config/gazebo/joint_position_hold.yaml')['joint_position_hold']
    control = load_control_parameters(src / 'robot_control/config/control.yaml', 'parameters')['control_node']['ros__parameters']
    return config, robot, physical, holding, control['base_velocity_lower'], control['base_velocity_upper']


def test_startup_uses_authoritative_limits_and_holding_reference():
    from robot_simulation.factory.actuation_factory import create_actuation_components
    config, robot, physical, holding, lower, upper = inputs()
    parts = create_actuation_components(config, robot, physical, holding, lower, upper, enabled=True)
    np.testing.assert_array_equal(parts.reference.position, list(holding['positions'].values()))
    np.testing.assert_array_equal(parts.reference.velocity_limit, [j['velocity'] for j in physical.values()])
    assert parts.guard.mode == 'normal'
    assert parts.names == [robot['joint_name_template'].format(key=key) for key in physical]


@pytest.mark.parametrize('name,value', [('wall_timeout', -1.), ('rate_hz', 0.),
                                       ('max_position_lead', float('nan')),
                                       ('diagnostic_rate_hz', -1.)])
def test_invalid_actuator_startup_is_rejected(name, value):
    from robot_simulation.factory.actuation_factory import create_actuation_components
    config, robot, physical, holding, lower, upper = inputs()
    config[name] = value
    with pytest.raises(ValueError):
        create_actuation_components(config, robot, physical, holding, lower, upper, enabled=True)
