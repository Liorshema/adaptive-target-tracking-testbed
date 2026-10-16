"""Assemble real filters from data without a ROS node or package lookup."""

from pathlib import Path
import numpy as np
import pytest
import yaml

from robot_estimation.factory.estimator_factory import create_robot_estimator, create_target_estimator


def configuration():
    root = Path(__file__).resolve().parents[2]
    parameters = {}
    for name in ['robot_estimator', 'target_estimator']:
        parameters.update(yaml.safe_load((root/'robot_estimation/config'/f'{name}.yaml').read_text())['estimation_node']['ros__parameters'])
    physical = yaml.safe_load((root/'robot_description/config/robot/robot.yaml').read_text())['robot']['manipulator']['joints']
    return parameters, physical


def test_robot_assembly_uses_authoritative_bounds_and_initial_distribution():
    parameters, physical = configuration()
    estimator, constraint = create_robot_estimator(parameters, physical)
    np.testing.assert_array_equal(estimator.state.mean, parameters['robot_initial_state'])
    np.testing.assert_array_equal(estimator.state.covariance, np.diag(parameters['robot_initial_covariance_diagonal']))
    np.testing.assert_array_equal(constraint.lower, [j['lower'] for j in physical.values()])
    assert estimator.state.timestamp is None
    assert estimator.state.dimension == 5+2*len(physical)


def test_disabled_constraint_requires_no_description_and_target_assembly_is_separate():
    parameters, _ = configuration()
    parameters['joint_bounds_enabled'] = False
    estimator, constraint = create_robot_estimator(parameters)
    assert constraint is None and estimator.state.dimension == 13
    target = create_target_estimator(parameters)
    np.testing.assert_array_equal(target.state.mean, parameters['target_initial_state'])
    np.testing.assert_array_equal(target.state.covariance, np.diag(parameters['target_initial_covariance_diagonal']))


def test_invalid_robot_initial_dimension_still_fails():
    parameters, physical = configuration()
    parameters['robot_initial_state'] = [0.]
    with pytest.raises(ValueError, match='dimension'):
        create_robot_estimator(parameters, physical)
