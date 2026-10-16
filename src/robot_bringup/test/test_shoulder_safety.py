"""Check the authoritative shoulder bound at model and execution boundaries."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

import pytest

from robot_control.factory.project_configuration import project_control_configuration
from robot_control.safety.safe_control_pipeline import SafeControlPipeline

from robot_description.model_parameters import load_robot_model_parameters

from robot_models.constraints.joint_limits import JointLimits
from robot_models.constraints.velocity_limits import VelocityLimits

from robot_simulation.actuation.position_reference import PositionReference
from robot_configuration.yaml_loader import load_yaml

from scipy.spatial.transform import Rotation

import xacro


ROOT = Path(__file__).resolve().parents[2]


def test_horizontal_shoulder_cannot_receive_downward_velocity_or_reference():
    """Description-derived bounds stop downward commands at the horizontal pose."""
    path = ROOT / 'robot_description/config/robot/robot.yaml'
    description = load_yaml(path)
    parameters = load_robot_model_parameters(path)
    physical = description['robot']['manipulator']['joints']
    shoulder = list(physical).index('shoulder')
    _, limits, _ = project_control_configuration(
        description, [-0.6, -1.0], [0.6, 1.0])
    joint = limits['joint']
    safety = SafeControlPipeline(
        VelocityLimits(**{key: np.asarray(value)
                          for key, value in limits['velocity'].items()}),
        joint_limits=JointLimits(np.asarray(joint['lower']), np.asarray(joint['upper'])),
        joint_velocity_indices=np.asarray(joint['velocity_indices']))
    q = np.zeros(len(physical))
    below = q.copy()
    below[shoulder] = -0.01
    assert not JointLimits(parameters.joint_limits.lower,
                           parameters.joint_limits.upper).contains(below)
    low, _ = safety.feasible_bounds(np.zeros(len(physical) + 2), q, 0.02)
    assert low[joint['velocity_indices'][shoulder]] >= 0.0
    reference = PositionReference(parameters.joint_limits.lower,
                                  parameters.joint_limits.upper,
                                  parameters.joint_limits.velocity, 0.1, 0.1)
    reference.initialize(q)
    velocity = np.zeros(len(physical))
    velocity[shoulder] = -1.0
    assert reference.step(velocity, q, 0.02)[shoulder] >= 0.0


@pytest.mark.parametrize('angle', [0.0, 0.4, 1.57])
def test_first_link_geometry_and_fk_raise_for_positive_shoulder(angle):
    """The real Xacro collision box and model agree on the shoulder convention."""
    path = ROOT / 'robot_description/config/robot/robot.yaml'
    document = xacro.process_file(
        str(ROOT / 'robot_description/urdf/robot.urdf.xacro'),
        mappings={'config_file': str(path)})
    urdf = ET.fromstring(document.toxml())
    shoulder = urdf.find("joint[@name='shoulder_joint']")
    np.testing.assert_allclose(
        np.fromstring(shoulder.find('origin').get('rpy'), sep=' '), [0., 0., 0.])
    axis = np.fromstring(shoulder.find('axis').get('xyz'), sep=' ')
    rotation = Rotation.from_rotvec(axis * angle).as_matrix()
    direction = rotation @ np.array([1., 0., 0.])
    assert direction[2] >= 0.0
    if angle == 0.0:
        np.testing.assert_allclose(direction, [1., 0., 0.])
    collision = urdf.find("link[@name='arm_link_1']/collision")
    size = np.fromstring(collision.find('geometry/box').get('size'), sep=' ')
    center = np.fromstring(collision.find('origin').get('xyz'), sep=' ')
    np.testing.assert_allclose(center, [size[0] / 2, 0., 0.])
    from robot_bringup.factory.model_factory import create_robot_model
    from robot_models.whole_body.kinematics import WholeBodyKinematics
    parameters = load_robot_model_parameters(path)
    model = create_robot_model(parameters)
    zero = WholeBodyKinematics.compute(model, np.eye(4), np.zeros(4))
    raised = WholeBodyKinematics.compute(model, np.eye(4), np.array([0., angle, 0., 0.]))
    assert raised[2, 3] >= zero[2, 3]
    np.testing.assert_allclose(raised[:3, :3], rotation, atol=1e-12)


def test_experiment_bringup_inherits_baseline_pipeline():
    """Changing experiment selection must retain the same control and estimation."""
    baseline = load_yaml(ROOT / 'robot_bringup/config/reactive_tracking.yaml')
    experiment = load_yaml(ROOT / 'robot_bringup/config/reactive_tracking_low_damping.yaml')
    assert experiment.pop('simulation_config') == (
        'config/scenarios/reactive_target_low_damping.yaml')
    baseline.pop('simulation_config')
    assert experiment == baseline
