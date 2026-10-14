"""Verify injected model composition and exactly one tracking-frame mount."""

from copy import deepcopy

import numpy as np
import pytest

from robot_models.base.differential_drive.kinematics import DifferentialDriveKinematics
from robot_models.common.rotations import rotation_z, rpy_to_rotation
from robot_models.common.transforms import make_transform
from robot_models.manipulator.serial_arm.model import SerialArmModel
from robot_models.whole_body.jacobian import WholeBodyJacobian
from robot_models.whole_body.kinematics import WholeBodyKinematics


def configuration():
    """Return component specs with a nontrivial mounted-frame transform."""
    return {
        'base': {'type': 'differential_drive', 'parameters': {
            'wheel_radius': .15, 'track_width': .68}},
        'manipulator': {'type': 'serial_arm', 'parameters': {
            'joint_axes': [[0., 0., 1.], [0., -1., 0.],
                           [0., -1., 0.], [0., -1., 0.]],
            'body_height': .2, 'arm_base_height': .08,
            'link_1_length': .4, 'link_2_length': .36, 'wrist_length': .12}},
        'tracking_frame': {'parent': 'wrist_link', 'translation': [.12, .03, -.04],
                           'rotation_rpy': [.2, -.3, .4]},
    }


def component_factory(category, specification):
    """Construct the real components passed through the injected boundary."""
    implementations = {'base': DifferentialDriveKinematics, 'manipulator': SerialArmModel}
    return implementations[category](**specification['parameters'])


def build(config, create=component_factory):
    """Resolve the new generic factory only when a test exercises it."""
    from robot_models.whole_body.factory import build_configured_model
    return build_configured_model(config, create)


def test_optional_mount_preserves_unmounted_pose_and_jacobian():
    """Omitting a mount must preserve the supplied component's end frame."""
    cfg = configuration()
    del cfg['tracking_frame']
    model = build(cfg)
    arm = component_factory('manipulator', cfg['manipulator'])
    q = np.array([.3, -.2, .4, .1])
    np.testing.assert_allclose(model.manipulator.forward_kinematics(q), arm.forward_kinematics(q))
    np.testing.assert_allclose(model.manipulator.jacobian(q), arm.jacobian(q))
    assert model.generalized_velocity_dimension == 6


def test_mount_matches_single_transform_and_shifted_whole_body_jacobian():
    """Missing or duplicated offsets must change both the pose and Jacobian."""
    cfg = configuration()
    original = deepcopy(cfg)
    model = build(cfg)
    base = component_factory('base', cfg['base'])
    arm = component_factory('manipulator', cfg['manipulator'])
    q = np.array([.3, -.2, .4, .1])
    transform = make_transform(rotation_z(.6), np.array([.4, -.2, .25]))
    parent_pose = arm.forward_kinematics(q)
    mount = make_transform(rpy_to_rotation(cfg['tracking_frame']['rotation_rpy']),
                           np.array(cfg['tracking_frame']['translation']))
    mounted_pose = parent_pose @ mount
    expected_pose = transform @ mounted_pose
    arm_jacobian = arm.jacobian(q).copy()
    offset = parent_pose[:3, :3] @ np.array(cfg['tracking_frame']['translation'])
    arm_jacobian[:3] += np.cross(arm_jacobian[3:].T, offset).T
    rotation = transform[:3, :3]
    combined_jacobian = np.hstack([
        base.spatial_jacobian(mounted_pose[:3, 3]), arm_jacobian,
    ])
    expected_jacobian = np.block([[rotation, np.zeros((3, 3))],
                                 [np.zeros((3, 3)), rotation]]) @ combined_jacobian
    np.testing.assert_allclose(WholeBodyKinematics.compute(model, transform, q), expected_pose)
    np.testing.assert_allclose(WholeBodyJacobian.compute(model, transform, q), expected_jacobian)
    assert cfg == original


@pytest.mark.parametrize('invalid_category', ['base', 'manipulator'])
def test_incompatible_injected_component_is_rejected(invalid_category):
    """A component of the wrong interface must fail before model use."""
    def create(category, specification):
        if category == invalid_category:
            return object()
        return component_factory(category, specification)
    with pytest.raises(TypeError, match=invalid_category):
        build(configuration(), create)


@pytest.mark.parametrize('invalid', [None, [], 'robot'])
def test_nonmapping_configuration_is_rejected(invalid):
    """Malformed configurations must give a clear contract error."""
    with pytest.raises(TypeError, match='configuration'):
        build(invalid)


@pytest.mark.parametrize('section', ['base', 'manipulator', 'tracking_frame'])
def test_nonmapping_component_or_mount_section_is_rejected(section):
    """Each declared section must retain its configuration mapping contract."""
    cfg = configuration()
    cfg[section] = []
    with pytest.raises(ValueError, match=section):
        build(cfg)


@pytest.mark.parametrize('field,value', [
    ('translation', [float('nan'), 0., 0.]),
    ('rotation_rpy', [0., float('inf'), 0.]),
    ('translation', [0., 0.]),
])
def test_invalid_mount_transform_is_rejected(field, value):
    """Nonfinite or malformed mounts cannot enter reusable model composition."""
    cfg = configuration()
    cfg['tracking_frame'][field] = value
    with pytest.raises(ValueError, match=field):
        build(cfg)
