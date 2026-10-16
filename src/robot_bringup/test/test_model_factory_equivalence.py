"""Preserve Bringup poses and Jacobians while sharing model construction."""

from dataclasses import asdict, replace

import numpy as np

from robot_bringup.factory.model_factory import create_robot_model
from robot_description.model_parameters import load_robot_model_parameters
from robot_models.base.differential_drive.kinematics import DifferentialDriveKinematics
from robot_models.spatial.rotations import rotation_z
from robot_models.spatial.transforms import make_transform
from robot_models.manipulator.fixed_mount.model import FixedMountModel
from robot_models.manipulator.serial_arm.model import SerialArmModel
from robot_models.whole_body.composer import WholeBodyComposer
from robot_models.whole_body.jacobian import WholeBodyJacobian
from robot_models.whole_body.kinematics import WholeBodyKinematics


def test_custom_parameters_match_previous_explicit_composition():
    """Custom physical parameters and a rotated offset retain prior behavior."""
    parameters = load_robot_model_parameters()
    parameters = replace(
        parameters,
        base=replace(parameters.base, wheel_radius=.23, track_width=.91),
        tracking_frame=replace(parameters.tracking_frame,
                               translation=np.array([.15, -.06, .04]),
                               rotation_rpy=np.array([.2, -.3, .1])),
    )
    previous = WholeBodyComposer(
        base=DifferentialDriveKinematics(**asdict(parameters.base)),
        manipulator=FixedMountModel(
            parent=SerialArmModel(**asdict(parameters.manipulator)),
            translation=parameters.tracking_frame.translation,
            rotation_rpy=parameters.tracking_frame.rotation_rpy,
        ),
    )
    current = create_robot_model(parameters)
    transform = make_transform(rotation_z(-.4), np.array([.5, -.3, .25]))
    for q in [np.zeros(4), np.array([.2, -.3, .4, -.15])]:
        np.testing.assert_allclose(WholeBodyKinematics.compute(current, transform, q),
                                   WholeBodyKinematics.compute(previous, transform, q))
        np.testing.assert_allclose(WholeBodyJacobian.compute(current, transform, q),
                                   WholeBodyJacobian.compute(previous, transform, q))
