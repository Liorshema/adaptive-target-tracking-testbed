"""Planar absolute-pose observations in fixed inertial frames; no ROS dependency."""
import numpy as np
from robot_estimation.interfaces.robot_measurement_model import RobotMeasurementModel


def transform_observation(value, covariance, transform, tolerance):
    """Transform [x,y,yaw] and its full covariance between planar fixed frames."""
    z = np.asarray(value, dtype=float)
    covariance = np.asarray(covariance, dtype=float)
    transform = np.asarray(transform, dtype=float)
    if (z.shape != (3,) or covariance.shape != (3, 3) or transform.shape != (4, 4)
            or not np.isfinite(np.r_[z, covariance.ravel(), transform.ravel()]).all()
            or not np.isfinite(tolerance) or tolerance <= 0):
        raise ValueError('Invalid planar pose dimensions or values')
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=tolerance)
            or not np.isclose(np.linalg.det(rotation), 1., atol=tolerance)
            or not np.allclose(rotation[2], [0, 0, 1], atol=tolerance)
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=tolerance)):
        raise ValueError('Pose observation requires a planar fixed-frame transform')
    if (not np.allclose(covariance, covariance.T, atol=tolerance)
            or np.linalg.eigvalsh(covariance).min() <= 0):
        raise ValueError('Pose covariance must be symmetric positive definite')
    jacobian = np.eye(3)
    jacobian[:2, :2] = rotation[:2, :2]
    xy = rotation[:2, :2] @ z[:2] + transform[:2, 3]
    yaw = z[2] + np.arctan2(rotation[1, 0], rotation[0, 0])
    return np.r_[xy, np.arctan2(np.sin(yaw), np.cos(yaw))], jacobian @ covariance @ jacobian.T


class PlanarPoseModel(RobotMeasurementModel):
    """Observe the existing robot state's [x,y,yaw] with a wrapped innovation."""

    def __init__(self, joint_count):
        self.dimension = 5 + 2 * joint_count

    @property
    def measurement_dimension(self):
        return 3

    def predict_measurement(self, state):
        state = np.asarray(state, dtype=float)
        if state.shape != (self.dimension,) or not np.isfinite(state).all():
            raise ValueError('Invalid robot pose state')
        return state[:3].copy()

    def jacobian(self, state):
        self.predict_measurement(state)
        return np.eye(self.dimension)[:3]

    def innovation(self, observation, state):
        residual = np.asarray(observation) - self.predict_measurement(state)
        residual[2] = np.arctan2(np.sin(residual[2]), np.cos(residual[2]))
        return residual


def validate_localization_parameters(parameters):
    """Legacy configurations remain disabled; enabled sources require explicit values."""
    enabled = parameters.get('localization_enabled', False)
    if type(enabled) is not bool:
        raise ValueError('localization_enabled must be boolean')
    if not enabled:
        return
    for name in ['localization_topic', 'localization_frame']:
        if not isinstance(parameters.get(name), str) or not parameters[name].strip():
            raise ValueError(f'{name} must be explicitly configured')
    for name in ['localization_max_age', 'localization_nis_limit', 'localization_transform_tolerance']:
        value = parameters.get(name)
        if type(value) not in (float, int) or not np.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite positive')
    if parameters['localization_topic'] == parameters.get('wheel_odometry_topic'):
        raise ValueError('Independent localization must not reuse wheel odometry')
