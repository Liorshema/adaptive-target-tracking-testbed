"""Planar absolute-pose observations in fixed inertial frames; no ROS dependency."""
from collections import deque

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


class PoseAcceptance:
    """Bounded causal evidence for one external pose source, never a second filter.

    Innovations are relative to the ordinary motion prior. Only a current
    observation supported by a recent consensus may establish/recover trust.
    Historical observations are used for validation, never fused again.
    """

    def __init__(self, window, inliers, span, max_age, trust_timeout, nis_target):
        self.window = window
        self.inliers = inliers
        self.span = span
        self.max_age = max_age
        self.trust_timeout = trust_timeout
        self.nis_target = nis_target
        self.history = deque(maxlen=window)
        self.mode = 'initializing'
        self.last_observation = None
        self.last_accepted = None
        self.support = 0
        self.reason = 'insufficient_evidence'

    def health(self, now):
        """Source-time trust expires independently of wheel/joint freshness."""
        if (self.mode == 'tracking' and now-self.last_accepted > self.trust_timeout):
            self.mode = 'degraded'
            self.history.clear()
            self.reason = 'source_timeout'
        return {'mode': self.mode, 'trusted': self.mode == 'tracking',
                'last_accepted_time': self.last_accepted, 'consensus_inliers': self.support,
                'pending_observations': len(self.history), 'reason': self.reason}

    def consider(self, measurement, innovation, nis, limit, relative_covariance):
        """Validate at the current acquisition epoch, with no future evidence."""
        stamp = measurement.timestamp
        if self.last_observation is not None and stamp <= self.last_observation:
            self.reason = 'nonincreasing_source_time'
            return False
        self.last_observation = stamp
        self.health(stamp)
        if self.mode == 'tracking' and nis <= limit:
            return True
        if self.mode == 'tracking':
            self.mode = 'validating'
            self.history.clear()
        self.history.append((stamp, innovation.copy(), measurement.covariance.copy()))
        while self.history and stamp-self.history[0][0] > self.max_age:
            self.history.popleft()
        self.reason = 'insufficient_evidence'
        if len(self.history) < self.window or stamp-self.history[0][0] < self.span-1e-9:
            return False
        times = np.array([row[0] for row in self.history])
        offsets = np.array([row[1] for row in self.history])
        # Work in one local angular chart; medoid is an actual causal sample.
        offsets[:, 2] = offsets[0, 2] + np.arctan2(
            np.sin(offsets[:, 2]-offsets[0, 2]), np.cos(offsets[:, 2]-offsets[0, 2]))
        center = np.median(offsets, axis=0)
        scale = np.median([np.diag(row[2]) for row in self.history], axis=0)
        reference = int(np.argmin(np.sum((offsets-center)**2/scale, axis=1)))
        compatible = []
        for index, (_, _, covariance) in enumerate(self.history):
            residual = offsets[index]-offsets[reference]
            residual[2] = np.arctan2(np.sin(residual[2]), np.cos(residual[2]))
            dt = abs(times[index]-times[reference])
            noise = relative_covariance(dt) if dt > 0 else np.zeros((3, 3))
            uncertainty = covariance+self.history[reference][2]+noise
            compatible.append(float(residual @ np.linalg.solve(uncertainty, residual)) <= limit)
        self.support = sum(compatible)
        enough = self.support >= self.inliers and compatible[-1]
        self.reason = 'corroborated' if enough else 'inconsistent_source'
        return enough

    def commit(self, stamp):
        """Only a successful EKF update can establish the trusted source epoch."""
        self.mode = 'tracking'
        self.last_accepted = stamp
        self.history.clear()
        self.reason = 'accepted'


def pose_acceptance_options(parameters):
    """Zero window preserves legacy behavior; active policy has explicit options."""
    window = parameters.get('localization_consensus_window', 0)
    if type(window) is not int or window < 0:
        raise ValueError('localization_consensus_window must be a nonnegative integer')
    if window == 0:
        return None
    values = {key: parameters.get('localization_'+key) for key in [
        'consensus_inliers', 'consensus_span', 'consensus_max_age',
        'trust_timeout', 'recovery_nis_target']}
    count = values['consensus_inliers']
    if type(count) is not int or not window/2 < count <= window or count < 3:
        raise ValueError('Localization consensus requires a strict majority of at least three')
    for key in ['consensus_span', 'consensus_max_age', 'trust_timeout', 'recovery_nis_target']:
        value = values[key]
        if type(value) not in (float, int) or not np.isfinite(value) or value <= 0:
            raise ValueError(f'localization_{key} must be finite positive')
    if (values['consensus_span'] >= values['consensus_max_age']
            or values['recovery_nis_target'] >= parameters['localization_nis_limit']):
        raise ValueError('Localization span/age or recovery NIS target is inconsistent')
    return dict(window=window, inliers=count, span=values['consensus_span'],
                max_age=values['consensus_max_age'], trust_timeout=values['trust_timeout'],
                nis_target=values['recovery_nis_target'])


def validate_localization_parameters(parameters):
    """Legacy configurations remain disabled; enabled sources require explicit values."""
    enabled = parameters.get('localization_enabled', False)
    if type(enabled) is not bool:
        raise ValueError('localization_enabled must be boolean')
    if not enabled:
        return
    pose_acceptance_options(parameters)
    for name in ['localization_topic', 'localization_frame']:
        if not isinstance(parameters.get(name), str) or not parameters[name].strip():
            raise ValueError(f'{name} must be explicitly configured')
    for name in ['localization_max_age', 'localization_nis_limit', 'localization_transform_tolerance']:
        value = parameters.get(name)
        if type(value) not in (float, int) or not np.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite positive')
    if parameters['localization_topic'] == parameters.get('wheel_odometry_topic'):
        raise ValueError('Independent localization must not reuse wheel odometry')
