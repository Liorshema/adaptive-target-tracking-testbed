"""
Range and two-dimensional spherical-log pointing geometry.

Spatial twists are [position rate, angular velocity] in world coordinates.
Pointing uses a fixed tangent basis at the local optical axis, leaving roll
free. The antipode has no unique logarithm: an explicit bounded escape chart
selects an achievable tangent direction rather than returning zero.
"""

import numpy as np

from robot_models.common.rotations import skew


class DistancePointingGeometry:
    """Compute three task errors and their robot/target velocity derivatives."""

    def __init__(self, radius, forward_axis_local, tangent_axis_local,
                 minimum_distance=1e-4, antipodal_tolerance=1e-6):
        """Configure separation, optical axis, chart direction and singular guards."""
        self.radius = float(radius)
        self.minimum_distance = float(minimum_distance)
        self.antipodal_tolerance = float(antipodal_tolerance)
        scalars = [self.radius, self.minimum_distance, self.antipodal_tolerance]
        if not np.all(np.isfinite(scalars)) or min(scalars) <= 0:
            raise ValueError('Geometry tolerances and radius must be finite and positive')
        self.axis = self._unit(forward_axis_local)
        tangent = np.asarray(tangent_axis_local, dtype=float)
        if tangent.shape != (3,) or not np.all(np.isfinite(tangent)):
            raise ValueError('Tangent axis must be a finite 3-vector')
        tangent = self._unit(tangent - self.axis * (self.axis @ tangent))
        self.basis = np.column_stack([tangent, np.cross(self.axis, tangent)])
        self.projector = np.eye(3) - np.outer(self.axis, self.axis)

    @staticmethod
    def _unit(vector):
        result = np.asarray(vector, dtype=float)
        if (result.shape != (3,) or not np.all(np.isfinite(result)) or
                np.linalg.norm(result) <= np.finfo(float).eps):
            raise ValueError('Axis must be a finite nonzero 3-vector')
        return result / np.linalg.norm(result)

    def evaluate(self, pose, target_position, spatial_jacobian, target_velocity):
        """Return error, generalized-velocity Jacobian and target-only error rate."""
        pose = np.asarray(pose, dtype=float)
        target = np.asarray(target_position, dtype=float)
        velocity = np.asarray(target_velocity, dtype=float)
        jacobian = np.asarray(spatial_jacobian, dtype=float)
        if (pose.shape != (4, 4) or target.shape != (3,) or velocity.shape != (3,) or
                jacobian.ndim != 2 or jacobian.shape[0] != 6 or
                not np.all(np.isfinite(np.r_[pose.ravel(), target, velocity, jacobian.ravel()]))):
            raise ValueError('Invalid distance/pointing geometry dimensions or values')
        rotation = pose[:3, :3]
        if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7) or
                not np.isclose(np.linalg.det(rotation), 1.)):
            raise ValueError('Tracking pose must have a proper rotation')
        delta = target - pose[:3, 3]
        distance = np.linalg.norm(delta)
        if distance < self.minimum_distance:
            raise ValueError('Target distance is too small to define pointing')
        direction = delta / distance
        local_direction = rotation.T @ direction
        cosine = np.clip(self.axis @ local_direction, -1., 1.)
        tangent = self.projector @ local_direction
        sine = np.linalg.norm(tangent)
        angle = np.arctan2(sine, cosine)
        direction_derivative = (np.eye(3) - np.outer(direction, direction)) / distance
        local_rate_jacobian = rotation.T @ (
            -direction_derivative @ jacobian[:3] + skew(direction) @ jacobian[3:])
        if cosine < 0 and sine < self.antipodal_tolerance:
            # No differentiable global log exists at the cut locus. Choose a
            # locally achievable escape chart, including robots with only one
            # available pointing direction. Anchor its sign to the local basis.
            log_derivative = -self.projector
            escape_jacobian = -self.basis.T @ local_rate_jacobian
            left, singular, _ = np.linalg.svd(escape_jacobian, full_matrices=False)
            escape = np.array([1., 0.])
            if singular.size and singular[0] > np.finfo(float).eps:
                escape = left[:, 0]
                anchor = int(np.argmax(np.abs(escape)))
                escape *= np.sign(escape[anchor])
            pointing = angle * escape
        elif sine < self.antipodal_tolerance:
            pointing = self.basis.T @ tangent
            log_derivative = self.projector
        else:
            factor = angle / sine
            derivative = (angle * cosine - sine) / sine**3
            pointing = self.basis.T @ (factor * tangent)
            log_derivative = factor * self.projector + derivative * np.outer(tangent, self.axis)
        pointing_map = self.basis.T @ log_derivative
        task_jacobian = np.vstack([-direction @ jacobian[:3],
                                  pointing_map @ local_rate_jacobian])
        target_rate = np.r_[direction @ velocity,
                            pointing_map @ rotation.T @ direction_derivative @ velocity]
        return np.r_[distance - self.radius, pointing], task_jacobian, target_rate
