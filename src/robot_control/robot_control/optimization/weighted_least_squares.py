"""Normalized bounded least squares for interchangeable tracking tasks."""

import numpy as np
from scipy.optimize import lsq_linear

from robot_control.interfaces.control_data import ControlMode, ControlOutput
from robot_control.interfaces.controller import Controller
from robot_control.safety.safe_control_pipeline import SafeControlPipeline
from robot_control.tracking.velocity_task import VelocityTask


class WeightedLeastSquaresController(Controller):
    """Allocate feasible velocity across base and arm with configurable costs."""

    def __init__(self, task, safety, gains, task_weights, task_scales, motion_weights,
                 damping, posture_weight, posture_gain, smoothness_weight,
                 limit_avoidance_weight, preferred_joint_positions=None):
        """Normalize task units and generalized speeds against shared physical limits."""
        if not isinstance(task, VelocityTask) or not isinstance(safety, SafeControlPipeline):
            raise TypeError('Optimizer requires a velocity task and shared safety bounds')
        self.task = task
        self.safety = safety
        self.gains = self._vector(gains)
        self.task_weights = self._vector(task_weights)
        self.task_scales = self._vector(task_scales)
        if not self.gains.shape == self.task_weights.shape == self.task_scales.shape:
            raise ValueError('Task gains, weights and scales must have equal dimension')
        self.motion_weights = self._vector(motion_weights)
        limits = safety.velocity_limits
        self.velocity_scales = np.maximum(np.abs(limits.lower), np.abs(limits.upper))
        if (self.motion_weights.shape != self.velocity_scales.shape or
                np.any(self.velocity_scales <= 0)):
            raise ValueError('Motion weights must match nonzero actuator velocity scales')
        self.damping = float(damping)
        self.posture_weight = float(posture_weight)
        self.posture_gain = float(posture_gain)
        self.smoothness_weight = float(smoothness_weight)
        self.limit_avoidance_weight = float(limit_avoidance_weight)
        values = [damping, posture_weight, posture_gain, smoothness_weight, limit_avoidance_weight]
        if not np.all(np.isfinite(values)) or damping <= 0 or min(values[1:]) < 0:
            raise ValueError('Invalid optimizer regularization or posture/smoothing weights')
        self.preferred_joints = None
        if safety.joint_limits is not None:
            joint = safety.joint_limits
            self.preferred_joints = (
                (joint.lower + joint.upper) / 2 if preferred_joint_positions is None
                else np.asarray(preferred_joint_positions, dtype=float))
            if not joint.contains(self.preferred_joints):
                raise ValueError('Preferred posture must lie within physical joint bounds')
        self.last_diagnostics = {}

    @staticmethod
    def _vector(values):
        result = np.asarray(values, dtype=float)
        if (result.ndim != 1 or not result.size or
                not np.all(np.isfinite(result)) or np.any(result <= 0)):
            raise ValueError('Weights, gains and scales must be finite positive vectors')
        return result

    def compute(self, control_input, model, model_data):
        """Solve the bounded convex objective, retaining independent final safety."""
        linearization = self.task.evaluate(control_input, model_data)
        error, jacobian, target_rate = (
            linearization.error, linearization.jacobian, linearization.target_rate)
        dimension = model.generalized_velocity_dimension
        if (jacobian.shape != (self.gains.size, dimension) or error.shape != self.gains.shape or
                not np.all(np.isfinite(np.r_[jacobian.ravel(), error, target_rate]))):
            raise ValueError('Task dimension or values disagree with the optimizer')
        indices = self.safety.joint_velocity_indices
        joints = (
            control_input.robot_state.configuration[-len(indices):]
            if indices is not None else None)
        lower, upper = self.safety.feasible_bounds(
            control_input.previous_velocity, joints, control_input.dt)
        previous = (control_input.robot_state.velocity if control_input.previous_velocity is None
                    else np.asarray(control_input.previous_velocity))
        if previous.shape != (dimension,) or not np.all(np.isfinite(previous)):
            raise ValueError('Previous velocity must match the robot dimension')
        task_scaling = np.sqrt(self.task_weights) / self.task_scales
        desired_rate = -self.gains * error - target_rate
        costs = self.motion_weights.copy()
        if joints is not None:
            joint_limits = self.safety.joint_limits
            fraction = np.minimum(joints - joint_limits.lower, joint_limits.upper - joints) / (
                joint_limits.upper - joint_limits.lower)
            costs[indices] *= 1 + self.limit_avoidance_weight / np.maximum(fraction, 1e-3)**2
        normalized = 1 / self.velocity_scales
        matrices = [task_scaling[:, None] * jacobian,
                    np.diag(self.damping * np.sqrt(costs) * normalized)]
        vectors = [task_scaling * desired_rate, np.zeros(dimension)]
        if self.smoothness_weight:
            scaling = np.sqrt(self.smoothness_weight) * normalized
            matrices.append(np.diag(scaling))
            vectors.append(scaling * previous)
        if joints is not None and self.posture_weight:
            scaling = np.sqrt(self.posture_weight) * normalized[indices]
            matrices.append(np.eye(dimension)[indices] * scaling[:, None])
            vectors.append(scaling * self.posture_gain * (self.preferred_joints - joints))
        matrix, vector = np.vstack(matrices), np.concatenate(vectors)
        # scipy requires strict intervals: fixed components are eliminated exactly.
        free = upper > lower
        best = None
        for candidate_error in (error, *linearization.alternative_errors):
            if (candidate_error.shape != error.shape or
                    not np.all(np.isfinite(candidate_error))):
                raise ValueError('Invalid alternative task chart')
            candidate_rate = -self.gains * candidate_error - target_rate
            candidate_vector = vector.copy()
            candidate_vector[:self.gains.size] = task_scaling * candidate_rate
            candidate = lower.copy()
            if np.any(free):
                result = lsq_linear(
                    matrix[:, free],
                    candidate_vector - matrix[:, ~free] @ candidate[~free],
                    bounds=(lower[free], upper[free]), method='bvls', tol=1e-9)
                if not result.success or not np.all(np.isfinite(result.x)):
                    raise ValueError('Constrained tracking solve failed')
                candidate[free] = result.x
            score = np.linalg.norm(matrix @ candidate - candidate_vector)**2
            if best is None or score < best[0]:
                best = (score, candidate, candidate_error, candidate_rate)
        _, command, error, desired_rate = best
        singular = np.linalg.svd(jacobian * self.velocity_scales, compute_uv=False)
        self.last_diagnostics = {
            'task_error': error.copy(), 'singular_values': singular,
            'solver_success': True,
            'weighted_motion_cost': float(np.sum(costs * (command * normalized)**2)),
            'active_bounds': int(np.count_nonzero(
                np.isclose(command, lower) | np.isclose(command, upper))),
            'task_residual': float(np.linalg.norm(jacobian @ command - desired_rate))}
        return ControlOutput(command=command, mode=ControlMode.VELOCITY)
