"""Condensed finite-horizon kinematic MPC, with shared task geometry and limits.

Freeze the current task Jacobian. Evaluate future unactuated errors b[k] using
existing task/reference geometry, then e[k] = b[k] + J sum(dt[i] u[i]).
Thus reference motion enters once via b[k], never again as target_rate.
Only the first command is executed through the independent final safety gate.
"""

from dataclasses import replace
from time import perf_counter

import numpy as np
from scipy.linalg import solve_triangular
from scipy.optimize import Bounds, LinearConstraint, minimize
from scipy.spatial.transform import Rotation

from robot_configuration.model_registration import ModelRegistration
from robot_control.interfaces.control_data import ControlMode, ControlOutput
from robot_control.interfaces.controller import Controller
from robot_control.interfaces.velocity_task import VelocityTask
from robot_control.safety.safe_control_pipeline import SafeControlPipeline
from robot_control.tracking.tracking_reference_builder import TrackingReferenceBuilder
from robot_control.tracking.velocity_task import CartesianVelocityTask, DistancePointingTask


def _positive_vector(value, name):
    result = np.asarray(value, dtype=float)
    if (
        result.ndim != 1
        or not result.size
        or not np.all(np.isfinite(result))
        or np.any(result <= 0)
    ):
        raise ValueError(f'MPC {name} must be a positive finite vector')
    return result.copy()


class ModelPredictiveController(Controller):
    """Strictly convex, normalized velocity MPC solved anew each cycle."""

    def __init__(
        self,
        *,
        task,
        safety,
        reference_builder,
        horizon,
        step_duration,
        task_weights,
        task_scales,
        motion_weight,
        smoothness_weight,
        terminal_weight,
        max_iterations,
        solver_tolerance,
        feasibility_tolerance,
    ):
        """Take numerical tuning externally; physical bounds belong to safety."""
        if not isinstance(task, VelocityTask) or not isinstance(safety, SafeControlPipeline):
            raise TypeError('MPC requires VelocityTask and shared SafeControlPipeline')
        if not isinstance(reference_builder, TrackingReferenceBuilder):
            raise TypeError('MPC requires the shared TrackingReferenceBuilder')
        for name, value in [('horizon', horizon), ('max_iterations', max_iterations)]:
            if type(value) is not int or value < (2 if name == 'horizon' else 1):
                raise ValueError(f'MPC {name} must be a valid integer')
        for name, value in [
            ('step_duration', step_duration),
            ('motion_weight', motion_weight),
            ('solver_tolerance', solver_tolerance),
            ('feasibility_tolerance', feasibility_tolerance),
        ]:
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'MPC {name} must be positive and finite')
        for name, value in [
            ('smoothness_weight', smoothness_weight),
            ('terminal_weight', terminal_weight),
        ]:
            if not np.isfinite(value) or value < 0:
                raise ValueError(f'MPC {name} must be nonnegative and finite')
        self.task, self.safety, self.reference_builder = task, safety, reference_builder
        self.horizon, self.step_duration = horizon, float(step_duration)
        self.weights = _positive_vector(task_weights, 'task_weights')
        self.scales = _positive_vector(task_scales, 'task_scales')
        if self.weights.shape != self.scales.shape:
            raise ValueError('MPC task weight/scale dimensions differ')
        self.motion_weight, self.smoothness_weight = motion_weight, smoothness_weight
        self.terminal_weight, self.max_iterations = terminal_weight, max_iterations
        self.solver_tolerance, self.feasibility_tolerance = solver_tolerance, feasibility_tolerance
        self.last_diagnostics = {}

    def _task_at(self, inputs, data, state):
        if isinstance(self.task, DistancePointingTask):
            # This task uses target position directly; it does not consume a
            # Cartesian desired pose. Avoid generating nine unused SO(3) poses.
            task = self.task.evaluate(replace(inputs, target_state=state), data)
            if task.alternative_errors:
                raise ValueError('MPC local angular model is undefined at the antipodal chart')
            return task, None
        bundle = self.reference_builder.build(data.end_effector_transform_world[:3, 3], state[:3])
        angular_error = None
        if isinstance(self.task, CartesianVelocityTask) and np.any(
            self.task.task.selection_matrix[:, 3:]
        ):
            relative = bundle.desired_pose[:3, :3] @ data.end_effector_transform_world[:3, :3].T
            angular_error = Rotation.from_matrix(relative).as_rotvec()
            if np.pi - np.linalg.norm(angular_error) <= np.sqrt(np.finfo(float).eps):
                raise ValueError('MPC Cartesian angular chart is undefined at pi')
        task = self.task.evaluate(
            replace(inputs, target_state=state, reference=bundle.reference),
            replace(
                data,
                desired_end_effector_transform_world=bundle.desired_pose,
                desired_end_effector_twist_world=bundle.desired_twist,
            ),
        )
        if task.alternative_errors:
            raise ValueError('MPC local angular model is undefined at the antipodal chart')
        return task, angular_error

    def compute(self, control_input, model, model_data):
        """Optimize bounded velocities using actual control and forecast timestamps."""
        started = perf_counter()
        self.last_diagnostics = {'solver_success': False}
        inputs, data = control_input, model_data
        if (
            inputs.evaluation_time is None
            or not np.isfinite(inputs.evaluation_time)
            or inputs.dt is None
            or not np.isfinite(inputs.dt)
            or inputs.dt <= 0
        ):
            raise ValueError('MPC requires finite evaluation_time and positive control dt')
        prediction = inputs.target_prediction
        if prediction is None or prediction.frame_id != inputs.robot_state.frame_id:
            raise ValueError('MPC requires a forecast in the robot state world frame')
        n, N = model.generalized_velocity_dimension, self.horizon
        previous = np.asarray(
            (
                inputs.previous_velocity
                if inputs.previous_velocity is not None
                else inputs.robot_state.velocity
            ),
            dtype=float,
        )
        if previous.shape != (n,) or not np.all(np.isfinite(previous)):
            raise ValueError('MPC previous velocity has invalid dimensions/values')
        # The first held command uses the real control interval. Subsequent planned
        # holds use the configured forecast discretization, independent of sample rate.
        durations = np.r_[inputs.dt, np.full(N - 1, self.step_duration)]
        times = inputs.evaluation_time + np.r_[0.0, np.cumsum(durations)]
        states = prediction.sample(times)
        evaluations = [self._task_at(inputs, data, state) for state in states]
        tasks = [item[0] for item in evaluations]
        if evaluations[0][1] is not None and np.any(
            np.linalg.norm(np.diff([item[1] for item in evaluations], axis=0), axis=1) >= np.pi
        ):
            raise ValueError('MPC forecast crosses a discontinuous full angular chart')
        m = len(self.weights)
        J = np.asarray(tasks[0].jacobian, dtype=float)
        errors = np.array([task.error for task in tasks], dtype=float)
        if (
            J.shape != (m, n)
            or errors.shape != (N + 1, m)
            or not np.all(np.isfinite(J))
            or not np.all(np.isfinite(errors))
        ):
            raise ValueError('MPC task dimensions/values do not match configured units')
        # Principal angular charts cannot be linearly crossed. Fail safely rather
        # than treat a 2*pi discontinuity as a large physical target displacement.
        angular = []
        if isinstance(self.task, DistancePointingTask):
            angular = [1, 2]
        elif isinstance(self.task, CartesianVelocityTask):
            selection = self.task.task.selection_matrix
            if np.any(np.any(selection[:, :3], axis=1) & np.any(selection[:, 3:], axis=1)):
                raise ValueError('MPC Cartesian rows must not mix linear/angular units')
            angular = np.flatnonzero(np.any(selection[:, 3:], axis=1))
        if len(angular) and np.any(
            np.linalg.norm(np.diff(errors[:, angular], axis=0), axis=1) >= np.pi
        ):
            raise ValueError('MPC forecast crosses a discontinuous angular chart')

        limits = self.safety.velocity_limits
        lower, upper = np.array(limits.lower, copy=True), np.array(limits.upper, copy=True)
        if lower.shape != (n,) or upper.shape != (n,):
            raise ValueError('MPC model and safety dimensions differ')
        velocity_scale = np.maximum(np.abs(lower), np.abs(upper))
        velocity_scale = np.where(velocity_scale > 0, velocity_scale, 1.0)
        joints = None
        indices = self.safety.joint_velocity_indices
        if indices is not None:
            joints = inputs.robot_state.configuration[-len(indices):]
        first_lower, first_upper = self.safety.feasible_bounds(previous, joints, inputs.dt)
        box_lower, box_upper = np.tile(lower / velocity_scale, N), np.tile(
            upper / velocity_scale, N
        )
        box_lower[:n], box_upper[:n] = first_lower / velocity_scale, first_upper / velocity_scale

        # z[k] = u[k]/velocity_scale. C integrates every preceding planned hold.
        C = np.tril(np.ones((N, N))) * durations[None, :]
        integration = np.kron(C, np.eye(n)) * np.tile(velocity_scale, N)[None, :]
        A = np.kron(np.eye(N), J) @ integration
        weights = np.tile(self.weights / self.scales**2, N)
        weights[-m:] *= 1.0 + self.terminal_weight
        b = errors[1:].ravel()
        difference = np.eye(N) - np.eye(N, k=-1)
        D = np.kron(difference, np.eye(n))
        d = np.r_[previous / velocity_scale, np.zeros((N - 1) * n)]
        H = A.T @ (weights[:, None] * A) + self.motion_weight * np.eye(N * n)
        H += self.smoothness_weight * (D.T @ D)
        g = A.T @ (weights * b) - self.smoothness_weight * D.T @ d

        rows, lows, highs = [], [], []
        acceleration = self.safety.acceleration_limits
        if acceleration is not None:
            rate = D * np.tile(velocity_scale, N)[None, :]
            offset = np.r_[previous, np.zeros((N - 1) * n)]
            rows.append(rate)
            lows.append(np.repeat(durations, n) * np.tile(acceleration.lower, N) + offset)
            highs.append(np.repeat(durations, n) * np.tile(acceleration.upper, N) + offset)
        if indices is not None:
            selection = np.eye(n)[indices]
            rows.append(np.kron(np.eye(N), selection) @ integration)
            lows.append(np.tile(self.safety.joint_limits.lower - joints, N))
            highs.append(np.tile(self.safety.joint_limits.upper - joints, N))
        M = np.vstack(rows) if rows else np.empty((0, N * n))
        lo, hi = (
            (np.concatenate(lows), np.concatenate(highs)) if rows else (np.array([]), np.array([]))
        )

        def violation(z):
            if z.shape != (N * n,) or not np.all(np.isfinite(z)):
                return np.inf
            return max(
                0.0,
                np.max(box_lower - z),
                np.max(z - box_upper),
                np.max(lo - M @ z, initial=0.0),
                np.max(M @ z - hi, initial=0.0),
            )

        # Exact SPD unconstrained minimizer is also the constrained optimum if
        # feasible. Otherwise SLSQP solves this same convex quadratic with analytic
        # derivatives and linear constraints; there is no controller fallback.
        z = np.linalg.solve(H, -g)
        iterations = 0
        if violation(z) > self.feasibility_tolerance:
            constraints = [LinearConstraint(M, lo, hi)] if len(M) else []
            initial = np.clip(np.tile(previous / velocity_scale, N), box_lower, box_upper)
            options = {
                'maxiter': self.max_iterations,
                'ftol': self.solver_tolerance,
                'disp': False,
            }
            if np.any(box_lower == box_upper) or np.any(lo == hi):
                # Preserve SciPy's fixed-coordinate elimination and equality
                # handling for degenerate envelopes; the objective is identical.
                result = minimize(
                    lambda x: float(x @ H @ x + 2 * g @ x),
                    initial,
                    jac=lambda x: 2 * (H @ x + g),
                    method='SLSQP',
                    bounds=Bounds(box_lower, box_upper),
                    constraints=constraints,
                    options=options,
                )
                z = np.asarray(result.x)
            else:
                # H=L L.T; z=z_free+L^-T y makes the same QP's objective
                # 0.5*||y||² (up to constant/positive scale). An identity Hessian
                # avoids iterative curvature learning in SLSQP. Transform ALL
                # linear constraints, including velocity boxes, without relaxing.
                transform = solve_triangular(np.linalg.cholesky(H).T, np.eye(N * n))
                full = np.vstack([np.eye(N * n), M])
                shifted = full @ z
                constraint = LinearConstraint(
                    full @ transform,
                    np.r_[box_lower, lo] - shifted,
                    np.r_[box_upper, hi] - shifted,
                )
                result = minimize(
                    lambda y: 0.5 * float(y @ y),
                    np.zeros(N * n),
                    jac=lambda y: y,
                    method='SLSQP',
                    constraints=[constraint],
                    options=options,
                )
                z = z + transform @ np.asarray(result.x)
            # SciPy's all-variables-fixed success has no iteration counter.
            iterations = getattr(result, 'nit', 0)
            if not result.success:
                raise ValueError(f'MPC optimization failed: {result.message}')
        residual = violation(z)
        if residual > self.feasibility_tolerance:
            raise ValueError(f'MPC infeasible/nonfinite solution: violation={residual}')
        commands = z.reshape(N, n) * velocity_scale
        self.last_diagnostics = {
            'solver_success': True,
            'solver_iterations': int(iterations),
            'solver_seconds': perf_counter() - started,
            'constraint_violation': float(residual),
            'task_error': errors[0].copy(),
            'singular_values': np.linalg.svd(J, compute_uv=False),
            'forecast_times': times.copy(),
            'predicted_task_errors': (A @ z + b).reshape(N, m),
        }
        return ControlOutput(commands[0], ControlMode.VELOCITY, commands)


_SCHEMA = {
    'type': 'object',
    'additionalProperties': False,
    'properties': {
        'horizon': {'type': 'integer', 'minimum': 2, 'unit': 'steps'},
        'step_duration': {'type': 'number', 'exclusiveMinimum': 0.0, 'unit': 's'},
        'task_weights': {
            'type': 'array',
            'minItems': 1,
            'items': {'type': 'number', 'exclusiveMinimum': 0.0},
            'unit': '1',
        },
        'task_scales': {
            'type': 'array',
            'minItems': 1,
            'items': {'type': 'number', 'exclusiveMinimum': 0.0},
            'unit': 'task row units (m or rad)',
        },
        'motion_weight': {'type': 'number', 'exclusiveMinimum': 0.0, 'unit': '1'},
        'smoothness_weight': {'type': 'number', 'minimum': 0.0, 'unit': '1'},
        'terminal_weight': {'type': 'number', 'minimum': 0.0, 'unit': '1'},
        'max_iterations': {'type': 'integer', 'minimum': 1, 'unit': 'iterations'},
        'solver_tolerance': {'type': 'number', 'exclusiveMinimum': 0.0, 'unit': '1'},
        'feasibility_tolerance': {'type': 'number', 'exclusiveMinimum': 0.0, 'unit': '1'},
    },
}
_SCHEMA['required'] = list(_SCHEMA['properties'])
COMPONENTS = (
    ModelRegistration(
        'mpc',
        ModelPredictiveController,
        _SCHEMA,
        dependencies=('task', 'safety', 'reference_builder'),
        description='Model Predictive Control',
        capabilities={
            'deployable': True,
            'runtime_validation': 'locally_linearized_kinematic_mpc',
            'display_name': 'Model Predictive Control',
            'task_binding': 'task',
            'command_layout': 'body_base_joint_velocity',
            'command_dimension': 'model.generalized_velocity_dimension',
            'safety': 'shared horizon bounds and independent final projection',
        },
    ),
)
