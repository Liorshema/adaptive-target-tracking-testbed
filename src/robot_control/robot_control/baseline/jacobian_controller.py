
"""Damped least-squares task-space velocity controller."""

import numpy as np
from numpy.typing import NDArray
from robot_models.interfaces.model_data import ModelData
from robot_models.interfaces.robot_model import RobotModel

from robot_control.interfaces.control_data import (
    ControlInput,
    ControlMode,
    ControlOutput,
)
from robot_control.interfaces.controller import Controller
from robot_control.tracking.task_space_error import TaskSpaceError

Array = NDArray[np.float64]


class JacobianController(Controller):
    """Compute generalized velocities using damped least squares.

    The controller supports arbitrary robot velocity dimensions
    and configurable task-space selection.

    Kinematics and Jacobians are supplied by robot_models.
    """

    def __init__(
        self,
        task: TaskSpaceError,
        gains: Array,
        damping: float,
    ) -> None:
        """Configure the tracking task, gains, and damping."""
        if not isinstance(task, TaskSpaceError):
            raise TypeError("task must be a TaskSpaceError")

        self._task = task

        gains = np.array(gains, dtype=np.float64, copy=True)

        task_dimension = task.selection_matrix.shape[0]

        if gains.shape != (task_dimension, task_dimension):
            raise ValueError(
                "gains must have shape "
                f"({task_dimension}, {task_dimension})"
            )

        if not np.all(np.isfinite(gains)):
            raise ValueError("gains must be finite")

        if not np.allclose(gains, gains.T):
            raise ValueError("gains must be symmetric")

        if np.any(np.linalg.eigvalsh(gains) <= 0.0):
            raise ValueError("gains must be positive definite")

        if not np.isfinite(damping) or damping <= 0.0:
            raise ValueError(
                "damping must be positive and finite"
            )

        self._gains = gains
        self._damping = float(damping)

    def compute(
        self,
        control_input: ControlInput,
        model: RobotModel,
        model_data: ModelData,
    ) -> ControlOutput:
        """Compute a generalized velocity tracking command.

        Args:
            control_input: Current robot state and reference contract.
            model: Robot model defining velocity dimensions.
            model_data: Current and desired task-space model data.

        Returns:
            Generalized velocity command for the whole robot.
        """
        if not isinstance(control_input, ControlInput):
            raise TypeError("control_input must be ControlInput")

        if not isinstance(model_data, ModelData):
            raise TypeError("model_data must be ModelData")

        velocity_dimension = model.generalized_velocity_dimension

        robot_velocity = control_input.robot_state.velocity

        if robot_velocity.shape != (velocity_dimension,):
            raise ValueError(
                "Robot velocity dimension does not match model"
            )

        jacobian = np.asarray(
            model_data.whole_body_jacobian_world,
            dtype=np.float64,
        )

        if jacobian.shape != (6, velocity_dimension):
            raise ValueError(
                "Whole-body Jacobian dimensions do not match model"
            )

        if not np.all(np.isfinite(jacobian)):
            raise ValueError("Jacobian must be finite")

        desired_twist = np.asarray(
            model_data.desired_end_effector_twist_world,
            dtype=np.float64,
        )

        if not np.all(np.isfinite(desired_twist)):
            raise ValueError("Desired twist must be finite")

        task_error = self._task.compute(
            current_pose=model_data.end_effector_transform_world,
            desired_pose=(
                model_data.desired_end_effector_transform_world
            ),
        )

        selected_jacobian = self._task.select_jacobian(
            jacobian
        )

        selected_twist = (
            self._task.selection_matrix @ desired_twist
        )

        desired_task_velocity = (
            selected_twist + self._gains @ task_error
        )

        task_dimension = selected_jacobian.shape[0]

        regularized_matrix = (
            selected_jacobian @ selected_jacobian.T
            + self._damping**2 * np.eye(task_dimension)
        )

        solution = np.linalg.solve(
            regularized_matrix,
            desired_task_velocity,
        )

        velocity_command = selected_jacobian.T @ solution

        if not np.all(np.isfinite(velocity_command)):
            raise ValueError("Computed velocity is not finite")

        return ControlOutput(
            command=velocity_command,
            mode=ControlMode.VELOCITY,
        )
