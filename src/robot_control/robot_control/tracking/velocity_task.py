"""Interchangeable task linearizations for whole-body velocity optimization."""

from robot_control.interfaces.velocity_task import TaskLinearization, VelocityTask


import numpy as np
from robot_models.task_space.distance_pointing import DistancePointingGeometry

from robot_control.tracking.task_space_error import TaskSpaceError


class CartesianVelocityTask(VelocityTask):
    """Adapt the existing pose-selection task to error-rate convention."""

    def __init__(self, selection_matrix):
        """Select Cartesian task components using the existing abstraction."""
        self.task = TaskSpaceError(np.asarray(selection_matrix))

    def evaluate(self, control_input, model_data):
        """Express current-minus-desired errors with spatial velocity derivatives."""
        error = -self.task.compute(model_data.end_effector_transform_world,
                                   model_data.desired_end_effector_transform_world)
        jacobian = self.task.select_jacobian(model_data.whole_body_jacobian_world)
        target_rate = -self.task.selection_matrix @ model_data.desired_end_effector_twist_world
        return TaskLinearization(error, jacobian, target_rate)


class DistancePointingTask(VelocityTask):
    """Select range and free-roll optical pointing using reusable geometry."""

    def __init__(self, **geometry_parameters):
        """Use externally supplied radius, axes and numerical chart tolerances."""
        self.geometry = DistancePointingGeometry(**geometry_parameters)

    def evaluate(self, control_input, model_data):
        """Linearize against the timestamp-consistent selected target state."""
        target = np.asarray(control_input.target_state, dtype=float)
        if target.shape != (6,):
            raise ValueError('Distance/pointing requires target [position, velocity]')
        error, jacobian, target_rate = self.geometry.evaluate(
            model_data.end_effector_transform_world, target[:3],
            model_data.whole_body_jacobian_world, target[3:])
        alternatives = ()
        if np.linalg.norm(error[1:]) >= np.pi - self.geometry.antipodal_tolerance:
            # Both signed charts describe the same antipodal direction. Let the
            # bounded optimizer select the chart compatible with available motion.
            alternate = error.copy()
            alternate[1:] *= -1
            alternatives = (alternate,)
        return TaskLinearization(error, jacobian, target_rate, alternatives)
