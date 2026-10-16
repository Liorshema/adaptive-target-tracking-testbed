"""Velocity-task contract and its linearization data."""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

@dataclass(frozen=True)
class TaskLinearization:
    """Error, robot-velocity derivative and target-only error derivative."""

    error: np.ndarray
    jacobian: np.ndarray
    target_rate: np.ndarray
    alternative_errors: tuple = ()



class VelocityTask(ABC):
    """Define a task without actuator, ROS or simulator assumptions."""

    @abstractmethod
    def evaluate(self, control_input, model_data):
        """Return a task linearization at the measured robot and selected target."""
