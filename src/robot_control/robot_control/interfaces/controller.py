
"""Generic interface for robot controllers."""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_models.interfaces.model_data import ModelData
    from robot_models.interfaces.robot_model import RobotModel

    from robot_control.interfaces.control_data import (
        ControlInput,
        ControlOutput,
    )


class Controller(ABC):
    """Abstract interface for interchangeable control algorithms."""

    @abstractmethod
    def compute(
        self,
        control_input: "ControlInput",
        model: "RobotModel",
        model_data: "ModelData",
    ) -> "ControlOutput":
        """Compute a robot command from the current control input.

        Args:
            control_input: Robot state and tracking reference.
            model: Robot mathematical model.
            model_data: Model quantities evaluated at the current state.

        Returns:
            A generalized robot control command.
        """
        raise NotImplementedError
