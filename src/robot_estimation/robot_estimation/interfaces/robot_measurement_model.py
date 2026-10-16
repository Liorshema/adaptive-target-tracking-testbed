"""Measurement function and Jacobian contract for robot-state observations."""

from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]

class RobotMeasurementModel(ABC):
    """
    Base interface for robot measurement models.

    A measurement model maps the complete robot state into the
    measurement space of a particular sensor.
    """

    @property
    @abstractmethod
    def measurement_dimension(self) -> int:
        """Return the measurement dimension."""
        raise NotImplementedError

    @abstractmethod
    def predict_measurement(
        self,
        state: Vector,
    ) -> Vector:
        """Predict the sensor measurement from the robot state."""
        raise NotImplementedError

    @abstractmethod
    def jacobian(
        self,
        state: Vector,
    ) -> Matrix:
        """Return the measurement Jacobian."""
        raise NotImplementedError
