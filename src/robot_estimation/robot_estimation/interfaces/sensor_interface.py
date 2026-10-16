from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

from robot_estimation.measurement import Measurement


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


class SensorInterface(ABC):
    """
    Generic interface for sensor adapters.

    A sensor adapter converts raw sensor values into the common
    Measurement representation used by the estimation pipeline.
    """

    @property
    @abstractmethod
    def measurement_dimension(self) -> int:
        """
        Return the dimension of the produced measurement vector.
        """
        raise NotImplementedError

    @abstractmethod
    def create_measurement(
        self,
        raw_value: Vector,
        timestamp: float,
    ) -> Measurement:
        """
        Convert raw sensor data into a Measurement.

        Parameters
        ----------
        raw_value:
            Raw sensor vector.

        timestamp:
            Sensor timestamp in seconds.

        Returns
        -------
        Measurement
            Validated measurement with associated covariance.
        """
        raise NotImplementedError