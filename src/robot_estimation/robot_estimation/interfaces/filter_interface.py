from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from robot_estimation.measurement import Measurement
from robot_estimation.state import StateEstimate


Vector = NDArray[np.float64]


class FilterInterface(ABC):
    """
    Generic interface for recursive state-estimation filters.

    A filter performs two fundamental operations:

    1. Prediction:
       propagate the current estimate through the process model.

    2. Update:
       correct the predicted estimate using a measurement.
    """

    @property
    @abstractmethod
    def state(self) -> StateEstimate:
        """
        Return the current filter state.
        """
        raise NotImplementedError

    @abstractmethod
    def predict(
        self,
        dt: float,
        control: Optional[Vector] = None,
    ) -> StateEstimate:
        """
        Perform the prediction step.

        Parameters
        ----------
        dt:
            Prediction time step in seconds.

        control:
            Optional control-input vector.

        Returns
        -------
        StateEstimate
            Predicted state estimate.
        """
        raise NotImplementedError

    @abstractmethod
    def update(
        self,
        measurement: Measurement,
    ) -> StateEstimate:
        """
        Perform the measurement-update step.

        Parameters
        ----------
        measurement:
            Measurement used to correct the predicted state.

        Returns
        -------
        StateEstimate
            Updated state estimate.
        """
        raise NotImplementedError