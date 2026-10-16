from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from robot_estimation.measurement import Measurement
from robot_estimation.state import StateEstimate


Vector = NDArray[np.float64]


class EstimatorInterface(ABC):
    """
    Generic interface for state estimators.

    An estimator maintains an internal state estimate and supports
    prediction and measurement-update steps.
    """

    @property
    @abstractmethod
    def state(self) -> StateEstimate:
        """
        Return the current state estimate.
        """
        raise NotImplementedError

    @abstractmethod
    def predict(
        self,
        dt: float,
        control: Optional[Vector] = None,
    ) -> StateEstimate:
        """
        Predict the state forward in time.

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
        Update the state estimate using a measurement.

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