from typing import Optional

import numpy as np
from numpy.typing import NDArray

from robot_estimation.interfaces.estimator_interface import EstimatorInterface
from robot_estimation.measurement import Measurement
from robot_estimation.state import StateEstimate
from robot_estimation.interfaces.filter_interface import FilterInterface


Vector = NDArray[np.float64]


class TargetStateEstimator(EstimatorInterface):
    """
    High-level target-state estimator.

    The estimator delegates the recursive estimation logic to a
    configured filter implementation such as a Kalman Filter or
    Extended Kalman Filter.
    """

    def __init__(
        self,
        filter_: FilterInterface,
    ) -> None:
        self._filter = filter_

    @property
    def state(self) -> StateEstimate:
        """
        Return the current target-state estimate.
        """
        return self._filter.state

    def predict(
        self,
        dt: float,
        control: Optional[Vector] = None,
    ) -> StateEstimate:
        """
        Predict the target state forward in time.
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive.")

        return self._filter.predict(
            dt=dt,
            control=control,
        )

    def update(
        self,
        measurement: Measurement,
    ) -> StateEstimate:
        """
        Update the target-state estimate using a target measurement.
        """
        return self._filter.update(measurement)