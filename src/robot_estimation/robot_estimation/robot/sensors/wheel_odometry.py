import numpy as np
from numpy.typing import NDArray

from robot_estimation.measurement import Measurement
from robot_estimation.interfaces.sensor_interface import SensorInterface


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


class WheelOdometrySensor(SensorInterface):
    """
    Wheel-odometry measurement adapter.

    Measurement vector
    ------------------
    z_odom = [v, omega]^T

    where

    v:
        Linear velocity.

    omega:
        Angular velocity.
    """

    def __init__(
        self,
        covariance: Matrix,
    ) -> None:
        self._covariance = np.asarray(
            covariance,
            dtype=float,
        )

        self._validate_covariance()

    @property
    def measurement_dimension(self) -> int:
        """Return the wheel-odometry measurement dimension."""
        return 2

    @property
    def covariance(self) -> Matrix:
        """Return a copy of the sensor covariance."""
        return self._covariance.copy()

    def create_measurement(
        self,
        raw_value: Vector,
        timestamp: float,
    ) -> Measurement:
        """
        Convert raw wheel-odometry data into a Measurement.
        """
        raw_value = np.asarray(
            raw_value,
            dtype=float,
        )

        if raw_value.shape != (
            self.measurement_dimension,
        ):
            raise ValueError(
                "Wheel-odometry measurement must have shape "
                f"({self.measurement_dimension},), "
                f"got {raw_value.shape}."
            )

        return Measurement(
            value=raw_value,
            covariance=self._covariance.copy(),
            timestamp=timestamp,
        )

    def _validate_covariance(self) -> None:
        """
        Validate the configured covariance matrix.
        """
        m = self.measurement_dimension

        if self._covariance.shape != (m, m):
            raise ValueError(
                f"Wheel-odometry covariance must have shape ({m}, {m}), "
                f"got {self._covariance.shape}."
            )