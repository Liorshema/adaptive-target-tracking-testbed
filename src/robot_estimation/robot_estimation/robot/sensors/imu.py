import numpy as np
from numpy.typing import NDArray

from robot_estimation.measurement import Measurement
from robot_estimation.interfaces.sensor_interface import SensorInterface


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


class ImuSensor(SensorInterface):
    """
    IMU measurement adapter.

    Measurement vector
    ------------------
    z_imu = [a, omega]^T

    For a 3D IMU:

    z_imu =
        [a_x, a_y, a_z,
         omega_x, omega_y, omega_z]^T
    """

    def __init__(
        self,
        covariance: Matrix,
        spatial_dimension: int = 3,
    ) -> None:
        if spatial_dimension <= 0:
            raise ValueError(
                "spatial_dimension must be positive."
            )

        self._spatial_dimension = spatial_dimension
        self._covariance = np.asarray(
            covariance,
            dtype=float,
        )

        self._validate_covariance()

    @property
    def spatial_dimension(self) -> int:
        """Return the physical spatial dimension."""
        return self._spatial_dimension

    @property
    def measurement_dimension(self) -> int:
        """Return the IMU measurement dimension."""
        return 2 * self._spatial_dimension

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
        Convert raw IMU data into a Measurement.
        """
        raw_value = np.asarray(
            raw_value,
            dtype=float,
        )

        if raw_value.shape != (
            self.measurement_dimension,
        ):
            raise ValueError(
                "IMU measurement must have shape "
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
        Validate the configured IMU covariance matrix.
        """
        m = self.measurement_dimension

        if self._covariance.shape != (m, m):
            raise ValueError(
                f"IMU covariance must have shape ({m}, {m}), "
                f"got {self._covariance.shape}."
            )