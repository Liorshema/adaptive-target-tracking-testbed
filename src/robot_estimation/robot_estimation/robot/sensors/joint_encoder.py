import numpy as np
from numpy.typing import NDArray

from robot_estimation.measurement import Measurement
from robot_estimation.interfaces.sensor_interface import SensorInterface


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


class JointEncoderSensor(SensorInterface):
    """
    Joint-encoder measurement adapter.

    Measurement vector
    ------------------
    z_joint = [q, q_dot]^T

    where

    q in R^n
        Joint positions.

    q_dot in R^n
        Joint velocities.

    The complete measurement therefore has dimension 2n.
    """

    def __init__(
        self,
        joint_count: int,
        covariance: Matrix,
    ) -> None:
        if joint_count <= 0:
            raise ValueError(
                "joint_count must be positive."
            )

        self._joint_count = joint_count

        self._covariance = np.asarray(
            covariance,
            dtype=float,
        )

        self._validate_covariance()

    @property
    def joint_count(self) -> int:
        """Return the number of joints."""
        return self._joint_count

    @property
    def measurement_dimension(self) -> int:
        """Return the joint measurement dimension."""
        return 2 * self._joint_count

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
        Convert raw joint encoder data into a Measurement.
        """
        raw_value = np.asarray(
            raw_value,
            dtype=float,
        )

        if raw_value.shape != (
            self.measurement_dimension,
        ):
            raise ValueError(
                "Joint measurement must have shape "
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
                f"Joint covariance must have shape ({m}, {m}), "
                f"got {self._covariance.shape}."
            )