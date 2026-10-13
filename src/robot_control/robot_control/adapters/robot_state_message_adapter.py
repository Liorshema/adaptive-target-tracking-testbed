
"""Convert ROS state-estimate messages into robot control contracts."""

import numpy as np

from robot_interfaces.msg import StateEstimate as StateEstimateMsg

from robot_control.adapters.robot_state_adapter import RobotStateAdapter
from robot_control.interfaces.control_data import RobotState


class RobotStateMessageAdapter:
    """Validate a ROS robot estimate and convert it to a control state."""

    def __init__(self, joint_count: int) -> None:
        """Configure the adapter for the estimator's robot-state layout."""
        self._state_adapter = RobotStateAdapter(joint_count=joint_count)

    def convert(self, message: StateEstimateMsg) -> RobotState:
        """Validate and convert a timestamped robot state estimate."""
        if not isinstance(message, StateEstimateMsg):
            raise TypeError("Expected a StateEstimate ROS message")

        state_dimension = self._state_adapter.state_dimension

        mean = np.asarray(message.mean, dtype=np.float64)
        covariance_flat = np.asarray(
            message.covariance,
            dtype=np.float64,
        )

        if mean.shape != (state_dimension,):
            raise ValueError(
                "Robot estimate mean must have shape "
                f"({state_dimension},), got {mean.shape}"
            )

        if covariance_flat.shape != (
            state_dimension * state_dimension,
        ):
            raise ValueError(
                "Robot estimate covariance must contain "
                f"{state_dimension * state_dimension} elements"
            )

        if not np.all(np.isfinite(mean)):
            raise ValueError("Robot estimate mean contains nonfinite values")

        if not np.all(np.isfinite(covariance_flat)):
            raise ValueError(
                "Robot estimate covariance contains nonfinite values"
            )

        covariance = covariance_flat.reshape(
            state_dimension,
            state_dimension,
        )

        if not np.allclose(
            covariance,
            covariance.T,
            rtol=1e-8,
            atol=1e-10,
        ):
            raise ValueError("Robot estimate covariance must be symmetric")

        if np.any(np.diag(covariance) < 0.0):
            raise ValueError(
                "Robot estimate covariance has negative diagonal values"
            )

        stamp = message.header.stamp

        if stamp.nanosec < 0 or stamp.nanosec >= 1_000_000_000:
            raise ValueError("Invalid ROS timestamp nanoseconds")

        timestamp = (
            float(stamp.sec)
            + float(stamp.nanosec) * 1e-9
        )

        frame_id = message.header.frame_id

        return self._state_adapter.convert(
            estimated_state=mean,
            timestamp=timestamp,
            frame_id=frame_id,
        )
