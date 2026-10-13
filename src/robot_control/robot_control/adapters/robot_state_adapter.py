
"""Convert estimated robot states into generic control data contracts."""

import numpy as np
from numpy.typing import NDArray

from robot_control.interfaces.control_data import RobotState

Array = NDArray[np.float64]


class RobotStateAdapter:
    """Map a differential-drive estimator state to a control state.

    The estimator state layout is:

        [x, y, yaw, linear_velocity, angular_velocity,
         joint_positions, joint_velocities]

    The number of joints is supplied through the adapter configuration.
    """

    BASE_STATE_DIMENSION = 5
    BASE_CONFIGURATION_DIMENSION = 3
    BASE_VELOCITY_DIMENSION = 2

    def __init__(self, joint_count: int) -> None:
        """Configure the estimator layout for the robot."""
        if isinstance(joint_count, (bool, np.bool_)) or not isinstance(
            joint_count, (int, np.integer)
        ) or joint_count <= 0:
            raise ValueError("joint_count must be a positive integer")

        self._joint_count = int(joint_count)

    @property
    def state_dimension(self) -> int:
        """Return the expected estimator state dimension."""
        return self.BASE_STATE_DIMENSION + 2 * self._joint_count

    def convert(
        self,
        estimated_state: Array,
        timestamp: float,
        frame_id: str,
    ) -> RobotState:
        """Convert an estimator state into a validated RobotState."""
        state = np.asarray(estimated_state, dtype=np.float64)

        if state.shape != (self.state_dimension,):
            raise ValueError(
                "Estimated robot state must have shape "
                f"({self.state_dimension},), got {state.shape}"
            )

        if not np.all(np.isfinite(state)):
            raise ValueError("Estimated robot state contains nonfinite values")

        joint_start = self.BASE_STATE_DIMENSION
        velocity_start = joint_start + self._joint_count

        base_configuration = state[
            :self.BASE_CONFIGURATION_DIMENSION
        ]
        base_velocity = state[
            self.BASE_CONFIGURATION_DIMENSION:
            self.BASE_STATE_DIMENSION
        ]

        joint_positions = state[joint_start:velocity_start]
        joint_velocities = state[velocity_start:]

        configuration = np.concatenate(
            (base_configuration, joint_positions)
        )

        velocity = np.concatenate(
            (base_velocity, joint_velocities)
        )

        return RobotState(
            configuration=configuration,
            velocity=velocity,
            timestamp=timestamp,
            frame_id=frame_id,
        )
