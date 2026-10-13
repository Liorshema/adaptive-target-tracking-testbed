
"""Convert ROS target-prediction messages into control data contracts."""

import numpy as np

from robot_interfaces.msg import PredictedTrajectory as PredictedTrajectoryMsg

from robot_control.interfaces.control_data import PredictedTargetTrajectory


class PredictionMessageAdapter:
    """Validate and convert timestamped ROS target trajectories."""

    def convert(
        self,
        message: PredictedTrajectoryMsg,
    ) -> PredictedTargetTrajectory:
        """Convert a ROS prediction into a validated target trajectory."""
        if not isinstance(message, PredictedTrajectoryMsg):
            raise TypeError("Expected a PredictedTrajectory ROS message")

        horizon = int(message.horizon)
        state_dimension = int(message.state_dimension)

        if horizon <= 0 or state_dimension <= 0:
            raise ValueError(
                "Prediction horizon and state dimension must be positive"
            )

        timestamps = np.asarray(message.timestamps, dtype=np.float64)
        states_flat = np.asarray(message.states, dtype=np.float64)

        if timestamps.shape != (horizon,):
            raise ValueError(
                f"Prediction timestamps must have shape ({horizon},)"
            )

        if states_flat.shape != (horizon * state_dimension,):
            raise ValueError(
                "Prediction states must contain "
                f"{horizon * state_dimension} elements"
            )

        if not np.all(np.isfinite(timestamps)):
            raise ValueError("Prediction timestamps contain nonfinite values")

        if not np.all(np.isfinite(states_flat)):
            raise ValueError("Prediction states contain nonfinite values")

        reference_timestamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )

        if not np.isfinite(reference_timestamp):
            raise ValueError("Prediction reference timestamp must be finite")

        if np.any(timestamps <= reference_timestamp):
            raise ValueError(
                "Prediction timestamps must follow the reference timestamp"
            )

        states = states_flat.reshape(horizon, state_dimension)

        return PredictedTargetTrajectory(
            states=states,
            timestamps=timestamps,
            frame_id=message.header.frame_id,
        )
