"""Convert target trajectories between fixed inertial coordinate frames."""

from robot_control.interfaces.control_data import PredictedTargetTrajectory


def transform_target_trajectory(prediction, rotation, offset, frame_id):
    """Transform [position, velocity]; moving-frame transport terms are unsupported."""
    states = prediction.states.copy()

    if states.shape[1] != 6:
        raise ValueError(
            "Frame conversion requires target state "
            "[x, y, z, vx, vy, vz]"
        )

    states[:, :3] = states[:, :3] @ rotation.T + offset
    states[:, 3:6] = states[:, 3:6] @ rotation.T

    return PredictedTargetTrajectory(
        states=states,
        timestamps=prediction.timestamps,
        frame_id=frame_id,
    )
