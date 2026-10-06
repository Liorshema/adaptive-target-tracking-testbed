from robot_prediction.history.vector_buffer import VectorBuffer


class TrajectoryBuffer(VectorBuffer):
    """Store a fixed-length history of target state vectors."""
