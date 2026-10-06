from robot_prediction.history.vector_buffer import VectorBuffer


class ResidualBuffer(VectorBuffer):
    """Store a fixed-length history of prediction residuals."""
