"""Task-space twist error utilities."""

import numpy as np


class TwistError:
    """Compute desired-minus-current frame twist error."""

    @staticmethod
    def compute(
        current_twist: np.ndarray,
        desired_twist: np.ndarray,
    ) -> np.ndarray:
        """Return 6D twist error e_V = V_d - V."""
        current_twist = np.asarray(
            current_twist,
            dtype=float,
        )

        desired_twist = np.asarray(
            desired_twist,
            dtype=float,
        )

        if current_twist.shape != (6,):
            raise ValueError(
                'current_twist must have shape (6,)'
            )

        if desired_twist.shape != (6,):
            raise ValueError(
                'desired_twist must have shape (6,)'
            )

        return (
            desired_twist
            - current_twist
        )
