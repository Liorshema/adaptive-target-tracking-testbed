"""Whole-body end-effector twist computation."""

import numpy as np


class WholeBodyTwist:
    """Compute end-effector twist from whole-body generalized velocity."""

    @staticmethod
    def compute(
        whole_body_jacobian: np.ndarray,
        generalized_velocity: np.ndarray,
    ) -> np.ndarray:
        """Compute end-effector twist V_E = J_WB @ nu."""
        whole_body_jacobian = np.asarray(
            whole_body_jacobian,
            dtype=float,
        )

        generalized_velocity = np.asarray(
            generalized_velocity,
            dtype=float,
        )

        if whole_body_jacobian.ndim != 2:
            raise ValueError(
                'whole_body_jacobian must be a 2D matrix'
            )

        if whole_body_jacobian.shape[0] != 6:
            raise ValueError(
                'whole_body_jacobian must have 6 rows'
            )

        if generalized_velocity.shape != (
            whole_body_jacobian.shape[1],
        ):
            raise ValueError(
                'generalized_velocity dimension must match '
                'jacobian columns'
            )

        return (
            whole_body_jacobian
            @ generalized_velocity
        )
