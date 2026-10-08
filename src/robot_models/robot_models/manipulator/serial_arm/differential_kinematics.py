"""Differential inverse kinematics for the serial manipulator."""

import numpy as np


class DifferentialKinematics:
    """Map desired end-effector twist to joint velocities."""

    @staticmethod
    def solve(
        jacobian: np.ndarray,
        desired_twist: np.ndarray,
        damping: float = 0.0,
    ) -> np.ndarray:
        """
        Compute desired joint rates from a desired 6D twist.

        Uses the Moore-Penrose pseudoinverse when damping is zero.
        Uses damped least squares when damping is positive.
        """
        jacobian = np.asarray(jacobian, dtype=float)
        desired_twist = np.asarray(
            desired_twist,
            dtype=float,
        )

        if jacobian.ndim != 2:
            raise ValueError(
                'jacobian must be a 2D matrix'
            )

        if desired_twist.shape != (jacobian.shape[0],):
            raise ValueError(
                'desired_twist dimension must match jacobian rows'
            )

        if damping < 0.0:
            raise ValueError(
                'damping must be non-negative'
            )

        if damping == 0.0:
            jacobian_pinv = np.linalg.pinv(
                jacobian
            )

        else:
            task_dim = jacobian.shape[0]

            regularized = (
                jacobian @ jacobian.T
                + (damping ** 2)
                * np.eye(task_dim)
            )

            jacobian_pinv = (
                jacobian.T
                @ np.linalg.solve(
                    regularized,
                    np.eye(task_dim),
                )
            )

        return jacobian_pinv @ desired_twist
