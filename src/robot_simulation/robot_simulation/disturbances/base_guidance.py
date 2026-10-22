"""Base-relative radial/vertical restoring and circulation acceleration."""
import numpy as np

from robot_simulation.disturbances.phase_schedule import PhaseSchedule
from robot_simulation.state.physical_context import PhysicalBaseContext


class BaseRelativeGuidance:
    """Acceleration only; damping and radial Fight/Flight remain in target dynamics."""

    context_requirements = frozenset({'physical_base'})

    def __init__(self, radial_gain, height_gain, radial_epsilon, reference_frame, schedule):
        """Validate explicit constructor parameters."""
        values = [radial_gain, height_gain, radial_epsilon]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in values):
            raise ValueError('guidance gains/epsilon must be numbers')
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError('guidance gains/epsilon must be finite positive')
        if not isinstance(reference_frame, str) or not reference_frame.strip():
            raise ValueError('guidance reference frame required')
        self.radial_gain, self.height_gain, self.radial_epsilon = values
        self.reference_frame = reference_frame
        self.schedule = (schedule if isinstance(schedule, PhaseSchedule)
                         else PhaseSchedule(schedule))

    def acceleration_with_context(self, position, time, context):
        """Evaluate guide in the configured world frame against measured physical base."""
        if not isinstance(context, PhysicalBaseContext):
            raise ValueError('guidance requires a typed physical base context')
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('guidance position must be finite 3D')
        s = position-context.position_at(time, self.reference_frame)
        rho = np.hypot(s[0], s[1])
        radial = np.array([s[0], s[1], 0.])/np.hypot(rho, self.radial_epsilon)
        tangent = np.array([-radial[1], radial[0], 0.])
        radius, height, acceleration = self.schedule.values(time)
        return (-self.radial_gain*(rho-radius)*radial
                + np.array([0., 0., -self.height_gain*(s[2]-height)])
                + acceleration*tangent)
