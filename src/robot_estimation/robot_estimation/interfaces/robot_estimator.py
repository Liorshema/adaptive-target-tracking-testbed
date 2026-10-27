"""Robot-specific operations required by the current sensor transport adapter."""

from typing import Protocol
from typing import runtime_checkable

from robot_estimation.measurement import Measurement
from robot_estimation.state import StateEstimate


@runtime_checkable
class RobotEstimatorProtocol(Protocol):
    """
    Planar mobile-manipulator estimate and typed sensor fusion operations.

    State layout is [x,y,yaw,v,omega,q[n],dq[n]]. Protocol conformance does not
    declare a different layout deployable: compatibility metadata checks that part.
    """

    @property
    def state(self) -> StateEstimate: ...

    @property
    def pose_diagnostics(self) -> dict: ...

    def predict(self, dt: float, control=None) -> StateEstimate: ...

    def update(self, measurement: Measurement) -> StateEstimate: ...

    def update_wheel_odometry(self, measurement: Measurement) -> StateEstimate: ...

    def update_imu(self, measurement: Measurement) -> StateEstimate: ...

    def update_joint_encoders(self, measurement: Measurement) -> StateEstimate: ...

    def update_planar_pose(self, measurement: Measurement, nis_limit: float) -> StateEstimate: ...

    def localization_health(self, now: float) -> dict: ...
