"""Check the tracking task against actual nonholonomic robot kinematics."""

import numpy as np
import pytest
from robot_bringup.factory.model_factory import create_robot_model
from robot_models.task_space.distance_pointing import DistancePointingGeometry
from robot_models.whole_body.jacobian import WholeBodyJacobian
from robot_models.whole_body.kinematics import WholeBodyKinematics
from scipy.spatial.transform import Rotation


@pytest.mark.parametrize('joints,pose_rank', [
    ([0., 0., 0., 0.], 4),
    ([0., .5235987756, -.4, 0.], 4),
    ([.3, .8, -.7, .4], 5),
    ([0., 1.4, -1.2, .8], 4),
])
@pytest.mark.parametrize('forward', [[0., 0., 1.], [1., 0., 0.]])
def test_actual_robot_task_rank_and_jacobian(joints, pose_rank, forward):
    """Six commands provide three tracking conditions, but only 4–5 pose DOFs."""
    model = create_robot_model()
    task = DistancePointingGeometry(1., forward, [0., -1., 0.])
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('z', .4).as_matrix()
    transform[:3, 3] = [.5, -.2, 0.]
    q = np.asarray(joints)
    target = np.array([2.1, .8, .6])
    pose = WholeBodyKinematics.compute(model, transform, q)
    spatial = WholeBodyJacobian.compute(model, transform, q)
    error, analytic, _ = task.evaluate(pose, target, spatial, np.zeros(3))
    assert np.linalg.matrix_rank(spatial) == pose_rank
    assert np.linalg.matrix_rank(analytic) == 3
    h = 1e-6
    numerical = np.empty_like(analytic)
    for column in range(model.generalized_velocity_dimension):
        values = []
        for step in [h, -h]:
            base, arm = transform.copy(), q.copy()
            if column == 0:
                base[:3, 3] += step * transform[:3, 0]
            elif column == 1:
                base[:3, :3] = (
                    Rotation.from_euler('z', step).as_matrix() @ transform[:3, :3])
            else:
                arm[column - 2] += step
            moved = WholeBodyKinematics.compute(model, base, arm)
            values.append(task.evaluate(moved, target, spatial, np.zeros(3))[0])
        numerical[:, column] = (values[0] - values[1]) / (2 * h)
    np.testing.assert_allclose(analytic, numerical, atol=2e-6)
    # A coordinate change must preserve the body-frame task and its rates.
    frame = Rotation.from_rotvec([.1, -.3, .2]).as_matrix()
    offset = np.array([.2, -.4, .1])
    changed_pose = pose.copy()
    changed_pose[:3, :3] = frame @ pose[:3, :3]
    changed_pose[:3, 3] = frame @ pose[:3, 3] + offset
    changed_jacobian = np.vstack([frame @ spatial[:3], frame @ spatial[3:]])
    changed = task.evaluate(changed_pose, frame @ target + offset,
                            changed_jacobian, np.zeros(3))
    np.testing.assert_allclose(changed[0], error, atol=1e-10)
    np.testing.assert_allclose(changed[1], analytic, atol=1e-10)
