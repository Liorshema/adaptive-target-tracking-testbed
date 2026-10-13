
"""Validate whole-body Jacobian against numerical kinematics.

Uses the configured robot model, including its tracking frame.
No ROS nodes or Gazebo execution are required.
"""

import numpy as np
import pytest
from robot_bringup.model_factory import create_robot_model
from robot_models.whole_body.jacobian import WholeBodyJacobian
from robot_models.whole_body.kinematics import WholeBodyKinematics


def base_transform(pose):
    """Convert planar base pose [x, y, yaw] to SE(3)."""
    x, y, yaw = pose

    c = np.cos(yaw)
    s = np.sin(yaw)

    transform = np.eye(4)
    transform[:3, :3] = [
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ]
    transform[:3, 3] = [x, y, 0.0]

    return transform


def tracking_pose(robot, base_pose, joints):
    """Evaluate configured whole-body forward kinematics."""
    return WholeBodyKinematics.compute(
        robot_model=robot,
        transform_world_base=base_transform(base_pose),
        joint_positions=joints,
    )


def numerical_jacobian(robot, base_pose, joints, step=1e-6):
    """Estimate the spatial Jacobian by central differences."""
    base_dof = robot.base.velocity_dimension
    total_dof = robot.generalized_velocity_dimension

    result = np.zeros((6, total_dof))

    current = tracking_pose(robot, base_pose, joints)
    rotation = current[:3, :3]

    for index in range(total_dof):
        velocity = np.zeros(total_dof)
        velocity[index] = 1.0

        base_velocity = velocity[:base_dof]
        joint_velocity = velocity[base_dof:]

        base_rate = robot.base.pose_derivative(
            base_pose,
            base_velocity,
        )

        pose_plus = tracking_pose(
            robot,
            base_pose + step * base_rate,
            joints + step * joint_velocity,
        )

        pose_minus = tracking_pose(
            robot,
            base_pose - step * base_rate,
            joints - step * joint_velocity,
        )

        linear_velocity = (
            pose_plus[:3, 3] - pose_minus[:3, 3]
        ) / (2.0 * step)

        rotation_rate = (
            pose_plus[:3, :3] - pose_minus[:3, :3]
        ) / (2.0 * step)

        angular_skew = rotation_rate @ rotation.T

        angular_velocity = np.array([
            angular_skew[2, 1] - angular_skew[1, 2],
            angular_skew[0, 2] - angular_skew[2, 0],
            angular_skew[1, 0] - angular_skew[0, 1],
        ]) / 2.0

        result[:, index] = np.concatenate(
            (linear_velocity, angular_velocity)
        )

    return result


@pytest.mark.parametrize(
    "base_pose,joints",
    [
        (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ),
        (
            [1.0, -0.5, 0.4],
            [0.2, 0.1, -0.15, 0.05],
        ),
        (
            [-0.7, 1.2, -0.6],
            [-0.3, 0.25, 0.1, -0.2],
        ),
    ],
)
def test_whole_body_jacobian_numerically(base_pose, joints):
    """Compare analytical Jacobian with FK finite differences."""
    robot = create_robot_model()

    base_pose = np.asarray(base_pose, dtype=float)
    joints = np.asarray(joints, dtype=float)

    analytical = WholeBodyJacobian.compute(
        robot_model=robot,
        transform_world_base=base_transform(base_pose),
        joint_positions=joints,
    )

    numerical = numerical_jacobian(
        robot,
        base_pose,
        joints,
    )

    assert analytical.shape == (
        6,
        robot.generalized_velocity_dimension,
    )

    np.testing.assert_allclose(
        analytical,
        numerical,
        rtol=1e-4,
        atol=1e-5,
    )


def test_jacobian_closed_loop_tracking():
    """Verify that DLS reduces tracking error on the real robot model."""
    from robot_models.interfaces.model_data import ModelData

    from robot_control.baseline.jacobian_controller import JacobianController
    from robot_control.interfaces.control_data import (
        ControlInput,
        ControlMode,
        RobotState,
        TrackingReference,
    )
    from robot_control.tracking.task_space_error import TaskSpaceError

    robot = create_robot_model()

    base_pose = np.zeros(3, dtype=float)
    joints = np.zeros(robot.manipulator.dof, dtype=float)

    base_dof = robot.base.velocity_dimension
    velocity_dof = robot.generalized_velocity_dimension

    dt = 0.05
    steps = 40

    controller = JacobianController(
        task=TaskSpaceError.position_only(),
        gains=np.eye(3),
        damping=0.1,
    )

    initial_pose = tracking_pose(robot, base_pose, joints)
    initial_jacobian = WholeBodyJacobian.compute(
        robot_model=robot,
        transform_world_base=base_transform(base_pose),
        joint_positions=joints,
    )

    # A small reachable first-order displacement along
    # the robot's initial forward-velocity Jacobian column.
    desired_pose = initial_pose.copy()
    desired_pose[:3, 3] += (
        0.1 * initial_jacobian[:3, 0]
    )

    errors = []

    for step in range(steps):
        current_pose = tracking_pose(
            robot,
            base_pose,
            joints,
        )

        jacobian = WholeBodyJacobian.compute(
            robot_model=robot,
            transform_world_base=base_transform(base_pose),
            joint_positions=joints,
        )

        error = (
            desired_pose[:3, 3]
            - current_pose[:3, 3]
        )
        errors.append(np.linalg.norm(error))

        control_input = ControlInput(
            robot_state=RobotState(
                configuration=np.concatenate(
                    (base_pose, joints)
                ),
                velocity=np.zeros(velocity_dof),
                timestamp=step * dt,
                frame_id="world",
            ),
            reference=TrackingReference(
                position=desired_pose[:3, 3],
            ),
        )

        model_data = ModelData(
            end_effector_transform_world=current_pose,
            end_effector_twist_world=np.zeros(6),
            desired_end_effector_transform_world=desired_pose,
            desired_end_effector_twist_world=np.zeros(6),
            whole_body_jacobian_world=jacobian,
        )

        output = controller.compute(
            control_input=control_input,
            model=robot,
            model_data=model_data,
        )

        assert output.mode == ControlMode.VELOCITY
        assert output.command.shape == (velocity_dof,)
        assert np.all(np.isfinite(output.command))

        base_velocity = output.command[:base_dof]
        joint_velocity = output.command[base_dof:]

        base_pose += dt * robot.base.pose_derivative(
            base_pose,
            base_velocity,
        )
        joints += dt * joint_velocity

    final_pose = tracking_pose(
        robot,
        base_pose,
        joints,
    )
    final_error = np.linalg.norm(
        desired_pose[:3, 3] - final_pose[:3, 3]
    )

    print(f"\nInitial tracking error: {errors[0]:.6f} m")
    print(f"Final tracking error:   {final_error:.6f} m")

    assert errors[0] > 0.0
    assert final_error < 0.5 * errors[0]


def test_jacobian_closed_loop_with_safety():
    """Verify closed-loop tracking under combined safety constraints."""
    from robot_models.constraints.acceleration_limits import AccelerationLimits
    from robot_models.constraints.joint_limits import JointLimits
    from robot_models.constraints.velocity_limits import VelocityLimits
    from robot_models.interfaces.model_data import ModelData

    from robot_control.baseline.jacobian_controller import JacobianController
    from robot_control.interfaces.control_data import (
        ControlInput,
        ControlMode,
        RobotState,
        TrackingReference,
    )
    from robot_control.safety.constraint_bounds import (
        acceleration_bounds,
        joint_position_bounds,
        velocity_bounds,
    )
    from robot_control.safety.constraint_projector import ConstraintProjector
    from robot_control.tracking.task_space_error import TaskSpaceError

    robot = create_robot_model()

    base_dof = robot.base.velocity_dimension
    arm_dof = robot.manipulator.dof
    velocity_dof = robot.generalized_velocity_dimension

    base_pose = np.zeros(3)
    joints = np.zeros(arm_dof)

    dt = 0.05
    steps = 80

    controller = JacobianController(
        task=TaskSpaceError.position_only(),
        gains=np.eye(3),
        damping=0.1,
    )
    projector = ConstraintProjector()

    # Conservative test limits.
    velocity_limits = VelocityLimits(
        lower=np.concatenate((
            np.full(base_dof, -0.08),
            np.full(arm_dof, -0.05),
        )),
        upper=np.concatenate((
            np.full(base_dof, 0.08),
            np.full(arm_dof, 0.05),
        )),
    )

    acceleration_limits = AccelerationLimits(
        lower=np.concatenate((
            np.full(base_dof, -0.8),
            np.full(arm_dof, -100.0),
        )),
        upper=np.concatenate((
            np.full(base_dof, 0.8),
            np.full(arm_dof, 100.0),
        )),
    )

    joint_limits = JointLimits(
        lower=np.full(arm_dof, -0.015),
        upper=np.full(arm_dof, 0.015),
    )

    joint_indices = np.arange(
        base_dof,
        velocity_dof,
    )

    initial_pose = tracking_pose(robot, base_pose, joints)

    initial_jacobian = WholeBodyJacobian.compute(
        robot_model=robot,
        transform_world_base=base_transform(base_pose),
        joint_positions=joints,
    )

    # Same reachable reference construction as the existing test.
    desired_pose = initial_pose.copy()
    desired_pose[:3, 3] += (
        0.1 * initial_jacobian[:3, 0]
    )

    previous_velocity = np.zeros(velocity_dof)
    initial_error = np.linalg.norm(
        desired_pose[:3, 3] - initial_pose[:3, 3]
    )

    limited_steps = 0
    tolerance = 1e-9

    for step in range(steps):
        current_pose = tracking_pose(
            robot,
            base_pose,
            joints,
        )

        jacobian = WholeBodyJacobian.compute(
            robot_model=robot,
            transform_world_base=base_transform(base_pose),
            joint_positions=joints,
        )

        control_input = ControlInput(
            robot_state=RobotState(
                configuration=np.concatenate(
                    (base_pose, joints)
                ),
                velocity=previous_velocity,
                timestamp=step * dt,
                frame_id="world",
            ),
            reference=TrackingReference(
                position=desired_pose[:3, 3],
            ),
        )

        model_data = ModelData(
            end_effector_transform_world=current_pose,
            end_effector_twist_world=np.zeros(6),
            desired_end_effector_transform_world=desired_pose,
            desired_end_effector_twist_world=np.zeros(6),
            whole_body_jacobian_world=jacobian,
        )

        requested = controller.compute(
            control_input=control_input,
            model=robot,
            model_data=model_data,
        )

        assert requested.mode == ControlMode.VELOCITY

        bounds = [
            velocity_bounds(velocity_limits),
            acceleration_bounds(
                acceleration_limits,
                previous_velocity,
                dt,
            ),
            joint_position_bounds(
                joint_limits,
                joints,
                joint_indices,
                velocity_limits,
                dt,
            ),
        ]

        safe_output = projector.project(
            output=requested,
            bounds=bounds,
        )

        command = safe_output.command

        if not np.allclose(
            requested.command,
            command,
            atol=1e-12,
            rtol=0.0,
        ):
            limited_steps += 1

        # Verify velocity limits.
        assert np.all(
            command >= velocity_limits.lower - tolerance
        )
        assert np.all(
            command <= velocity_limits.upper + tolerance
        )

        # Verify acceleration limits.
        acceleration = (
            command - previous_velocity
        ) / dt

        assert np.all(
            acceleration
            >= acceleration_limits.lower - tolerance
        )
        assert np.all(
            acceleration
            <= acceleration_limits.upper + tolerance
        )

        base_velocity = command[:base_dof]
        joint_velocity = command[base_dof:]

        predicted_joints = joints + dt * joint_velocity

        # Verify next-step joint positions.
        assert np.all(
            predicted_joints >= joint_limits.lower - tolerance
        )
        assert np.all(
            predicted_joints <= joint_limits.upper + tolerance
        )

        # Apply the admissible command to the kinematic model.
        base_pose += dt * robot.base.pose_derivative(
            base_pose,
            base_velocity,
        )

        joints = predicted_joints
        previous_velocity = command.copy()

    final_pose = tracking_pose(
        robot,
        base_pose,
        joints,
    )

    final_error = np.linalg.norm(
        desired_pose[:3, 3] - final_pose[:3, 3]
    )

    print(
        f"\nInitial tracking error: {initial_error:.6f} m"
    )
    print(
        f"Final tracking error:   {final_error:.6f} m"
    )
    print(
        f"Safety-limited steps:   {limited_steps}/{steps}"
    )

    assert initial_error > 0.0
    assert limited_steps > 0
    assert final_error < 0.7 * initial_error
