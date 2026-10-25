"""Reproduce single-mount, authoritative-model WLS allocation without ROS/Gazebo.

Run from any directory: python3 /path/to/ros2_ws/tools/validation/
reactive_challenge_allocation_verified.py

Output is written beside this script. The prior allocation_audit.json probe is
superseded because its ModelDataBuilder applied the tracking mount twice.
"""

import hashlib
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
for package in sorted((ROOT / 'src').iterdir()):
    if package.is_dir():
        sys.path.insert(0, str(package))

from robot_configuration.yaml_loader import load_yaml
from robot_bringup.factory.model_factory import create_robot_model
from robot_description.model_parameters import robot_model_parameters_from_description
from robot_control.factory.project_configuration import project_control_configuration
from robot_control.safety.safe_control_pipeline import SafeControlPipeline
from robot_models.constraints.velocity_limits import VelocityLimits
from robot_models.constraints.joint_limits import JointLimits
from robot_control.adapters.model_data_builder import ModelDataBuilder
from robot_control.tracking.velocity_task import DistancePointingTask
from robot_control.controllers.weighted_least_squares import WeightedLeastSquaresController
from robot_control.interfaces.control_data import ControlInput, RobotState, TrackingReference


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    """Solve the actual configured objective at four measured arm yaw positions."""
    description_path = ROOT / 'src/robot_description/config/robot/robot.yaml'
    control_path = ROOT / 'src/robot_control/config/control.yaml'
    nominal_path = ROOT / 'src/robot_control/config/whole_body.yaml'
    priority_path = ROOT / 'src/robot_control/config/whole_body_manipulation_arm_priority.yaml'
    existing_path = ROOT / 'src/robot_control/config/whole_body_arm_preferred.yaml'
    description = load_yaml(description_path)
    control = load_yaml(control_path)['control_node']['ros__parameters']
    nominal = load_yaml(nominal_path)
    model = create_robot_model(robot_model_parameters_from_description(description))
    _, bounds, _ = project_control_configuration(
        description, control['base_velocity_lower'], control['base_velocity_upper'])
    safety = SafeControlPipeline(
        VelocityLimits(np.asarray(bounds['velocity']['lower']), np.asarray(bounds['velocity']['upper'])),
        joint_limits=JointLimits(np.asarray(bounds['joint']['lower']), np.asarray(bounds['joint']['upper'])),
        joint_velocity_indices=np.asarray(bounds['joint']['velocity_indices']))
    # create_robot_model already wraps SerialArmModel in FixedMountModel.
    # Supplying a TrackingFrameKinematics here would incorrectly add it twice.
    builder = ModelDataBuilder(model)
    base = np.eye(4)
    base[:3, 3] = load_yaml(ROOT / 'src/robot_simulation/config/gazebo/robot.yaml')['robot']['pose'][:3]
    outputs = []
    for yaw in [0., 2.5, 3.13, 3.14]:
        q = np.array([yaw, .7, -.7, 0.])
        data = builder.build(base, q, np.zeros(6), np.eye(4), np.zeros(6))
        camera = data.end_effector_transform_world
        target = camera[:3, 3] + camera[:3, :3] @ np.array([np.cos(.1), np.sin(.1), 0.])
        inputs = ControlInput(
            RobotState(np.r_[base[:2, 3], 0., q], np.zeros(6), 0., 'world'),
            TrackingReference(target), target_state=np.r_[target, np.zeros(3)],
            dt=.02, previous_velocity=np.zeros(6))
        for policy, path in [('nominal', nominal_path), ('existing_arm_preferred', existing_path),
                             ('yaw_priority_candidate', priority_path)]:
            strategy = load_yaml(path)
            geometry = strategy['controller']['task']['parameters']
            task = DistancePointingTask(
                radius=control['tracking_radius'],
                forward_axis_local=control['tracking_forward_axis'],
                tangent_axis_local=control['tracking_up_axis'], **geometry)
            parameters = strategy['controller']['parameters']
            controller = WeightedLeastSquaresController(task, safety, **parameters)
            command = controller.compute(inputs, model, data).command
            linearization = task.evaluate(inputs, data)
            jacobian = linearization.jacobian
            lower, upper = safety.feasible_bounds(np.zeros(6), q, .02)
            assert np.all(np.isfinite(command))
            assert np.all(command >= lower - 1e-12) and np.all(command <= upper + 1e-12)
            np.testing.assert_allclose(jacobian[:, 1], jacobian[:, 2], atol=1e-12)
            fractions = np.minimum(q - safety.joint_limits.lower,
                                   safety.joint_limits.upper - q) / (
                                       safety.joint_limits.upper - safety.joint_limits.lower)
            costs = np.asarray(parameters['motion_weights'], dtype=float)
            costs[2:] *= 1 + parameters['limit_avoidance_weight'] / np.maximum(fractions, 1e-3)**2
            outputs.append({
                'policy': policy, 'arm_yaw_rad': yaw, 'joint_positions': q.tolist(),
                'motion_weights': parameters['motion_weights'],
                'effective_motion_weights': costs.tolist(),
                'camera_transform_world': camera.tolist(), 'target_world_m': target.tolist(),
                'task_error': linearization.error.tolist(),
                'task_target_rate': linearization.target_rate.tolist(),
                'task_jacobian': jacobian.tolist(), 'command_v_omega_dq': command.tolist(),
                'arm_fraction_of_abs_yaw_speed': float(abs(command[2]) / (
                    abs(command[1]) + abs(command[2]))),
                'yaw_task_rate_base': (jacobian[:, 1] * command[1]).tolist(),
                'yaw_task_rate_arm': (jacobian[:, 2] * command[2]).tolist(),
                'task_residual_norm': controller.last_diagnostics['task_residual'],
                'feasible_lower': lower.tolist(), 'feasible_upper': upper.tolist(),
                'yaw_columns_equal': True})
    sources = [description_path, control_path, nominal_path, priority_path, existing_path,
               ROOT / 'src/robot_bringup/robot_bringup/factory/model_factory.py',
               ROOT / 'src/robot_models/robot_models/factory/whole_body_factory.py',
               ROOT / 'src/robot_control/robot_control/controllers/weighted_least_squares.py',
               ROOT / 'src/robot_control/robot_control/adapters/model_data_builder.py']
    report = {
        'schema_version': 1, 'authoritative': True,
        'supersedes': 'reactive_challenge_allocation_audit.json',
        'superseded_reason': 'Earlier probe applied the tracking mount a second time in ModelDataBuilder.',
        'reproducer': Path(__file__).name,
        'source_sha256': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in sources},
        'mount_application_count': 1,
        'model_construction': 'create_robot_model(description parameters) includes FixedMountModel; ModelDataBuilder(model) adds no mount.',
        'generalized_order': ['base_linear_m_s', 'base_yaw_rad_s', 'arm_yaw_rad_s',
                              'shoulder_rad_s', 'elbow_rad_s', 'wrist_rad_s'],
        'velocity_scales': np.maximum(abs(safety.velocity_limits.lower),
                                      abs(safety.velocity_limits.upper)).tolist(),
        'setup': {'base_transform_world': base.tolist(), 'dt_s': .02,
                  'previous_velocity': [0.] * 6, 'local_target_yaw_error_rad': .1,
                  'tracking_radius_m': control['tracking_radius'],
                  'nominal_strategy': nominal},
        'objective': '||sqrt(task_weights)/task_scales * (J*u + gains*error + target_rate)||^2 + damping^2*sum(effective_motion_weights*(u/velocity_scales)^2) + smoothness_weight*sum(((u-previous)/velocity_scales)^2) + posture_weight*sum(((u_joint-posture_gain*(preferred_joint-q))/velocity_scales_joint)^2), bounded by safety.feasible_bounds.',
        'findings': [
            'Base yaw has lower normalized nominal damping cost than arm yaw; midpoint posture and smoothness additionally affect allocation.',
            'Changing only the base yaw weight from 2 to 12 shifts centered yaw demand toward arm yaw.',
            'At large arm yaw, midpoint posture may counterrotate arm and base; absolute speed share is descriptive, not a success metric.',
            'Near limits, increased arm cost and hard next-step bounds transfer outward yaw demand to the base.',
            'Jacobian task contributions are directly comparable here because the base/arm yaw columns coincide.'],
        'comparisons': outputs,
        'checks': {'solves': len(outputs), 'all_finite': True,
                   'all_commands_within_shared_feasible_bounds': True,
                   'all_yaw_columns_equal': True},
        'limitations': ['Single-step numerical allocation only; no physical dynamics/safety claim.',
                        'Existing arm_preferred remains historically rejected for measured wrist overspeed.',
                        'No mixed linear/angular activity norm is used.',
                        'Solver weighted_motion_cost excludes damping and other objective terms.']}
    output = args.output
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    for row in outputs:
        if row['policy'] != 'existing_arm_preferred':
            print(row['arm_yaw_rad'], row['policy'], row['command_v_omega_dq'][1:3],
                  row['arm_fraction_of_abs_yaw_speed'])
    print(output)


if __name__ == '__main__':
    main()
