"""Read-only actual-model feasibility study; run after source install/setup.bash.

Only the sibling JSON is written. Camera +X and control radius 1m are fixed.
A solution satisfies target = camera_position + 1m * camera_forward_axis.
Multi-start bounded optimization establishes examples, not infeasibility proofs.
No collision/dynamics/actuation/controller performance claims are made.
"""
from dataclasses import asdict
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares, minimize
import yaml

from robot_description.model_parameters import load_robot_model_parameters
from robot_bringup.factory.model_factory import create_robot_model
from robot_models.task_space.distance_pointing import DistancePointingGeometry
from robot_models.whole_body.jacobian import WholeBodyJacobian

ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT / 'src/robot_description/config/robot/robot.yaml'
DEPLOYMENT = ROOT / 'src/robot_simulation/config/gazebo/robot.yaml'
PARAMETERS = load_robot_model_parameters(DESCRIPTION)
ROBOT = create_robot_model(PARAMETERS)
ARM = ROBOT.manipulator
BASE_Z = yaml.safe_load(DEPLOYMENT.read_text())['robot']['pose'][2]
SHOULDER = np.array([0., 0., PARAMETERS.manipulator.body_height / 2 +
                     PARAMETERS.manipulator.arm_base_height])
GEOMETRY = DistancePointingGeometry(1., [1., 0., 0.], [0., -1., 0.])


def virtual_target(q):
    pose = ARM.forward_kinematics(q)
    return pose[:3, 3] + pose[:3, 0]


def virtual_jacobian(q):
    pose = ARM.forward_kinematics(q)
    jac = ARM.jacobian(q)
    return jac[:3] + np.cross(jac[3:].T, pose[:3, 0]).T


def reach(q):
    return np.linalg.norm(ARM.forward_kinematics(q)[:3, 3] - SHOULDER)


def describe(q, target):
    pose = ARM.forward_kinematics(q)
    error, jac, _ = GEOMETRY.evaluate(pose, target, ARM.jacobian(q), np.zeros(3))
    whole_j = WholeBodyJacobian.compute(ROBOT, np.eye(4), q)
    _, whole_task, _ = GEOMETRY.evaluate(pose, target, whole_j, np.zeros(3))
    arm_s = np.linalg.svd(jac, compute_uv=False)
    whole_s = np.linalg.svd(whole_task, compute_uv=False)
    return dict(q_rad=q.tolist(), q_degrees=np.rad2deg(q).tolist(),
                camera_base_xyz_m=pose[:3, 3].tolist(),
                virtual_target_residual_m=float(np.linalg.norm(virtual_target(q)-target)),
                distance_error_m=float(error[0]), pointing_error_degrees=float(np.rad2deg(np.linalg.norm(error[1:]))),
                shoulder_camera_reach_m=float(reach(q)),
                arm_task_singular_values=arm_s.tolist(), arm_task_condition=float(arm_s[0]/max(arm_s[-1], 1e-16)),
                whole_body_task_singular_values=whole_s.tolist(),
                whole_body_task_condition=float(whole_s[0]/max(whole_s[-1], 1e-16)))


def bounds(wrist):
    lo, hi = PARAMETERS.joint_limits.lower.copy(), PARAMETERS.joint_limits.upper.copy()
    lo[3], hi[3] = max(lo[3], -wrist), min(hi[3], wrist)
    return lo, hi


def seeds(wrist, azimuth):
    lo, hi = bounds(wrist)
    yaw = (azimuth + np.pi + np.pi) % (2*np.pi) - np.pi
    for angle in [azimuth, yaw]:
        for s, e, w in itertools.product([.03, .78, 1.54], [-1.54, 0., 1.54], [-.98*wrist, 0., .98*wrist]):
            yield np.clip([angle, s, e, w], lo + 1e-8, hi - 1e-8)


def solve(target, wrist, optimize_extrema=True, seed_override=None):
    lo, hi = bounds(wrist)
    azimuth = np.arctan2(target[1], target[0])
    starts = list(seeds(wrist, azimuth)) if seed_override is None else seed_override
    # Rank all start points with actual FK, retaining each shoulder/elbow family.
    ranked = sorted(starts, key=lambda q: np.linalg.norm(virtual_target(q)-target))
    chosen = []
    for angle in sorted(set(float(q[0]) for q in starts)):
        family = [q for q in ranked if float(q[0]) == angle]
        chosen.extend(family[:7])
        for seed in family:
            if all(np.linalg.norm(seed[1:]-old[1:]) > 1.2 for old in chosen
                   if float(old[0]) == angle):
                chosen.append(seed)
    solved = []
    for seed in chosen:
        result = least_squares(lambda q: virtual_target(q)-target, seed,
                               jac=virtual_jacobian, bounds=(lo, hi),
                               ftol=1e-11, xtol=1e-11, gtol=1e-11, max_nfev=220)
        solved.append(result.x)
    best = min(solved, key=lambda q: np.linalg.norm(virtual_target(q)-target))
    exact = [q for q in solved if np.linalg.norm(virtual_target(q)-target) < 1e-7]
    output = dict(feasible=bool(exact), best=describe(best, target))
    output['distinct_yaw_branch_examples'] = []
    for q in exact:
        if all(abs(q[0]-item['q_rad'][0]) > .1 for item in output['distinct_yaw_branch_examples']):
            output['distinct_yaw_branch_examples'].append(describe(q, target))
    if not exact:
        return output
    extrema = []
    for sign in [-1., 1.]:
        candidates = exact.copy()
        if optimize_extrema:
            for seed in sorted(exact, key=lambda q: sign*reach(q))[:3]:
                fit = minimize(lambda q: sign*reach(q), seed, method='SLSQP',
                               bounds=list(zip(lo, hi)),
                               constraints=[dict(type='eq', fun=lambda q: virtual_target(q)-target,
                                                 jac=virtual_jacobian)],
                               options=dict(maxiter=150, ftol=1e-12))
                if np.linalg.norm(virtual_target(fit.x)-target) < 1e-7:
                    candidates.append(fit.x)
        extrema.append(min(candidates, key=lambda q: sign*reach(q)))
    output['maximum_reach'] = describe(extrema[0], target)
    output['minimum_reach'] = describe(extrema[1], target)
    return output


def supplement_report(report):
    """Add selected schedule geometry and constructed yaw-limit examples."""
    report['selected_schedule'] = dict(low=dict(rho=.55, height_above_base=1.6, world_z=1.85),
                                     high=dict(rho=.6, height_above_base=1.85, world_z=2.1),
                                     ring=dict(rho=1.3, height_above_base=1.35, world_z=1.6))
    for name, wrist in [('nominal', 1.57), ('wrist30', np.pi/6), ('wrist15', np.pi/12)]:
        group = report['cases'][name]
        group['selected_ring'] = group['orbit']
        if 'selected_low' in group:
            continue
        target = np.array([.55*np.cos(.2), .55*np.sin(.2), 1.6])
        lo, hi = bounds(wrist)
        starting = [np.array(group[case][extreme]['q_rad'])
                    for case in ['low_common_a', 'low_common_b']
                    for extreme in ['minimum_reach', 'maximum_reach']]
        candidates = []
        for sign in [1., -1.]:
            valid = []
            for seed in starting:
                fit = minimize(lambda q: sign*reach(q), seed, method='SLSQP',
                               bounds=list(zip(lo, hi)),
                               constraints=[dict(type='eq', fun=lambda q: virtual_target(q)-target,
                                                 jac=virtual_jacobian)],
                               options=dict(maxiter=100, ftol=1e-12))
                if np.linalg.norm(virtual_target(fit.x)-target) < 1e-7:
                    valid.append(fit.x)
            if not valid:
                raise RuntimeError('selected low refinement failed: '+name)
            candidates.append(min(valid, key=lambda q: sign*reach(q)))
        group['selected_low'] = dict(feasible=True, best=describe(candidates[0], target),
                                    minimum_reach=describe(candidates[0], target),
                                    maximum_reach=describe(candidates[1], target),
                                    target_rho_m=.55, target_world_z_m=1.85, azimuth_rad=.2,
                                    target_base_xyz_m=target.tolist(),
                                    method='SLSQP exact constraint refinement from neighboring actual-model solutions')
    # A solution under tighter wrist bounds is also a valid nominal witness.
    for name, wrist in [('nominal', 1.57), ('wrist30', np.pi/6)]:
        lo, hi = bounds(wrist)
        for key, case in report['cases'][name].items():
            if not case['feasible']:
                continue
            for tighter in ['wrist30', 'wrist15']:
                other = report['cases'][tighter].get(key, {})
                if not other.get('feasible'):
                    continue
                for extreme, direction in [('minimum_reach', 1.), ('maximum_reach', -1.)]:
                    witness = other[extreme]
                    q = np.array(witness['q_rad'])
                    if (np.all(q >= lo-1e-8) and np.all(q <= hi+1e-8) and
                            direction*witness['shoulder_camera_reach_m'] <
                            direction*case[extreme]['shoulder_camera_reach_m']):
                        case[extreme] = witness.copy()
    report['selected_low_constructed_yaw_examples'] = []
    q = np.array(report['cases']['wrist15']['selected_low']['minimum_reach']['q_rad'])
    for az in [0., .2, 3.13, np.pi, -np.pi]:
        rotated = q.copy()
        rotated[0] = np.clip((az+2*np.pi) % (2*np.pi)-np.pi, -3.14, 3.14)
        target = np.array([.55*np.cos(az), .55*np.sin(az), 1.6])
        result = describe(rotated, target)
        result['azimuth_rad'] = az
        report['selected_low_constructed_yaw_examples'].append(result)
    q = np.array([-.8, .7, 1.2, .2])
    eps = 1e-6
    numeric = np.column_stack([(virtual_target(q+eps*e)-virtual_target(q-eps*e))/(2*eps)
                               for e in np.eye(4)])
    report['virtual_jacobian_finite_difference_max_difference'] = float(np.max(np.abs(numeric-virtual_jacobian(q))))
    return report


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    candidates = {
        'low_too_low': (.4, 1.4),
        'low_nominal': (.5, 1.5),
        'low_wrist30_boundary': (.4, 1.75),
        'selected_low': (.55, 1.85),
        'low_common_a': (.5, 1.85),
        'low_common_b': (.6, 1.85),
        'low_common_c': (.6, 1.9),
        'high': (.6, 2.1),
        'high_a': (.9, 2.0),
        'extension': (1.7, 1.2),
        'extension_low': (1.8, .8),
        'orbit': (1.3, 1.6),
        'ring_lower_alternative': (1.3, 1.25),
    }
    report = dict(
        source_description=str(DESCRIPTION.relative_to(ROOT)),
        source_sha256=hashlib.sha256(DESCRIPTION.read_bytes()).hexdigest(),
        deployment=str(DEPLOYMENT.relative_to(ROOT)),
        base_world_z_m=BASE_Z, shoulder_base_xyz_m=SHOULDER.tolist(),
        shoulder_world_z_m=float(BASE_Z+SHOULDER[2]), radius_m=1., camera_forward_axis=[1., 0., 0.],
        source_limits=asdict(PARAMETERS.joint_limits), joint_order=['base_yaw', 'shoulder', 'elbow', 'wrist'],
        method='actual robot_models FK/Jacobian; bounded multi-start scipy least_squares; SLSQP reach extrema',
        conditioning='Unscaled range[m] + pointing[rad] Jacobians; singular values depend on units. Whole body columns use [v,omega,qdot].',
        limitations=['Static kinematic existence only; no collision, full-link base clearance, dynamics, speed, acceleration, controller or estimator validation.',
                     'Optimization failures are evidence of difficulty, not a formal global infeasibility proof.',
                     'Yaw coordinate cannot remain continuous through a complete orbit with fixed base and physical +/-3.14 limit; base heading or branch management is required.'],
        cases={}, yaw_cases={})
    for name, wrist in [('nominal', 1.57), ('wrist30', np.pi/6), ('wrist15', np.pi/12)]:
        group = {}
        for case, (rho, world_z) in candidates.items():
            # Positive azimuth allows both signed radial branches within physical yaw bounds.
            az = .2
            target = np.array([rho*np.cos(az), rho*np.sin(az), world_z-BASE_Z])
            result = solve(target, wrist)
            result.update(target_rho_m=rho, target_world_z_m=world_z, azimuth_rad=az, target_base_xyz_m=target.tolist())
            group[case] = result
            print(name, case, result['feasible'], round(result['best']['virtual_target_residual_m'], 7),
                  'reach', round(result.get('minimum_reach', result['best'])['shoulder_camera_reach_m'], 4),
                  round(result.get('maximum_reach', result['best'])['shoulder_camera_reach_m'], 4), flush=True)
        report['cases'][name] = group
        yaw_group = {}
        for yaw in [-np.pi, -3.13, -2.8, 0., 2.8, 3.13, np.pi]:
            target = np.array([1.3*np.cos(yaw), 1.3*np.sin(yaw), 1.6-BASE_Z])
            result = solve(target, wrist, optimize_extrema=False)
            result['azimuth_rad'] = yaw
            yaw_group[str(yaw)] = result
        report['yaw_cases'][name] = yaw_group
    # Authoritative FK extrema: optimize reach independent of a task target.
    report['global_reach'] = {}
    for name, wrist in [('nominal', 1.57), ('wrist30', np.pi/6), ('wrist15', np.pi/12)]:
        lo, hi = bounds(wrist)
        optimized = []
        for sign in [1., -1.]:
            fits = [minimize(lambda q: sign*reach(q), seed, bounds=list(zip(lo, hi)),
                             method='L-BFGS-B') for seed in list(seeds(wrist, 0.))[::6]]
            q = min(fits, key=lambda fit: sign*reach(fit.x)).x
            optimized.append(dict(q_rad=q.tolist(), shoulder_camera_reach_m=float(reach(q))))
        report['global_reach'][name] = dict(minimum=optimized[0], maximum=optimized[1])
    def convert(value):
        if isinstance(value, np.ndarray): return value.tolist()
        if isinstance(value, np.generic): return value.item()
        raise TypeError(type(value).__name__)
    path = args.output
    supplement_report(report)
    path.write_text(json.dumps(report, indent=2, default=convert)+'\n')
    print(path)


if __name__ == '__main__':
    main()
