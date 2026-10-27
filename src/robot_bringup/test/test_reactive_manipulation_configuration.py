"""Challenge variants preserve authoritative limits and select complete deployments."""

from copy import deepcopy
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from robot_experiments.launching import launch_profile
from robot_bringup.factory.system_configuration import load_system_configuration
from robot_description.model_parameters import load_robot_model_parameters
from robot_experiments.configuration import load_experiment, materialize_run
from functools import partial
from robot_experiments.composition import load_resource
from robot_experiments.configuration import resolve_system_resource
load_yaml = partial(load_resource, share=lambda package: Path(__file__).resolve().parents[2] / package)


SOURCE = Path(__file__).resolve().parents[2]


def share(package):
    """Resolve package resources without a build or ROS runtime."""
    return SOURCE / package


@pytest.mark.parametrize('degrees', [15, 30])
def test_wrist_variant_is_complete_description_with_only_position_bounds_changed(degrees):
    """Xacro and every plain YAML consumer must see identical owner parameters."""
    root = share('robot_description') / 'config/robot'
    nominal = load_yaml(root / 'robot.yaml')
    path = resolve_system_resource(root / f'robot_wrist{degrees}.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package)
    actual = yaml.safe_load(path.read_text())
    expected = deepcopy(nominal)
    expected['robot']['manipulator']['joints']['wrist'].update(
        lower=-math.radians(degrees), upper=math.radians(degrees))
    assert actual == expected
    parameters = load_robot_model_parameters(path)
    np.testing.assert_allclose(parameters.joint_limits.lower[-1], -math.radians(degrees))
    np.testing.assert_array_equal(parameters.joint_limits.velocity, [1., 1., 1., 1.5])


@pytest.mark.parametrize('degrees', [15, 30])
def test_wrist_description_renders_matching_physical_urdf_limits(degrees):
    """The simulator's unchanged xacro reader must honor the selected owner bounds."""
    xacro = pytest.importorskip('xacro')
    root = share('robot_description')
    config = resolve_system_resource(root / f'config/robot/robot_wrist{degrees}.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package)
    document = xacro.process_file(
        str(root / 'urdf/robot.urdf.xacro'), mappings={'config_file': str(config)})
    physical = ET.fromstring(document.toxml())
    wrist_limit = physical.find("joint[@name='wrist_joint']/limit")
    assert wrist_limit is not None
    assert float(wrist_limit.attrib['lower']) == -math.radians(degrees)
    assert float(wrist_limit.attrib['upper']) == math.radians(degrees)
    assert float(wrist_limit.attrib['velocity']) == 1.5
    assert float(wrist_limit.attrib['effort']) == 10.


def test_arm_priority_changes_only_normalized_base_yaw_cost():
    """The experiment cannot alter task gains, physical bounds or wrist penalties."""
    expected = launch_profile(
        'closed_loop', lambda p: SOURCE / p, controller='whole_body')['strategy']
    expected['controller']['parameters']['motion_weights'][1] = 12.
    assert launch_profile(
        'closed_loop', lambda p: SOURCE / p,
        controller='whole_body_manipulation_arm_priority')['strategy'] == expected


def test_nominal_remains_identical_and_challenge_direct_launch_has_physical_feedback():
    """Wrist selection must preserve challenge deployment, including enabled truth."""
    experiment = load_experiment(
        share('robot_experiments') / 'config/reactive_manipulation.yaml', share)
    nominal = next(v for v in experiment['variants'] if v['name'] == 'A')
    assert nominal['system_config'] == (
        resolve_system_resource(share('robot_experiments') / 'config/systems/nominal_tracking.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package))
    base_deployment = load_yaml(share('robot_simulation') / 'config/gazebo/robot_manipulation.yaml')
    for name in ['B', 'C', 'D', 'E']:
        plan = load_system_configuration(
            resolve_system_resource(share('robot_experiments') / f'config/systems/manipulation_{name}.yaml', Path.cwd(), share), share)
        assert plan['runtime']['enable_actuation'] is False
        scenario = load_yaml(plan['profile']['simulation_config'])
        assert scenario['tracking']['pose_source'] == 'physical'
        deployment = load_yaml(share('robot_simulation') / scenario['robot']['config'])
        expected = deepcopy(base_deployment)
        actual_description = deployment['robot']['description']['config']
        assert (share('robot_description') / actual_description).resolve() == plan['description_path']
        expected['robot']['description']['config'] = actual_description
        assert deployment == expected
        assert deployment['robot']['ground_truth_odometry']['enabled'] is True
        assert plan['strategy_path'] == share('robot_control') / 'config/control.yaml'
        expected_yaw_cost = 12. if name == 'D' else 2.
        assert plan['strategy']['controller']['parameters']['motion_weights'][1] == (
            expected_yaw_cost)


@pytest.mark.parametrize('variant,degrees', [('C', 15), ('E', 30)])
def test_restricted_wrist_rejects_out_of_bounds_initial_hold_before_launch(
        tmp_path, variant, degrees):
    """A selected physical bound must reject a previously nominal-valid posture."""
    holding = tmp_path / 'holding.yaml'
    holding.write_text(yaml.safe_dump({
        'extends': str(share('robot_simulation') / 'config/gazebo/joint_position_hold.yaml'),
        'joint_position_hold': {'positions': {'wrist': math.radians(degrees) + .01}}}))
    scenario = tmp_path / 'scenario.yaml'
    scenario.write_text(yaml.safe_dump({
        'extends': str(resolve_system_resource(share('robot_simulation') /
                       f'config/scenarios/reactive_manipulation_wrist{degrees}.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package)),
        'robot': {'joint_position_hold': str(holding)}}))
    manifest = tmp_path / 'system.yaml'
    data = load_yaml(resolve_system_resource(share('robot_experiments') / f'config/systems/manipulation_{variant}.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package))
    data['simulation']['config'] = str(scenario)
    manifest.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match=r'positions\.wrist=.*outside authoritative limits'):
        load_system_configuration(resolve_system_resource(manifest, Path.cwd(), share), share)


def test_study_variants_preflight_and_materialize_consistent_physical_consumers(tmp_path):
    """The public M08 API must resolve all five M07 variants into replay inputs."""
    spec = load_experiment(share('robot_experiments') / 'config/reactive_manipulation.yaml', share)
    assert spec['duration'] == 40. and spec['settling_time'] == 0.
    assert spec['seeds'] == [42, 917]
    assert [variant['name'] for variant in spec['variants']] == ['A', 'B', 'C', 'D', 'E']
    contracts = {}
    for variant, wrist_degrees in zip(spec['variants'], [None, None, 15, 15, 30]):
        directory = tmp_path / variant['name']
        directory.mkdir()
        result = materialize_run(variant, 917, directory, share, spec['runtime'])
        plan = result['plan']
        physical_path = plan['description_path']
        assert Path(plan['control_parameters']['robot_description_file']) == physical_path
        assert Path(plan['estimation_parameters']['robot_description_file']) == physical_path
        simulation = load_yaml(result['simulation_path'])
        robot = load_yaml(simulation['robot']['config'])['robot']
        assert Path(robot['description']['config']) == physical_path
        description = load_yaml(physical_path)
        expected = 1.57 if wrist_degrees is None else math.radians(wrist_degrees)
        assert description['robot']['manipulator']['joints']['wrist']['upper'] == expected
        assert plan['runtime']['enable_actuation'] is True
        contracts[variant['name']] = result['comparison_contract']
    assert contracts['C'] == contracts['D']
    assert contracts['A'] != contracts['B']
    assert contracts['B'] != contracts['C']
    assert contracts['C'] != contracts['E']


def test_physical_context_is_part_of_m08_comparison_identity(tmp_path):
    from robot_experiments.configuration import load_experiment, materialize_run
    root = Path(__file__).resolve().parents[2]
    share = lambda name: root / name
    spec = load_experiment(root/'robot_experiments/config/reactive_manipulation.yaml', share)
    variant = next(v for v in spec['variants'] if v['name'] == 'B')
    result = materialize_run(variant,42,tmp_path,share,spec['runtime'])
    sim = load_yaml(result['simulation_path'])
    assert result['comparison_contract']['physical_context'] == sim['physical_context']
