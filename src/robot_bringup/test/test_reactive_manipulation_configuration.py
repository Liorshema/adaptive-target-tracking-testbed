"""Challenge variants preserve authoritative limits and select complete deployments."""

from copy import deepcopy
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from robot_configuration.yaml_loader import load_yaml
from robot_bringup.factory.system_configuration import load_system_configuration
from robot_description.model_parameters import load_robot_model_parameters
from robot_experiments.configuration import load_experiment, materialize_run


SOURCE = Path(__file__).resolve().parents[2]


def share(package):
    """Resolve package resources without a build or ROS runtime."""
    return SOURCE / package


@pytest.mark.parametrize('degrees', [15, 30])
def test_wrist_variant_is_complete_description_with_only_position_bounds_changed(degrees):
    """Xacro and every plain YAML consumer must see identical owner parameters."""
    root = share('robot_description') / 'config/robot'
    nominal = load_yaml(root / 'robot.yaml')
    path = root / f'robot_wrist{degrees}.yaml'
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
    config = root / f'config/robot/robot_wrist{degrees}.yaml'
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
    root = share('robot_control') / 'config'
    expected = load_yaml(root / 'whole_body.yaml')
    expected['controller']['parameters']['motion_weights'][1] = 12.
    assert load_yaml(root / 'whole_body_manipulation_arm_priority.yaml') == expected


def test_nominal_remains_identical_and_challenge_direct_launch_has_physical_feedback():
    """Wrist selection must preserve challenge deployment, including enabled truth."""
    root = share('robot_bringup') / 'config'
    assert load_yaml(root / 'system_manipulation_A.yaml') == load_yaml(root / 'system_nominal.yaml')
    base_deployment = load_yaml(share('robot_simulation') / 'config/gazebo/robot_manipulation.yaml')
    for name in ['B', 'C', 'D', 'E']:
        plan = load_system_configuration(root / f'system_manipulation_{name}.yaml', share)
        assert plan['runtime']['enable_actuation'] is False
        scenario = load_yaml(plan['profile']['simulation_config'])
        assert scenario['tracking']['pose_source'] == 'physical'
        deployment = load_yaml(share('robot_simulation') / scenario['robot']['config'])
        expected = deepcopy(base_deployment)
        expected['robot']['description']['config'] = str(
            plan['description_path'].relative_to(share('robot_description')))
        assert deployment == expected
        assert deployment['robot']['ground_truth_odometry']['enabled'] is True
        expected_strategy = ('whole_body_manipulation_arm_priority.yaml'
                             if name == 'D' else 'whole_body.yaml')
        assert plan['strategy_path'].name == expected_strategy


@pytest.mark.parametrize('variant,degrees', [('C', 15), ('E', 30)])
def test_restricted_wrist_rejects_out_of_bounds_initial_hold_before_launch(
        tmp_path, variant, degrees):
    """A selected physical bound must reject a previously nominal-valid posture."""
    root = share('robot_bringup') / 'config'
    holding = tmp_path / 'holding.yaml'
    holding.write_text(yaml.safe_dump({
        'extends': str(share('robot_simulation') / 'config/gazebo/joint_position_hold.yaml'),
        'joint_position_hold': {'positions': {'wrist': math.radians(degrees) + .01}}}))
    scenario = tmp_path / 'scenario.yaml'
    scenario.write_text(yaml.safe_dump({
        'extends': str(share('robot_simulation') /
                       f'config/scenarios/reactive_manipulation_wrist{degrees}.yaml'),
        'robot': {'joint_position_hold': str(holding)}}))
    profile = tmp_path / 'profile.yaml'
    profile.write_text(yaml.safe_dump({
        'extends': str(root / f'reactive_manipulation_wrist{degrees}.yaml'),
        'simulation_config': str(scenario)}))
    manifest = tmp_path / 'system.yaml'
    manifest.write_text(yaml.safe_dump({
        'extends': str(root / f'system_manipulation_{variant}.yaml'),
        'simulation': {'config': str(profile)}}))
    with pytest.raises(ValueError, match=r'positions\.wrist=.*outside authoritative limits'):
        load_system_configuration(manifest, share)


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
    from robot_configuration.yaml_loader import load_yaml
    root = Path(__file__).resolve().parents[2]
    share = lambda name: root / name
    spec = load_experiment(root/'robot_experiments/config/reactive_manipulation.yaml', share)
    variant = next(v for v in spec['variants'] if v['name'] == 'B')
    result = materialize_run(variant,42,tmp_path,share,spec['runtime'])
    sim = load_yaml(result['simulation_path'])
    assert result['comparison_contract']['physical_context'] == sim['physical_context']
