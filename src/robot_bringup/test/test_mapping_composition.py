"""Mapping assembly preserves the file-based preflight contracts."""
from copy import deepcopy
from pathlib import Path

import pytest

from robot_bringup.factory import system_configuration as configuration
from functools import partial
from robot_experiments.composition import load_resource
from robot_experiments.configuration import resolve_system_resource
load_yaml = partial(load_resource, share=lambda package: Path(
    __file__).resolve().parents[2] / package)

SRC = Path(__file__).resolve().parents[2]


def share(name):
    return SRC / name


def test_mapping_and_file_share_validation_and_do_not_mutate_input():
    source = resolve_system_resource(
        share('robot_experiments') /
        'config/systems/nominal_tracking.yaml',
        Path.cwd(),
        lambda package: Path(__file__).resolve().parents[2] /
        package)
    document = load_yaml(source)
    original = deepcopy(document)
    expected = configuration.load_system_configuration(
        resolve_system_resource(source, Path.cwd(), share), share)
    assert hasattr(configuration, 'assemble_system_configuration')
    actual = configuration.assemble_system_configuration(document, source, share)
    for key in [
        'identities', 'selection', 'estimation_parameters', 'control_parameters', 'runtime'
    ]:
        assert actual[key] == expected[key]
    assert document == original
    assert actual['manifest']['robot']['description'] == expected['description_path']


def test_mapping_reference_origin_is_declaration_directory(tmp_path):
    source = resolve_system_resource(
        share('robot_experiments') /
        'config/systems/nominal_tracking.yaml',
        Path.cwd(),
        lambda package: Path(__file__).resolve().parents[2] /
        package)
    document = load_yaml(source)
    description = share('robot_description') / 'config/robot/robot.yaml'
    document['robot']['description'] = 'description.yaml'
    (tmp_path / 'description.yaml').symlink_to(description)
    assert hasattr(configuration, 'assemble_system_configuration')
    actual = configuration.assemble_system_configuration(
        document, tmp_path / 'catalog.yaml', share)
    assert actual['description_path'] == description.resolve()
    document['prediction']['type'] = 'unsupported'
    with pytest.raises(ValueError, match='prediction'):
        configuration.assemble_system_configuration(
            document, tmp_path / 'catalog.yaml', share)


def test_grouped_owner_defaults_require_explicit_control_selections():
    source = resolve_system_resource(
        share('robot_experiments') /
        'config/systems/nominal_tracking.yaml',
        Path.cwd(),
        lambda package: Path(__file__).resolve().parents[2] /
        package)
    document = load_yaml(source)
    assert document['control']['config'].get('group') == 'parameters'
    assert document['estimation']['robot']['config']['path'] == 'config/estimation.yaml'
    actual = configuration.load_system_configuration(
        resolve_system_resource(source, Path.cwd(), share), share)
    assert actual['control_parameters']['controller_type'] == document['control']['type']
    assert actual['strategy']['controller']['parameters']['motion_weights'] == (
        [1., 2., 3., 3., 3., 2.])
    document['control'].pop('reference')
    with pytest.raises(ValueError, match='reference'):
        configuration.assemble_system_configuration(document, source, share)


def test_grouped_owner_configuration_cannot_select_unknown_group():
    source = resolve_system_resource(
        share('robot_experiments') /
        'config/systems/nominal_tracking.yaml',
        Path.cwd(),
        lambda package: Path(__file__).resolve().parents[2] /
        package)
    document = load_yaml(source)
    document['estimation']['robot']['config']['group'] = 'nonexistent'
    with pytest.raises(ValueError, match='group'):
        configuration.assemble_system_configuration(document, source, share)


def test_grouped_estimator_description_binding_comes_from_selected_robot(tmp_path):
    import yaml
    source = resolve_system_resource(
        share('robot_experiments') /
        'config/systems/mobile_manipulator_pose_tracking.yaml',
        Path.cwd(),
        lambda package: Path(__file__).resolve().parents[2] /
        package)
    document = load_yaml(source)
    calibration = load_yaml(
        share('robot_experiments') / 'config/calibration/manipulation_pose.yaml')
    calibration.pop('robot_description_config', None)
    path = tmp_path / 'calibration.yaml'
    path.write_text(yaml.safe_dump(calibration))
    document['estimation']['calibration'] = str(path)
    actual = configuration.assemble_system_configuration(document, source, share)
    selected = load_yaml(actual['description_path'])
    assert load_yaml(actual['estimation_parameters']['robot_description_file']) == selected
    assert selected['robot']['manipulator']['joints']['wrist']['upper'] < .27


def test_explicit_group_survives_string_path_round_trip():
    """Normalized group selection cannot depend on a Python Path instance."""
    source = resolve_system_resource(
        share('robot_experiments') /
        'config/systems/nominal_tracking.yaml',
        Path.cwd(),
        lambda package: Path(__file__).resolve().parents[2] /
        package)
    expected = configuration.load_system_configuration(
        resolve_system_resource(source, Path.cwd(), share), share)
    document = deepcopy(expected['manifest'])
    for entry, key in [
        (document['control'], 'config'),
        (document['control'], 'strategy'),
        (document['estimation']['robot'], 'config'),
        (document['estimation']['target'], 'config'),
    ]:
        entry[key] = str(entry[key])
    actual = configuration.assemble_system_configuration(document, source, share)
    assert actual['control_parameters'] == expected['control_parameters']
    assert actual['estimation_parameters'] == expected['estimation_parameters']
    assert actual['strategy'] == expected['strategy']
