"""Inventory must describe selected configuration, never invent live sensor health."""

import json
from copy import deepcopy
from pathlib import Path

import pytest

from robot_bringup.factory.system_configuration import load_system_configuration

from robot_configuration.yaml_loader import load_yaml

import yaml
from robot_experiments.configuration import resolve_system_resource


SRC = Path(__file__).resolve().parents[2]


def share(package):
    """Resolve fixtures from authoritative source packages, not an older install."""
    return SRC / package


def system(name='system_nominal.yaml'):
    """Choose a real M07 system fixture."""
    from robot_experiments.configuration import resolve_system_resource
    return resolve_system_resource(
        {'package': 'robot_bringup', 'path': 'config/'+name}, SRC, share)


def inventory(plan):
    """Require the new assembly output before examining source contracts."""
    assert 'sensor_inventory' in plan, 'M07 must expose its effective sensor selection'
    return plan['sensor_inventory']['sources']


def write_yaml(path, value):
    """Write isolated owner overrides without modifying installed resources."""
    from robot_experiments.reporting import plain
    path.write_text(yaml.safe_dump(plain(value)))
    return str(path)


def legacy_profile_fixture(tmp_path, source, profile):
    """Exercise saved schema-1 readers without shipping selection profiles."""
    document = load_yaml(source)
    document['schema_version'] = 1
    document['estimation'].pop('calibration', None)
    document['simulation'] = {'config': write_yaml(tmp_path / 'profile.yaml', profile)}
    return write_yaml(tmp_path / 'system.yaml', document)


def modified_localization(tmp_path, *, publisher=None, fusion=None, omit_sensor=False):
    """Change independent explicit publisher and fusion selections."""
    source = system('system_manipulation_D_localized.yaml')
    data = load_yaml(source)
    plan = load_system_configuration(resolve_system_resource(source, Path.cwd(), share), share)
    deployment = load_yaml(plan['profile']['simulation_config'])
    if omit_sensor:
        deployment.pop('localization_sensor_config')
        data['simulation'].pop('localization_sensor', None)
    elif publisher is not None:
        sensor = load_yaml(share('robot_simulation') / deployment['localization_sensor_config'])
        sensor['localization_sensor']['enabled'] = publisher
        path = write_yaml(tmp_path / 'sensor.yaml', sensor)
        deployment['localization_sensor_config'] = path
        data['simulation']['localization_sensor'] = path
    if fusion is not None:
        data['estimation']['robot']['parameters']['localization_enabled'] = fusion
    data['simulation']['config'] = write_yaml(tmp_path / 'simulation.yaml', deployment)
    return Path(write_yaml(tmp_path / 'system.yaml', data))


@pytest.mark.parametrize('name,wheel,holdback', [
    ('system_nominal.yaml', [1e-6, 1e-6], .03),
    ('system_manipulation_D_localized.yaml', [.02, .04], .08),
])
def test_inventory_uses_effective_overrides_not_base_estimator_defaults(name, wheel, holdback):
    """Base YAML values must not leak past the selected bringup calibration."""
    plan = load_system_configuration(
        resolve_system_resource(
            system(name), Path.cwd(), share), share)
    sources = inventory(plan)
    odometry = sources['wheel_odometry']
    assert odometry['measurement']['assumed_variance'] == wheel
    assert odometry['timing']['reorder_holdback'] == holdback
    assert odometry['producer']['rate_hz'] == 50.
    assert odometry['transport']['topic'] == plan['estimation_parameters']['wheel_odometry_topic']
    provenance = odometry['configuration']['parameter_sources']
    assert provenance['wheel_linear_velocity_variance'] == str(
        plan['profile']['estimation_overrides'])
    joints = sources['joint_feedback']
    assert joints['producer']['configured'] is True
    assert joints['fusion']['enabled'] is True
    assert joints['measurement']['position_variance'] == 1e-6
    assert joints['measurement']['joint_names'] == plan['estimation_parameters']['joint_names']
    assert joints['producer']['rate_hz'] == 'not configured'
    assert joints['safety_feedback']['required_for_active_actuation'] is True
    assert joints['safety_feedback']['actuation_enabled'] is False


def test_imu_subscription_does_not_imply_builtin_publisher_or_health():
    """A consumer without a built-in producer is external-dependent, not live."""
    plan = load_system_configuration(resolve_system_resource(system(), Path.cwd(), share), share)
    sources = inventory(plan)
    imu = sources['imu']
    assert imu['producer']['configured'] is False
    assert imu['fusion']['subscription_configured'] is True
    assert imu['fusion']['enabled'] is True
    assert imu['external_provider'] == 'required for observations'
    assert imu['measurement']['fields'] == ['angular_velocity.z']
    assert imu['measurement']['assumed_variance'] == [.01]
    assert imu['live_status'] == 'not observed'
    assert sources['external_planar_localization']['fusion']['enabled'] is False
    assert sources['external_planar_localization']['producer']['configured'] is False


def test_localized_inventory_preserves_generator_and_transported_covariance_contracts():
    """Keep generated imperfections distinct from incoming full pose covariance."""
    plan = load_system_configuration(
        resolve_system_resource(
            system('system_manipulation_D_localized.yaml'),
            Path.cwd(),
            share),
        share)
    source = inventory(plan)['external_planar_localization']
    assert source['producer']['configured'] is True
    assert source['fusion']['subscription_configured'] is True
    assert source['fusion']['enabled'] is True
    assert source['producer']['rate_hz'] == 10.
    assert source['simulated_effects']['latency'] == .04
    assert source['simulated_effects']['latency_jitter'] == .01
    assert source['simulated_effects']['dropout_probability'] == .05
    assert source['simulated_effects']['white_noise_std'] == [.04, .04, .03490658503988659]
    assert source['simulated_effects']['calibration_bias_std'] == [.01, .01, .008726646259971648]
    assert source['measurement']['covariance_source'] == 'message pose covariance: x, y, yaw'
    assert source['timing']['localization_max_age'] == .15
    assert source['transport']['frames'] == {
        'source': 'world', 'child': 'base_link', 'fusion': 'odom'}
    assert source['timing']['stamp'] == 'acquisition time retained through delayed delivery'


@pytest.mark.parametrize('publisher,fusion', [(False, True), (True, False), (False, False)])
def test_publisher_and_fusion_are_independent_configured_choices(tmp_path, publisher, fusion):
    """A publisher outage must not be mislabeled as disabled fusion."""
    plan = load_system_configuration(resolve_system_resource(modified_localization(
        tmp_path, publisher=publisher, fusion=fusion), Path.cwd(), share), share)
    source = inventory(plan)['external_planar_localization']
    assert source['producer']['configured'] is publisher
    assert source['fusion']['enabled'] is fusion
    assert source['fusion']['subscription_configured'] is fusion
    assert source['live_status'] == 'not observed'
    if fusion and not publisher:
        assert source['external_provider'] == 'required for observations'


def test_missing_optional_sensor_resource_is_external_not_fabricated(tmp_path):
    """Missing source settings cannot become invented rates or noise models."""
    plan = load_system_configuration(
        resolve_system_resource(
            modified_localization(
                tmp_path,
                omit_sensor=True),
            Path.cwd(),
            share),
        share)
    source = inventory(plan)['external_planar_localization']
    assert source['producer']['configured'] is False
    assert source['producer']['rate_hz'] == 'not configured'
    assert source['simulated_effects'] == 'not configured'
    assert source['configuration']['producer_resource'] == 'not configured'
    assert source['fusion']['enabled'] is True
    assert source['external_provider'] == 'required for observations'


@pytest.mark.parametrize('name,truth,pose_source,context', [
    ('system_nominal.yaml', False, 'wheel_odometry', False),
    ('system_manipulation_D_localized.yaml', True, 'physical', True),
])
def test_truth_context_and_derived_tf_are_not_independent_fused_sensors(
    name, truth, pose_source, context,
):
    """Physical truth may feed simulation context without direct EKF fusion."""
    sources = inventory(
        load_system_configuration(
            resolve_system_resource(
                system(name),
                Path.cwd(),
                share),
            share))
    base = sources['physical_base_context']
    assert base['producer']['configured'] is truth
    assert base['fusion']['enabled'] is False
    assert ('target contextual integration' in base['consumers']) is context
    tf = sources['tracking_frame_tf']
    assert tf['classification'] == 'derived tracking context'
    assert tf['fusion']['enabled'] is False
    assert tf['measurement']['pose_source'] == pose_source
    assert tf['producer']['configured'] is True
    assert sources['simulation_clock']['classification'] == 'timing infrastructure'
    assert sources['target_ground_truth']['classification'] == 'evaluation truth'
    target = sources['target_position']
    assert target['producer']['configured'] is True
    assert target['fusion']['enabled'] is True
    assert target['measurement']['generated_covariance'] == [
        [.01, 0., 0.], [0., .01, 0.], [0., 0., .01]]
    assert target['measurement']['assumed_variance'] == [.01, .01, .01]


def test_inventory_uses_changed_topics_and_last_override_resource(tmp_path):
    """Inventory must follow the selected owner rather than scenario literals."""
    plan = load_system_configuration(
        resolve_system_resource(
            system('system_manipulation_D_localized.yaml'),
            Path.cwd(),
            share),
        share)
    profile = dict(plan['profile'])
    deployment = load_yaml(plan['profile']['simulation_config'])
    robot = load_yaml(share('robot_simulation') / deployment['robot']['config'])
    robot['robot']['differential_drive']['odom_topic'] = '/study/wheels'
    deployment['robot']['config'] = write_yaml(tmp_path / 'robot.yaml', robot)
    overrides = load_yaml(plan['profile']['estimation_overrides'])
    overrides.update(wheel_odometry_topic='/study/wheels', wheel_linear_velocity_variance=.123)
    override_path = write_yaml(tmp_path / 'calibration.yaml', overrides)
    profile['estimation_overrides'] = override_path
    profile['simulation_config'] = write_yaml(tmp_path / 'simulation.yaml', deployment)
    path = legacy_profile_fixture(
        tmp_path, system('system_manipulation_D_localized.yaml'), profile)
    wheel = inventory(
        load_system_configuration(
            resolve_system_resource(
                path,
                Path.cwd(),
                share),
            share))['wheel_odometry']
    assert wheel['transport']['topic'] == '/study/wheels'
    assert wheel['measurement']['assumed_variance'] == [.123, .04]
    assert (
        wheel['configuration']['parameter_sources']['wheel_linear_velocity_variance']
        == override_path)


def test_inventory_is_deterministic_json_metadata_and_does_not_mutate_inputs(monkeypatch):
    """Metadata construction and later consumer edits cannot alter owner state."""
    from robot_bringup.factory.sensor_inventory import build_sensor_inventory

    plan = load_system_configuration(resolve_system_resource(system(), Path.cwd(), share), share)
    deployment = load_yaml(plan['profile']['simulation_config'])
    inputs = {
        'deployment': deployment,
        'robot': load_yaml(share('robot_simulation') / deployment['robot']['config'])['robot'],
        'parameters': plan['estimation_parameters'],
        'measurements': load_yaml(
            share('robot_simulation') / deployment['simulation']['measurement_config']),
        'integration': load_yaml(share('robot_simulation') / deployment['simulation']['config']),
        'localization': None, 'runtime': plan['runtime'],
        'tracking_tf': plan['profile']['tracking_tf'],
        'actuation': load_yaml(plan['profile']['actuation_config'])['actuation'],
        'context_required': False,
        'resources': {'description': plan['description_path'],
                      'actuation': plan['profile']['actuation_config']},
        'parameter_sources': {},
    }
    before = deepcopy(inputs)

    def no_processes(*args, **kwargs):
        pytest.fail('Inventory construction must not start simulator or discovery processes')

    monkeypatch.setattr('subprocess.Popen', no_processes)
    first = build_sensor_inventory(**inputs)
    assert inputs == before
    first['sources']['joint_feedback']['measurement']['joint_names'].clear()
    first['sources']['target_position']['measurement']['generated_covariance'][0][0] = 999.
    assert inputs == before, 'Consumer edits to metadata must not mutate owner values'
    target = build_sensor_inventory(**inputs)['sources']['target_position']
    assert target['measurement']['generated_covariance'][0][0] == .01
    again = load_system_configuration(resolve_system_resource(system(), Path.cwd(), share), share)
    assert json.dumps(plan['sensor_inventory'], sort_keys=True) == json.dumps(
        again['sensor_inventory'], sort_keys=True)


def test_target_timing_does_not_invent_robot_buffer_age_admission():
    """The target filter has separate chronology, not robot-buffer age admission."""
    source = inventory(
        load_system_configuration(
            resolve_system_resource(
                system(),
                Path.cwd(),
                share),
            share))['target_position']
    assert source['timing']['admission_max_age'] == 'not configured'
    assert 'separate target filter chronology' in source['timing']['ordering']


def test_absent_calibration_override_reports_selected_estimator_owner(tmp_path):
    """Without a calibration overlay, both effective R and provenance revert."""
    plan = load_system_configuration(resolve_system_resource(system(), Path.cwd(), share), share)
    profile = dict(plan['profile'])
    profile.pop('estimation_overrides')
    path = legacy_profile_fixture(tmp_path, system(), profile)
    source = inventory(
        load_system_configuration(
            resolve_system_resource(
                path,
                Path.cwd(),
                share),
            share))['wheel_odometry']
    assert source['measurement']['assumed_variance'] == [.05, .02]
    assert source['configuration']['parameter_sources']['wheel_linear_velocity_variance'] == str(
        share('robot_estimation') / 'config/estimation.yaml')


def test_static_tracking_does_not_claim_tf_producer_selected(tmp_path):
    """An available broadcaster is not a selected producer in static mode."""
    plan = load_system_configuration(resolve_system_resource(system(), Path.cwd(), share), share)
    profile = dict(plan['profile'])
    deployment = load_yaml(plan['profile']['simulation_config'])
    deployment['tracking']['source'] = 'static'
    profile['simulation_config'] = write_yaml(tmp_path / 'simulation.yaml', deployment)
    path = legacy_profile_fixture(tmp_path, system(), profile)
    source = inventory(
        load_system_configuration(
            resolve_system_resource(
                path,
                Path.cwd(),
                share),
            share))['tracking_frame_tf']
    assert source['producer']['configured'] is False
    assert source['producer']['rate_hz'] == 'not applicable'
    assert source['measurement']['pose_source'] == 'static'


def test_summary_is_configured_not_live_and_shows_effective_observation_noise():
    """Startup output must qualify availability and fit a concise log entry."""
    from robot_bringup.factory.sensor_inventory import format_sensor_inventory

    plan = load_system_configuration(
        resolve_system_resource(
            system('system_manipulation_D_localized.yaml'),
            Path.cwd(),
            share),
        share)
    summary = format_sensor_inventory(plan['sensor_inventory'])
    assert 'not live' in summary
    assert 'external' in summary.lower()
    assert '0.02' in summary and '0.04' in summary
    assert len(summary.splitlines()) <= 4
