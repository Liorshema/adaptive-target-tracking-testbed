"""Check installed localization module/config origins and ordinary pipeline contracts."""
import argparse
from importlib import import_module
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from robot_bringup.factory.system_configuration import load_system_configuration
from robot_experiments.configuration import resolve_system_resource
from robot_configuration.yaml_loader import load_yaml
from robot_simulation.gazebo.physics_engine import physics_engine_library
import hashlib


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prefix', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--copied', action='store_true')
    a = p.parse_args()
    share = lambda package: Path(get_package_share_directory(package))
    origins = {}
    for name in ['robot_estimation.robot.planar_pose',
                 'robot_simulation.measurements.planar_localization',
                 'robot_simulation.nodes.localization_sensor']:
        path = Path(import_module(name).__file__).resolve()
        if a.copied:
            assert path.is_relative_to(a.prefix.resolve()), (name, path)
        origins[name] = str(path)
    base = load_system_configuration(resolve_system_resource(
        {'package': 'robot_experiments', 'path': 'config/systems/manipulation_D.yaml'},
        share('robot_experiments'), share), share)
    localized = load_system_configuration(resolve_system_resource(
        {'package': 'robot_experiments', 'path': 'config/systems/mobile_manipulator_pose_tracking.yaml'},
        share('robot_experiments'), share), share)
    assert base['control_parameters'] == localized['control_parameters']
    assert base['description_path'] == localized['description_path']
    assert base['identities'] == localized['identities']
    assert not base['estimation_parameters'].get('localization_enabled', False)
    assert localized['estimation_parameters']['localization_enabled'] is True
    sensor = share('robot_simulation')/'config/sensors/planar_localization.yaml'
    assert sensor.exists()
    assert load_yaml(sensor)['localization_sensor']['white_noise_std'][0] == .04
    assert (a.prefix/'robot_simulation/lib/robot_simulation/localization_sensor').is_file()
    binary = Path(physics_engine_library('dartsim_recovery'))
    a.output.write_text(json.dumps({'modules': origins,
        'sensor_config': str(sensor), 'ordinary_controller_input': localized['control_parameters']['robot_state_topic'],
        'estimation': localized['estimation_parameters'], 'identities': localized['identities'],
        'backend_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}, indent=2)+'\n')
    print('Localization module/config/entrypoint and ordinary baseline/physics contracts verified.')


if __name__ == '__main__':
    main()
