"""Check a copied installed workspace without importing its source packages."""

import argparse
from importlib import import_module
from importlib.metadata import distribution
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml

ROOT = Path(__file__).resolve().parents[2]
ENTRYPOINTS = {
    'robot_estimation/estimation_node': 'robot_estimation.nodes.estimation_node:main',
    'robot_prediction/prediction_node': 'robot_prediction.nodes.prediction_node:main',
    'robot_control/control_node': 'robot_control.nodes.control_node:main',
    'robot_simulation/simulation_node': 'robot_simulation.nodes.simulation_node:main',
    'robot_simulation/velocity_actuation': 'robot_simulation.nodes.velocity_actuation:main',
    'robot_bringup/tracking_feedback_tf': 'robot_bringup.nodes.tracking_feedback_tf:main',
    'robot_bringup/tracking_visualization': 'robot_bringup.nodes.tracking_visualization:main',
}


def verify(prefix):
    plan = json.loads((ROOT / 'results/validation/m06_migration_map.json').read_text())
    result = dict(prefix=str(prefix), entrypoints={}, obsolete_imports_absent=[], resources=[], xacro=False)
    for qualified, expected in ENTRYPOINTS.items():
        package, executable = qualified.split('/')
        entries = [e for e in distribution(package).entry_points
                   if e.group == 'console_scripts' and e.name == executable]
        assert len(entries) == 1, qualified
        assert entries[0].value == expected, (entries[0], expected)
        assert callable(entries[0].load())
        actual = import_module(expected.split(':')[0])
        assert Path(actual.__file__).resolve().is_relative_to(prefix), actual.__file__
        result['entrypoints'][qualified] = expected
    for old, new in plan['modules'].items():
        canonical = import_module(new)
        try:
            import_module(old)
        except ModuleNotFoundError as error:
            assert old == error.name or old.startswith(error.name + '.'), (old, error)
        else:
            raise AssertionError('Obsolete internal API still installed: ' + old)
        assert Path(canonical.__file__).resolve().is_relative_to(prefix), canonical.__file__
        result['obsolete_imports_absent'].append(old)
    semantic = json.loads((ROOT / 'results/validation/m06_1_semantic_moves.json').read_text())
    canonical = set(semantic['module_import_replacements'].values())
    canonical.update(semantic['domain_record_destinations'])
    canonical.update('.'.join(Path(move['new']).with_suffix('').parts[2:])
                     for move in semantic['module_moves'] if 'classes' in move)
    for name in sorted(canonical):
        actual = import_module(name)
        assert Path(actual.__file__).resolve().is_relative_to(prefix), actual.__file__
    removed = list(semantic['module_import_replacements']) + ['robot_models.types']
    for name in removed:
        try:
            import_module(name)
        except ModuleNotFoundError as error:
            assert name == error.name or name.startswith(error.name + '.'), (name, error)
        else:
            raise AssertionError('Removed semantic path still installed: ' + name)
    result['semantic_imports'] = sorted(canonical)
    result['removed_semantic_imports'] = removed
    for package in sorted((ROOT / 'src').iterdir()):
        if not (package / 'package.xml').exists():
            continue
        share = Path(get_package_share_directory(package.name))
        assert share.resolve().is_relative_to(prefix), share
        for directory in ['config', 'urdf', 'models', 'worlds', 'launch']:
            for source in (package / directory).rglob('*'):
                if not source.is_file() or '__pycache__' in source.parts or source.name.startswith('.'):
                    continue
                target = share / source.relative_to(package)
                assert target.is_file() and target.read_bytes() == source.read_bytes(), target
                if source.suffix == '.yaml' and isinstance(yaml.safe_load(source.read_text()), dict):
                    import_module('robot_configuration.yaml_loader').load_yaml(target)
                result['resources'].append(str(target.relative_to(prefix)))
        assert not any(p.name.endswith(('.bak', '.backup', '.orig', '.pyc', '.pyo'))
                       for p in share.rglob('*')), share
    share = Path(get_package_share_directory('robot_description'))
    urdf = subprocess.run(['xacro', str(share / 'urdf/robot.urdf.xacro'),
                           'config_file:='+str(share / 'config/robot/robot.yaml')],
                          check=True, capture_output=True, text=True).stdout
    robot = ET.fromstring(urdf)
    physical = yaml.safe_load((share / 'config/robot/robot.yaml').read_text())['robot']['manipulator']['joints']
    for key, limits in physical.items():
        joint = robot.find("joint[@name='"+key+"_joint']")
        assert joint is not None
        for field in ['lower', 'upper', 'velocity', 'effort']:
            assert float(joint.find('limit').get(field)) == limits[field]
    result['xacro'] = True
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = verify(args.prefix.resolve())
    if args.output:
        args.output.write_text(json.dumps(result, indent=2)+'\n')
    print('Verified', len(result['entrypoints']), 'entry points,', len(result['obsolete_imports_absent']),
          'removed legacy APIs,', len(result['resources']), 'installed resources and Xacro limits.')
    print('Verified', len(result['semantic_imports']), 'semantic module locations and',
          len(result['removed_semantic_imports']), 'removed semantic imports.')
