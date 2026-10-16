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


def verify(prefix):
    plan = json.loads((ROOT / 'docs/validation/m06_migration_map.json').read_text())
    old_entries = json.loads((ROOT / 'docs/validation/m06_audit_verification.json').read_text())['entrypoints']
    result = dict(prefix=str(prefix), entrypoints={}, compatibility={}, resources=[], xacro=False)
    for qualified, old in old_entries.items():
        package, executable = qualified.split('/')
        entries = [e for e in distribution(package).entry_points
                   if e.group == 'console_scripts' and e.name == executable]
        assert len(entries) == 1, qualified
        module, function = old.split(':')
        expected = plan['modules'].get(module, module)+':'+function
        assert entries[0].value == expected, (entries[0], expected)
        assert callable(entries[0].load())
        actual = import_module(expected.split(':')[0])
        assert Path(actual.__file__).resolve().is_relative_to(prefix), actual.__file__
        result['entrypoints'][qualified] = expected
    for old, new in plan['modules'].items():
        legacy, canonical = import_module(old), import_module(new)
        assert legacy is canonical, (old, new)
        assert Path(canonical.__file__).resolve().is_relative_to(prefix), canonical.__file__
        result['compatibility'][old] = new
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
    args = parser.parse_args()
    result = verify(args.prefix.resolve())
    (ROOT / 'docs/validation/m06_installed_checks.json').write_text(json.dumps(result, indent=2)+'\n')
    print('Verified', len(result['entrypoints']), 'entry points,', len(result['compatibility']),
          'legacy identities,', len(result['resources']), 'installed resources and Xacro limits.')
