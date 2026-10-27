"""Validate copied-install backend selection, notices and existing package gates."""
import argparse
import hashlib
from importlib import import_module
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from functools import partial
from robot_experiments.composition import load_resource
load_yaml = partial(load_resource, share=lambda package: Path(get_package_share_directory(package)))
from robot_simulation.gazebo.physics_engine import physics_engine_library


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();prefix=args.prefix.resolve()
    with tempfile.TemporaryDirectory(prefix='recovery_install_') as tmp:
        previous=Path(tmp)/'previous.json'
        subprocess.run([sys.executable,str(Path(__file__).with_name('reactive_challenge_verify_installation.py')),
                        '--prefix',str(prefix),'--output',str(previous)],check=True)
        result=json.loads(previous.read_text())
    for name in ['robot_simulation.gazebo.physics_engine',
                 'robot_simulation.factory.physics_configuration']:
        assert Path(import_module(name).__file__).resolve().is_relative_to(prefix)
    share=Path(get_package_share_directory('robot_simulation')).resolve()
    binary=Path(physics_engine_library('dartsim_recovery'))
    assert share.is_relative_to(prefix) and binary.is_relative_to(prefix)
    notices=share/'third_party/dartsim_recovery'
    assert all((notices/name).is_file() for name in ['LICENSE','UPSTREAM.json','README.md'])
    provenance=json.loads((notices/'UPSTREAM.json').read_text())
    assert provenance['commit']=='2c0295ef56d09089a20be33344b659aa8654e287'
    selected=load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml',
                        'group': 'resources.deployment_standalone'})['world']['physics']['engine']
    assert selected=='dartsim_recovery'
    result['joint_limit_recovery']={'binary':str(binary),'sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
        'provenance':provenance,'selected_engine':selected,'notices_installed':True}
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print('Copied ten-package gates and recovery backend/provenance verified.')


if __name__=='__main__':main()
