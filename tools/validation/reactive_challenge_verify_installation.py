"""Verify installed challenge resources outside the checkout, including M02/M09 gates."""

from robot_experiments.catalog import ExperimentCatalog
import argparse
from importlib import import_module
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from robot_configuration.yaml_loader import load_yaml
from robot_experiments.configuration import materialize_run


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    prefix=args.prefix.resolve()
    with tempfile.TemporaryDirectory(prefix='challenge_install_') as tmp:
        previous=Path(tmp)/'m02.json'
        subprocess.run([sys.executable,str(Path(__file__).with_name('m02_verify_installation.py')),
                        '--prefix',str(prefix),'--output',str(previous)],check=True)
        result=json.loads(previous.read_text())
        for name in ['robot_simulation.disturbances.base_guidance',
                     'robot_simulation.disturbances.phase_schedule',
                     'robot_simulation.state.physical_context',
                     'robot_experiments.metrics.manipulation']:
            assert Path(import_module(name).__file__).resolve().is_relative_to(prefix)
        share=lambda name:Path(get_package_share_directory(name))
        spec=ExperimentCatalog(share=share).prepare('EXP-MANIPULATION')
        variants={}
        for variant in spec['variants']:
            directory=Path(tmp)/variant['name'];directory.mkdir()
            run=materialize_run(variant,42,directory,share,spec['runtime'],record_actuator_trace=True)
            physical=run['context']['description']['robot']['manipulator']['joints']['wrist']
            sim=load_yaml(run['simulation_path'])
            variants[variant['name']]={'wrist_bounds':[physical['lower'],physical['upper']],
                'strategy':str(variant['plan']['strategy_path']),
                'pose_source':sim['tracking'].get('pose_source','wheel_odometry'),
                'physical_context':sim.get('physical_context'),
                'description_selected_consistently':True}
        assert variants['A']['wrist_bounds']==variants['B']['wrist_bounds']
        assert variants['C']['wrist_bounds']==variants['D']['wrist_bounds']
        assert variants['C']['wrist_bounds'][1]<variants['E']['wrist_bounds'][1]<variants['A']['wrist_bounds'][1]
        assert variants['D']['strategy']!=variants['C']['strategy']
        assert (share('robot_experiments')/'config/manipulation_analysis.yaml').is_file()
        result['reactive_challenge']={'variants':variants,'all_modules_from_copied_prefix':True,
                                      'trace_is_run_local':True}
        args.output.write_text(json.dumps(result,indent=2)+'\n')
    print('Copied ten-package M02/M09 checks and all five challenge variants verified.')


if __name__=='__main__':
    main()
