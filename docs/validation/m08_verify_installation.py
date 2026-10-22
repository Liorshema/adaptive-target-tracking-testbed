"""Verify ten copied packages and the experiment CLI outside the source checkout."""
import argparse
from importlib import import_module
from importlib.metadata import distribution
import json
from pathlib import Path
import subprocess
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from m07_verify_installation import run as verify_m07


def run(prefix):
    result=verify_m07(prefix)
    for name in ['configuration','execution','recording','metrics','benchmarking','reporting','runner','cli']:
        module=import_module('robot_experiments.'+name)
        assert Path(module.__file__).resolve().is_relative_to(prefix),module.__file__
    entries=[e for e in distribution('robot_experiments').entry_points if e.group=='console_scripts' and e.name=='run']
    assert len(entries)==1 and entries[0].value=='robot_experiments.cli:main'
    assert callable(entries[0].load())
    share=lambda name:Path(get_package_share_directory(name))
    from robot_experiments.configuration import load_experiment,materialize_run
    config=share('robot_experiments')/'config/paired_prediction.yaml'
    spec=load_experiment(config,share)
    with tempfile.TemporaryDirectory(prefix='m08-installed-') as temporary:
        prepared=materialize_run(spec['variants'][0],917,Path(temporary),share,spec['runtime'])
        assert prepared['snapshot']['seed_consumers']['measurements.target_position.random_seed']==917
        assert prepared['plan']['description_path'].is_relative_to(temporary)
    executable=Path(get_package_prefix('robot_experiments'))/'lib/robot_experiments/run'
    invocation=subprocess.run([str(executable),'--config','config/paired_prediction.yaml','--validate-only'],capture_output=True,text=True,check=True)
    parsed=json.loads(invocation.stdout)
    assert parsed['valid'] and len(parsed['variants'])==2
    result['experiment_cli']={'entry_point':entries[0].value,'validation':parsed,'seed_materialization':917}
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    result=run(args.prefix.resolve())
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print('Verified ten packages, eight console entries, installed examples, real component identities and seed 917 materialization.')
