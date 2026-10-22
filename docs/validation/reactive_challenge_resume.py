"""Run only missing M08 variant/seed pairs after an interrupted study; retain originals."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from robot_experiments.configuration import load_experiment
from robot_experiments.reporting import create_study_directory, write_json
from robot_experiments.runner import run_one


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--previous',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    share=lambda package:Path(get_package_share_directory(package))
    spec=load_experiment(share('robot_experiments')/'config/reactive_manipulation.yaml',share)
    previous=json.loads((args.previous/'resolved_experiment.json').read_text())
    for key in ['duration','settling_time','seeds','metrics','runtime','execution']:
        if previous[key]!=spec[key]:
            raise ValueError(f'cannot resume changed experiment {key}')
    study=create_study_directory(args.output,'reactive_manipulation_resume')
    (study/'runs').mkdir()
    shutil.copyfile(spec['source'],study/'experiment.yaml')
    write_json(study/'resolved_experiment.json',spec)
    results=[]
    print(study,flush=True)
    for seed in spec['seeds']:
        for variant in spec['variants']:
            name=f'{variant["name"]}_seed{seed}'
            prior=args.previous/'runs'/name
            status_path=prior/'status.json'
            status=json.loads(status_path.read_text()) if status_path.exists() else {}
            if status.get('execution_status')=='completed':
                metadata=json.loads((prior/'metadata.json').read_text())
                assert metadata['variant']==variant['name'] and metadata['seed']==seed
                assert status['window']['window_end']-status['window']['window_start']==spec['duration']
                result={'variant':variant['name'],'seed':seed,'reused':str(prior.resolve()),
                        'status_sha256':hashlib.sha256(status_path.read_bytes()).hexdigest()}
            else:
                result=run_one(spec,variant,seed,study/'runs'/name,share)
            results.append(result)
            write_json(study/'resume_results.json',results)
            print(name,'reused' if 'reused' in result else result['execution_status'],flush=True)
    print(study,flush=True)


if __name__=='__main__':
    main()
