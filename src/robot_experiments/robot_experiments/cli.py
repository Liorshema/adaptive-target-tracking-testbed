"""Installed experiment CLI; code 0 accepted, 1 interrupted, 2 invalid, 3 conditional."""
import argparse
import json
from pathlib import Path

from .configuration import load_experiment
from .runner import component_identities, run_experiment


def main(argv=None, *, share=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True,type=Path)
    parser.add_argument('--output',type=Path,help='Override study parent directory; never overwrites a study')
    parser.add_argument('--validate-only',action='store_true')
    args=parser.parse_args(argv)
    if share is None:
        from ament_index_python.packages import get_package_share_directory
        share=lambda name:Path(get_package_share_directory(name))
    try:
        path=args.config.expanduser()
        if not path.is_file() and not path.is_absolute():path=share('robot_experiments')/path
        spec=load_experiment(path,share,args.output)
        if args.validate_only:
            print(json.dumps({'schema_version':1,'valid':True,'variants':{v['name']:v['plan']['identities'] for v in spec['variants']}},indent=2))
            return 0
        directory,code=run_experiment(spec,share)
        print(json.dumps({'schema_version':1,'results':str(directory),'exit_code':code}))
        return code
    except (ValueError,OSError,KeyError,TypeError) as exc:
        print(json.dumps({'schema_version':1,'error':str(exc)}));return 2


if __name__=='__main__':
    raise SystemExit(main())
