"""Installed experiment CLI; code 0 accepted, 1 interrupted, 2 invalid, 3 conditional."""
import argparse
from contextlib import redirect_stdout
import json
import sys
from pathlib import Path

from .configuration import load_experiment
from .explain import explain_experiment
from .runner import component_identities, run_experiment


def main(argv=None, *, share=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True,type=Path)
    parser.add_argument('--output',type=Path,help='Override study parent directory; never overwrites a study')
    parser.add_argument('--observer-diagnostics', action='store_true',
                        help='Record bounded observer diagnostics; scientific policy unchanged')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--validate-only',action='store_true')
    mode.add_argument('--explain',action='store_true',help='Inspect effective seeded inputs without launching')
    args=parser.parse_args(argv)
    if share is None:
        from ament_index_python.packages import get_package_share_directory
        share=lambda name:Path(get_package_share_directory(name))
    try:
        path=args.config.expanduser()
        if not path.is_file() and not path.is_absolute():path=share('robot_experiments')/path
        if args.explain:
            with redirect_stdout(sys.stderr):
                spec=load_experiment(path,share,args.output)
                explanation=explain_experiment(spec,share)
            print(json.dumps(explanation,indent=2,allow_nan=False))
            return 0
        spec=load_experiment(path,share,args.output)
        if args.validate_only:
            print(json.dumps({'schema_version':1,'valid':True,'variants':{v['name']:v['plan']['identities'] for v in spec['variants']}},indent=2))
            return 0
        if args.observer_diagnostics:
            directory,code=run_experiment(spec,share,observer_diagnostics=True)
        else:
            directory,code=run_experiment(spec,share)
        print(json.dumps({'schema_version':1,'results':str(directory),'exit_code':code}))
        return code
    except (ValueError,OSError,KeyError,TypeError) as exc:
        print(json.dumps({'schema_version':1,'error':str(exc)}),
              file=sys.stderr if args.explain else sys.stdout);return 2


if __name__=='__main__':
    raise SystemExit(main())
