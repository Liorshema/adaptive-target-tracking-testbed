"""Installed experiment CLI; code 0 accepted, 1 interrupted, 2 invalid, 3 conditional."""
import argparse
from contextlib import redirect_stdout
import json
from pathlib import Path
import sys

from .configuration import load_experiment
from .explain import explain_experiment
from .runner import component_identities as component_identities
from .runner import run_experiment

__all__ = ['component_identities', 'main', 'manage']


def main(argv=None, *, share=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == 'compose':
        from .launching import compose
        return compose(argv[1:], share=share)
    if argv and argv[0] == 'components':
        from .components import main as component_main
        return component_main(argv[1:])
    if argv and argv[0] in {'catalog', 'start', 'status', 'stop', 'history'}:
        return manage(argv, share=share)
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument(
        '--config', type=Path, help='Import an M08 document or historical catalog entry')
    selection.add_argument('--experiment', help='Stable catalog experiment ID')
    parser.add_argument('--catalog', type=Path)
    parser.add_argument('--revision', type=int)
    parser.add_argument(
        '--output', type=Path,
        help='Override study parent directory; never overwrites a study')
    parser.add_argument(
        '--observer-diagnostics', action='store_true',
        help='Record bounded observer diagnostics; scientific policy unchanged')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--validate-only', action='store_true')
    mode.add_argument(
        '--explain', action='store_true',
        help='Inspect effective seeded inputs without launching')
    args = parser.parse_args(argv)
    if share is None:
        from ament_index_python.packages import get_package_share_directory

        def share(name):
            return Path(get_package_share_directory(name))
    try:

        def load():
            if args.experiment:
                from .catalog import ExperimentCatalog
                return ExperimentCatalog(args.catalog, share).prepare(
                    args.experiment, args.revision, output=args.output)
            if args.catalog or args.revision:
                raise ValueError('--catalog/--revision require --experiment')
            path = args.config.expanduser()
            if not path.is_file() and (not path.is_absolute()):
                path = share('robot_experiments') / path
            return load_experiment(path, share, args.output)
        if args.explain:
            with redirect_stdout(sys.stderr):
                spec = load()
                explanation = explain_experiment(spec, share)
            print(json.dumps(explanation, indent=2, allow_nan=False))
            return 0
        spec = load()
        if args.validate_only:
            print(json.dumps({
                'schema_version': 1, 'valid': True,
                'variants': {v['name']: v['plan']['identities'] for v in spec['variants']},
            }, indent=2))
            return 0
        if args.observer_diagnostics:
            directory, code = run_experiment(spec, share, observer_diagnostics=True)
        else:
            directory, code = run_experiment(spec, share)
        print(json.dumps({'schema_version': 1, 'results': str(directory), 'exit_code': code}))
        return code
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(json.dumps({'schema_version': 1, 'error': str(exc)}),
              file=sys.stderr if args.explain else sys.stdout)
        return 2


def manage(argv, *, share=None):
    """Stable JSON headless management; imports no dashboard dependencies."""
    from robot_configuration.yaml_loader import load_yaml
    from .catalog import ExperimentCatalog
    from .jobs import ExperimentJobs
    from .jobs import history
    from .reporting import plain
    parser = argparse.ArgumentParser(description='Experiment catalog and owned runs')
    parser.add_argument('operation', choices=['catalog', 'start', 'status', 'stop', 'history'])
    parser.add_argument('action', nargs='?')
    parser.add_argument('--catalog', type=Path)
    parser.add_argument('--experiment')
    parser.add_argument('--revision', type=int)
    parser.add_argument('--expected-revision', type=int)
    parser.add_argument('--name')
    parser.add_argument('--definition', type=Path, help='Self-contained M08 document')
    parser.add_argument('--overrides', type=Path, help='Owner override mapping')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--jobs', type=Path, default=Path('results/jobs'))
    parser.add_argument('--results', type=Path, default=Path('results/experiments'))
    args = parser.parse_args(argv)
    try:
        catalog = ExperimentCatalog(args.catalog, share)
        if args.operation == 'catalog':
            if args.action == 'init':
                result = catalog.initialize()
            elif args.action == 'list':
                result = catalog.list()
            elif args.action == 'show':
                result = catalog.get(args.experiment, args.revision)
            elif args.action in {'create', 'revise'}:
                if not args.definition:
                    raise ValueError('--definition required')
                definition = load_yaml(args.definition)
                overrides = load_yaml(args.overrides) if args.overrides else {}
                if args.action == 'create':
                    result = catalog.create(args.experiment, args.name, definition, overrides)
                else:
                    if args.expected_revision is None:
                        raise ValueError('--expected-revision required')
                    result = catalog.revise(
                        args.experiment, definition, overrides,
                        expected_revision=args.expected_revision)
            elif args.action in {'delete', 'purge'}:
                if args.expected_revision is None:
                    raise ValueError('--expected-revision required')
                getattr(catalog, args.action)(
                    args.experiment, expected_revision=args.expected_revision)
                result = {'id': args.experiment, 'operation': args.action}
            else:
                raise ValueError('catalog action: init/list/show/create/revise/delete/purge')
        elif args.operation == 'start':
            result = ExperimentJobs(args.jobs).start(
                catalog, args.experiment, args.revision, output=args.output)
        elif args.operation in {'status', 'stop'}:
            result = getattr(ExperimentJobs(args.jobs), args.operation)(args.action)
        else:
            result = history(args.results, args.experiment)
        print(json.dumps(plain(result), indent=2, allow_nan=False))
        return 0
    except (ValueError, OSError, KeyError, TypeError, TimeoutError) as error:
        print(json.dumps({'error': str(error)}), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
