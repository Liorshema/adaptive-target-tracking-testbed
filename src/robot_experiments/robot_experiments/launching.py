"""Select experiment-owned direct-launch cases through the shared preflight."""

from copy import deepcopy
from pathlib import Path

from robot_bringup.factory.system_configuration import assemble_system_configuration
from robot_configuration.yaml_loader import load_yaml_group


def launch_profile(name, share, *, controller='', overrides=None):
    """Select a declared case and allocation, with no algorithm-specific branches."""
    from robot_configuration.yaml_loader import load_yaml
    from robot_experiments.configuration import resolve_system_resource

    source = Path(share('robot_experiments')) / 'config/experiments.yaml'
    reference = load_yaml_group(source, 'direct_launch.cases')[name]
    document_path = resolve_system_resource(reference, source.parent, share)
    document = load_yaml(document_path)
    if controller:
        selected = load_yaml_group(source, 'direct_launch.controllers.' + controller)
        document['control'].pop('strategy_overrides', None)
        document['control'].update(deepcopy(selected))
    return assemble_system_configuration(document, source, share, overrides)


def compose(argv, *, share=None):
    """Freeze a named continuous-launch case using the existing run-input materializer."""
    import argparse
    import json

    from robot_experiments.catalog import package_share
    from robot_experiments.configuration import materialize_run

    parser = argparse.ArgumentParser(description=compose.__doc__)
    parser.add_argument('--case', required=True)
    parser.add_argument('--controller', default='')
    parser.add_argument('--seed', required=True, type=int)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if not 0 <= args.seed < 2**32:
            raise ValueError('seed must be uint32')
        share = share or package_share
        plan = launch_profile(args.case, share, controller=args.controller)
        args.output.mkdir(parents=True, exist_ok=False)
        result = materialize_run(
            {'name': args.case, 'plan': plan},
            args.seed,
            args.output,
            share,
            {'enable_actuation': False},
        )
        print(
            json.dumps(
                {
                    'system_config': str(result['system_path']),
                    'simulation_config': str(result['simulation_path']),
                    'identities': result['plan']['identities'],
                }
            )
        )
        return 0
    except (ValueError, KeyError, TypeError, OSError) as error:
        print(json.dumps({'error': str(error)}))
        return 2
