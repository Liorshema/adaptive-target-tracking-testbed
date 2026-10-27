"""Verify unified component selection against a copied install from outside the workspace."""

import argparse
import json
from importlib import import_module
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from robot_bringup.factory.system_configuration import load_system_configuration
from robot_experiments.configuration import resolve_system_resource
from robot_experiments.composition import load_resource, resource_sources
from m06_verify_installation import verify


def run(prefix):
    result = verify(prefix)
    for name in ['robot_configuration.resources', 'robot_configuration.parameter_validation',
                 'robot_bringup.factory.system_configuration', 'robot_control.parameter_schema',
                 'robot_estimation.parameter_schema',
                 'robot_simulation.factory.deployment_configuration',
                 'robot_simulation.factory.actuation_factory']:
        assert Path(import_module(name).__file__).resolve().is_relative_to(prefix), name
    share = lambda package: Path(get_package_share_directory(package))
    root = share('robot_experiments') / 'config/systems'
    cases = {}
    for name, filename in [('system_nominal','nominal_tracking'), ('system_cv','cv_prediction'), ('system_dls','dls_control')]:
        selected = resolve_system_resource(root / (filename + '.yaml'), root, share)
        plan = load_system_configuration(selected, share)
        for key in ['description_path', 'prediction_path']:
            origins = resource_sources(plan[key])
            assert origins and all(Path(origin['path']).is_relative_to(prefix)
                                   for origin in origins), (key, origins)
        assert all(not value for value in plan['runtime'].values())
        cases[name] = plan['identities']
    assert cases['system_nominal']['predictor'] != cases['system_cv']['predictor']
    assert cases['system_nominal']['controller'] != cases['system_dls']['controller']
    try:
        from robot_bringup.factory.system_configuration import assemble_system_configuration
        invalid = load_resource(root / 'nominal_tracking.yaml', share=share)
        invalid['prediction']['type'] = 'constant_acceleration'
        assemble_system_configuration(invalid, root/'invalid-test.yaml', share)
    except ValueError as error:
        assert 'prediction' in str(error) and 'constant_acceleration' in str(error)
        result['negative_error'] = str(error)
    else:
        raise AssertionError('Invalid installed manifest silently accepted')
    result['systems'] = cases
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = run(args.prefix.resolve())
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print('Verified 7 console entries,', len(result['resources']),
          'resources and installed nominal/CV/DLS identities; invalid manifest rejected.')
