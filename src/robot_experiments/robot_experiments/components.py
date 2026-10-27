"""Headless component discovery delegates all model knowledge to owning packages."""

from robot_control.factory.project_components import create_project_registries
from robot_estimation.factory.estimator_factory import create_estimation_registries
from robot_prediction.factory.component_registry import create_prediction_registries
from robot_simulation.factory.simulation_factory import SimulationFactory


def component_registries():
    """Aggregate families, never individual algorithm IDs or model-selection branches."""
    control = create_project_registries()
    return {
        **create_estimation_registries(),
        **create_prediction_registries(),
        'dynamics': SimulationFactory.with_defaults().dynamics_registry,
        'controller': control['controller'],
        'task': control['task'],
        'reference_policy': control['policy'],
        'reference_geometry': control['geometry'],
    }


def _registry(family):
    registries = component_registries()
    if family not in registries:
        raise ValueError(f'Unknown component family {family!r}; available: {sorted(registries)}')
    return registries[family]


def list_components(family=None):
    """Return stable, detached descriptions/schemas/defaults/units/capabilities."""
    if family is not None:
        return _registry(family).describe_all()
    return {name: registry.describe_all() for name, registry in component_registries().items()}


def describe_component(family, component_id):
    """Retrieve one trusted registration without importing any YAML-supplied code."""
    return _registry(family).describe(component_id)


def validate_component(family, component_id, parameters=None, context=None):
    """Return schema/adapter compatibility; construction enforces coupled domain constraints."""
    registry = _registry(family)
    try:
        values = registry.validate(component_id, parameters, context)
        return {
            'valid': True,
            'parameters': values,
            'errors': [],
            'validation_scope': 'parameter schema and declared adapter requirements',
        }
    except (ValueError, TypeError, KeyError) as error:
        return {
            'valid': False,
            'parameters': None,
            'errors': [str(error)],
            'validation_scope': 'parameter schema and declared adapter requirements',
        }


def main(argv):
    """Serve JSON metadata for CLI and future clients; no dashboard dependency."""
    import argparse
    import json
    from pathlib import Path

    from robot_configuration.yaml_loader import load_yaml

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['list', 'show', 'validate'])
    parser.add_argument('family', nargs='?')
    parser.add_argument('component_id', nargs='?')
    parser.add_argument('--parameters', type=Path)
    parser.add_argument('--context', type=Path)
    args = parser.parse_args(argv)
    try:
        if args.action == 'list':
            result = list_components(args.family)
        elif args.action == 'show':
            result = describe_component(args.family, args.component_id)
        else:
            result = validate_component(
                args.family,
                args.component_id,
                load_yaml(args.parameters) if args.parameters else None,
                load_yaml(args.context) if args.context else None,
            )
        print(json.dumps(result, indent=2, allow_nan=False))
        return 2 if isinstance(result, dict) and result.get('valid') is False else 0
    except (ValueError, TypeError, KeyError, OSError) as error:
        print(json.dumps({'error': str(error)}))
        return 2
