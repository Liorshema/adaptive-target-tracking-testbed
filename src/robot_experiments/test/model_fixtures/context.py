"""Isolate test-only owner modules/defaults from source and installed production inventory."""

from contextlib import contextmanager
from contextlib import ExitStack
from importlib import import_module
from pathlib import Path
import shutil
import sys
from unittest.mock import patch

from robot_configuration.yaml_loader import load_yaml
import yaml


@contextmanager
def model_context(directory, share):
    """Temporarily extend trusted owner namespaces; restore paths/modules on exit."""
    import robot_configuration.resources as resources
    import robot_control.configuration as control
    import robot_prediction.factory.component_registry as prediction
    import robot_estimation.configuration as estimation

    fixture = Path(__file__).parent
    additions = load_yaml(fixture / 'owner_parameters.yaml')
    roots = {}
    for package, groups in additions.items():
        original, root = Path(share(package)), Path(directory) / package
        root.mkdir(parents=True)
        for child in original.iterdir():
            if child.name == 'config':
                shutil.copytree(child, root / child.name)
            else:
                (root / child.name).symlink_to(child, target_is_directory=child.is_dir())
        filename = {
            'robot_simulation': 'dynamics.yaml',
            'robot_prediction': 'prediction.yaml',
            'robot_control': 'control.yaml',
            'robot_estimation': 'estimation.yaml',
        }[package]
        path = root / 'config' / filename
        document = load_yaml(path)
        for group, entries in groups.items():
            document[group].update(entries)
        path.write_text(yaml.safe_dump(document, sort_keys=False))
        roots[package] = root

    def lookup(package):
        return roots[package] if package in roots else Path(share(package))

    original_config_path = resources.package_config_path

    def config_path(module, package, filename):
        if package in roots and (roots[package] / 'config' / filename).is_file():
            return roots[package] / 'config' / filename
        return original_config_path(module, package, filename)

    namespaces = {
        'robot_simulation.dynamics.target': 'dynamics',
        'robot_prediction.baseline': 'prediction',
        'robot_control.controllers': 'control',
        'robot_estimation.filters': 'estimation',
    }
    modules = [name + ('.fading_filter' if folder == 'estimation' else '.mathematical_example')
               for name, folder in namespaces.items()]
    assert not any(name in sys.modules for name in modules)
    try:
        with ExitStack() as stack:
            stack.enter_context(patch.object(resources, 'package_config_path', config_path))
            stack.enter_context(patch.object(prediction, 'package_config_path', config_path))
            stack.enter_context(patch.object(
                estimation, 'estimation_configuration_path',
                lambda: roots['robot_estimation'] / 'config/estimation.yaml'))
            stack.enter_context(
                patch.object(
                    control,
                    'control_configuration_path',
                    lambda: roots['robot_control'] / 'config/control.yaml',
                )
            )
            for namespace, folder in namespaces.items():
                package = import_module(namespace)
                stack.enter_context(
                    patch.object(package, '__path__', [*package.__path__, str(fixture / folder)])
                )
            yield lookup
    finally:
        for name in modules:
            sys.modules.pop(name, None)
