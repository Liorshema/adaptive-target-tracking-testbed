"""Deployment preflight rejects ignored or incompatible numerical settings."""

from copy import deepcopy
from pathlib import Path

import pytest

from functools import partial
from robot_experiments.composition import load_resource
load_yaml = partial(load_resource, share=lambda package: Path(
    __file__).resolve().parents[2] / package)


def configurations():
    root = Path(__file__).parents[1]
    deployment = load_yaml({'package': 'robot_experiments',
                            'path': 'config/experiments.yaml',
                            'group': 'resources.deployment_oscillatory_reactive_tracking'})
    return [deployment, load_yaml(root / deployment['target']['config']),
            load_yaml(root / deployment['simulation']['environment_config']),
            load_yaml(root / deployment['simulation']['config']),
            load_yaml(root / deployment['simulation']['measurement_config'])]


def test_valid_current_deployment_passes_owner_validation():
    from robot_simulation.factory.deployment_configuration import validate_deployment_configuration
    validate_deployment_configuration(*configurations())


@pytest.mark.parametrize('index,path,value', [
    (0, ('tracking', 'max_aggge'), .1),
    (1, ('target', 'dynamcis'), {}),
    (2, ('environment', 'stochastic_disturbances'), [{'fake': True}]),
    (3, ('simulation', 'integration', 'max_steps'), 0),
    (4, ('measurements', 'target_position', 'rate_hz'), float('nan')),
    (4, ('measurements', 'target_position', 'noise', 'covariance'), [[1.]]),
])
def test_invalid_numerical_deployment_is_not_ignored(index, path, value):
    from robot_simulation.factory.deployment_configuration import validate_deployment_configuration
    data = deepcopy(configurations())
    item = data[index]
    for key in path[:-1]:
        item = item[key]
    item[path[-1]] = value
    with pytest.raises(ValueError):
        validate_deployment_configuration(*data)


@pytest.mark.parametrize('change', [
    {'max_age': 0.}, {'reference_frame': 'odom'}, {'unused': True}])
def test_contextual_deployment_rejects_bad_physical_context(change):
    from robot_simulation.factory.deployment_configuration import validate_deployment_configuration
    root = Path(__file__).parents[1]
    d = load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml',
                  'group': 'resources.deployment_reactive_manipulation'})
    d['physical_context'].update(change)
    with pytest.raises(ValueError):
        validate_deployment_configuration(d, load_yaml(root/d['target']['config']),
                                          load_yaml(root/d['simulation']['environment_config']),
                                          load_yaml(root/d['simulation']['config']),
                                          load_yaml(root/d['simulation']['measurement_config']))
