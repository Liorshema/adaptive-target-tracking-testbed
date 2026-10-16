"""Legacy imports must share canonical globals used by real instrumentation."""

from importlib import import_module
from pathlib import Path

import pytest
from setuptools import find_packages


MODULES = [
    ('robot_control.ros.control_node', 'robot_control.nodes.control_node'),
    ('robot_estimation.estimation_node', 'robot_estimation.nodes.estimation_node'),
    ('robot_prediction.prediction_node', 'robot_prediction.nodes.prediction_node'),
    ('robot_simulation.actuation.velocity_actuation', 'robot_simulation.nodes.velocity_actuation'),
    ('robot_bringup.tracking_feedback_tf', 'robot_bringup.nodes.tracking_feedback_tf'),
    ('robot_bringup.tracking_visualization', 'robot_bringup.nodes.tracking_visualization'),
    ('robot_bringup.model_factory', 'robot_bringup.factory.model_factory'),
    ('robot_prediction.system_factory', 'robot_prediction.factory.system_factory'),
    ('robot_prediction.config_loader', 'robot_prediction.configuration.prediction_config'),
    ('robot_simulation.factory.config_loader', 'robot_configuration.yaml_loader'),
]


@pytest.mark.parametrize('legacy,canonical', MODULES)
def test_legacy_import_is_canonical_module_with_shared_globals(legacy, canonical, monkeypatch):
    """Patching an old module must affect the module invoked by the console script."""
    new = import_module(canonical)
    old = import_module(legacy)
    assert old is new
    monkeypatch.setattr(old, '_migration_probe', object(), raising=False)
    assert new._migration_probe is old._migration_probe
    if hasattr(new, 'main'):
        assert new.main.__globals__ is new.__dict__


@pytest.mark.parametrize('subpackage', ['robot_control.factory', 'robot_control.pipeline'])
def test_control_assembly_is_packaged_without_symlink_install(subpackage):
    """A source directory import is insufficient if setuptools omits its contents."""
    package = Path(__file__).resolve().parents[2] / 'robot_control'
    assert subpackage in find_packages(where=str(package), exclude=['test'])
