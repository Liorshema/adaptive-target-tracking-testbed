"""Experimental inheritance changes only circulation and selected motion costs."""

from copy import deepcopy
from pathlib import Path

from robot_configuration.yaml_loader import load_yaml
from robot_simulation.factory.simulation_factory import SimulationFactory

import yaml

SOURCE = Path(__file__).resolve().parents[2]


def test_reactive_3d_uses_live_tf_same_initial_state_and_environment():
    """The saturated baseline remains an unchanged, independently usable parent."""
    base = load_yaml(SOURCE / 'robot_simulation/config/scenarios/reactive_target_saturated.yaml')
    experiment = load_yaml(SOURCE / 'robot_simulation/config/scenarios/reactive_target_3d.yaml')
    expected = deepcopy(base)
    expected['target']['config'] = 'config/scenarios/target_3d.yaml'
    assert experiment == expected
    assert experiment['tracking']['source'] == 'tf'
    target = load_yaml(SOURCE / 'robot_simulation' / experiment['target']['config'])
    baseline_target = load_yaml(SOURCE / 'robot_simulation' / base['target']['config'])
    configured = target['target']['dynamics']['parameters'].pop('tangential')
    assert configured == {'acceleration': .06, 'axis_world': [.3, -.8, .5]}
    assert target == baseline_target
    target = load_yaml(SOURCE / 'robot_simulation' / experiment['target']['config'])
    environment = load_yaml(SOURCE / 'robot_simulation' /
                            experiment['simulation']['environment_config'])
    integration = load_yaml(SOURCE / 'robot_simulation' / experiment['simulation']['config'])
    engine, dt = SimulationFactory.with_defaults().create(target, environment, integration)
    assert dt > 0. and engine.dynamics.tangential is not None
    master = load_yaml(SOURCE / 'robot_bringup/config/reactive_tracking_3d.yaml')
    assert master['simulation_config'] == 'config/scenarios/reactive_target_3d.yaml'


def test_arm_preference_changes_only_existing_base_motion_weights():
    """Tasks, physical bounds, posture, damping and arm penalties are retained."""
    baseline = load_yaml(SOURCE / 'robot_control/config/whole_body.yaml')
    experiment = yaml.safe_load(
        (SOURCE / 'robot_control/config/whole_body_arm_preferred.yaml').read_text())
    expected = deepcopy(baseline)
    expected['controller']['parameters']['motion_weights'] = [2., 4., 3., 3., 3., 2.]
    assert experiment == expected
    assert baseline['controller']['parameters']['motion_weights'] == [1., 2., 3., 3., 3., 2.]
