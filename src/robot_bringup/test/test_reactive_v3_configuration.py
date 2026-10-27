"""Experimental inheritance changes only circulation and selected motion costs."""

from copy import deepcopy
from pathlib import Path

from robot_experiments.launching import launch_profile
from robot_simulation.factory.simulation_factory import SimulationFactory
from functools import partial
from robot_experiments.composition import load_resource
from robot_experiments.configuration import resolve_system_resource
load_yaml = partial(load_resource, share=lambda package: Path(
    __file__).resolve().parents[2] / package)

SOURCE = Path(__file__).resolve().parents[2]


def test_reactive_3d_uses_live_tf_same_initial_state_and_environment():
    """The saturated baseline remains an unchanged, independently usable parent."""
    base = load_yaml(
        resolve_system_resource(
            SOURCE /
            'robot_simulation/config/scenarios/reactive_target_saturated.yaml',
            Path.cwd(),
            lambda package: Path(__file__).resolve().parents[2] /
            package))
    experiment = load_yaml(
        resolve_system_resource(
            SOURCE /
            'robot_simulation/config/scenarios/reactive_target_3d.yaml',
            Path.cwd(),
            lambda package: Path(__file__).resolve().parents[2] /
            package))
    expected = deepcopy(base)
    expected['target']['config'] = str(resolve_system_resource(
        SOURCE / 'robot_simulation/config/scenarios/target_3d.yaml', Path.cwd(),
        lambda p: SOURCE / p))
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
    master = launch_profile('reactive_tracking_3d', lambda p: SOURCE / p)['profile']
    assert load_yaml(master['simulation_config']) == experiment


def test_arm_preference_changes_only_existing_base_motion_weights():
    """Tasks, physical bounds, posture, damping and arm penalties are retained."""
    baseline = launch_profile(
        'closed_loop', lambda p: SOURCE / p, controller='whole_body')['strategy']
    experiment = launch_profile(
        'closed_loop', lambda p: SOURCE / p, controller='whole_body_arm_preferred')['strategy']
    expected = deepcopy(baseline)
    expected['controller']['parameters']['motion_weights'] = [2., 4., 3., 3., 3., 2.]
    assert experiment == expected
    assert baseline['controller']['parameters']['motion_weights'] == [1., 2., 3., 3., 3., 2.]
