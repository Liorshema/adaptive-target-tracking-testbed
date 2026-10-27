"""Experiment API selects owner components and freezes variations without source YAMLs."""

from copy import deepcopy
from pathlib import Path

from robot_configuration.yaml_loader import load_yaml
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.configuration import materialize_run

SRC = Path(__file__).resolve().parents[2]


def share(package):
    return SRC / package


def reference(group):
    return {'package': 'robot_experiments', 'path': 'config/experiments.yaml', 'group': group}


def definition():
    document = ExperimentCatalog(share=share).document('EXP-NOMINAL')
    document['experiment']['variants'] = [
        {
            'name': 'custom',
            'system_config': {
                'base': reference(
                    ['compositions.common', 'compositions.selections.nominal_tracking']
                ),
                'overrides': {},
            },
        }
    ]
    return document


def test_composition_selects_dynamics_prediction_control_and_owner_overrides(tmp_path):
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    defaults = {p: p.read_bytes() for p in SRC.rglob('*.yaml')}
    for index, (dynamics, predictor, controller) in enumerate(
        [
            ('target_fight_flight_low_damping', 'constant_velocity', 'jacobian'),
            ('target_saturated', 'imm', 'weighted_least_squares'),
        ]
    ):
        doc = definition()
        recipe = doc['experiment']['variants'][0]['system_config']
        recipe['overrides'] = {
            'prediction': {'type': predictor, 'adaptation': 'none', 'fusion': 'none'},
            'control': {'type': controller},
            'simulation': {
                'config': {
                    'base': reference('resources.deployment_oscillatory_reactive_tracking'),
                    'overrides': {'target': {'config': reference('resources.' + dynamics)}},
                }
            },
        }
        if controller == 'jacobian':
            recipe['overrides']['control']['strategy'] = None
        overrides = {
            'target': {'target': {'dynamics': {'parameters': {'damping': 0.23}}}},
            'measurements': {
                'measurements': {
                    'target_position': {
                        'noise': {
                            'covariance': [[0.002, 0.0, 0.0], [0.0, 0.002, 0.0], [0.0, 0.0, 0.002]]
                        }
                    }
                }
            },
        }
        if controller == 'weighted_least_squares':
            overrides['strategy'] = {
                'controller': {'parameters': {'motion_weights': [2.0, 4.0, 3.0, 3.0, 3.0, 2.0]}}
            }
        identifier = f'EXP-COMPOSE-{index}'
        catalog.create(identifier, 'Selection workflow', doc, overrides)
        spec = catalog.prepare(identifier)
        directory = tmp_path / str(index)
        directory.mkdir()
        prepared = materialize_run(spec['variants'][0], 42, directory, share, spec['runtime'])
        assert prepared['plan']['selection']['predictor_type'] == predictor
        assert prepared['plan']['control_parameters']['controller_type'] == controller
        target = load_yaml(directory / 'inputs/target.yaml')['target']['dynamics']
        assert target['parameters']['damping'] == 0.23
        assert target['type'] == ('fight_flight' if index == 0 else 'saturated_fight_flight')
        draft = catalog.draft(identifier)
        assert catalog.reset(draft)['overrides'] == {}
        assert catalog.get(identifier)['overrides'] == overrides
    assert defaults == {p: p.read_bytes() for p in defaults}


def test_registered_extensions_are_selected_through_catalog_without_runner_branches(
    tmp_path, monkeypatch
):
    import robot_bringup.factory.system_configuration as assembly
    import robot_estimation.factory.estimator_factory as estimation
    import robot_prediction.factory.system_factory as prediction
    from robot_simulation.factory.simulation_factory import SimulationFactory

    calls = set()

    def extend(registry, original, family):
        entry = registry._entry(original)
        metadata = {k: deepcopy(v) for k, v in entry.items() if k not in {'id', 'builder'}}

        def build(**kwargs):
            calls.add(family)
            return entry['builder'](**kwargs)

        registry.register('test_' + family, build, **metadata)
        return registry

    make_estimation = estimation.create_estimation_registries
    make_prediction = prediction.create_prediction_registries
    make_control = assembly.create_project_registries
    make_simulation = SimulationFactory.with_defaults

    def estimator_registries(*args, **kwargs):
        registries = make_estimation(*args, **kwargs)
        extend(registries['target_estimator'], 'kalman', 'estimator')
        return registries

    def prediction_registries(*args, **kwargs):
        registries = make_prediction(*args, **kwargs)
        extend(registries['predictor'], 'constant_velocity', 'predictor')
        return registries

    def control_registries(*args, **kwargs):
        registries = make_control(*args, **kwargs)
        extend(registries['controller'], 'weighted_least_squares', 'controller')
        return registries

    def simulation_factory(*args, **kwargs):
        factory = make_simulation(*args, **kwargs)
        extend(factory.dynamics_registry, 'fight_flight', 'dynamics')
        return factory

    monkeypatch.setattr(estimation, 'create_estimation_registries', estimator_registries)
    monkeypatch.setattr(prediction, 'create_prediction_registries', prediction_registries)
    monkeypatch.setattr(assembly, 'create_project_registries', control_registries)
    monkeypatch.setattr(SimulationFactory, 'with_defaults', simulation_factory)
    doc = definition()
    doc['experiment']['variants'][0]['system_config']['overrides'] = {
        'estimation': {'target': {'type': 'test_estimator'}},
        'prediction': {'type': 'test_predictor', 'adaptation': 'none', 'fusion': 'none'},
        'control': {'type': 'test_controller'},
        'simulation': {
            'config': {
                'base': reference('resources.deployment_oscillatory_reactive_tracking'),
                'overrides': {
                    'target': {
                        'config': {
                            'base': reference('resources.target_independent'),
                            'selection': {
                                'target.dynamics': {'type': 'test_dynamics', 'parameters': {}}
                            },
                            'overrides': {},
                        }
                    }
                },
            }
        },
    }
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    catalog.create('EXP-EXTENSION', 'Registered test adapters', doc)
    assert catalog.prepare('EXP-EXTENSION')['variants']
    assert calls == {'dynamics', 'predictor', 'estimator', 'controller'}
