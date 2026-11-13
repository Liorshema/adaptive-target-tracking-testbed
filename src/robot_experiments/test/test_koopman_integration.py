"""Exercise registered EDMD Scenario provenance and the actual MPC consumer."""

from copy import deepcopy
from pathlib import Path

import numpy as np


from robot_control.factory.control_factory import ControlFactory
from robot_control.factory.project_components import create_project_registries
from robot_control.factory.project_configuration import configured_control
from robot_control.interfaces.control_data import PredictedTargetTrajectory, RobotState

from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.configuration import materialize_run

from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.factory.system_factory import PredictionSystemFactory
from robot_prediction.interfaces.prediction_data import PredictionInput

import yaml

SRC = Path(__file__).resolve().parents[2]


def test_edmd_schema_scenario_roundtrip_immutable_revision_and_mpc(tmp_path):
    """Save/restore the actual component and pass its real forecast through MPC."""
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share=lambda p: SRC / p)
    catalog.initialize()
    backend = ExperimentBackend(tmp_path / 'state', catalog=catalog)
    option = next(
        o for o in backend.list_component_options('predictor') if o['id'] == 'koopman_edmd'
    )
    assert option['capabilities']['display_name'] == 'Koopman (EDMD)'
    assert option['defaults']['polynomial_degree'] == 2
    assert option['schema']['properties']['training_interval']['unit'] == 's'
    draft = backend.new_draft('EXP-EDMD', 'Koopman test', 'EXP-NOMINAL')
    draft['experiment']['components'] = {
        'predictor': 'koopman_edmd',
        'adaptation': 'none',
        'fusion': 'none',
        'controller': 'mpc',
        'task': 'distance_pointing',
    }
    draft['experiment']['parameters'] = {'predictor': {'refit_interval': 3}}
    preview = backend.preview_experiment(draft)
    assert preview['identities']['predictor'].endswith('.KoopmanEDMDPredictor')
    r1 = backend.save_experiment(draft)
    original = deepcopy(catalog.get('EXP-EDMD', 1))
    draft['experiment']['parameters']['predictor']['refit_interval'] = 4
    assert backend.save_experiment(draft, expected_revision=1)['revision'] == 2
    assert catalog.get('EXP-EDMD', 1) == original == r1
    pinned = backend.new_draft('EXP-EDMD-PIN', 'Pinned Koopman', 'EXP-EDMD', base_revision=1)
    restored = backend.preview_experiment(pinned)
    assert restored['components']['predictor']['effective']['refit_interval'] == 3
    plan = catalog.prepare('EXP-EDMD', 1)
    run = tmp_path / 'run'
    run.mkdir()
    materialize_run(plan['variants'][0], 42, run, catalog.share, plan['runtime'])
    prediction = PredictionSystemFactory(PredictionConfig(run / 'inputs/prediction.yaml')).create(
        predictor_type='koopman_edmd', adaptation_type='none', fusion_type='none'
    )
    state = np.array([2.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    output = prediction.predict(PredictionInput(state, 0.0, 0.1, 20))
    for k in range(1, 41):
        prediction.update(state, observation_timestamp=k * 0.1)
        output = prediction.predict(PredictionInput(state, k * 0.1, 0.1, 20))
    assert output.model_name == 'koopman_edmd'
    params = yaml.safe_load((run / 'inputs/control.yaml').read_text())['control_node'][
        'ros__parameters'
    ]
    description = yaml.safe_load((run / 'inputs/robot_description.yaml').read_text())
    strategy = yaml.safe_load((run / 'inputs/strategy.yaml').read_text())
    pipeline = ControlFactory(create_project_registries()).build(
        configured_control(params, description, strategy)
    )
    joints = list(description['robot']['manipulator']['joints'].values())
    q = np.array([(j['lower'] + j['upper']) / 2 for j in joints])
    robot = RobotState(np.r_[np.zeros(3), q], np.zeros(2 + len(joints)), 4.0, 'world')
    forecast = PredictedTargetTrajectory(output.states, output.timestamps, 'world')
    command = pipeline.compute(robot, forecast, 4.0, previous_velocity=robot.velocity, dt=0.05)
    assert np.all(np.isfinite(command.safe_output.command))
    np.testing.assert_allclose(command.raw_output.command, command.safe_output.command, atol=1e-7)
    invalid = deepcopy(draft)
    invalid['experiment']['parameters']['predictor']['regularization'] = 0.0
    assert not backend.validate_experiment(invalid)['valid']


def test_experiment_lab_renders_registered_koopman_controls(tmp_path):
    """The existing generic UI shows the owner name and numerical schema."""
    import pytest

    pytest.importorskip('dash')
    from robot_dashboard.app import component_cards
    from robot_dashboard.forms import begin_draft

    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share=lambda p: SRC / p)
    catalog.initialize()
    backend = ExperimentBackend(tmp_path / 'state', catalog=catalog)
    state = begin_draft(backend, 'EXP-NOMINAL', 3, 'imm', 'create', 'EXP-LAB-EDMD', 'EDMD Lab')
    state['inherited_components'] = deepcopy(state['components'])
    state['spec']['experiment']['components'] = {
        'predictor': 'koopman_edmd',
        'adaptation': 'none',
        'fusion': 'none',
    }
    state['components'] = backend.preview_experiment(state['spec'])['components']
    stack = list(component_cards(backend, state))
    nodes = []
    while stack:
        item = stack.pop()
        if isinstance(item, (list, tuple)):
            stack.extend(item)
        elif hasattr(item, 'to_plotly_json'):
            nodes.append(item)
            stack.append(item.to_plotly_json()['props'].get('children'))
    selection = next(
        n for n in nodes if getattr(n, 'id', None) == {'type': 'component', 'role': 'predictor'}
    )
    assert any(
        o['value'] == 'koopman_edmd' and o['label'] == 'Koopman (EDMD)' for o in selection.options
    )
    regularization = next(
        n
        for n in nodes
        if getattr(n, 'id', None)
        == {'type': 'param', 'role': 'predictor', 'key': 'regularization', 'kind': 'number'}
    )
    assert regularization.value == 0.0001
