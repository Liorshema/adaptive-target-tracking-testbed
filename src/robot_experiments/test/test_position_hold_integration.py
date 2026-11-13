"""Exercise None through actual Author/Factory and all requested controller families."""

from pathlib import Path

import numpy as np

import pytest

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


@pytest.mark.parametrize('controller', ['jacobian', 'weighted_least_squares', 'mpc'])
def test_position_hold_uses_existing_author_factory_and_controller(tmp_path, controller):
    """Materialization is configuration validation; no experiment is executed."""
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share=lambda p: SRC / p)
    catalog.initialize()
    backend = ExperimentBackend(tmp_path / 'state', catalog=catalog)
    draft = backend.new_draft('EXP-HOLD', 'Position hold', 'EXP-NOMINAL')
    draft['experiment']['components'] = {
        'predictor': 'none',
        'adaptation': 'none',
        'fusion': 'none',
        'controller': controller,
        'task': 'cartesian' if controller == 'jacobian' else 'distance_pointing',
    }
    checked = backend.validate_experiment(draft)
    assert checked['valid'], checked
    record = backend.save_experiment(draft)
    assert record['revision'] == 1
    plan = catalog.prepare(record['id'], 1)
    output = tmp_path / 'materialized'
    output.mkdir()
    materialize_run(plan['variants'][0], 42, output, catalog.share, plan['runtime'])
    system = PredictionSystemFactory(PredictionConfig(output / 'inputs/prediction.yaml')).create(
        predictor_type='none', adaptation_type='none', fusion_type='none'
    )
    prediction = system.predict(
        PredictionInput(np.array([2.0, 0.0, 1.0, 4.0, 5.0, 6.0]), 4.0, 0.1, 20)
    )
    np.testing.assert_allclose(prediction.states[:, 3:], 0.0)
    params = yaml.safe_load((output / 'inputs/control.yaml').read_text())['control_node'][
        'ros__parameters'
    ]
    description = yaml.safe_load((output / 'inputs/robot_description.yaml').read_text())
    strategy = yaml.safe_load((output / 'inputs/strategy.yaml').read_text())
    pipeline = ControlFactory(create_project_registries()).build(
        configured_control(params, description, strategy)
    )
    joints = list(description['robot']['manipulator']['joints'].values())
    q = np.array([(j['lower'] + j['upper']) / 2 for j in joints])
    robot = RobotState(np.r_[np.zeros(3), q], np.zeros(2 + len(joints)), 4.0, 'world')
    command = pipeline.compute(
        robot,
        PredictedTargetTrajectory(prediction.states, prediction.timestamps, 'world'),
        4.0,
        previous_velocity=robot.velocity,
        dt=0.05,
    )
    assert np.all(np.isfinite(command.safe_output.command))
