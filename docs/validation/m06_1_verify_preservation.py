"""Compare extracted assembly with baseline and preserve original experiment assets."""

import hashlib
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
import sys

import numpy as np
import yaml

from robot_estimation.measurement import Measurement
from robot_estimation.factory.estimator_factory import create_robot_estimator, create_target_estimator
from archived_estimation_imports import canonicalize_imports

ROOT = Path(__file__).resolve().parents[2]
BEFORE = ROOT / 'docs/validation/m06_1_before'


def verify():
    hashes = json.loads((BEFORE / 'hashes.json').read_text())
    assets = []
    for name, digest in hashes.items():
        path = ROOT / name
        if path.suffix in {'.yaml', '.xacro', '.sdf', '.msg'}:
            if name == 'src/robot_simulation/config/disturbances.yaml':
                assert not (BEFORE / name).read_text().strip() and not path.exists()
            else:
                assert hashlib.sha256(path.read_bytes()).hexdigest() == digest, name
                assets.append(name)
    parameters = {}
    for kind in ['robot', 'target']:
        parameters.update(yaml.safe_load((ROOT / f'src/robot_estimation/config/{kind}_estimator.yaml').read_text())['estimation_node']['ros__parameters'])
    description = ROOT / 'src/robot_description/config/robot/robot.yaml'
    parameters['robot_description_file'] = str(description)
    physical = yaml.safe_load(description.read_text())['robot']['manipulator']['joints']
    module = ModuleType('m06_1_baseline_node')
    sys.modules[module.__name__] = module
    source = BEFORE / 'src/robot_estimation/robot_estimation/nodes/estimation_node.py'
    exec(compile(canonicalize_imports(source.read_text()), str(source), 'exec'), module.__dict__)
    node = object.__new__(module.EstimationNode)
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    old_robot = node._create_robot_estimator()
    new_robot, _ = create_robot_estimator(parameters, physical)
    old_target = node._create_target_estimator()
    new_target = create_target_estimator(parameters)
    for estimator in [old_robot, new_robot]:
        estimator.predict(.02)
        estimator.update_wheel_odometry(Measurement(np.array([.2, .1]), np.eye(2)*.01, 1.))
        estimator.update_joint_encoders(Measurement(
            np.array([.1, .2, -.1, .2, 0., 0., 0., 0.]), np.eye(8)*.01, 1.))
        estimator.predict(.02)
    for estimator in [old_target, new_target]:
        estimator.predict(.02)
        estimator.update(Measurement(np.array([2., .1, 1.]), np.eye(3)*.01, 1.))
        estimator.predict(.02)
    for old, new in [(old_robot, new_robot), (old_target, new_target)]:
        np.testing.assert_allclose(old.state.mean, new.state.mean, rtol=0., atol=0.)
        np.testing.assert_allclose(old.state.covariance, new.state.covariance, rtol=0., atol=0.)
        assert old.state.timestamp == new.state.timestamp
    result = dict(unchanged_assets=assets, empty_stub_removed=True,
                  robot_and_target_assembly_distribution_equivalent=True)
    (ROOT / 'docs/validation/m06_1_preservation.json').write_text(json.dumps(result, indent=2)+'\n')
    print('Verified', len(assets), 'unchanged configuration/physical/message assets and exact baseline filter distributions.')


if __name__ == '__main__':
    verify()
