"""Check the geometric certificate independently of live tracking accuracy."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from analyze_aggressive_oscillatory import reachability_certificate

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def snapshot():
    """Use authoritative deployment geometry and the selected optical task."""
    def load(path):
        return yaml.safe_load((ROOT / path).read_text())
    return {
        'description': load('src/robot_description/config/robot/robot.yaml')['robot'],
        'robot': load('src/robot_simulation/config/gazebo/robot.yaml')['robot'],
        'control': load('src/robot_control/config/control.yaml')['control_node']['ros__parameters']}


def test_valid_virtual_endpoint_family_constructs_exact_distance_pointing_poses(tmp_path, snapshot):
    """Separate free-base geometric feasibility from current-base tracking error."""
    result = reachability_certificate(tmp_path, snapshot,
                                      np.array([[2.2, 0., .85], [-1., .3, 1.], [.2, -.3, 1.4]]))
    assert result['certified_samples'] == 3 and result['unresolved_samples'] == 0
    assert result['checked_ik_max_residual'] < 1e-8
    assert not result['dynamic_reachability_certified']


def test_outside_sufficient_family_is_unresolved_not_declared_unreachable(tmp_path, snapshot):
    """A limited posture family must not masquerade as a complete IK solver."""
    result = reachability_certificate(tmp_path, snapshot, np.array([[2., 0., 5.]]))
    assert result['certified_samples'] == 0 and result['unresolved_samples'] == 1
    assert result['checked_ik_max_residual'] is None
