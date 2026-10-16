"""Check native Gazebo timestamp decoding and independent physical metrics."""

import json
from pathlib import Path

import pytest
import yaml

from shoulder_experiment_metrics import experiment_metrics


@pytest.mark.parametrize('seconds', ['11', 11])
def test_native_pose_decode_with_optional_tf_and_known_optical_geometry(tmp_path, seconds):
    root = Path(__file__).resolve().parents[2]
    description = yaml.safe_load(
        (root / 'src/robot_description/config/robot/robot.yaml').read_text())['robot']
    scene = {'header': {'stamp': {'sec': seconds, 'nsec': 0}}, 'pose': [
        {'name': 'robot', 'orientation': {'w': 1.}},
        {'name': 'arm_link_1', 'position': {'z': .18}, 'orientation': {'w': 1.}},
        {'name': 'wrist_link', 'position': {'z': .18}, 'orientation': {'w': 1.}}]}
    (tmp_path / 'gazebo_poses.jsonl').write_text(json.dumps(scene) + '\n')
    streams = {'joints': [[11., [0., 0., 0., 0.], [0., .4, .7, 1.6]]],
               'targets': [[11., [1.12, 0., .18]], [12., [1.12, 0., .18]]],
               'tracking_tf': []}
    result = experiment_metrics(
        tmp_path, streams, description, {'entity_name': 'robot'}, 1.,
        {'settling_time': 10., 'duration': 40., 'max_feedback_skew': .08,
         'physical_numerical_tolerance': .001},
        control={'tracking_forward_axis': [1., 0., 0.], 'tracking_radius': 1.})
    assert result['minimum_link_elevation'] == 0.
    assert result['minimum_first_link_box_base_clearance'] == pytest.approx(.04)
    assert result['mean_physical_distance_error'] == pytest.approx(0.)
    assert result['mean_physical_pointing_degrees'] == pytest.approx(0.)
    assert result['mean_target_speed'] == 0.
    assert result['tracking_tf_samples'] == 0
    assert result['raw_joint_speed_peaks'] == [0., .4, .7, 1.6]
    json.dumps(result)
