"""Verify that visualization displays the selected physical frame axes."""

import importlib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


@pytest.mark.parametrize('forward', [[0., 0., 1.], [1., 0., 0.]])
def test_optical_marker_uses_selected_axis_and_actual_rotation(forward):
    """The optical marker must be perpendicular to a sideways camera's shaft."""
    module = importlib.import_module('robot_bringup.nodes.tracking_visualization')
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('y', .72).as_matrix()
    pose[:3, 3] = [1., 2., .13]
    configuration = {'axis_length': .3, 'forward_length': .8, 'width': .01,
                     'origin_size': .03, 'lifetime': .3,
                     'colors': {'x': [1., 0., 0., 1.], 'y': [0., 1., 0., 1.],
                                'z': [0., 0., 1., 1.], 'optical': [1., 1., 0., 1.],
                                'target': [1., 0., 1., 1.], 'origin': [1., 1., 1., 1.]}}
    target = pose[:3, 3] + 1.1 * pose[:3, 2]
    markers = module.build_markers(pose, target, forward, configuration, 'world')
    by_name = {marker.ns: marker for marker in markers.markers}
    optical = by_name['optical'].points
    endpoint = np.array([optical[1].x, optical[1].y, optical[1].z])
    np.testing.assert_allclose(endpoint, pose[:3, 3] + .8 * pose[:3, :3] @ forward)
    assert pose[2, 0] < 0 < pose[2, 2]
    assert set(by_name) == {'origin', 'x', 'y', 'z', 'optical', 'target'}
    assert all(marker.header.frame_id == 'world' for marker in markers.markers)
