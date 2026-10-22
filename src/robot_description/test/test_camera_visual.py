"""Verify the schematic camera and unchanged M08 physical description."""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / 'config' / 'robot' / 'robot.yaml'
# Complete expanded bf71fdb URDF, excluding only visuals/materials. This includes
# joint origins, connectivity, axes, limits, dynamics, collisions and inertials.
M08_PHYSICAL_SHA256 = (
    '97cc0d03b785aaa68cd9c35c3cd9c5aafb4f7107aca7679d35a05e45cab169bf'
)


def _render(config_path=CONFIG):
    document = xacro.process_file(
        str(PACKAGE / 'urdf' / 'robot.urdf.xacro'),
        mappings={'config_file': str(config_path)},
    )
    return ET.fromstring(document.toxml())


def _physical_tree(node):
    return [
        node.tag, sorted(node.attrib.items()),
        ' '.join((node.text or '').split()),
        [_physical_tree(child) for child in node
         if child.tag not in {'visual', 'material'}],
    ]


def _numbers(value):
    return tuple(float(item) for item in value.split())


def _sdf(urdf, path):
    ET.ElementTree(urdf).write(path, encoding='unicode')
    result = subprocess.run(
        ['gz', 'sdf', '-p', str(path)], check=True,
        capture_output=True, text=True, timeout=30,
    )
    return ET.fromstring(result.stdout)


@pytest.fixture
def robot():
    """Expand the actual installed-format robot resources."""
    return _render()


def test_camera_housing_and_lens_identify_local_positive_x(robot):
    """Catch missing camera geometry or a lens facing the wrong control axis."""
    tracking = robot.find("link[@name='tracking_frame']")
    visuals = {item.get('name'): item for item in tracking.findall('visual')}
    assert set(visuals) == {'schematic_camera_housing', 'schematic_camera_lens'}
    housing = visuals['schematic_camera_housing']
    lens = visuals['schematic_camera_lens']
    assert housing.find('material').get('name') == 'black'
    assert _numbers(lens.find('material/color').get('rgba')) == (0, 1, 1, 1)

    # The camera frame lies at the housing's front plane. The cylinder points
    # along local +X; its front surface extends in the tracked control direction.
    size = _numbers(housing.find('geometry/box').get('size'))
    housing_x = _numbers(housing.find('origin').get('xyz'))[0]
    assert housing_x + size[0] / 2 == pytest.approx(0)
    lens_x = _numbers(lens.find('origin').get('xyz'))[0]
    length = float(lens.find('geometry/cylinder').get('length'))
    assert lens_x - length / 2 == pytest.approx(0)
    assert lens_x > 0
    roll, pitch, yaw = _numbers(lens.find('origin').get('rpy'))
    axis = (
        math.cos(yaw) * math.sin(pitch) * math.cos(roll)
        + math.sin(yaw) * math.sin(roll),
        math.sin(yaw) * math.sin(pitch) * math.cos(roll)
        - math.cos(yaw) * math.sin(roll),
        math.cos(pitch) * math.cos(roll),
    )
    assert axis == pytest.approx((1, 0, 0))
    assert tracking.find('collision') is None
    assert tracking.find('inertial') is None
    assert robot.find('.//sensor') is None


def test_camera_geometry_scales_from_wrist_and_preserves_configured_mount(tmp_path):
    """Catch hard-coded camera dimensions or an overridden authoritative mount."""
    config = yaml.safe_load(CONFIG.read_text())
    config['robot']['manipulator']['wrist'].update(
        length=0.24, width=0.08, height=0.10,
    )
    config['robot']['tracking_frame'].update(
        translation=[0.21, 0.02, 0.03], rotation_rpy=[0.1, 0.2, 0.3],
    )
    path = tmp_path / 'robot.yaml'
    path.write_text(yaml.safe_dump(config))
    robot = _render(path)
    housing = robot.find(".//visual[@name='schematic_camera_housing']")
    lens = robot.find(".//visual[@name='schematic_camera_lens']")
    assert housing is not None
    assert lens is not None
    assert _numbers(housing.find('geometry/box').get('size')) == pytest.approx(
        (0.12, 0.12, 0.15),
    )
    assert float(lens.find('geometry/cylinder').get('length')) == pytest.approx(0.024)
    assert float(lens.find('geometry/cylinder').get('radius')) == pytest.approx(0.08 / 3)
    joint = robot.find("joint[@name='tracking_frame_joint']")
    assert joint.find('parent').get('link') == 'wrist_link'
    assert _numbers(joint.find('origin').get('xyz')) == (0.21, 0.02, 0.03)
    assert _numbers(joint.find('origin').get('rpy')) == (0.1, 0.2, 0.3)


def test_complete_urdf_physical_description_matches_m08(robot):
    """Catch any cosmetic change to M08 geometry, inertia, joints or dynamics."""
    signature = json.dumps(_physical_tree(robot), separators=(',', ':'))
    assert hashlib.sha256(signature.encode()).hexdigest() == M08_PHYSICAL_SHA256


@pytest.mark.skipif(shutil.which('gz') is None, reason='Gazebo SDF tools unavailable')
def test_gazebo_fixed_joint_merge_retains_camera_without_physical_change(robot, tmp_path):
    """Catch massless-link visual loss or any physical SDF conversion change."""
    baseline = deepcopy(robot)
    tracking = baseline.find("link[@name='tracking_frame']")
    for visual in list(tracking.findall('visual')):
        tracking.remove(visual)
    before = _sdf(baseline, tmp_path / 'before.urdf')
    after = _sdf(robot, tmp_path / 'after.urdf')
    assert _physical_tree(after) == _physical_tree(before)
    assert len(after.findall('.//collision')) == 9
    assert len(after.findall('.//inertial')) == 9
    camera_visuals = [
        visual for visual in after.findall('.//visual')
        if 'schematic_camera_' in visual.get('name', '')
    ]
    assert len(camera_visuals) == 2
    wrist = after.find("model/link[@name='wrist_link']")
    assert all(visual in list(wrist) for visual in camera_visuals)
    assert float(wrist.find('inertial/mass').text) == pytest.approx(0.4)
