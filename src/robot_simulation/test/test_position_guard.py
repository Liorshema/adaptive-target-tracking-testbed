"""Physical excursions require directed, bounded recovery or a latched fault."""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import pytest

from robot_simulation.actuation.position_guard import PositionGuard

import yaml


def guard():
    """Create a deliberately asymmetric generic test envelope."""
    return PositionGuard([0., -1.], [1., 1.], [1., 2.],
                         numerical_tolerance=1e-9, recoverable_violation=.001,
                         recovery_speed=.1, recovery_inset=.04, release_margin=.005)


@pytest.mark.parametrize('q', [[0., -1.], [1., 1.], [.5, 0.]])
def test_exact_limits_are_legal(q):
    """Keep exact physical boundaries legal without inactive release authority."""
    g = guard()
    assert g.observe(q) == 'normal'


@pytest.mark.parametrize('q,mode', [
    ([-1e-13, 0.], 'numerical_recovery'), ([-.0005, 0.], 'recovery'),
    ([1.0005, 0.], 'recovery')])
def test_excursions_are_classified_and_only_affected_reference_moves_inward(q, mode):
    """Classify both sides and respect physical reference speed and position bounds."""
    g = guard()
    assert g.observe(q) == mode
    reference = np.clip(q, g.lower, g.upper)
    result = g.step(reference, q, .02)
    assert abs(result[0] - reference[0]) <= .1 * .02 + 1e-15
    assert (result[0] - reference[0]) * (.5 - q[0]) > 0
    assert result[1] == reference[1]
    assert np.all(result >= g.lower) and np.all(result <= g.upper)
    assert g.events == 1


def test_recovery_is_hysteretic_and_does_not_chatter_at_zero():
    """Release only after measured motion clears the interior margin."""
    g = guard()
    g.observe([-1e-13, 0.])
    for q in [1e-13, 0., -.0001, .0049]:
        assert g.observe([q, 0.]) != 'normal'
    assert g.observe([.005, 0.]) == 'normal'
    assert g.events == 1


@pytest.mark.parametrize('q', [[-.001001, 0.], [1.002, 0.], [0., -1.01]])
def test_serious_fault_cannot_be_cleared_by_good_feedback_or_step(q):
    """A new sample cannot acknowledge a serious physical fault."""
    g = guard()
    assert g.observe(q) == 'fault'
    assert g.observe([.5, 0.]) == 'fault'
    with pytest.raises(ValueError, match='latched'):
        g.step([.5, 0.], [.5, 0.], .02)


def test_no_outward_reference_even_if_previous_hold_is_further_inside():
    """Recovery must not reverse an already safer reference."""
    g = guard()
    np.testing.assert_allclose(g.step([.07, 0.], [-.0001, 0.], .02), [.07, 0.])


@pytest.mark.parametrize('dt', [0., -.1, float('nan')])
def test_recovery_rejects_invalid_intervals(dt):
    """Invalid time cannot advance recovery."""
    with pytest.raises(ValueError):
        guard().step([0., 0.], [-1e-13, 0.], dt)


@pytest.mark.parametrize('q,direction', [(0., 1), (1e-13, 1), (1., -1), (1. - 1e-13, -1)])
def test_inactive_boundary_release_handles_legal_physical_roundoff(q, direction):
    """An invalid estimate must not deadlock a measured joint at its hard stop."""
    g = guard()
    assert g.observe([q, 0.]) == 'normal'
    assert g.observe([q, 0.], release_boundary=True) == 'numerical_recovery'
    result = g.step([q, 0.], [q, 0.], .02)
    assert (result[0] - q) * direction > 0


def test_boundary_release_does_not_move_ordinary_legal_positions():
    """The numerical inside band is narrow and explicit."""
    g = guard()
    assert g.observe([1e-7, 0.], release_boundary=True) == 'normal'


def test_legacy_pid_recovery_offset_can_lift_the_horizontal_loaded_arm():
    """Legacy PID offset rationale only; native velocity servo needs physical tests."""
    root = Path(__file__).resolve().parents[2]
    simulation = root / 'robot_simulation'
    robot_path = root / 'robot_description/config/robot/robot.yaml'
    hold_path = simulation / 'config/gazebo/joint_position_hold.yaml'
    robot = yaml.safe_load(robot_path.read_text())['robot']
    hold = yaml.safe_load(hold_path.read_text())['joint_position_hold']
    cfg = yaml.safe_load((simulation / 'config/actuation.yaml').read_text())['actuation']
    arm = robot['manipulator']
    world = ET.parse(simulation / 'worlds/tracking_testbed.sdf.xacro')
    gravity = np.linalg.norm([float(x) for x in world.find('world/gravity').text.split()])
    lever1 = arm['link1']['length'] / 2
    lever2 = arm['link1']['length'] + arm['link2']['length'] / 2
    lever3 = arm['link1']['length'] + arm['link2']['length'] + arm['wrist']['length'] / 2
    gravity_load = gravity * (arm['link1']['mass'] * lever1 +
                              arm['link2']['mass'] * lever2 +
                              arm['wrist']['mass'] * lever3)
    pid = dict(hold['pid'], **hold['joint_overrides']['shoulder']['pid'])
    recovery = cfg['position_recovery']
    available = pid['p_gain'] * (recovery['recovery_inset'] - recovery['release_margin'])
    # Integral saturation is expressed in effort units by gz-math PID.
    assert available - max(abs(pid['i_min']), abs(pid['i_max'])) > gravity_load
    assert gravity_load < arm['joints']['shoulder']['effort']
    assert recovery['recovery_inset'] <= cfg['max_position_lead']
