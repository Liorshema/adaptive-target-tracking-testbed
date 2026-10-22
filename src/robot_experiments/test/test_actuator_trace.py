"""Analytical physical trace acceptance, independent of ROS delivery quality."""
import csv

import pytest

from robot_experiments.metrics.actuator_trace import analyze_trace


def trace(tmp_path, *, times=(0.002, 0.004, 0.006), speeds=(0.0, 0.0, 0.0), names=('arm',)):
    """Write known physical observations with unavailable native motor torque."""
    path = tmp_path / 'actuator.csv'
    keys = ['time', 'iteration', 'dt', 'name', 'q_post', 'v_post',
            'position_reference', 'commanded_effort', 'saturation', 'invalid_commands']
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        for i, (t, v) in enumerate(zip(times, speeds)):
            for name in names:
                writer.writerow({
                    'time': t, 'iteration': i + 1, 'dt': 0.002, 'name': name,
                    'q_post': 0.0, 'v_post': v, 'position_reference': 0.0,
                    'commanded_effort': 'nan', 'saturation': 0, 'invalid_commands': 0})
    context = {'joint_names': ['arm'], 'description': {'robot': {'manipulator': {
        'joints': {'generic': {'lower': -1.0, 'upper': 1.0, 'velocity': 1.0,
                               'effort': 2.0}}}}}}
    return (path, context)


def test_complete_trace_and_unavailable_torque_are_explicit(tmp_path):
    """Unavailable native torque is distinct from a reported zero command."""
    p, c = trace(tmp_path)
    result = analyze_trace(p, c, 0.004)
    assert result['accepted'] and result['complete']
    assert result['joints']['arm']['max_abs_written_effort_nm'] is None
    assert result['joints']['arm']['coverage_fraction'] == 1.0


@pytest.mark.parametrize('speeds,accepted', [
    ((1.0009, 0.0, 0.0), True), ((1.002, 0.0, 0.0), False)])
def test_startup_speed_uses_unchanged_tolerance(tmp_path, speeds, accepted):
    """A startup violation cannot disappear through later valid tracking."""
    p, c = trace(tmp_path, speeds=speeds)
    r = analyze_trace(p, c, 0.004)
    assert r['accepted'] is accepted


@pytest.mark.parametrize('times', [(0.002, 0.006, 0.008), (0.1, 0.102, 0.104)])
def test_feedback_gap_or_missing_startup_prevents_pass(tmp_path, times):
    """Even zero measured speed does not certify an uncovered interval."""
    p, c = trace(tmp_path, times=times)
    result = analyze_trace(p, c, 0.104)
    assert not result['accepted'] and (not result['complete'])


def test_missing_joint_does_not_pass(tmp_path):
    """Every configured arm joint needs actual physical observations."""
    p, c = trace(tmp_path, names=('other',))
    result = analyze_trace(p, c, 0.004)
    assert not result['accepted'] and (not result['complete'])


def test_inconsistent_layout_is_rejected(tmp_path):
    """An extra configured name cannot disappear through zip truncation."""
    p, c = trace(tmp_path)
    c['joint_names'].append('missing')
    with pytest.raises(ValueError, match='layout'):
        analyze_trace(p, c, 0.004)


def test_truncated_final_joint_iteration_is_incomplete(tmp_path):
    """One joint cannot silently have a shorter observed shutdown horizon."""
    p, c = trace(tmp_path, names=('arm', 'other'))
    c['joint_names'].append('other')
    c['description']['robot']['manipulator']['joints']['second'] = {
        'lower': -1., 'upper': 1., 'velocity': 1., 'effort': 2.}
    p.write_text('\n'.join(p.read_text().splitlines()[:-1]) + '\n')
    result = analyze_trace(p, c, .004)
    assert not result['accepted'] and not result['complete']
