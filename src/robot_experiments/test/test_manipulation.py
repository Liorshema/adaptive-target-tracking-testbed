"""Analytical phase metrics use physical names, frames and observed intervals."""

from pathlib import Path
import math

import numpy as np
import pytest
import yaml


def context():
    """Use the actual authoritative arm and camera mount."""
    path = Path(__file__).resolve().parents[2] / 'robot_description/config/robot/robot.yaml'
    return {'description': yaml.safe_load(path.read_text()),
            'joint_names': ['base_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint'],
            'world_frame': 'world',
            'control': {'tracking_radius': 1., 'tracking_forward_axis': [1., 0., 0.]},
            'physical_numerical_tolerance': .001}


def record(time, data, frame='world'):
    """Only normalized source-time fields are needed by the pure analysis."""
    return {'source_time': time, 'frame_id': frame, 'data': data}


def fixture(times, azimuths=None, headings=None):
    """Provide independent physical base, target and measured joint observations."""
    azimuths = [0.] * len(times) if azimuths is None else azimuths
    headings = [0.] * len(times) if headings is None else headings
    streams = {'base_truth': [], 'target_truth': [], 'joints': []}
    for time, angle, heading in zip(times, azimuths, headings):
        streams['base_truth'].append(record(time, {
            'position': [0., 0., .25],
            'orientation': [0., 0., math.sin(heading / 2), math.cos(heading / 2)]}))
        streams['target_truth'].append(record(time, {
            'position': [.55 * math.cos(angle), .55 * math.sin(angle), 1.85]}))
        streams['joints'].append(record(time, {
            'names': context()['joint_names'], 'position': [0., .5, -.4, 0.],
            'velocity': [0., 0., 0., 0.]}, ''))
    return streams


def analyze(streams, start=0., end=.2, phases=None, **kwargs):
    """Import the feature at call time so missing implementation is a test failure."""
    from robot_experiments.metrics.manipulation import analyze_phases
    kwargs.setdefault("overhead_region", {"rho":[.35,.8],"height":[1.4,1.8]})
    return analyze_phases(streams, context(), phases or [
        {'name': 'test', 'start': start, 'end': end}], start, end, **kwargs)


def test_joint_names_reorder_per_record_and_physical_utilization_is_not_a_mixed_norm():
    streams = fixture([0., .1, .2])
    for index, sample in enumerate(streams['joints']):
        names = ['wheel', 'wrist_joint', 'elbow_joint', 'shoulder_joint', 'base_yaw_joint']
        sample['data'] = {'names': names, 'position': [90., .1, -.4, .5 + .1 * index, 0.],
                          'velocity': [90., .3, .2, .4, .1]}
    result = analyze(streams)
    joints = result['windows']['test']['joints']
    assert list(joints) == context()['joint_names']
    assert joints['shoulder_joint']['angular_range_rad'] == pytest.approx(.2)
    assert joints['wrist_joint']['peak_speed_utilization'] == pytest.approx(.2)
    assert joints['wrist_joint']['q_fraction_of_range']['min'] == pytest.approx((.1 + 1.57) / 3.14)
    assert joints['shoulder_joint']['observed_duration_s'] == pytest.approx(.2)


def test_gap_exclusion_does_not_invent_rear_crossings_or_angular_coverage():
    streams = fixture([0., .1, 1.], [0., .1, math.pi])
    result = analyze(streams, end=1.)['windows']['test']['target']
    assert result['rear_hemisphere_observed'] is True
    assert result['rear_entries'] == 0
    assert result['rear_dwell_observed_s'] == 0.
    assert result['relative_azimuth']['observed_travel_rad'] == pytest.approx(.1)
    assert result['relative_azimuth']['angular_coverage_rad'] == pytest.approx(.1)
    assert result['relative_azimuth']['observed_duration_s'] == pytest.approx(.1)
    assert result['relative_azimuth']['segment_count'] == 2


def test_behind_is_relative_to_physical_heading_and_base_unwrap_is_separate():
    streams = fixture([0., .1, .2], [0., math.pi / 2, math.pi],
                      [0., math.pi / 2, math.pi])
    result = analyze(streams)['windows']['test']
    assert result['target']['rear_hemisphere_observed'] is False
    assert result['target']['relative_azimuth']['unwrapped_excursion_rad'] == pytest.approx(0.)
    assert result['base']['yaw']['unwrapped_excursion_rad'] == pytest.approx(math.pi)
    streams = fixture([0., .1], [0., 0.], [math.pi, math.pi])
    result = analyze(streams)['windows']['test']['target']
    assert result['rear_hemisphere_observed'] is True
    assert result['rear_dwell_observed_s'] == pytest.approx(.1)


def test_aft_axis_crossing_unwraps_branch_cut_without_a_full_turn():
    result = analyze(fixture([0., .1], [2.9, -3.0]))['windows']['test']['target']
    assert result['aft_axis_crossings'] == 1
    assert result['rear_entries'] == 0
    assert result['relative_azimuth']['observed_travel_rad'] == pytest.approx(2 * math.pi - 5.9)


def test_physical_alignment_is_causal_and_stale_or_wrong_frame_is_unavailable():
    streams = fixture([0., .1, .2])
    streams['base_truth'][0]['source_time'] = .01
    streams['joints'][0]['source_time'] = .01
    streams['base_truth'][2]['frame_id'] = 'odom'
    result = analyze(streams, max_skew=.05)
    assert result['series'][0]['rho_m'] is None
    assert result['series'][0]['reach_m'] is None
    assert result['series'][1]['rho_m'] == pytest.approx(.55)
    assert result['series'][2]['rho_m'] is None
    assert result['availability']['base_alignment_missing'] == 2
    assert 'causal' in result['alignment_policy']


def test_authoritative_camera_fk_single_mount_and_shoulder_relative_reach():
    streams = fixture([0., .1])
    for sample in streams['joints']:
        sample['data']['position'] = [0.] * 4
    for sample in streams['target_truth']:
        sample['data']['position'] = [1.88, 0., .43]
    result = analyze(streams)
    assert result['series'][0]['reach_m'] == pytest.approx(.88)
    assert result['series'][0]['distance_error_m'] == pytest.approx(0., abs=1e-12)
    assert result['series'][0]['pointing_error_rad'] == pytest.approx(0., abs=1e-12)


def test_vertical_velocity_and_overhead_dwell_do_not_bridge_target_gaps():
    streams = fixture([0., .1, 1.])
    streams['target_truth'][1]['data']['position'][2] += .2
    result = analyze(streams, end=1., overhead_region={'rho': [.35, .8], 'height': [1.4, 1.8]})
    assert result['series'][0]['vertical_velocity_m_s'] is None
    assert result['series'][1]['vertical_velocity_m_s'] == pytest.approx(2.)
    assert result['series'][2]['vertical_velocity_m_s'] is None
    assert result['windows']['test']['target']['overhead_dwell_observed_s'] == pytest.approx(.1)


def test_startup_and_phase_joint_motion_remain_separate():
    streams = fixture([0., .1, .2, .3])
    for index, sample in enumerate(streams['joints']):
        sample['data']['position'][0] = index * .1
    result = analyze(streams, start=.1, end=.3, phases=[
        {'name': 'first', 'start': .1, 'end': .2},
        {'name': 'second', 'start': .2, 'end': .3}])
    assert result['windows']['startup']['joints']['base_yaw_joint']['angular_range_rad'] == pytest.approx(.1)
    assert result['windows']['first']['joints']['base_yaw_joint']['angular_range_rad'] == pytest.approx(.1)
    assert result['windows']['active']['joints']['base_yaw_joint']['angular_range_rad'] == pytest.approx(.2)


def test_missing_feedback_is_explicit_and_json_contains_no_nan():
    import json
    result = analyze({})
    assert result['windows']['test']['target']['rho']['available'] is False
    assert result['windows']['test']['joints']['wrist_joint']['available'] is False
    assert result['windows']['test']['reach']['available'] is False
    json.dumps(result, allow_nan=False)


def test_clock_resets_and_overlapping_phase_windows_are_rejected():
    with pytest.raises(ValueError, match='epoch|backwards|increas'):
        analyze(fixture([.1, 0.]))
    with pytest.raises(ValueError, match='overlap'):
        analyze(fixture([0., .1]), phases=[
            {'name': 'one', 'start': 0., 'end': .15},
            {'name': 'two', 'start': .1, 'end': .2}])


def test_latched_position_stop_is_counted_from_actual_guard_reason_contract():
    streams=fixture([0.,.1,.2])
    streams['actuation']=[record(t,{'active':False,'reason':'Serious position violation (latched)'})
                          for t in [0.,.1,.2]]
    result=analyze(streams)['windows']['test']['health']['actuation']
    assert result['fault_observed_duration_s']==pytest.approx(.2)
    assert result['inactive_observed_duration_s']==pytest.approx(.2)


def test_exported_relative_azimuth_is_unwrapped_only_inside_observed_segments():
    result=analyze(fixture([0.,.1,1.],[2.9,-3.,0.]),end=1.)
    rows=result['series']
    assert rows[1]['relative_azimuth_rad']>math.pi
    assert rows[1]['relative_azimuth_rad_segment']==rows[0]['relative_azimuth_rad_segment']
    assert rows[2]['relative_azimuth_rad_segment']!=rows[1]['relative_azimuth_rad_segment']
