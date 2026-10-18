"""Analytical fixtures for physical, timestamped experiment evaluation."""

import json
from copy import deepcopy
from pathlib import Path

import numpy as np

import pytest

import yaml


def context():
    path = Path(__file__).parents[2] / 'robot_description/config/robot/robot.yaml'
    description = yaml.safe_load(path.read_text())
    return {'description': description, 'joint_names': ['base_yaw', 'shoulder', 'elbow', 'wrist'],
            'robot': {'pose': [0., 0., .25, 0., 0., 0.]},
            'control': {'tracking_radius': 1., 'tracking_forward_axis': [1., 0., 0.]},
            'world_frame': 'world', 'target_frame': 'world', 'estimator_frame': 'odom'}


def record(t, data, frame='world', arrival=None, clock=None):
    return {'source_time': t, 'arrival_monotonic': t if arrival is None else arrival,
            'clock': t if clock is None else clock, 'frame_id': frame, 'data': data}


def joints(t, q=None, dq=None):
    return record(t, {'names': context()['joint_names'], 'position': q or [0.] * 4,
                      'velocity': dq or [0.] * 4}, '')


def base(t, position=None, orientation=None):
    return record(t, {'position': position or [0., 0., .25],
                      'orientation': orientation or [0., 0., 0., 1.],
                      'linear_velocity': [0., 0., 0.], 'angular_velocity': [0., 0., 0.]})


def evaluate(streams, ctx=None, start=0., end=1., skew=.08):
    from robot_experiments.metrics import evaluate as implementation
    return implementation(streams, context() if ctx is None else ctx, start, end, skew)


def test_physical_fk_tracking_has_single_mount_and_optical_axis():
    # Wrist joint at x=.4+.36; mount adds .12; world height=.25+.1+.08.
    result = evaluate({'joints': [joints(.5)], 'base_truth': [base(.5)],
                       'target_truth': [record(.5, {'position': [2., 0., .43]})]})
    assert result['metrics']['tracking_distance']['mean'] == pytest.approx(.12, abs=1e-12)
    assert result['metrics']['pointing_error']['mean'] == pytest.approx(0., abs=1e-12)
    assert result['metrics']['tracking_distance']['unit'] == 'm'
    assert result['time_series']['tracking_distance']['samples'][0]['time'] == .5


def test_tracking_uses_world_rotation_and_signed_distance_is_absolute():
    half = 2 ** -.5
    result = evaluate({
        'joints': [joints(.5)],
        'base_truth': [base(.5, [3., 4., .25], [0., 0., half, half])],
        'target_truth': [record(.5, {'position': [3., 5.5, .43]})]})
    assert result['metrics']['tracking_distance']['mean'] == pytest.approx(.38)
    assert result['metrics']['pointing_error']['mean'] == pytest.approx(0., abs=1e-7)


def test_missing_truth_and_wrong_frames_are_unavailable_not_estimate_fallback():
    streams = {'joints': [joints(.5)], 'robot_estimate': [record(.5, {'mean': [0.] * 13})],
               'target_truth': [record(.5, {'position': [2., 0., .43]}, 'odom')]}
    result = evaluate(streams)
    assert not result['metrics']['tracking_distance']['available']
    assert result['metrics']['tracking_distance']['sample_count'] == 0
    assert result['metrics']['tracking_distance']['reason']
    assert result['safety']['observations_accepted'] is True
    assert result['safety']['accepted'] is None  # sparse evidence cannot cover window


def test_prediction_matches_absolute_future_truth_not_issue_estimate():
    truth = [record(t, {'position': [t, 0., 0.]}) for t in [.1, .2, .3, .4, .5]]
    pred = record(.1, {'timestamps': [.2, .4], 'states': [
        [.2, 0., 0., 1., 0., 0.], [1.4, 0., 0., 1., 0., 0.]]})
    result = evaluate({'target_truth': truth, 'prediction': [pred]}, skew=.11)
    horizons = result['metrics']['prediction_rmse']['horizons']
    assert horizons['0.1']['rmse'] == pytest.approx(0.)
    assert horizons['0.3']['rmse'] == pytest.approx(1.)
    assert result['time_series']['prediction_error']['samples'][1]['source_time'] == .1
    assert result['time_series']['prediction_error']['samples'][1]['time'] == .4


def test_prediction_horizons_group_nanosecond_source_stamp_quantization():
    second = {'timestamps': [.3], 'states': [[2., 0., 0., 0., 0., 0.]]}
    forecasts = [record(.1, {'timestamps': [.2], 'states': [[1., 0., 0., 0., 0., 0.]]}),
                 record(.199999999, second)]
    truth = [record(.2, {'position': [0., 0., 0.]}),
             record(.3, {'position': [0., 0., 0.]})]
    result = evaluate({'prediction': forecasts, 'target_truth': truth})
    horizons = result['metrics']['prediction_rmse']['horizons']
    assert list(horizons) == ['0.1']
    assert horizons['0.1']['sample_count'] == 2
    assert horizons['0.1']['rmse'] == pytest.approx(2.5 ** .5)
    samples = result['time_series']['prediction_error']['samples']
    assert samples[1]['source_time'] == .199999999
    assert samples[1]['time'] == .3


def test_prediction_interpolates_brackets_but_rejects_gaps_and_extrapolation():
    pred = record(.1, {'timestamps': [.3, .8, .95], 'states': [[.3, 0., 0., 0., 0., 0.]] * 3})
    result = evaluate({
        'target_truth': [record(.2, {'position': [.2, 0., 0.]}),
                         record(.4, {'position': [.4, 0., 0.]}),
                         record(.9, {'position': [.9, 0., 0.]})],
        'prediction': [pred]}, skew=.11)
    metric = result['metrics']['prediction_rmse']
    assert metric['sample_count'] == 1
    assert metric['rmse'] == pytest.approx(0.)
    assert metric['rejected_samples'] == 2


def test_future_issue_epoch_and_nonfinite_prediction_are_rejected():
    result = evaluate({
        'prediction': [record(.5, {'timestamps': [.4], 'states': [[0.] * 6]}),
                       record(.5, {'timestamps': [.6], 'states': [[float('nan')] * 6]})],
        'target_truth': [record(.4, {'position': [0.] * 3}), record(.6, {'position': [0.] * 3})]})
    assert not result['metrics']['prediction_rmse']['available']


def test_actual_joint_overspeed_fails_acceptance_with_irregular_bounded_duration():
    result = evaluate({'joints': [joints(.1, dq=[1.5, 0., 0., 0.]),
                                  joints(.13, dq=[2., 0., 0., 0.]), joints(.5)]})
    metric = result['metrics']['joint_speed_violation']
    assert metric['violation_samples'] == 2
    assert metric['max_overshoot'] == 1.
    assert metric['violation_duration'] == pytest.approx(.11)  # .03 + capped .08
    assert metric['observed_duration'] == pytest.approx(.19)
    assert result['safety']['accepted'] is False


def test_position_violation_and_reordered_joint_names_are_measured():
    sample = joints(.4, q=[0., -.02, 0., 0.])
    sample['data']['names'].reverse()
    sample['data']['position'].reverse()
    sample['data']['velocity'].reverse()
    result = evaluate({'joints': [sample]})
    assert result['metrics']['joint_position_violation']['max_overshoot'] == pytest.approx(.02)
    assert result['safety']['accepted'] is False


def test_missing_velocity_cannot_be_replaced_by_safe_commands():
    sample = joints(.4)
    sample['data']['velocity'] = []
    result = evaluate({'joints': [sample], 'command': [record(.4, {'velocity': [0.] * 6})]})
    assert not result['metrics']['joint_speed_violation']['available']
    assert result['safety']['accepted'] is None


def test_duplicate_source_stamp_cannot_hide_a_measured_speed_peak():
    result = evaluate({'joints': [joints(.4), joints(.4, dq=[4., 0., 0., 0.]), joints(.5)],
                       'command': [record(.4, {'velocity': [0.] * 6})]})
    assert result['metrics']['joint_speed_violation']['max_overshoot'] == pytest.approx(3.)
    assert result['safety']['accepted'] is False


def test_joint_estimation_error_does_not_require_unrelated_base_truth():
    state = [0.] * 5 + [.1, 0., 0., 0.] + [0.] * 4
    result = evaluate({'joints': [joints(.5)],
                       'robot_estimate': [record(.5, {'mean': state}, 'odom')]})
    assert result['metrics']['robot_joint_error']['mean'] == pytest.approx(.1)
    assert not result['metrics']['robot_position_error']['available']


def test_unsorted_duplicate_nonfinite_stale_records_are_auditable():
    truth = [record(.5, {'position': [2., 0., .43]}),
             record(.1, {'position': [2., 0., .43]}),
             record(.1, {'position': [2., 0., .43]}),
             record(float('nan'), {'position': [2., 0., .43]})]
    result = evaluate({'target_truth': truth, 'joints': [joints(.3)], 'base_truth': [base(.3)]})
    assert result['streams']['target_truth']['out_of_order'] == 1
    assert result['streams']['target_truth']['duplicates'] == 1
    assert result['streams']['target_truth']['invalid_timestamps'] == 1
    assert not result['metrics']['tracking_distance']['available']
    assert result['streams']['target_truth']['uncovered_duration'] == pytest.approx(.84)


def test_target_and_robot_estimation_align_documented_initial_pose_frame():
    ctx = context()
    ctx['robot']['pose'] = [3., 4., .25, 0., 0., np.pi / 2]
    half = 2 ** -.5
    streams = {
        'base_truth': [base(.5, [3., 5., .25], [0., 0., half, half])],
        'joints': [joints(.5)],
        'robot_estimate': [record(.5, {'mean': [1., 0., 0.] + [0.] * 10}, 'odom')],
        'target_truth': [record(.5, {'position': [2., 0., .43]})],
        'target_estimate': [record(.5, {'mean': [3., 0., .43, 0., 0., 0.]})]}
    result = evaluate(streams, ctx)
    assert result['metrics']['robot_position_error']['mean'] == pytest.approx(0., abs=1e-12)
    assert result['metrics']['robot_yaw_error']['mean'] == pytest.approx(0., abs=1e-12)
    assert result['metrics']['target_position_error']['mean'] == pytest.approx(1.)
    assert not result['metrics']['nees']['available']
    assert not result['metrics']['nis']['available']


def test_compute_rtf_event_deltas_and_freshness_use_distinct_clocks():
    clocks = [record(.1, {'time': .1}, arrival=10.), record(.3, {'time': .3}, arrival=10.4)]
    result = evaluate({
        'clock': clocks, 'control': [record(.2, {
            'valid': False, 'reason': 'stale', 'computation_time': .003}, clock=.25)],
        'actuation': [record(0., {'timeout_events': 4, 'rejected_commands': 3}),
                      record(.5, {'timeout_events': 6, 'rejected_commands': 4})]})
    assert result['metrics']['realtime_factor']['mean'] == pytest.approx(.5)
    assert result['metrics']['control_compute_time']['p95'] == pytest.approx(.003)
    assert result['metrics']['invalid_commands']['count'] == 1
    assert result['metrics']['watchdog_events']['count'] == 2
    assert result['metrics']['rejected_commands']['count'] == 1
    assert result['streams']['control']['freshness']['mean'] == pytest.approx(.05)


def test_batched_clock_delivery_does_not_inflate_realtime_factor_mean():
    clocks = [record(0., {'time': 0.}, arrival=10.),
              record(.1, {'time': .1}, arrival=10.0001),
              record(.2, {'time': .2}, arrival=10.4)]
    result = evaluate({'clock': clocks})
    assert result['metrics']['realtime_factor']['mean'] == pytest.approx(.5)
    assert result['metrics']['realtime_factor']['max'] == pytest.approx(.5)


def test_safety_tolerance_changes_acceptance_without_hiding_raw_overshoot():
    ctx = context()
    ctx['physical_numerical_tolerance'] = .001
    samples = [joints(i / 20., q=[0., -.0005, 0., 0.], dq=[1.0005, 0., 0., 0.])
               for i in range(21)]
    result = evaluate({'joints': samples}, ctx)
    assert result['safety']['accepted'] is True
    assert result['metrics']['joint_speed_violation']['max_overshoot'] == pytest.approx(.0005)
    assert result['metrics']['joint_position_violation']['max_overshoot'] == pytest.approx(.0005)
    assert result['metrics']['joint_speed_violation']['tolerance'] == .001


def test_safe_sparse_joint_samples_cannot_accept_an_unobserved_window():
    result = evaluate({'joints': [joints(0.), joints(.02)]})
    assert result['safety']['observations_accepted'] is True
    assert result['safety']['accepted'] is None
    assert result['safety']['coverage_complete'] is False
    coverage = result['safety']['coverage']['joint_speed_violation']
    assert coverage['observed_duration'] == pytest.approx(.1)
    assert coverage['window_duration'] == 1.
    assert coverage['coverage_fraction'] == pytest.approx(.1)
    assert coverage['max_gap'] == pytest.approx(.98)


def test_dense_safe_joint_samples_cover_the_declared_window():
    result = evaluate({'joints': [joints(i / 20.) for i in range(21)]})
    assert result['safety']['accepted'] is True
    assert result['safety']['observations_accepted'] is True
    assert result['safety']['coverage_complete'] is True
    coverage = result['safety']['coverage']['joint_speed_violation']
    assert coverage['coverage_fraction'] == pytest.approx(1.)
    assert coverage['max_gap'] == pytest.approx(.05)


def test_sparse_physical_violation_remains_failure_despite_missing_coverage():
    result = evaluate({'joints': [joints(.1, dq=[3., 0., 0., 0.])]})
    assert result['safety']['accepted'] is False
    assert result['safety']['observations_accepted'] is False
    assert result['safety']['coverage_complete'] is False


def test_last_prestart_joint_sample_covers_only_its_remaining_forward_hold():
    samples = [joints(-.01 + i / 20.) for i in range(21)]
    result = evaluate({'joints': samples})
    assert result['safety']['accepted'] is True
    assert result['safety']['coverage']['joint_speed_violation']['observed_duration'] == (
        pytest.approx(1.))
    first = result['time_series']['joint_speed_violation']['samples'][0]
    assert first['time'] == 0.
    assert first['source_time'] == -.01


def test_prestart_speed_peak_gates_only_when_its_bounded_hold_overlaps_window():
    recent = evaluate({'joints': [joints(-.01, dq=[2., 0., 0., 0.]), joints(.02)]})
    expired = evaluate({'joints': [joints(-.2, dq=[2., 0., 0., 0.]), joints(.02)]})
    assert recent['safety']['accepted'] is False
    assert recent['metrics']['joint_speed_violation']['violation_duration'] == pytest.approx(.02)
    assert expired['safety']['accepted'] is None
    assert expired['safety']['observations_accepted'] is True


def test_missing_metrics_serialize_strictly_and_input_is_not_mutated():
    streams = {'target_truth': [record(.5, {'position': [float('nan'), 0., 0.]})]}
    original = deepcopy(streams)
    result = evaluate(streams)
    assert result['schema_version'] == 1
    assert result['safety']['accepted'] is None
    json.dumps(result, allow_nan=False)
    assert streams['target_truth'][0]['source_time'] == original['target_truth'][0]['source_time']


@pytest.mark.parametrize('start,end,skew', [(1., 0., .08), (0., 1., 0.), (0., float('nan'), .08)])
def test_invalid_window_rejected(start, end, skew):
    with pytest.raises(ValueError):
        evaluate({}, start=start, end=end, skew=skew)
