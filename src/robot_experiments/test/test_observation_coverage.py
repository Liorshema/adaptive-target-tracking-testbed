"""Physical coverage contracts using D2.9 gap epochs and synthetic test payloads.

Only the timestamps/gap endpoints come from retained evidence. These fixtures
never create, repair or stand in for experimental Run observations.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from robot_experiments.recording import TelemetryStore
from robot_experiments.recording.observer_timing import ObserverTiming
from robot_experiments.runner import _acceptance
from test_metrics import evaluate, joints
from test_recording import fake_ros, header, queued_executor, recording


START, END = 10.476, 10.660
# Exact local CV epochs around its retained 10.484 -> 10.628 gap.
CV_TIMES = [10.476, 10.478, 10.480, 10.482, 10.484, 10.628, 10.644, 10.660]


def coverage(records, start=START, end=END):
    """Use the production evaluator and its unchanged default 80 ms hold."""
    return evaluate({'joints': records}, start=start, end=end)


def message(time):
    """Synthetic valid JointState understood by actual normalization/storage."""
    values = joints(time)['data']
    return NS(header=header(time, ''), name=values['names'],
              position=values['position'], velocity=values['velocity'], effort=[])


def test_retained_cv_internal_gap_has_exact_64ms_unobserved_interval():
    result = coverage([joints(time) for time in CV_TIMES])
    assert result['safety']['observations_accepted'] is True
    assert result['safety']['accepted'] is None
    for name in ('joint_position_violation', 'joint_speed_violation'):
        observed = result['safety']['coverage'][name]
        assert observed['max_gap'] == pytest.approx(.144)
        assert observed['observed_duration'] == pytest.approx(.120)
        assert (END - START) - observed['observed_duration'] == pytest.approx(.064)
        assert observed['coverage_fraction'] == pytest.approx(.120 / .184)
        assert observed['coverage_complete'] is False


def test_retained_imm_20ms_gap_is_below_unchanged_hold_budget():
    times = [7.650, 7.652, 7.654, 7.674, 7.676, 7.678]
    result = coverage([joints(time) for time in times], start=times[0], end=times[-1])
    assert result['safety']['accepted'] is True
    assert result['safety']['coverage']['joint_speed_violation']['max_gap'] == pytest.approx(.020)
    assert result['safety']['coverage']['joint_speed_violation']['coverage_fraction'] == pytest.approx(1.)


def test_missing_start_is_not_covered_backwards_by_later_observations():
    result = coverage([joints(time) for time in [10.500, 10.550, 10.600, 10.650, END]])
    assert result['safety']['accepted'] is None
    assert result['safety']['coverage']['joint_speed_violation']['observed_duration'] == pytest.approx(.160)


@pytest.mark.parametrize('future_observation', [False, True])
def test_missing_end_is_not_backfilled_or_extrapolated_beyond_hold(future_observation):
    records = [joints(time) for time in CV_TIMES[:5]]
    if future_observation:
        records.append(joints(10.680))
    result = coverage(records)
    observed = result['safety']['coverage']['joint_speed_violation']
    assert observed['observed_duration'] == pytest.approx(.088)
    assert observed['max_gap'] == pytest.approx(.176)
    assert result['safety']['accepted'] is None


@pytest.mark.parametrize('prestart,accepted', [(10.474, True), (10.390, None)])
def test_only_unexpired_prestart_observation_can_cover_window_start(prestart, accepted):
    records = [joints(time) for time in [prestart, 10.500, 10.550, 10.600, 10.650, END]]
    result = coverage(records)
    assert result['safety']['accepted'] is accepted
    assert result['safety']['coverage']['joint_speed_violation']['observed_duration'] == pytest.approx(
        .184 if accepted else .160)


def test_reordering_and_duplicate_epochs_do_not_create_extra_coverage():
    original = [joints(time) for time in CV_TIMES]
    records = list(reversed(original)) + [deepcopy(original[3])]
    before = deepcopy(records)
    result = coverage(records)
    assert records == before
    assert result['streams']['joints']['out_of_order'] > 0
    assert result['streams']['joints']['duplicates'] == 1
    assert result['safety']['coverage']['joint_speed_violation']['observed_duration'] == pytest.approx(.120)
    assert result['safety']['accepted'] is None


def test_duplicate_epoch_cannot_hide_measured_violation_despite_missing_coverage():
    records = [joints(time) for time in CV_TIMES]
    records.append(joints(10.484, dq=[100., 0., 0., 0.]))
    result = coverage(records)
    assert result['safety']['coverage_complete'] is False
    assert result['safety']['observations_accepted'] is False
    assert result['safety']['accepted'] is False
    assert result['metrics']['joint_speed_violation']['violation_samples'] == 1


def test_position_and_velocity_require_independent_valid_observations():
    records = [joints(time) for time in CV_TIMES]
    records[5]['data']['velocity'] = []
    result = coverage(records)
    assert result['safety']['coverage']['joint_position_violation']['observed_duration'] == pytest.approx(.120)
    assert result['safety']['coverage']['joint_speed_violation']['observed_duration'] == pytest.approx(.104)
    assert result['metrics']['joint_speed_violation']['rejected_samples'] == 1
    assert result['safety']['accepted'] is None


def test_wall_blackout_alone_does_not_erase_delivered_source_coverage():
    records = [joints(time) for time in [10.476, 10.496, 10.516, 10.536, 10.556, 10.576, 10.596, 10.616, 10.636, 10.656, END]]
    for index, record in enumerate(records):
        record['arrival_monotonic'] = 100. + index * .001 + (.215206 if index >= 5 else 0.)
        record['clock'] = 10.486  # Legitimate asynchronous lag; source stamps stay authoritative.
    before = deepcopy(records)
    result = coverage(records)
    assert result['safety']['accepted'] is True
    assert result['safety']['coverage']['joint_speed_violation']['observed_duration'] == pytest.approx(.184)
    assert records == before


def test_store_preserves_supplied_observations_and_does_not_fill_source_gap(tmp_path):
    store = TelemetryStore({'topics': {'joints': '/joint_states'}}, tmp_path)
    for index, time in enumerate(CV_TIMES):
        store.record('joints', message(time), arrival_monotonic=100. + index)
    store.close()
    saved = [json.loads(line) for line in (tmp_path / 'joints.jsonl').read_text().splitlines()]
    assert saved == store.streams['joints']
    assert [r['source_time'] for r in saved] == pytest.approx(CV_TIMES)
    assert len(saved) == len(CV_TIMES)
    assert store.availability['joints']['invalid_count'] == 0
    assert store.availability['joints']['record_count'] == len(CV_TIMES)
    assert coverage(saved)['safety']['accepted'] is None


def test_unobserved_transport_loss_is_not_invented_as_recorder_rejections(tmp_path, monkeypatch):
    resources = fake_ros(monkeypatch)
    recorder = recording().Recorder({'topics': {'joints': '/joint_states'}}, tmp_path)
    queue = queued_executor(resources, monkeypatch, [('/joint_states', message(time)) for time in CV_TIMES])
    recorder.spin_once()
    recorder.close()
    assert not queue
    metadata = recorder.availability['joints']
    assert metadata['record_count'] == len(CV_TIMES) and metadata['invalid_count'] == 0
    assert metadata['out_of_order_count'] == 0
    assert [r['source_time'] for r in recorder.streams['joints']] == pytest.approx(CV_TIMES)
    assert coverage(recorder.streams['joints'])['safety']['accepted'] is None
    # No publisher sequence evidence was supplied: local counts cannot locate upstream loss.
    assert not recorder.events


def test_optional_dds_sequence_evidence_exposes_loss_without_repairing_samples(tmp_path, monkeypatch):
    resources = fake_ros(monkeypatch)
    timing = ObserverTiming(tmp_path / 'diagnostics', ['joints'], monitor=False)
    recorder = recording().Recorder({'topics': {'joints': '/joint_states'}}, tmp_path / 'raw', diagnostics=timing)
    callback = resources.nodes[0].subscriptions[0][2]
    # Test-only 500 Hz publisher: omit precisely 71 epochs inside the actual CV gap.
    epochs = [10476, 10478, 10480, 10482, 10484, *range(10628, 10661, 2)]
    for received, milliseconds in enumerate(epochs, 1):
        callback(message(milliseconds / 1000), NS(
            publisher_gid=bytes([1] * 24), publication_sequence_number=(milliseconds - 10476) // 2 + 1,
            reception_sequence_number=received, source_timestamp=1, received_timestamp=2))
    recorder.close()
    metadata = timing.snapshot()['middleware']['streams']['joints']
    assert metadata['publication_sequence_gaps'] == 71
    assert len(recorder.streams['joints']) == len(epochs)
    assert coverage(recorder.streams['joints'])['safety']['accepted'] is None
    assert 'not loss-location proof' in timing.snapshot()['middleware']['sequence_gap_semantics']


def acceptance_document(safety):
    """Test-only availability for unrelated metrics; no experimental Run is created."""
    return {'safety': safety, 'metrics': {
        **{name: {'available': True, 'count': 0} for name in (
            'invalid_commands', 'watchdog_events', 'rejected_commands', 'feedback_rejections')},
        **{name: {'available': True} for name in (
            'tracking_distance', 'pointing_error', 'prediction_rmse', 'target_position_error')}}}


def test_finished_window_with_safe_observed_samples_still_has_unknown_acceptance():
    result = coverage([joints(time) for time in CV_TIMES])
    accepted, gates = _acceptance(acceptance_document(result['safety']), True)
    assert accepted is None and gates['physical_joint_limits'] is None
    assert result['safety']['observations_accepted'] is True


def test_complete_coverage_does_not_demonstrate_disabled_actuation():
    result = coverage([joints(time) for time in [10.476, 10.526, 10.576, 10.626, END]])
    accepted, gates = _acceptance(acceptance_document(result['safety']), False)
    assert result['safety']['accepted'] is True
    assert gates['physical_joint_limits'] is True
    assert gates['physical_actuation_demonstrated'] is None and accepted is None
