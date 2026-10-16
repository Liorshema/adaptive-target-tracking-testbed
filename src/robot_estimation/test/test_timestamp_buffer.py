"""A shared filter must receive distinct observations in causal stamp order."""

import numpy as np

import pytest

from robot_estimation.common.timestamp_buffer import TimestampBuffer


def buffer(delay=.03):
    """Use independent source identities, without sensor-specific policies."""
    return TimestampBuffer(delay, .2, .01, 10)


def test_interleaved_delayed_streams_are_sorted_and_equal_time_sources_both_survive():
    """Receipt order cannot move the shared EKF past pending earlier evidence."""
    b = buffer()
    assert b.push('one', 1.02, 'later', 1.024)
    assert b.push('two', 1.01, 'earlier', 1.026)
    assert b.push('two', 1.02, 'same-time', 1.028)
    assert b.pop_ready(1.03) == []
    assert [e[2] for e in b.pop_ready(1.06)] == ['earlier', 'later', 'same-time']
    assert b.counters['two']['accepted'] == 2
    assert b.counters['two']['delayed'] == 1


def test_duplicate_pending_and_processed_observations_are_not_fused_twice():
    """Duplicate pending and processed observations are not fused twice."""
    b = buffer()
    assert b.push('source', 1., None, 1.)
    assert not b.push('source', 1., None, 1.01)
    b.pop_ready(1.04)
    assert not b.push('source', 1., None, 1.05)
    assert b.counters['source']['duplicate'] == 2


def test_too_late_stale_future_and_nonfinite_are_rejected_without_rewind():
    """Too late stale future and nonfinite are rejected without rewind."""
    b = buffer()
    b.push('a', 1., None, 1.)
    b.pop_ready(1.04)
    assert not b.push('b', .999, None, 1.04)
    assert not b.push('b', .5, None, 1.04)
    assert not b.push('b', 2., None, 1.04)
    assert not b.push('b', np.nan, None, 1.04)
    assert b.committed == 1.
    assert b.counters['b']['late'] == 1
    assert b.counters['b']['stale'] == 1
    assert b.counters['b']['future'] == 1


def test_reset_clears_old_epoch_queue_and_allows_lower_time():
    """Reset clears old epoch queue and allows lower time."""
    b = buffer()
    b.push('a', 10., 'old epoch', 10.)
    b.reset()
    assert b.push('a', 1., 'new epoch', 1.)
    assert [e[2] for e in b.pop_ready(1.04)] == ['new epoch']


def test_missing_stream_does_not_block_other_sources_or_unbound_memory():
    """Missing stream does not block other sources or unbound memory."""
    b = TimestampBuffer(.03, .2, .01, 2)
    assert b.push('a', 1., 1, 1.)
    assert b.push('a', 1.01, 2, 1.01)
    assert not b.push('a', 1.02, 3, 1.02)
    assert len(b.pop_ready(1.1)) == 2
    assert b.counters['a']['overflow'] == 1


@pytest.mark.parametrize('values', [(-.1, .2, .01, 10), (.3, .2, .01, 10),
                                    (.03, .2, .01, 0)])
def test_invalid_buffer_budget_is_rejected(values):
    """Invalid buffer budget is rejected."""
    with pytest.raises(ValueError):
        TimestampBuffer(*values)


def test_reordering_within_one_source_is_valid_before_commit():
    """A source-specific arrival watermark must not reject a still-buffered stamp."""
    b = buffer()
    b.push('source', 1.02, 'second', 1.025)
    b.push('source', 1.01, 'first', 1.027)
    assert [e[2] for e in b.pop_ready(1.06)] == ['first', 'second']
