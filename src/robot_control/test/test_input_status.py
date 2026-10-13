
"""Test control-input readiness states and transitions."""

import pytest

from robot_control.safety.input_status import (
    InputStatus,
    InputStatusSnapshot,
    InputStatusTracker,
)


def test_initial_status():
    """Start in WAITING without readiness for control."""
    tracker = InputStatusTracker()

    assert tracker.snapshot.status == InputStatus.WAITING
    assert tracker.snapshot.reason == "Waiting for control inputs"
    assert not tracker.is_ready


def test_transition_to_valid():
    """Mark inputs ready following successful validation."""
    tracker = InputStatusTracker()

    assert tracker.set_valid() is True
    assert tracker.snapshot.status == InputStatus.VALID
    assert tracker.snapshot.reason == ""
    assert tracker.is_ready


def test_repeated_valid_does_not_change():
    """Avoid repeated notifications for an unchanged valid state."""
    tracker = InputStatusTracker()
    tracker.set_valid()

    assert tracker.set_valid() is False


def test_transition_to_invalid():
    """Reject invalid inputs and retain their failure reason."""
    tracker = InputStatusTracker()

    assert tracker.set_invalid("Robot state is stale") is True
    assert tracker.snapshot.status == InputStatus.INVALID
    assert tracker.snapshot.reason == "Robot state is stale"
    assert not tracker.is_ready


def test_repeated_invalid_reason_does_not_change():
    """Avoid duplicate updates for an identical failure."""
    tracker = InputStatusTracker()
    tracker.set_invalid("Robot state is stale")

    assert tracker.set_invalid("Robot state is stale") is False


def test_changed_invalid_reason_is_reported():
    """Report a meaningful change in validation failure."""
    tracker = InputStatusTracker()
    tracker.set_invalid("Robot state is stale")

    assert tracker.set_invalid("Frame mismatch") is True
    assert tracker.snapshot.reason == "Frame mismatch"


def test_recover_from_invalid_to_valid():
    """Recover readiness when new inputs pass validation."""
    tracker = InputStatusTracker()
    tracker.set_invalid("Stale prediction")

    assert tracker.set_valid() is True
    assert tracker.is_ready


def test_return_to_waiting():
    """Return to waiting when input availability is reset."""
    tracker = InputStatusTracker()
    tracker.set_valid()

    assert tracker.set_waiting() is True
    assert tracker.snapshot.status == InputStatus.WAITING
    assert not tracker.is_ready


def test_repeated_waiting_does_not_change():
    """Avoid repeated updates for identical waiting status."""
    tracker = InputStatusTracker()

    assert tracker.set_waiting() is False


def test_snapshot_readiness():
    """Expose readiness directly from immutable snapshots."""
    assert InputStatusSnapshot(InputStatus.VALID).is_ready
    assert not InputStatusSnapshot(InputStatus.WAITING).is_ready
    assert not InputStatusSnapshot(InputStatus.INVALID).is_ready


@pytest.mark.parametrize("invalid_status", ["valid", None, 1, True])
def test_reject_invalid_status_type(invalid_status):
    """Require status values to use the InputStatus enum."""
    tracker = InputStatusTracker()

    with pytest.raises(TypeError, match="InputStatus"):
        tracker.update(invalid_status)


@pytest.mark.parametrize("invalid_reason", [None, 1, False])
def test_reject_invalid_reason_type(invalid_reason):
    """Require human-readable status reasons."""
    tracker = InputStatusTracker()

    with pytest.raises(TypeError, match="reason"):
        tracker.update(InputStatus.INVALID, invalid_reason)


def test_invalid_status_requires_reason():
    """Prevent marking inputs invalid without an explanation."""
    tracker = InputStatusTracker()

    with pytest.raises(ValueError, match="requires a reason"):
        tracker.set_invalid("")


def test_snapshot_is_immutable():
    """Prevent accidental mutation of a readiness snapshot."""
    tracker = InputStatusTracker()

    with pytest.raises(AttributeError):
        tracker.snapshot.status = InputStatus.VALID
