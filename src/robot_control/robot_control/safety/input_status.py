
"""Track control-input readiness independently of ROS 2."""

from dataclasses import dataclass
from enum import Enum


class InputStatus(str, Enum):
    """Readiness of external data required for robot control."""

    WAITING = "waiting"
    VALID = "valid"
    INVALID = "invalid"


@dataclass(frozen=True)
class InputStatusSnapshot:
    """Immutable snapshot of control-input readiness."""

    status: InputStatus
    reason: str = ""

    @property
    def is_ready(self) -> bool:
        """Return whether the inputs are valid for control."""
        return self.status == InputStatus.VALID


class InputStatusTracker:
    """Track readiness and report meaningful state transitions."""

    def __init__(self) -> None:
        """Start in the waiting state."""
        self._snapshot = InputStatusSnapshot(
            status=InputStatus.WAITING,
            reason="Waiting for control inputs",
        )

    @property
    def snapshot(self) -> InputStatusSnapshot:
        """Return the current readiness snapshot."""
        return self._snapshot

    @property
    def is_ready(self) -> bool:
        """Return whether the latest inputs passed validation."""
        return self._snapshot.is_ready

    def update(
        self,
        status: InputStatus,
        reason: str = "",
    ) -> bool:
        """Update readiness and report whether the snapshot changed.

        Args:
            status: New input-readiness status.
            reason: Human-readable explanation of the status.

        Returns:
            True if status or reason changed, otherwise False.
        """
        if not isinstance(status, InputStatus):
            raise TypeError("status must be an InputStatus")

        if not isinstance(reason, str):
            raise TypeError("reason must be a string")

        next_snapshot = InputStatusSnapshot(
            status=status,
            reason=reason,
        )

        if next_snapshot == self._snapshot:
            return False

        self._snapshot = next_snapshot
        return True

    def set_waiting(self, reason: str = "Waiting for control inputs") -> bool:
        """Mark inputs as incomplete."""
        return self.update(InputStatus.WAITING, reason)

    def set_valid(self) -> bool:
        """Mark inputs as ready for control."""
        return self.update(InputStatus.VALID)

    def set_invalid(self, reason: str) -> bool:
        """Mark inputs as unusable for control."""
        if not reason:
            raise ValueError("Invalid status requires a reason")

        return self.update(InputStatus.INVALID, reason)
