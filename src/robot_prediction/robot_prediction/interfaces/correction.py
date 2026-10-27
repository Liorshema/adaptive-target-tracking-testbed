"""Domain contracts for residual adaptation and trajectory correction fusion."""

from typing import Protocol
from typing import runtime_checkable


@runtime_checkable
class ResidualCorrection(Protocol):
    """Produce a horizon-by-state correction from baseline residual statistics."""

    def compute_correction(self, statistics, horizon): ...


@runtime_checkable
class CorrectionFusion(Protocol):
    """Apply a correction while retaining absolute trajectory sample times."""

    def apply(self, baseline, correction): ...
