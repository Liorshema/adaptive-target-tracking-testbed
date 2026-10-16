"""Configurable time-periodic acceleration field."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SinusoidalFieldParameters:
    """Independent sinusoidal acceleration components."""

    amplitude: np.ndarray
    frequency: np.ndarray
    phase: np.ndarray

    def __post_init__(self):
        amplitude = np.asarray(self.amplitude, dtype=float)
        frequency = np.asarray(self.frequency, dtype=float)
        phase = np.asarray(self.phase, dtype=float)

        if (amplitude.ndim != 1 or amplitude.size == 0
                or frequency.shape != amplitude.shape
                or phase.shape != amplitude.shape
                or not np.all(np.isfinite(amplitude))
                or not np.all(np.isfinite(frequency))
                or not np.all(np.isfinite(phase))
                or np.any(frequency < 0)):
            raise ValueError("Invalid sinusoidal field parameters")

        object.__setattr__(self, "amplitude", amplitude.copy())
        object.__setattr__(self, "frequency", frequency.copy())
        object.__setattr__(self, "phase", phase.copy())


class SinusoidalAccelerationField:
    """Produce a time-dependent acceleration of configurable dimension."""

    def __init__(self, parameters: SinusoidalFieldParameters):
        self.parameters = parameters

    def acceleration(self, position: np.ndarray, time: float = 0.0) -> np.ndarray:
        position = np.asarray(position, dtype=float)

        if position.shape != self.parameters.amplitude.shape:
            raise ValueError("Position dimension mismatch")
        if not np.all(np.isfinite(position)) or not np.isfinite(time):
            raise ValueError("Invalid position or time")

        return self.parameters.amplitude * np.sin(
            2 * np.pi * self.parameters.frequency * time
            + self.parameters.phase
        )
