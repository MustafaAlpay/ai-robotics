"""Safety gates and runtime protections."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from ai_robotics_core.data_model import Action, State


class SafetyGate(ABC):
    @abstractmethod
    def filter(self, action: Action, state: State) -> Action:
        raise NotImplementedError

    @abstractmethod
    def on_timing_violation(self, elapsed_s: float) -> None:
        raise NotImplementedError


@dataclass
class SpeedLimits:
    linear_mps: float = 1.0
    angular_rps: float = 1.0


class BasicSafetyGate(SafetyGate):
    def __init__(self, limits: SpeedLimits) -> None:
        self._limits = limits

    def filter(self, action: Action, state: State) -> Action:
        _ = state
        limited = dict(action.command)
        if "linear" in limited:
            limited["linear"] = max(
                -self._limits.linear_mps,
                min(self._limits.linear_mps, limited["linear"]),
            )
        if "angular" in limited:
            limited["angular"] = max(
                -self._limits.angular_rps,
                min(self._limits.angular_rps, limited["angular"]),
            )
        return Action(timestamp_s=action.timestamp_s, command=limited, frame_id=action.frame_id)

    def on_timing_violation(self, elapsed_s: float) -> None:
        _ = elapsed_s
