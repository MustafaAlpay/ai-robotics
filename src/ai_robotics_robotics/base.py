"""Base classes for robot hardware adapters."""

from __future__ import annotations

from abc import ABC, abstractmethod


class RobotBridge(ABC):
    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        raise NotImplementedError
