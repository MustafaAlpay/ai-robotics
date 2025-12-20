"""Base classes for simulation bridges."""

from __future__ import annotations

from abc import ABC, abstractmethod


class SimulationBridge(ABC):
    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def step(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        raise NotImplementedError
