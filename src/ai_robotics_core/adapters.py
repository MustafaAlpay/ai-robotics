"""Adapter interfaces for sensors, robots, simulators, and policies."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from ai_robotics_core.data_model import Action, Observation, State


class SensorAdapter(ABC):
    @abstractmethod
    def start(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def read(self) -> Observation:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError


class RobotAdapter(ABC):
    @abstractmethod
    def start(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def read_state(self) -> State:
        raise NotImplementedError

    @abstractmethod
    def apply(self, action: Action) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError


class SimAdapter(ABC):
    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def step(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        raise NotImplementedError


class PolicyAdapter(ABC):
    @abstractmethod
    def start(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def act(self, observation: Observation, state: State) -> Action:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    def load_policy(self, policy_path: Optional[str] = None) -> None:
        _ = policy_path
