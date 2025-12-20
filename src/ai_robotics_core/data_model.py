"""Shared data structures for AtlasForge Robotics."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional


@dataclass(frozen=True)
class Observation:
    timestamp_s: float
    sensors: Mapping[str, Any]
    frame_id: str


@dataclass(frozen=True)
class State:
    timestamp_s: float
    pose: Mapping[str, float]
    velocity: Mapping[str, float]
    covariance: Optional[List[float]] = None


@dataclass(frozen=True)
class Action:
    timestamp_s: float
    command: Mapping[str, float]
    frame_id: str


@dataclass
class EpisodeMeta:
    episode_id: str
    robot_id: str
    sim_backend: Optional[str]
    tags: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
