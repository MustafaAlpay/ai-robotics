"""Structured logging for runtime episodes."""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Optional

from ai_robotics_core.data_model import Action, Observation, State


class RunLogger:
    def __init__(self, output_dir: Path, run_id: str) -> None:
        self._output_dir = output_dir
        self._run_id = run_id
        self._file: Optional[Path] = None
        self._handle = None

    def start(self) -> None:
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._file = self._output_dir / f"{self._run_id}.jsonl"
        self._handle = self._file.open("w", encoding="utf-8")

    def log_step(self, observation: Observation, state: State, action: Action) -> None:
        if not self._handle:
            return
        record = {
            "observation": asdict(observation),
            "state": asdict(state),
            "action": asdict(action),
        }
        self._handle.write(json.dumps(record) + "\n")

    def stop(self) -> None:
        if self._handle:
            self._handle.close()
            self._handle = None
