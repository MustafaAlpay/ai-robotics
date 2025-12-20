"""Runtime loop orchestration for AtlasForge Robotics."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from ai_robotics_core.adapters import PolicyAdapter, RobotAdapter, SensorAdapter
from ai_robotics_core.logging import RunLogger
from ai_robotics_core.safety import SafetyGate


@dataclass
class RuntimeConfig:
    loop_hz: float = 20.0
    max_cycle_time_s: float = 0.1


class RuntimeLoop:
    def __init__(
        self,
        sensors: SensorAdapter,
        robot: RobotAdapter,
        policy: PolicyAdapter,
        safety: SafetyGate,
        logger: Optional[RunLogger] = None,
        config: Optional[RuntimeConfig] = None,
    ) -> None:
        self._sensors = sensors
        self._robot = robot
        self._policy = policy
        self._safety = safety
        self._logger = logger
        self._config = config or RuntimeConfig()
        self._running = False

    def start(self) -> None:
        self._sensors.start()
        self._robot.start()
        self._policy.start()
        if self._logger:
            self._logger.start()
        self._running = True

    def stop(self) -> None:
        self._running = False
        if self._logger:
            self._logger.stop()
        self._policy.stop()
        self._robot.stop()
        self._sensors.stop()

    def run_once(self) -> None:
        start = time.perf_counter()
        observation = self._sensors.read()
        state = self._robot.read_state()
        action = self._policy.act(observation, state)
        safe_action = self._safety.filter(action, state)
        self._robot.apply(safe_action)
        if self._logger:
            self._logger.log_step(observation, state, safe_action)
        elapsed = time.perf_counter() - start
        if elapsed > self._config.max_cycle_time_s:
            self._safety.on_timing_violation(elapsed)

    def run_forever(self) -> None:
        self.start()
        try:
            period = 1.0 / self._config.loop_hz
            while self._running:
                loop_start = time.perf_counter()
                self.run_once()
                sleep_for = period - (time.perf_counter() - loop_start)
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            self.stop()
