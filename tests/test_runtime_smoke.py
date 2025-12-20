import json

from ai_robotics_core import (
    Action,
    Observation,
    PolicyAdapter,
    RobotAdapter,
    RunLogger,
    RuntimeLoop,
    SafetyGate,
    SensorAdapter,
    State,
)


class DummySensors(SensorAdapter):
    def __init__(self) -> None:
        self._t = 0.0

    def start(self) -> None:
        return None

    def read(self) -> Observation:
        self._t += 0.1
        return Observation(
            timestamp_s=self._t,
            sensors={"camera_front": "frame_0"},
            frame_id="base_link",
        )

    def stop(self) -> None:
        return None


class DummyRobot(RobotAdapter):
    def __init__(self) -> None:
        self.last_action: Action | None = None

    def start(self) -> None:
        return None

    def read_state(self) -> State:
        return State(
            timestamp_s=0.0,
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            velocity={"linear": 0.0, "angular": 0.0},
        )

    def apply(self, action: Action) -> None:
        self.last_action = action

    def stop(self) -> None:
        return None


class DummyPolicy(PolicyAdapter):
    def start(self) -> None:
        return None

    def act(self, observation: Observation, state: State) -> Action:
        _ = state
        return Action(
            timestamp_s=observation.timestamp_s,
            command={"linear": 0.1, "angular": 0.0},
            frame_id=observation.frame_id,
        )

    def stop(self) -> None:
        return None


class DummySafety(SafetyGate):
    def filter(self, action: Action, state: State) -> Action:
        _ = state
        return action

    def on_timing_violation(self, elapsed_s: float) -> None:
        _ = elapsed_s


def test_runtime_smoke(tmp_path) -> None:
    logger = RunLogger(tmp_path, "smoke")
    loop = RuntimeLoop(
        sensors=DummySensors(),
        robot=DummyRobot(),
        policy=DummyPolicy(),
        safety=DummySafety(),
        logger=logger,
    )

    loop.start()
    loop.run_once()
    loop.stop()

    log_file = tmp_path / "smoke.jsonl"
    assert log_file.exists()
    lines = log_file.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 1
    payload = json.loads(lines[0])
    assert "observation" in payload
    assert "state" in payload
    assert "action" in payload
