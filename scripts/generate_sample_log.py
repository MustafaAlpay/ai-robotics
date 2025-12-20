#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC_PATH = ROOT / "src"
if SRC_PATH.exists():
    sys.path.insert(0, str(SRC_PATH))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a sample AtlasForge JSONL log.")
    parser.add_argument("--output-dir", default="data/runs", help="Directory for JSONL logs.")
    parser.add_argument("--run-id", default="sample-run", help="Run identifier for the log file.")
    parser.add_argument("--steps", type=int, default=5, help="Number of log records to write.")
    return parser.parse_args()


def main() -> None:
    from ai_robotics_core import Action, Observation, RunLogger, State

    args = parse_args()
    logger = RunLogger(Path(args.output_dir), args.run_id)
    logger.start()
    for step in range(args.steps):
        ts = float(step) * 0.1
        observation = Observation(
            timestamp_s=ts,
            sensors={
                "camera_front": f"frames/front_{step:04d}.png",
                "lidar": f"pointclouds/lidar_{step:04d}.pcd",
            },
            frame_id="base_link",
        )
        state = State(
            timestamp_s=ts,
            pose={"x": ts, "y": 0.0, "yaw": 0.0},
            velocity={"linear": 0.2, "angular": 0.0},
        )
        action = Action(
            timestamp_s=ts,
            command={"linear": 0.2, "angular": 0.0},
            frame_id="base_link",
        )
        logger.log_step(observation, state, action)
    logger.stop()


if __name__ == "__main__":
    main()
