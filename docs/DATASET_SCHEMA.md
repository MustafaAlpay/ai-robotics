# Dataset and Logging Schema

AtlasForge logs each runtime step as a JSON Lines record to enable streaming datasets. Every line represents a single control cycle.

To generate a sample log that matches this schema, run `./scripts/generate_sample_log.py`.

## Record Structure
```json
{
  "observation": {
    "timestamp_s": 123.45,
    "sensors": {
      "camera_front": "path/or/inline",
      "lidar": "path/or/inline"
    },
    "frame_id": "base_link"
  },
  "state": {
    "timestamp_s": 123.45,
    "pose": {"x": 1.2, "y": 0.4, "yaw": 0.1},
    "velocity": {"linear": 0.2, "angular": 0.1},
    "covariance": [0.0, 0.0, 0.0]
  },
  "action": {
    "timestamp_s": 123.45,
    "command": {"linear": 0.2, "angular": 0.1},
    "frame_id": "base_link"
  }
}
```

## Episode Manifest (Proposed)
Store a `manifest.json` alongside logs to capture metadata:
```json
{
  "episode_id": "2025-01-01T12-00-00Z",
  "robot_id": "turtlebot3",
  "sim_backend": "gazebo",
  "config_paths": ["config/runtime/default.yaml", "config/robots/turtlebot3.yaml"],
  "tags": ["mapping", "baseline"],
  "git_commit": "<sha>"
}
```

## Storage Conventions
- Use `data/runs/<episode_id>.jsonl` for log records.
- Store large sensor blobs separately and reference paths in `sensors`.
