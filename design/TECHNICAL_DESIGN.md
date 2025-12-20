# Technical Design

## Architecture Overview
AtlasForge is organized as a layered stack with clear interfaces between runtime, perception, planning, control, and learning. Every subsystem exposes adapters so it can be swapped without changing upstream logic.

```
Runtime Loop -> Sensor IO -> Perception -> Mapping -> Planning -> Control -> Actuation
                        \-> Logging -> Dataset Builder -> Policy Training
```

## Core Packages
- `ai_robotics_core`: runtime scheduler, data buses, safety gates, logging.
- `ai_robotics_sim`: simulator bridges and virtual sensor streams.
- `ai_robotics_robotics`: hardware adapters, drivers, and calibration hooks.

## Key Interfaces
- `SensorAdapter`: stream images, point clouds, IMU, GPS, encoders.
- `StateEstimator`: fuse sensor data into a consistent state.
- `Planner`: produce trajectories and high-level actions.
- `Controller`: convert trajectories to actuator commands.
- `SafetyGate`: enforce limits and stop conditions.
- `Logger`: write structured logs and dataset metadata.

## Data Model (Proposed)
- `Observation`: timestamped sensor bundle + transforms.
- `State`: robot pose, velocity, and covariance.
- `Action`: command primitives (velocity, joint targets, gripper).
- `Episode`: ordered sequence of `Observation`, `State`, `Action`.

## Example Adapter Contract (Python)
```python
class SensorAdapter:
    def start(self) -> None: ...
    def read(self) -> "Observation": ...
    def stop(self) -> None: ...
```

## Simulation Backends
Adapters normalize sim APIs into a shared sensor/action contract:
- ROS 2 + Gazebo: uses ROS topics and `tf` for transforms.
- Isaac Sim: uses OmniGraph sensors and synthetic data.
- AirSim: uses RPC sensors and vehicle control APIs.

## Training Pipeline
- Dataset builder reads logs and emits episode shards.
- Policy trainer consumes shards and exports a policy package.
- Runtime loads policy packages via a `PolicyAdapter`.

## Reliability & Safety
- Runtime loop includes watchdog timers and fault states.
- SafetyGate enforces actuator bounds and overrides on anomalies.
- All runs emit a manifest with configs and git metadata.
