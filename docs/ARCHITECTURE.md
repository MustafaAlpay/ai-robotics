# Architecture Overview

AtlasForge separates concerns into adapters, core runtime services, and application logic. The goal is to make switching between simulator and hardware as painless as possible.

## Layers
1. **Adapters**: simulator and hardware interfaces (`ai_robotics_sim`, `ai_robotics_robotics`).
2. **Runtime**: scheduling, safety, logging, and data buses (`ai_robotics_core`).
3. **Applications**: perception, planning, control, and policy execution.

## Data Flow
Sensor streams enter through adapters, are fused into `Observation` and `State`, then routed to planners and controllers. A `SafetyGate` validates each action before actuation and logs every step for dataset creation.

## Extensibility
Add a new robot by implementing adapter interfaces and registering the platform configuration in `config/robots/`.
