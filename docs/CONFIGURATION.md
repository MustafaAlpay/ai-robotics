# Configuration

AtlasForge uses explicit configuration files for reproducibility. Avoid hidden defaults.

## Directory Layout
- `config/robots/`: hardware-specific settings (URDF, sensors, limits).
- `config/sim/`: simulator bridges and scene settings.
- `config/runtime/`: safety policies, loop rates, and logging settings.

## Environment Variables
- `AF_DATA_ROOT`: dataset and log storage location.
- `AF_SIM_BACKEND`: selected simulator backend (for example: `gazebo`).
- `AF_ROBOT_ID`: hardware profile identifier.

## Configuration Tips
- Keep production and research settings separate.
- Store calibration values in version-controlled files.
- For secrets, use `.env` locally and keep `.env.example` updated.
