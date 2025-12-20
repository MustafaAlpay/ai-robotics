# Security

Robotics systems interact with physical environments. Treat every change as safety-critical.

## Responsible Use
- Do not deploy untested autonomy stacks on real robots.
- Validate on simulation and controlled test benches first.

## Secrets
- Never commit credentials, keys, or robot access tokens.
- Use `.env` files for local secrets and document variables in `.env.example`.

## Vulnerability Reporting
- If you find a security issue, open a private report via the repository owner.
