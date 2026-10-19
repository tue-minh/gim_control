# Taste

## Workflow
- Maintains task/TODO work as a dedicated markdown file (e.g. `ADMITTANCE_CONTROL_TODO.md`) and expects the agent to read it and complete all listed tasks end-to-end. Confidence: 0.6

## Documentation
- Wants every new node/component accompanied by a dedicated markdown instruction/user-guide file (e.g. a new `ADMITTANCE_TUNER.md` for a new tuner node), following the project's existing doc conventions. Confidence: 0.65

## Project Conventions
- When asking for a new node/package, expects it to mirror the structure and conventions of existing sibling nodes (points the agent at reference folders and expects the same file layout, naming, and style). Confidence: 0.65
- Expects new controller packages to keep the established node trio: a calibration node, a calibration-trajectory node, and a run/execution node. Confidence: 0.65
- Wants calibration logging/export to keep writing into `config/calibration_logs`, and motor parameters to keep being read from the shared `config/motor_params.yaml` rather than being re-declared per node. Confidence: 0.7
