# AGENTS.md/CLAUDE.md

This file provides guidance to AI Agents. Note that CLAUDE.md and AGENTS.md are symlinked together.

## Overview

MADSci node module for the Brooks Automation PreciseFlex 400 (PF400) robot arm. Exposes a FastAPI REST server that translates MADSci actions into telnet commands sent to the physical robot.

## Installation and Running

```bash
# Install (use pip, not PDM)
pip install .

# Configure (see Configuration section below), then run
python -m pf400_rest_node

# Run via Docker
docker compose up
```

Configuration is loaded automatically via MADSci's walk-up settings discovery. The Docker `compose.yaml` sets `PF400_IP` via environment variable.

## Linting

```bash
# Lint
ruff check src/

# Format
ruff format src/

# Fix auto-fixable issues
ruff check --fix src/
```

No pytest test suite — tests are Jupyter notebooks in `tests/` (`test_pf400_interface.ipynb`, `test_pf400_node.ipynb`) and require a live robot connection.

## Code Architecture

Three layers, each building on the previous:

### 1. `src/pf400_rest_node.py` — MADSci REST Node
`PF400Node(RestNode)` implements the MADSci node interface:
- `startup_handler`: initializes MADSci resource templates (gripper slot, lid slot, plate lid asset) and instantiates `PF400`
- `state_handler`: polls `movement_state` (0=power off, 1=ready, >1=busy)
- Actions decorated with `@action`: `transfer`, `pick_plate`, `place_plate`, `remove_lid`, `replace_lid`
- Config via `PF400NodeConfig(RestNodeConfig)` — adds `pf400_ip`, `pf400_port` (10100), `pf400_status_port` (10000)

### 2. `src/pf400_interface/pf400.py` — Robot Driver
`PF400(KINEMATICS)` communicates with the robot via **two telnet connections**:
- Command connection: port 10100 — sends GPL (robot programming language) commands
- Status connection: port 10000 — reads robot state

Key driver concepts:
- Locations are represented as 6-element joint-angle arrays: `[z_mm, shoulder_deg, elbow_deg, wrist_deg, gripper_mm, rail_mm]`
- Motion profiles (from `pf400_constants.py`): profile 1 (slow), profile 2 (fast, 120 speed), profile 3 (straight-line)
- `grip_wide=True` → gripper opens to 130mm / closes to 127mm; `False` → 90mm / 85mm
- Plate rotation: `""` or `"narrow"` → 0° wrist offset; `"wide"` → 90° wrist offset
- `default_approach_height = 15.0` steps above/below the target position for approach moves

Key methods:
- `transfer(source, target, ...)` — full pick+place with optional approach waypoints and plate rotation
- `pick_plate(source, ...)` / `place_plate(target, ...)` — individual pick/place
- `remove_lid(source, target, lid_height=7.0, ...)` / `replace_lid(...)` — lid handling
- `check_incorrect_plate_orientation(representation, rotation)` — corrects joint angles when orientation mismatches

### 3. `src/pf400_interface/pf400_kinematics.py` — Kinematics
`KINEMATICS` base class with forward/inverse kinematics:
- Arm segments: shoulder=302mm, elbow=289mm, end-effector=162mm
- Joint coordinate order: `[z, shoulder, elbow, wrist, gripper, rail]`
- Cartesian coordinate order: `[X, Y, Z, yaw, pitch=90, roll=180]`
- Rail (joint[5]) is the horizontal position and must be subtracted from X before calling `inverse_kinematics`

### Supporting files
- `pf400_constants.py` — `ERROR_CODES`, `MOTION_PROFILES`, `OUTPUT_CODES` dicts
- `pf400_errors.py` — `Pf400ConnectionError`, `Pf400CommandError`, `Pf400ResponseError`
- `src/keyboard_control.py` — interactive keyboard teleoperation utility (arrow keys = XY, W/S = Z, N = neutral)

## Configuration

Configuration uses MADSci's Pydantic Settings system. Settings are loaded via walk-up file discovery (searching from CWD up to the `.madsci/` sentinel directory), with this priority order: CLI args → environment variables → `node.settings.yaml` → `settings.yaml`.

Example `node.settings.yaml`:

```yaml
node_name: pf400
node_url: http://0.0.0.0:2000
pf400_ip: 192.168.1.100
pf400_port: 10100
pf400_status_port: 10000
```

All settings can also be set via environment variables (e.g. `PF400_IP`, `NODE_URL`).

The node's stable identity (ID) is persisted in the `.madsci/registry.json` file found by the same walk-up discovery. The node registers itself under its `node_name` on first startup and reuses the same ULID on subsequent restarts.

`definitions/pf400.info.yaml` is auto-generated from the node's declared actions and capabilities — do not edit it manually.

| Parameter | Default | Description |
|---|---|---|
| `pf400_ip` | None (required) | Robot IP address |
| `pf400_port` | 10100 | Command telnet port |
| `pf400_status_port` | 10000 | Status telnet port |
| `node_url` | `http://127.0.0.1:2000/` | REST API base URL |
| `status_update_interval` | 2.0s | State polling interval |
