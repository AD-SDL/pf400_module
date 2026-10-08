# AGENTS.md/CLAUDE.md

This file provides guidance to AI Agents. Note that CLAUDE.md and AGENTS.md are symlinked together.

## Overview

MADSci node module for the Brooks Automation PreciseFlex 400 (PF400) robot arm. Exposes a FastAPI REST server that translates MADSci actions into telnet commands sent to the physical robot.

## Installation and Running

```bash
# Install
pdm install

# Configure (see Configuration section below), then run
pdm run python -m pf400_rest_node

# Run via Docker
docker compose up
```

Configuration is loaded automatically via MADSci's walk-up settings discovery. Settings come from `.env` or a `node.settings.yaml` beside the compose file.

## Linting

```bash
# Lint
ruff check src/

# Format
ruff format src/

# Fix auto-fixable issues
ruff check --fix src/
```

No pytest test suite. Tests are Jupyter notebooks in `tests/`:

| Notebook | Needs a robot? |
|---|---|
| `test_pf400_interface.ipynb` | yes |
| `test_pf400_node.ipynb` | yes |
| `test_simulation_node.ipynb` | **no**, only a reachable resource manager |

`test_simulation_node.ipynb` starts the node itself and covers all seven actions, the
joint soft limit refusals, and a multi-step sequence. Run it with:

```bash
cd tests
RESOURCE_SERVER_URL=http://localhost:8003 \
  jupyter nbconvert --to notebook --execute --inplace test_simulation_node.ipynb
```

It creates and removes its own test resources. Point it at a sandbox, not a live lab.

## Code Architecture

Two layers, each building on the previous:

### 1. `src/pf400_rest_node.py` — MADSci REST Node
`PF400Node(RestNode)` implements the MADSci node interface:
- `startup_handler`: initializes MADSci resource templates (gripper slot, lid slot, plate lid asset) and instantiates `PF400`
- `state_handler`: polls `movement_state` (0=power off, 1=ready, >1=busy)
- Actions decorated with `@action`: `transfer`, `pick_plate`, `place_plate`, `move_to_location`, `move_neutral`, `remove_lid`, `replace_lid`
- Config via `PF400NodeConfig(RestNodeConfig)` — adds `pf400_ip`, `pf400_port` (10100), `pf400_status_port` (10000)

### 2. `src/pf400_interface/pf400.py` — Robot Driver
`PF400` communicates with the robot via **two telnet connections**:
- Command connection: port 10100 — sends GPL (robot programming language) commands
- Status connection: port 10000 — reads robot state

Key driver concepts:
- Locations are represented as 6-element joint-angle arrays: `[z_mm, shoulder_deg, elbow_deg, wrist_deg, gripper_mm, rail_mm]`
- Motion profiles (from `pf400_constants.py`): profile 1 (slow), profile 2 (fast, 120 speed), profile 3 (straight-line)
- `grip_wide=True` → gripper opens to 130mm / closes to 127mm; `False` → 90mm / 85mm
- Plate rotation: `None` or `"narrow"` → 0° wrist offset; `"wide"` → 90° wrist offset. Any other value, **including the empty string**, raises `ValueError`
- `default_approach_height = 15.0` steps above/below the target position for approach moves

Key methods:
- `transfer(source, target, ...)` — full pick+place with optional approach waypoints and plate rotation
- `pick_plate(source, ...)` / `place_plate(target, ...)` — individual pick/place
- `remove_lid(source, target, lid_height=7.0, ...)` / `replace_lid(...)` — lid handling

### Kinematics run on the robot, not here
There is no local kinematics module. `joint_to_cart` and `cart_to_joint` call the
`JointToCart` and `CartToJoint` custom TCS commands, which use the controller's own
kinematics. Both are queries and move nothing.
- Joint coordinate order: `[z, shoulder, elbow, wrist, gripper, rail]`
- Cartesian coordinate order: `[X, Y, Z, yaw, pitch, roll]`

### Supporting files
- `pf400_constants.py` — `ERROR_CODES`, `MOTION_PROFILES`, `OUTPUT_CODES` dicts
- `pf400_errors.py` — `Pf400ConnectionError`, `Pf400CommandError`, `Pf400ResponseError`
- `src/keyboard_control.py` — interactive keyboard teleoperation utility (arrow keys = XY, W/S = Z, N = neutral)

## Joint soft limits

Read from controller parameters 16077 (max) and 16078 (min). The node checks every
location, approach waypoint, and computed above-position against these before sending
anything. Past them the robot raises a soft envelope error, `ERROR_CODES["-1012"]`,
which arrives only after the command is already sent.

| Joint | Soft min | Soft max | Unit |
|---|---|---|---|
| z | 1.5 | 1161.5 | mm |
| shoulder | -93 | 93 | deg |
| elbow | 12 | 348 | deg |
| wrist | -960 | 960 | deg |
| gripper | 69 | 134 | mm |
| rail | -1000 | 1000 | mm |

Hard stops (16075 and 16076) sit just outside these and are not used for checking.
Override with `joint_soft_limit_min` and `joint_soft_limit_max` if the controller
parameters change.

## Simulation mode

Set `simulation: true` to start the node with no hardware connection. Use it to validate
workflow steps while the real node is busy with an experiment.

`PF400.__init__` performs no I/O. A real node constructs the object, then calls
`connect()` and `initialize_robot()`. A simulation node constructs the same object and
never connects. `send_command` and `send_status_command` return canned replies instead
of reaching the network, so every sequencing and resource-tracking path runs unchanged.
There is no second class and no second set of actions.

**A simulation node still updates resources.** The plate moves in the resource graph
exactly as it would for a real transfer, which is what lets a multi-step workflow be
checked: step 2 sees the state that step 1 produced. So point a simulation node at a
**sandbox resource manager**, never at the one the real lab is using.

Two things a simulation node cannot tell you. Its reported joint position is an echo of
the last commanded move, not a prediction, so never read a position from it and act on
it. And its view of the lab is a copy, so it cannot know the real lab changed underneath
it.

Reachability is not simulated. Every pose is checked against the joint soft limits
before any command is issued, in simulation and on real hardware alike, which is why the
canned replies do not need to be geometrically meaningful.

### Starting a node in simulation

Three equivalent routes, highest priority first:

```bash
# CLI. The flag is typed bool, so it needs a value; bare --simulation fails.
pdm run python -m pf400_rest_node --simulation=true

# Environment variable, note the NODE_ prefix
NODE_SIMULATION=true pdm run python -m pf400_rest_node
```

```yaml
# node.settings.yaml
node_name: pf400_piper_sim
node_url: http://0.0.0.0:2010
simulation: true
```

Or in Docker, which runs on a distinct name and port so it sits alongside the real node:

```bash
docker compose -f compose.simulation.yaml up
```

`pf400_ip` is not required when `simulation` is true.

**Environment variable names are not always what you expect.** The prefix is `NODE_`, so
`simulation` is `NODE_SIMULATION`. But a field whose own name already starts with `node_`
is not doubled: `node_url` is `NODE_URL`, and `NODE_NODE_URL` is silently ignored, which
leaves the node on its default port 2000 where it will collide with whatever is there.
`.env.example` is generated from the config model and is the authoritative list.

Simulation is a startup flag rather than a per-request one on purpose. A flag toggled
per request on a live node races with concurrent real actions. A node that never opened
a connection cannot move anything whatever its state.

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

All settings can also be set via environment variables. **The prefix is `NODE_`**, inherited from `NodeConfig`, so the variable for `pf400_ip` is `NODE_PF400_IP` and for `simulation` it is `NODE_SIMULATION`. An unprefixed `PF400_IP` is ignored.

The node's stable identity (ID) is persisted in the `.madsci/registry.json` file found by the same walk-up discovery. The node registers itself under its `node_name` on first startup and reuses the same ULID on subsequent restarts.

`definitions/pf400.info.yaml` is auto-generated from the node's declared actions and capabilities — do not edit it manually.

| Parameter | Default | Description |
|---|---|---|
| `pf400_ip` | None (required) | Robot IP address |
| `pf400_port` | 10100 | Command telnet port |
| `pf400_status_port` | 10000 | Status telnet port |
| `node_url` | `http://127.0.0.1:2000/` | REST API base URL |
| `status_update_interval` | 2.0s | State polling interval |
| `simulation` | false | Start with no hardware connection; actions validate but do not move |
| `simulation_mirrors_node` | None | Real node whose gripper a simulation node observes |
| `joint_soft_limit_min` | see table above | Per-joint minimum soft stop |
| `joint_soft_limit_max` | see table above | Per-joint maximum soft stop |
