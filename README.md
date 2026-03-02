# pf400_module

Implementation of a MADSci Node Module for integrating a Brooks Automation PreciseFlex 400 (PF400).

## Installation and Usage

### Python

```bash
# Create a virtual environment named .venv
python -m venv .venv
# Activate the virtual environment on Linux or macOS
source .venv/bin/activate
# Alternatively, activate the virtual environment on Windows
# .venv\Scripts\activate
# Install the module and dependencies in the venv
pip install .
# Create a settings file (see Configuration below), then start the node
python -m pf400_rest_node
```

### Configuration

Settings are loaded automatically via MADSci's walk-up file discovery. Create a `node.settings.yaml` in your working directory (or any parent up to the `.madsci/` sentinel):

```yaml
node_name: pf400
node_url: http://0.0.0.0:2000
pf400_ip: 192.168.1.100
pf400_port: 10100
pf400_status_port: 10000
```

All settings can also be provided as environment variables (e.g. `PF400_IP`, `NODE_URL`). The node's stable ID is stored in `.madsci/registry.json` and reused across restarts.

### Docker

- We provide a `Dockerfile` and example docker compose file (`compose.yaml`) to run this node dockerized.
- There is also a pre-built image available as `ghcr.io/ad-sdl/pf400_module`.
- You can control the container user's id and group id by setting the `USER_ID` and `GROUP_ID`
