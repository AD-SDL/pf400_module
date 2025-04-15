# pf400_module

Implementation of a MADSci Node Module for integrating a Brooks Automation PreciseFlex 400 (PF400).

See `definitions/pf400.node.yaml` for an example node definition file, and `definitions/pf400.node.info.yaml` for a description of the capabilities of the node.

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
# Start the node
python -m pf400_rest_node --host=<HOSTNAME> --port <PORT> --pf400_ip <IP> --pf400_port <PORT>
```

You can use `0.0.0.0` as the hostname to connect from any device on the local network, or `127.0.0.1` to limit it only to local connections.

### Docker

- We provide a `Dockerfile` and example docker compose file (`compose.yaml`) to run this node dockerized.
- There is also a pre-built image available as `ghcr.io/ad-sdl/pf400_module`.
- You can control the container user's id and group id by setting the `USER_ID` and `GROUP_ID`
