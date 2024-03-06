# pf400_module

Driver for communicating with the PF400.

## Installation

```
git clone https://github.com/AD-SDL/pf400_module.git
cd pf400_module
python -m venv .venv && source .venv/bin/activate
pip install -e .
```

## Running the PF400 Module

```
python3 -m pf400_rest_node --host=<HOSTNAME> --port <PORT> --alias <ALIAS> --pf400_ip <IP> --pf400_port <PORT>
```

You can use `0.0.0.0` as the hostname to connect from any device on the local network.
