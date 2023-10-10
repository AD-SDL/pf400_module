# pf400_module

Driver for communicating with the PF400.

## Installation

```
git clone https://github.com/AD-SDL/pf400_module.git
cd pf400_module
pip install -r requirements.txt
pip install -e .
```

## Running the PF400 Module

In `/scripts`, run the following

```
python3 pf400_rest_client.py --host=<hostname> --port 3000 --alias pf400
```

You can use `0.0.0.0` as the hostname to connect from any device on the local network.
