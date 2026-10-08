# Adding simulation mode to a MADSci device module

A node in simulation mode opens no connection to its device. Every action still runs
its argument parsing, its precondition checks, and its resource bookkeeping, then
returns without issuing a command. This lets an agent validate a workflow while the
real node is busy with an experiment.

`pf400_module` is the worked example. This is the procedure for doing the same to
another module.

## The one rule

**A simulation node must still update resources.** It is not read-only. A multi-step
workflow can only be checked if step 2 sees what step 1 did, so plates move in the
resource graph exactly as they would for a real run.

That is why a simulation node must point at a **sandbox resource manager**, never at
the one a live lab is using.

## Does your device fit this pattern?

| Device shape | What to do |
|---|---|
| Arm, or any device whose driver talks over a socket | Follow this procedure |
| Device with a vendor simulator, e.g. the OT-2 | Delegate to it instead. `opentrons_simulate` is a real protocol simulator and beats canned replies |
| Simple state machine, e.g. a sealer, peeler, plate reader | Follow this procedure. It is much shorter, usually a handful of canned replies |

## Four changes

### 1. Add the config flag

```python
class YourNodeConfig(RestNodeConfig):
    simulation: bool = False
    """Run with no hardware connection. Actions validate but do not drive the device."""
```

### 2. Take I/O out of the driver's `__init__`

This is the change that makes everything else possible, and it is usually the only
part that touches existing behaviour. A constructor should not open sockets.

Move the connection, any device configuration, and any startup handshake out of
`__init__` and into `connect()`. In `pf400.py` that meant `connect()`,
`_configure_robot()`, `set_gripper_open()` and `set_gripper_close()` all moving, with
the raw socket setup becoming `_open_connections()`.

Then the node calls it explicitly:

```python
self.device = YourDriver(..., simulation=self.config.simulation)
if self.config.simulation:
    self.logger.log_info("Started in SIMULATION mode.")
    return
self.device.connect()
self.device.initialize()
```

This is an API change for anyone constructing the driver directly. Check the notebooks
in `tests/`.

### 3. Return canned replies below the transport

Find the one or two methods that actually touch the wire. In `pf400.py` they are
`send_command` and `send_status_command`. Intercept at the top, before any
auto-reconnect:

```python
def send_command(self, command: str) -> str:
    if self.simulation:
        return self._simulated_response(command)
    ...
```

**Only cover what callers actually parse.** Do not simulate the device. Read each
caller and work out the minimum reply that keeps it moving. For the PF400 that was
four cases out of dozens of commands:

- a status query that must report "not moving", or the wait loop never exits
- the grip command, whose reply gates the resource pop
- the release command, where a later check compares the gripper position to the
  width that was requested
- the position query, which echoes the last commanded move

Everything else returns the success code.

The replies do **not** need to be physically meaningful, because physical limits are
checked before anything is sent. See change 4.

### 4. Report a distinct state

`state_handler` usually reads a value that only a real command round trip refreshes.
In simulation it will sit at its initial value and report something alarming. The
PF400 reported `POWER OFF` and logged an error twice a second until this was fixed.

```python
def state_handler(self) -> None:
    if self.device.simulation:
        self.node_state = {"status_code": "SIMULATION", "simulation": True}
        return
    ...
```

## Declare the device's limits while you are here

Simulation mode makes a workflow runnable without hardware. It does not make it
*correct*. Correctness comes from checking the device's limits before a command goes
out, and that check runs in simulation and on real hardware alike.

For the PF400 that is the joint soft envelope, read off the controller and declared as
config. For your device it might be a temperature range, a shaker frequency range, or
a set of ordering preconditions such as "the drawer must be open before a plate is
placed in it".

Put the limits in config so they can be corrected without a code change, and make the
refusal message say which limit was exceeded and by how much. An agent can repair from
"gripper is 140 mm, limit is 69 to 134 mm". It cannot repair from "action failed".

## Test it

Copy `tests/test_simulation_node.ipynb`. It starts the node itself, so it needs only a
reachable resource manager and no hardware. Keep its shape:

- one check per action, each stating the outcome it expects
- a refusal test for every limit you declared
- a multi-step sequence, to prove step 2 sees what step 1 did
- teardown that removes the resources it created

Run it **twice in a row**. Running it once tells you much less. The second run is what
caught `remove_lid` leaving a resource behind and then failing on every call after the
first.

## Three mistakes to avoid

**A per-request flag instead of a startup flag.** Tempting, and unsafe. A flag toggled
on a live node races with concurrent real actions, so a validation request can move the
arm during someone's experiment. A node that never opened a connection cannot.

**Environment variable names.** The prefix is `NODE_`, so `simulation` is
`NODE_SIMULATION`. A field whose own name already starts with `node_` is not doubled:
it is `NODE_URL`, never `NODE_NODE_URL`. Getting this wrong fails silently and leaves
the node on its default port, where it collides with whatever is already there.
`.env.example` is generated from the config model and is the authoritative list.

**Creating resources before validating.** If an action creates a resource and then
refuses, it leaks one per refusal. Validate first, or clean up on the refusal path.
