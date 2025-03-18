#! /usr/bin/env python3
"""The server for the PF400 robot that takes incoming WEI flow requests from the experiment application"""

import datetime
import traceback
from time import sleep
from typing import List

from madsci.common.types.action_types import ActionFailed, ActionSucceeded
from madsci.common.types.node_types import RestNodeConfig
from madsci.node_module.abstract_node_module import action
from madsci.node_module.rest_node_module import RestNode
from pf400_interface.pf400 import PF400
from pf400_interface.pf400_errors import ConnectionException
from typing_extensions import Annotated


class PF400NodeConfig(RestNodeConfig):
    """Configuration for the pf400 node module."""

    __test__ = False

    ip: str = "146.137.240.35"
    """Required Robot IP"""
    port: int = 10100
    """Required Robot Port"""


class PF400Node(RestNode):
    """A Rest Node object to control PF400 robots"""

    __test__ = False

    pf400_interface = PF400 = None
    config_model = PF400NodeConfig

    def startup_handler(self) -> None:
        """Called to (re)initialize the node. Should be used to open connections to devices or initialize any other resources."""
        try:
            self.logger.log("Node initializing...")
            self.pf400_interface = PF400(
                host=self.config_model.ip, port=self.config_model.port
            )
            self.pf400_interface.initialize_robot()
        except Exception as err:
            self.logger.log_error(f"Error starting the PF400 Node: {err}")
            self.startup_has_run = False
        else:
            self.startup_has_run = True
            self.logger.log("Test node initialized!")

    def shutdown_handler(self) -> None:
        """Called to shutdown the node. Should be used to close connections to devices or release any other resources."""
        try:
            self.logger.log("Shutting down")
            self.pf400_interface.disconnect()
            self.shutdown_has_run = True
            del self.pf400_interface
            self.pf400_interface = None
            self.logger.log("Shutdown complete.")
        except Exception as err:
            self.logger.log_error(f"Error shutting down the PF400 Node: {err}")

    def state_handler(self) -> None:
        """Periodically called to update the current state of the node."""
        if self.pf400_interface is not None:
            # Getting robot state
            robot_state = self.pf400_interface.movement_state
            if robot_state == 0:
                self.node_state = {
                    "pf400_status_code": "POWER OFF",
                }
                self.logger.log_error("PF400 POWER OFF")
            elif robot_state == 1:
                self.node_state = {
                    "pf400_status_code": "READY",
                }
            elif robot_state > 1:
                self.node_state = {
                    "pf400_status_code": "BUSY",
                }
            else:
                self.node_state = {
                    "pf400_status_code": self.pf400_interface.robot_state,
                }

    @action
    def test_action(self, test_param: int) -> bool:
        """A test action."""
        result = self.test_interface.run_command(
            f"Test action with param {test_param}."
        )
        if result:
            return ActionSucceeded()
        return ActionFailed(
            errors=f"`run_command` returned '{result}'. Expected 'True'."
        )

    @action(
        name="transfer", description="Transfer a plate from one location to another"
    )
    def transfer(
        self,
        source: Annotated[List[float], "Location to pick a plate from"],
        target: Annotated[List[float], "Location to place a plate to"],
        source_plate_rotation: Annotated[
            str, "Orientation of the plate at the source, wide or narrow"
        ],
        target_plate_rotation: Annotated[
            str, "Final orientation of the plate at the target, wide or narrow"
        ],
    ):
        """A doc string, but not the actual description of the action."""

        self.pf400_interface.transfer(
            source_loc=source,
            target_loc=target,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )
        result = self.test_interface.run_command(
            f"Test action with param {test_param}.", fail=True
        )
        if result:
            return ActionSucceeded()
        return ActionFailed(
            errors=f"`run_command` returned '{result}'. Expected 'True'."
        )

    def pause(self) -> None:
        """Pause the node."""
        self.logger.log("Pausing node...")
        self.node_status.paused = True
        self.logger.log("Node paused.")
        return True

    def resume(self) -> None:
        """Resume the node."""
        self.logger.log("Resuming node...")
        self.node_status.paused = False
        self.logger.log("Node resumed.")
        return True

    def shutdown(self) -> None:
        """Shutdown the node."""
        self.shutdown_handler()
        return True

    def reset(self) -> None:
        """Reset the node."""
        self.logger.log("Resetting node...")
        result = super().reset()
        self.logger.log("Node reset.")
        return result

    def safety_stop(self) -> None:
        """Stop the node."""
        self.logger.log("Stopping node...")
        self.node_status.stopped = True
        self.logger.log("Node stopped.")
        return True

    def cancel(self) -> None:
        """Cancel the node."""
        self.logger.log("Canceling node...")
        self.node_status.cancelled = True
        self.logger.log("Node cancelled.")
        return True


# ------------------------------------------------------------------------------#
rest_module = RESTModule(
    name="pf400_node",
    version="0.0.1",
    description="A node to control the pf400 plate moving robot",
    model="pf400",
)
rest_module.arg_parser.add_argument(
    "--pf400_ip", type=str, help="pf400 ip value", default="146.137.240.35"
)
rest_module.arg_parser.add_argument(
    "--pf400_port", type=int, help="pf400 port value", default=10100
)


@rest_module.startup()
def pf400_startup(state: State):
    """Example startup handler."""
    try:
        state.pf400 = PF400(state.pf400_ip, state.pf400_port)
        state.pf400.initialize_robot()
        state.status[ModuleStatus.READY] = True
        state.status[ModuleStatus.INIT] = False
        state.action_start = None
    except Exception:
        state.status = [ModuleStatus.ERROR] = True
        state.status[ModuleStatus.INIT] = False
        traceback.print_exc()

    else:
        print("PF400 online")

    # Do any cleanup here


def check_state(state: State):
    """Updates the PF400 state

    Parameters:
    -----------
        None
    Returns
    -------
        None
    """
    # TODO: Simplify this function

    try_connect = False
    err = None

    try:
        movement_state = state.pf400.movement_state

    except UnboundLocalError as local_var_err:
        traceback.print_exc()
        err = local_var_err

    except AttributeError as attribute_err:
        traceback.print_exc()
        err = attribute_err
        try_connect = True

    except Exception as general_err:
        traceback.print_exc()
        err = general_err

    finally:
        if try_connect:
            state.status[ModuleStatus.READY] = False
            state.status[ModuleStatus.ERROR] = True
            try:
                state.pf400 = PF400(state.pf400_ip, state.pf400_port)
                state.pf400.initialize_robot()
                state.status[ModuleStatus.READY] = True
                state.status[ModuleStatus.ERROR] = False

            except ConnectionException as error_msg:
                state.status[ModuleStatus.READY] = False
                state.status[ModuleStatus.ERROR] = True
                print(error_msg)

            except Exception as err:
                state.status[ModuleStatus.READY] = False
                state.status[ModuleStatus.ERROR] = True
                print(err)
            else:
                print("PF400 online")

    if err:
        state.status[ModuleStatus.READY] = False
        state.status[ModuleStatus.ERROR] = True
        return

    # Check if robot wasn't attached to the software after recovering from Power Off state
    if state.pf400.attach_state == "-1":
        state.status[ModuleStatus.READY] = False
        state.status[ModuleStatus.ERROR] = True
        state.pf400.force_initialize_robot()

    # Publishing robot warning messages if the job wasn't completed successfully
    if (
        state.pf400.robot_warning.upper() != "CLEAR"
        and len(state.pf400.robot_warning) > 0
    ):
        state.status[ModuleStatus.READY] = False
        state.status[ModuleStatus.ERROR] = True
        state.pf400.robot_warning = "CLEAR"

    # Checking real robot state parameters and publishing the current state
    if movement_state == 0:
        state.status[ModuleStatus.READY] = False
        state.status[ModuleStatus.ERROR] = True
        state.pf400.force_initialize_robot()

    elif state.pf400.robot_state == "ERROR" or state.status == ModuleStatus.ERROR:
        state.status[ModuleStatus.READY] = False
        state.status[ModuleStatus.ERROR] = True
    elif (
        movement_state >= 1 and state.status == ModuleStatus.BUSY
    ) or movement_state >= 2:
        state.status[ModuleStatus.READY] = False
        state.status[ModuleStatus.BUSY] = True


@rest_module.state_handler()
def state(state: State):
    """Returns the current state of the Pf400 module"""

    if not (state.status[ModuleStatus.BUSY]) or (
        state.action_start
        and (datetime.datetime.now() - state.action_start > datetime.timedelta(0, 2))
    ):
        check_state(state)
    return JSONResponse(content={"status": state.status, "error": state.error})


@rest_module.action(
    name="transfer", description="Transfer a plate from one location to another"
)
def transfer(
    state: State,
    action: ActionRequest,
    source: Annotated[List[float], "Location to pick a plate from"],
    target: Annotated[List[float], "Location to place a plate to"],
    source_plate_rotation: Annotated[
        str, "Orientation of the plate at the source, wide or narrow"
    ],
    target_plate_rotation: Annotated[
        str, "Final orientation of the plate at the target, wide or narrow"
    ],
) -> StepResponse:
    """Transfer a plate from one location to another"""
    sleep(0.3)
    err = None
    if len(source) != 6:
        err = True
        msg = "Position 1 should be six joint angles length. Canceling the job!"
    elif len(target) != 6:
        err = True
        msg = "Position 2 should be six joint angles length. Canceling the job!"
    if err:
        return StepResponse.step_failed(error=msg)
    sleep(0.3)
    state.action_start = datetime.datetime.now()
    state.pf400.transfer(source, target, source_plate_rotation, target_plate_rotation)
    state.action_start = None
    return StepResponse.step_succeeded()


@rest_module.action()
def pick_plate(
    state: State,
    source: Annotated[List[float], "Locationto pick a plate from"],
    source_plate_rotation: Annotated[
        str, "Orientation of the plate at the source, wide or narrow"
    ],
) -> StepResponse:
    """Picks a plate from a location"""
    sleep(0.3)
    err = None
    if len(source) != 6:
        err = True
        msg = "Source should be six joint angles length. Canceling the job!"
    if err:
        return StepResponse.step_failed(error=msg)
    sleep(0.3)
    state.action_start = datetime.datetime.now()
    state.pf400.robot_warning = "CLEAR"

    if source_plate_rotation.lower() == "wide":
        plate_source_rotation = 90

    elif source_plate_rotation.lower() == "narrow" or source_plate_rotation == "":
        plate_source_rotation = 0
    source = state.pf400.check_incorrect_plate_orientation(
        source, plate_source_rotation
    )
    state.pf400.force_initialize_robot()
    state.pf400.pick_plate(source)
    state.action_start = None
    if state.pf400.plate_state == -1:
        state.pf400.robot_warning = "MISSING PLATE"
        state.pf400.move_all_joints_neutral()
        sleep(5)
        return StepResponse.step_failed(error="No plate detected after pick.")
    return StepResponse.step_succeeded()


@rest_module.action()
def place_plate(
    state: State,
    target: Annotated[List[float], "Location to place the plate"],
    target_plate_rotation: Annotated[
        str, "Orientation of the plate at the target, wide or narrow"
    ],
) -> StepResponse:
    """Picks a plate from a location"""
    sleep(0.3)
    err = None
    if len(target) != 6:
        err = True
        msg = "Target should be six joint angles length. Canceling the job!"
    if err:
        return StepResponse.step_failed(error=msg)
    sleep(0.3)
    state.action_start = datetime.datetime.now()
    if target_plate_rotation.lower() == "wide":
        plate_target_rotation = 90

    elif target_plate_rotation.lower() == "narrow" or target_plate_rotation == "":
        plate_target_rotation = 0
    target = state.pf400.check_incorrect_plate_orientation(
        target, plate_target_rotation
    )
    state.pf400.force_initialize_robot()
    state.pf400.place_plate(target)
    state.action_start = None
    return StepResponse.step_succeeded()


@rest_module.action(name="remove_lid", description="Remove a lid from a plate")
def remove_lid(
    state: State,
    action: ActionRequest,
    target: Annotated[List[float], "Location to remove a plate lid from"],
    target_plate_rotation: Annotated[
        str, " Orientation of the plate at the target, wide or narrow"
    ],
    lid_height: Annotated[float, "height of the lid, in steps"] = 7.0,
) -> StepResponse:
    """Remove a lid from a plate"""
    sleep(0.3)
    state.action_start = datetime.datetime.now()
    state.pf400.remove_lid(target, lid_height, target_plate_rotation)
    state.action_start = None
    return StepResponse.step_succeeded()


@rest_module.action(name="replace_lid", description="Replace a lid on a plate")
def replace_lid(
    state: State,
    action: ActionRequest,
    target: Annotated[List[float], "Location to place a plate to"],
    target_plate_rotation: Annotated[
        str, "Orientation of the plate at the target, wide or narrow"
    ],
    lid_height: Annotated[float, "height of the lid, in steps"] = 7.0,
) -> StepResponse:
    """Replace a lid on a plate"""
    sleep(0.3)
    state.action_start = datetime.datetime.now()
    state.pf400.replace_lid(target, lid_height, target_plate_rotation)
    state.action_start = None
    return StepResponse.step_succeeded()


rest_module.start()
