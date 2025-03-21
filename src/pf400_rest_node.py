#! /usr/bin/env python3
"""The server for the PF400 robot that takes incoming WEI flow requests from the experiment application"""

import datetime
import traceback
from time import sleep
from typing import List, Optional, Union

from fastapi.datastructures import State
from fastapi.responses import JSONResponse
from pf400_driver.pf400_driver import PF400
from pf400_driver.pf400_errors import ConnectionException
from typing_extensions import Annotated
from wei.modules.rest_module import RESTModule
from wei.types.module_types import ModuleStatus
from wei.types.step_types import ActionRequest, StepResponse

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
    state.pf400 = None
    state.action_start = None
    try:
        state.pf400 = PF400(state.pf400_ip, state.pf400_port)
        state.pf400.initialize_robot()
    except Exception:
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
    err = False

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

    if (
        not (state.status[ModuleStatus.BUSY])
        and not (state.status[ModuleStatus.INIT])
        or (
            state.action_start
            and (
                datetime.datetime.now() - state.action_start > datetime.timedelta(0, 2)
            )
        )
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
    source_approach: Annotated[List[float], "Location to approach from"] = None,
    target_approach: Annotated[List[float], "Location to approach from"] = None,
    # source_approach=None,
    # target_approach=None,
    source_plate_rotation: Annotated[
        str, "Orientation of the plate at the source, wide or narrow"
    ] = "",
    target_plate_rotation: Annotated[
        str, "Final orientation of the plate at the target, wide or narrow"
    ] = "",
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
    state.pf400.transfer(
        source=source,
        target=target,
        source_approach=source_approach,
        target_approach=target_approach,
        source_plate_rotation=source_plate_rotation,
        target_plate_rotation=target_plate_rotation,
    )
    state.action_start = None
    return StepResponse.step_succeeded()


@rest_module.action(
    name="pick_plate", description="Pick a plate from a source location"
)
def pick_plate(
    state: State,
    source: Annotated[List[float], "Location to pick a plate from"],
    source_approach: Optional[
        Annotated[
            Union[List[float], List[List[float]]], "Approach location(s) for source"
        ]
    ] = None,
    source_plate_rotation: Annotated[
        str, "Orientation of the plate at the source, wide or narrow"
    ] = "",
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
    state.pf400.pick_plate(source=source, source_approach=source_approach)
    state.action_start = None
    if state.pf400.plate_state == -1:
        state.pf400.robot_warning = "MISSING PLATE"
        state.pf400.move_all_joints_neutral()
        sleep(5)
        return StepResponse.step_failed(error="No plate detected after pick.")
    return StepResponse.step_succeeded()


@rest_module.action(
    name="place_plate", description="Place a plate to a target location"
)
def place_plate(
    state: State,
    target: Annotated[List[float], "Location to place the plate"],
    target_approach: Annotated[
        Optional[Union[List[float], List[List[float]]]],
        "Approach location(s) for target",
    ] = None,
    target_plate_rotation: Annotated[
        str, "Orientation of the plate at the target, wide or narrow"
    ] = "",
) -> StepResponse:
    """Places a plate at a location"""
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
    state.pf400.place_plate(target=target, target_approach=target_approach)
    state.action_start = None
    return StepResponse.step_succeeded()


@rest_module.action(name="remove_lid", description="Remove a lid from a plate")
def remove_lid(
    state: State,
    action: ActionRequest,
    source: Annotated[List[float], "Location to pick a plate from"],
    target: Annotated[List[float], "Location to place a plate to"],
    source_approach: Optional[
        Annotated[
            Union[List[float], List[List[float]]], "Approach location(s) for source"
        ]
    ] = None,
    target_approach: Optional[
        Annotated[
            Union[List[float], List[List[float]]], "Approach location(s) for target"
        ]
    ] = None,
    source_plate_rotation: Annotated[
        str, "Orientation of the plate at the source, wide or narrow"
    ] = "",
    target_plate_rotation: Annotated[
        str, "Final orientation of the plate at the target, wide or narrow"
    ] = "",
    lid_height: Annotated[float, "height of the lid, in steps"] = 7.0,
) -> StepResponse:
    """Remove a lid from a plate"""
    sleep(0.3)
    state.action_start = datetime.datetime.now()
    state.pf400.remove_lid(
        source=source,
        target=target,
        lid_height=lid_height,
        source_approach=source_approach,
        target_approach=target_approach,
        source_plate_rotation=source_plate_rotation,
        target_plate_rotation=target_plate_rotation,
    )
    state.action_start = None
    return StepResponse.step_succeeded()


@rest_module.action(name="replace_lid", description="Replace a lid on a plate")
def replace_lid(
    state: State,
    action: ActionRequest,
    source: Annotated[List[float], "Location to pick a plate from"],
    target: Annotated[List[float], "Location to place a plate to"],
    source_approach: Optional[
        Annotated[
            Union[List[float], List[List[float]]], "Approach location(s) for source"
        ]
    ] = None,
    target_approach: Optional[
        Annotated[
            Union[List[float], List[List[float]]], "Approach location(s) for target"
        ]
    ] = None,
    source_plate_rotation: Annotated[
        str, "Orientation of the plate at the source, wide or narrow"
    ] = "",
    target_plate_rotation: Annotated[
        str, "Final orientation of the plate at the target, wide or narrow"
    ] = "",
    lid_height: Annotated[float, "height of the lid, in steps"] = 7.0,
) -> StepResponse:
    """Replace a lid on a plate"""
    sleep(0.3)
    state.action_start = datetime.datetime.now()
    state.pf400.replace_lid(
        source=source,
        target=target,
        lid_height=lid_height,
        source_approach=source_approach,
        target_approach=target_approach,
        source_plate_rotation=source_plate_rotation,
        target_plate_rotation=target_plate_rotation,
    )
    state.action_start = None
    return StepResponse.step_succeeded()


rest_module.start()
