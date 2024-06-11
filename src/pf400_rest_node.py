#! /usr/bin/env python3
"""The server for the PF400 robot that takes incoming WEI flow requests from the experiment application"""

import datetime
import json
import traceback
from argparse import ArgumentParser, Namespace
from contextlib import asynccontextmanager
from pathlib import Path
from time import sleep

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from pf400_driver.pf400_driver import PF400
from pf400_driver.pf400_errors import ConnectionException
from wei.types import (
    ModuleAbout,
    ModuleAction,
    ModuleActionArg,
)
from wei.utils import extract_version


def parse_args() -> Namespace:
    """Parses the command line arguments for the PF400 REST node"""
    parser = ArgumentParser()
    parser.add_argument("--alias", type=str, help="Name of the Node", default="pf400")
    parser.add_argument("--host", type=str, help="Host for rest", default="0.0.0.0")
    parser.add_argument("--port", type=int, help="port value")
    parser.add_argument(
        "--pf400_ip", type=str, help="pf400 ip value", default="146.137.240.35"
    )
    parser.add_argument(
        "--pf400_port", type=int, help="pf400 port value", default=10100
    )

    return parser.parse_args()


global pf400_ip, pf400_port, state, action_start


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initial run function for the app, parses the workcell argument
    Parameters
    ----------
    app : FastApi
       The REST API app being initialized

    Returns
    -------
    None"""
    global pf400, state, pf400_ip, pf400_port

    args = parse_args()
    pf400_ip = args.pf400_ip
    pf400_port = args.pf400_port

    try:
        pf400 = PF400(pf400_ip, pf400_port)
        pf400.initialize_robot()
        state = "IDLE"
    except Exception:
        state = "ERROR"
        traceback.print_exc()
    else:
        print("PF400 online")
    yield

    # Do any cleanup here
    pass


app = FastAPI(
    lifespan=lifespan,
)


def check_state():
    """Updates the PF400 state

    Parameters:
    -----------
        None
    Returns
    -------
        None
    """
    # TODO: Simplify this function
    global state, pf400, pf400_ip, pf400_port
    try_connect = False
    err = None

    try:
        movement_state = pf400.movement_state

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
            state = "ERROR"
            try:
                pf400 = PF400(pf400_ip, pf400_port)
                pf400.initialize_robot()
                state = "IDLE"

            except ConnectionException as error_msg:
                state = "ERROR"
                print(error_msg)

            except Exception as err:
                state = "ERROR"
                print(err)
            else:
                print("PF400 online")

    if err:
        state = "ERROR"
        return

    # Check if robot wasn't attached to the software after recovering from Power Off state
    if pf400.attach_state == "-1":
        state = "ERROR"
        pf400.force_initialize_robot()

    # Publishing robot warning messages if the job wasn't completed successfully
    if pf400.robot_warning.upper() != "CLEAR" and len(pf400.robot_warning) > 0:
        state = "ERROR"
        pf400.robot_warning = "CLEAR"

    # Checking real robot state parameters and publishing the current state
    if movement_state == 0:
        state = "ERROR"
        pf400.force_initialize_robot()

    elif pf400.robot_state == "ERROR" or state == "ERROR":
        state = "ERROR"
        state = "UNKNOWN"

    elif (movement_state >= 1 and state == "BUSY") or movement_state >= 2:
        state = "BUSY"


@app.get("/state")
def state():
    """Returns the current state of the Pf400 module"""
    global state, action_start
    if not (state == "BUSY") or (
        action_start
        and (datetime.datetime.now() - action_start > datetime.timedelta(0, 2))
    ):
        check_state()
    return JSONResponse(content={"State": state})


@app.get("/resources")
async def resources():
    """Returns info about the resources the module has access to"""
    global pf400
    return JSONResponse(content={"State": pf400.get_status()})


@app.get("/about")
async def about() -> JSONResponse:
    """Returns a description of the actions and resources the module supports"""
    global state
    about = ModuleAbout(
        name="Pf400 Robotic Arm",
        model="Precise Automation PF400",
        description="pf400 is a robot module that moves plates between two robot locations.",
        interface="wei_rest_node",
        version=extract_version(Path(__file__).parent.parent / "pyproject.toml"),
        actions=[
            ModuleAction(
                name="transfer",
                description="This action transfers a plate from a source robot location to a target robot location.",
                args=[
                    ModuleActionArg(
                        name="source",
                        description="Source location in the workcell for pf400 to grab plate from.",
                        type="str",
                        required=True,
                    ),
                    ModuleActionArg(
                        name="target",
                        description="Transfer location in the workcell for pf400 to transfer plate to.",
                        type="str",
                        required=True,
                    ),
                    ModuleActionArg(
                        name="source_plate_rotation",
                        description="Plate rotation for source location in the workcell.",
                        type="str",
                        required=True,
                    ),
                    ModuleActionArg(
                        name="target_plate_rotation",
                        description="Plate rotation for target location in the workcell.",
                        type="str",
                        required=True,
                    ),
                ],
            ),
            ModuleAction(
                name="remove_lid",
                description="This action removes the lid off of a plate",
                args=[
                    ModuleActionArg(
                        name="target",
                        description="Target location in the workcell that the plate is currently at.",
                        type="str",
                        required=True,
                    ),
                    ModuleActionArg(
                        name="lid_height",
                        description="Lid height of the target plate.",
                        type="str",
                        required=True,
                    ),
                    ModuleActionArg(
                        name="target_plate_rotation",
                        description="Rotation of plate at target location in the workcell.",
                        type="str",
                        required=True,
                    ),
                ],
            ),
            ModuleAction(
                name="replace_lid",
                description="This action places a lid on a plate with no lid.",
                args=[
                    ModuleActionArg(
                        name="target",
                        description="Target location in workcell that plate is currently at.",
                        type="str",
                        required=True,
                    ),
                    ModuleActionArg(
                        name="lid_height",
                        description="Lid height of the target plate.",
                        type="str",
                        required=True,
                    ),
                    ModuleActionArg(
                        name="target_plate_rotation",
                        description="Rotation of plate at target location in the workcell.",
                        type="str",
                        required=True,
                    ),
                ],
            ),
        ],
        resource_pools=[],
    )
    return JSONResponse(content=about.model_dump(mode="json"))


@app.post("/action")
def do_action(action_handle: str, action_vars: str):
    """Executes the action requested by the user"""
    response = {"action_response": "", "action_msg": "", "action_log": ""}
    print(action_vars)
    global pf400, state, action_start
    if state == "BUSY":
        return
    action_start = datetime.datetime.now()
    if state == "PF400 CONNECTION ERROR":
        response["action_response"] = "failed"
        response["action_log"] = "Connection error, cannot accept a job!"
        return response

    sleep(
        0.3
    )  # Before starting the action, wait for stateRefresherCallback function to cycle for at least once to avoid data loss.

    vars = json.loads(action_vars)

    err = False
    state = "BUSY"
    if action_handle == "transfer":
        source_plate_rotation = ""
        target_plate_rotation = ""

        if "source" not in vars.keys():
            err = True
            msg = "Pick up location is not provided. Canceling the job!"
        elif "target" not in vars.keys():
            err = True
            msg = "Drop off up location is not provided. Canceling the job!"
        elif len(vars.get("source")) != 6:
            err = True
            msg = "Position 1 should be six joint angles length. Canceling the job!"
        elif len(vars.get("target")) != 6:
            err = True
            msg = "Position 2 should be six joint angles length. Canceling the job!"

        if err:
            response["action_response"] = "failed"
            response["action_log"] = msg
            state = "ERROR"
            return response

        if "source_plate_rotation" in vars.keys():
            source_plate_rotation = str(vars.get("source_plate_rotation"))

        if "target_plate_rotation" in vars.keys():
            target_plate_rotation = str(vars.get("target_plate_rotation"))

        source = vars.get("source")
        target = vars.get("target")

        try:
            pf400.transfer(source, target, source_plate_rotation, target_plate_rotation)
        except Exception:
            state = "ERROR"
            traceback.print_exc()
            response["action_response"] = "failed"
        else:
            state = "IDLE"
            response["action_response"] = "succeeded"
        return response

    elif action_handle == "remove_lid":
        target_plate_rotation = ""

        if "target" not in vars.keys():
            err = 1
            msg = "Target location is not provided. Canceling the job!"
            state = "ERROR"

        if len(vars.get("target")) != 6:
            err = 1
            msg = (
                "Target position should be six joint angles length. Canceling the job!"
            )
            state = "ERROR"

        if err:
            response["action_response"] = "failed"
            response["action_log"] = msg
            state = "ERROR"
            return response

        if "target_plate_rotation" not in vars.keys():
            pass
        else:
            target_plate_rotation = str(vars.get("target_plate_rotation"))

        target = vars.get("target")

        lid_height = vars.get("lid_height", 7.0)

        try:
            pf400.remove_lid(target, lid_height, target_plate_rotation)
        except Exception:
            response["action_response"] = "failed"
            state = "ERROR"
        else:
            state = "IDLE"
            response["action_response"] = "succeeded"
        return response

    elif action_handle == "replace_lid":
        target_plate_rotation = ""

        if "target" not in vars.keys():
            err = 1
            msg = "Target location is not provided. Canceling the job!"
            state = "ERROR"

        if len(vars.get("target")) != 6:
            err = 1
            msg = (
                "Target position should be six joint angles length. Canceling the job!"
            )
            state = "ERROR"

        if err:
            response["action_response"] = "failed"
            response["action_log"] = msg
            state = "ERROR"
            return response

        if "target_plate_rotation" not in vars.keys():
            pass
        else:
            target_plate_rotation = str(vars.get("target_plate_rotation"))

        if "lid_height" not in vars.keys():
            lid_height = 7.0

        else:
            lid_height = vars.get("lid_height")

        try:
            pf400.replace_lid(target, lid_height, target_plate_rotation)
        except Exception:
            response["action_response"] = "failed"
            state = "ERROR"
        else:
            state = "IDLE"
            response["action_response"] = "succeeded"
        return response

    else:
        msg = "UNKNOWN ACTION REQUEST! Available actions: explore_workcell, transfer, remove_lid, replace_lid"
        state = "ERROR"
        response["action_response"] = "failed"
        response["action_log"] = msg
        return response


if __name__ == "__main__":
    import uvicorn

    args = parse_args()

    uvicorn.run(
        "pf400_rest_node:app",
        host=args.host,
        port=args.port,
        reload=True,
        ws_max_size=100000000000000000000000000000000000000,
    )
