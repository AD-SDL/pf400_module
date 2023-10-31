#! /usr/bin/env python3
"""The server for the PF400 arm that takes incoming WEI flow requests from the experiment application"""

import datetime
import json
from argparse import ArgumentParser
from contextlib import asynccontextmanager
from time import sleep

from fastapi import FastAPI
from fastapi.responses import JSONResponse

from pf400_driver.errors import ConnectionException
from pf400_driver.pf400_driver import PF400

workcell = None
global pf400, state, action_start
serial_port = "/dev/ttyUSB0"
local_ip = "parker.alcf.anl.gov"
local_port = "8000"


@asynccontextmanager
async def lifespan(app: FastAPI):
    global pf400, state
    """Initial run function for the app, parses the workcell argument
        Parameters
        ----------
        app : FastApi
           The REST API app being initialized

        Returns
        -------
        None"""
    ip = "127.0.0.1"
    port = 8085

    ip = "146.137.240.35"
    port = 10100

    try:
        pf400 = PF400(ip, port)
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
    yield

    # Do any cleanup here
    pass


app = FastAPI(
    lifespan=lifespan,
)


def check_state():
    """updates the Pf400 state

    Parameters:
    -----------
        None
    Returns
    -------
        None
    """
    global state, pf400
    try_connect = False
    err = None

    try:
        movement_state = pf400.movement_state

    except UnboundLocalError as local_var_err:
        err = local_var_err

    except AttributeError as attribute_err:
        err = attribute_err
        try_connect = True

    except Exception as general_err:
        err = general_err

    finally:
        if try_connect:
            state = "ERROR"
            try:
                ip = "146.137.240.35"
                port = 10100
                pf400 = PF400(ip, port)
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
    global state, action_start
    if not (state == "BUSY") or (
        action_start
        and (datetime.datetime.now() - action_start > datetime.timedelta(0, 2))
    ):
        check_state()
    return JSONResponse(content={"State": state})


@app.get("/description")
async def description():
    global pf400
    return JSONResponse(content={"State": pf400.get_status()})


@app.get("/resources")
async def resources():
    global pf400
    return JSONResponse(content={"State": pf400.get_status()})


@app.post("/action")
def do_action(action_handle: str, action_vars):
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
            msg = "Position 1 should be six joint angles lenght. Canceling the job!"
        elif len(vars.get("target")) != 6:
            err = True
            msg = "Position 2 should be six joint angles lenght. Canceling the job!"

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
            response["action_response"] = "failed"
        else:
            state = "IDLE"
            response["action_response"] = "succeeded"
        finally:
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
                "Target position should be six joint angles lenght. Canceling the job!"
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
        finally:
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
                "Target position should be six joint angles lenght. Canceling the job!"
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
        finally:
            return response

    else:
        msg = "UNKNOWN ACTION REQUEST! Available actions: explore_workcell, transfer, remove_lid, replace_lid"
        state = "ERROR"
        response["action_response"] = "failed"
        response["action_log"] = msg
        return response


if __name__ == "__main__":
    import uvicorn

    parser = ArgumentParser()
    parser.add_argument("--alias", type=str, help="Name of the Node")
    parser.add_argument("--host", type=str, help="Host for rest")
    parser.add_argument("--port", type=int, help="port value")
    args = parser.parse_args()
    uvicorn.run(
        "pf400_rest_client:app",
        host=args.host,
        port=args.port,
        reload=True,
        ws_max_size=100000000000000000000000000000000000000,
    )
