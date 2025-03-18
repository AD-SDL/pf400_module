#! /usr/bin/env python3
"""The server for the PF400 robot that takes incoming WEI flow requests from the experiment application"""

import datetime
from time import sleep
from typing import Any, List, Optional, Union

from madsci.client.resource_client import ResourceClient
from madsci.common.types.action_types import ActionFailed, ActionSucceeded
from madsci.common.types.location_types import Location
from madsci.common.types.node_types import RestNodeConfig
from madsci.node_module.abstract_node_module import action
from madsci.node_module.rest_node_module import RestNode
from pf400_interface.pf400 import PF400
from typing_extensions import Annotated


class PF400NodeConfig(RestNodeConfig):
    """Configuration for the pf400 node module."""

    __test__ = False

    pf400_ip: str = "146.137.240.35"
    """Required Robot IP"""
    pf400_port: int = None


class PF400Node(RestNode):
    """A Rest Node object to control PF400 robots"""

    __test__ = False

    pf400_interface = PF400 = None
    config_model = PF400NodeConfig

    def startup_handler(self) -> None:
        """Called to (re)initialize the node. Should be used to open connections to devices or initialize any other resources."""
        try:
            self.logger.log("Node initializing...")
            self.pf400_interface = PF400(host=self.config_model.ip)
            self.pf400_interface.initialize_robot()
            self.resource_client = ResourceClient(url="http://testserver")
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
        source: Annotated[dict[str, Any], "Location to pick a plate from"],
        target: Annotated[dict[str, Any], "Location to place a plate to"],
        source_approach: Annotated[dict[str, Any], "Location to approach from"] = None,
        target_approach: Annotated[dict[str, Any], "Location to approach from"] = None,
        source_plate_rotation: Annotated[
            str, "Orientation of the plate at the source, wide or narrow"
        ] = "",
        target_plate_rotation: Annotated[
            str, "Final orientation of the plate at the target, wide or narrow"
        ] = "",
    ):
        """A doc string, but not the actual description of the action."""
        try:
            source = Location.model_validate(source)
            target = Location.model_validate(target)
            source_approach = (
                Location.model_validate(source_approach) if source_approach else None
            )
            target_approach = (
                Location.model_validate(target_approach) if target_approach else None
            )
        except Exception as e:
            return ActionFailed(errors=f"Invalid location data: {e}")
        # resource = self.resource_client.get_resource(resource_id=source.resource_id)
        popped_plate, resource = self.resource_client.pop(source.resource_id)
        self.pf400_interface.transfer(
            source_loc=source.look_up,
            target_loc=target.look_up,
            source_approach=source_approach.look_up,
            target_approach=target_approach.look_up,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )
        self.resource_client.push(target.resource_id, popped_plate)

        # if result:
        return ActionSucceeded()
        # return ActionFailed(
        # errors=f"`run_command` returned '{result}'. Expected 'True'."
        # )

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
