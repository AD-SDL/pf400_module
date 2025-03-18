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
            source=source.look_up,
            target=target.look_up,
            source_approach=source_approach.look_up,
            target_approach=target_approach.look_up,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )
        self.resource_client.push(target.resource_id, popped_plate)

        return ActionSucceeded()
        # return ActionFailed(
        # errors=f"`run_command` returned '{result}'. Expected 'True'."
        # )
        
    @action(
    name="pick_plate", description="Pick a plate from a source location"
    )
    def pick_plate(
        self,
        source: Annotated[dict[str, Any], "Location to pick a plate from"],
        source_approach: Annotated[dict[str, Any], "Location to approach from"] = None,

    ):
        """A doc string, but not the actual description of the action."""
        try:
            source = Location.model_validate(source)
            source_approach = (
                Location.model_validate(source_approach) if source_approach else None
            )
        except Exception as e:
            return ActionFailed(errors=f"Invalid location data: {e}")
        # resource = self.resource_client.get_resource(resource_id=source.resource_id)
        popped_plate, resource = self.resource_client.pop(source.resource_id)
        self.pf400_interface.pick_plate(
            source_loc=source.look_up,
            source_approach=source_approach.look_up,
        )
        
        return ActionSucceeded()
    
    @action(
    name="place_plate", description="Place a plate to a target location"
    )
    def place_plate(
        self,
        target: Annotated[dict[str, Any], "Location to place a plate to"],
        target_approach: Annotated[dict[str, Any], "Location to approach from"] = None,
    ):
        """A doc string, but not the actual description of the action."""
        try:
            source = Location.model_validate(source)
            source_approach = (
                Location.model_validate(source_approach) if source_approach else None
            )
        except Exception as e:
            return ActionFailed(errors=f"Invalid location data: {e}")
        # resource = self.resource_client.get_resource(resource_id=source.resource_id)
        self.pf400_interface.place_plate(
            target=target.look_up,
            target_approach=target_approach.look_up,
        )
        # self.resource_client.push(target.resource_id, popped_plate)

        return ActionSucceeded()
    
    @action(
       name="remove_lid", description="Remove a lid from a plate"
    )
    def remove_lid(
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
        lid_height: Annotated[float, "height of the lid, in steps"] = 7.0,

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
        self.pf400_interface.remove_lid(
            source=source.look_up,
            target=target.look_up,
            lid_height=lid_height,
            source_approach=source_approach.look_up,
            target_approach=target_approach.look_up,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )
        self.resource_client.push(target.resource_id, popped_plate)

        return ActionSucceeded()
    @action(
        name="replace_lid", description="Replace a lid on a plate"
    )
    def replace_lid(
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
        lid_height: Annotated[float, "height of the lid, in steps"] = 7.0,

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
        self.pf400_interface.replace_lid(
            source=source.look_up,
            target=target.look_up,
            lid_height=lid_height,
            source_approach=source_approach.look_up,
            target_approach=target_approach.look_up,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )
        self.resource_client.push(target.resource_id, popped_plate)

        return ActionSucceeded()
            
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
