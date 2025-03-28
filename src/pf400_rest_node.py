#! /usr/bin/env python3
"""The server for the PF400 robot that takes incoming WEI flow requests from the experiment application"""

from typing import Annotated, Optional

from madsci.client.resource_client import ResourceClient
from madsci.common.types.action_types import ActionSucceeded
from madsci.common.types.auth_types import OwnershipInfo
from madsci.common.types.location_types import LocationArgument
from madsci.common.types.node_types import RestNodeConfig
from madsci.common.types.resource_types.definitions import SlotResourceDefinition
from madsci.node_module.helpers import action
from madsci.node_module.rest_node_module import RestNode
from pf400_interface.pf400 import PF400
from pydantic.networks import AnyUrl


class PF400NodeConfig(RestNodeConfig):
    """Configuration for the pf400 node module."""

    pf400_ip: str
    resource_manager_url: Optional[AnyUrl] = None


class PF400Node(RestNode):
    """A Rest Node object to control PF400 robots"""

    pf400_interface: PF400 = None
    config_model = PF400NodeConfig

    def startup_handler(self) -> None:
        """Called to (re)initialize the node. Should be used to open connections to devices or initialize any other resources."""

        try:
            if self.config.resource_manager_url:
                self.resource_client = ResourceClient(self.config.resource_manager_url)
                self.gripper_resource = self.resource_client.init_resource(
                    SlotResourceDefinition(
                        resource_name="pf400_gripper",
                        owner=OwnershipInfo(node_id=self.node_definition.node_id),
                    )
                )
            else:
                self.resource_client = None
                self.gripper_resource = None

            self.logger.log("Node initializing...")
            self.pf400_interface = PF400(
                host=self.config.pf400_ip,
                resource_client=self.resource_client,
                gripper_resource_id=self.gripper_resource.resource_id
                if self.gripper_resource
                else None,
            )
            self.pf400_joint_state = PF400(host=self.config.pf400_ip, port=10000)
            self.pf400_interface.initialize_robot()

        except Exception as err:
            self.logger.log_error(f"Error starting the PF400 Node: {err}")
            self.startup_has_run = False
        else:
            self.startup_has_run = True
            self.logger.log("PF400 node initialized!")

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
            current_location = self.pf400_joint_state.get_joint_states()
            if robot_state == 0:
                self.node_state = {
                    "pf400_status_code": "POWER OFF",
                    "current_joint_angles": current_location,
                }
                self.logger.log_error("PF400 POWER OFF")
            elif robot_state == 1:
                self.node_state = {
                    "pf400_status_code": "READY",
                    "current_joint_angles": current_location,
                }
            elif robot_state > 1:
                self.node_state = {
                    "pf400_status_code": "BUSY",
                    "current_joint_angles": current_location,
                }
            else:
                self.node_state = {
                    "pf400_status_code": self.pf400_interface.robot_state,
                    "current_joint_angles": current_location,
                }

    @action(
        name="transfer", description="Transfer a plate from one location to another"
    )
    def transfer(
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        target: Annotated[LocationArgument, "Location to place a plate to"],
        source_approach: Annotated[
            Optional[LocationArgument], "Location to approach from"
        ] = None,
        target_approach: Annotated[
            Optional[LocationArgument], "Location to approach from"
        ] = None,
        source_plate_rotation: Annotated[
            str, "Orientation of the plate at the source, wide or narrow"
        ] = "",
        target_plate_rotation: Annotated[
            str, "Final orientation of the plate at the target, wide or narrow"
        ] = "",
    ):
        """A doc string, but not the actual description of the action."""
        try:
            self.pf400_interface.transfer(
                source=source,
                target=target,
                source_approach=source_approach if source_approach else None,
                target_approach=target_approach if target_approach else None,
                source_plate_rotation=source_plate_rotation,
                target_plate_rotation=target_plate_rotation,
            )
        except Exception as err:
            self.logger.log_error(err)
        return ActionSucceeded()

    @action(name="pick_plate", description="Pick a plate from a source location")
    def pick_plate(
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        source_approach: Annotated[
            Optional[LocationArgument], "Location to approach from"
        ] = None,
    ):
        """A doc string, but not the actual description of the action."""

        source_approach = None
        self.pf400_interface.pick_plate(
            source=source,
            source_approach=source_approach if source_approach else None,
        )

        return ActionSucceeded()

    @action(name="place_plate", description="Place a plate to a target location")
    def place_plate(
        self,
        target: Annotated[LocationArgument, "Location to place a plate to"],
        target_approach: Annotated[
            Optional[LocationArgument], "Location to approach from"
        ] = None,
    ):
        """A doc string, but not the actual description of the action."""

        self.pf400_interface.place_plate(
            target=target,
            target_approach=target_approach if target_approach else None,
        )

        return ActionSucceeded()

    @action(name="remove_lid", description="Remove a lid from a plate")
    def remove_lid(
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        target: Annotated[LocationArgument, "Location to place a plate to"],
        source_approach: Annotated[
            Optional[LocationArgument], "Location to approach from"
        ] = None,
        target_approach: Annotated[
            Optional[LocationArgument], "Location to approach from"
        ] = None,
        source_plate_rotation: Annotated[
            str, "Orientation of the plate at the source, wide or narrow"
        ] = "",
        target_plate_rotation: Annotated[
            str, "Final orientation of the plate at the target, wide or narrow"
        ] = "",
        lid_height: Annotated[float, "height of the lid, in steps"] = 7.0,
    ):
        """A doc string, but not the actual description of the action."""

        self.pf400_interface.remove_lid(
            source=source,
            target=target,
            lid_height=lid_height,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )

        return ActionSucceeded()

    @action(name="replace_lid", description="Replace a lid on a plate")
    def replace_lid(
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        target: Annotated[LocationArgument, "Location to place a plate to"],
        source_approach: Annotated[
            Optional[LocationArgument], "Location to approach from"
        ] = None,
        target_approach: Annotated[
            Optional[LocationArgument], "Location to approach from"
        ] = None,
        source_plate_rotation: Annotated[
            str, "Orientation of the plate at the source, wide or narrow"
        ] = "",
        target_plate_rotation: Annotated[
            str, "Final orientation of the plate at the target, wide or narrow"
        ] = "",
        lid_height: Annotated[float, "height of the lid, in steps"] = 7.0,
    ):
        """A doc string, but not the actual description of the action."""

        self.pf400_interface.replace_lid(
            source=source,
            target=target,
            lid_height=lid_height,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )

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


if __name__ == "__main__":
    pf400_node = PF400Node()
    pf400_node.start_node()
