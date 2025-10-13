#! /usr/bin/env python3
"""The server for the PF400 robot that takes incoming WEI flow requests from the experiment application"""

from typing import Annotated, Optional

from madsci.common.types.auth_types import OwnershipInfo
from madsci.common.types.location_types import LocationArgument
from madsci.common.types.node_types import RestNodeConfig
from madsci.common.types.resource_types import Slot
from madsci.common.types.resource_types.definitions import (
    AssetResourceDefinition,
    SlotResourceDefinition,
    
)
from madsci.node_module.helpers import action
from madsci.node_module.rest_node_module import RestNode
from pf400_interface.pf400 import PF400


class PF400NodeConfig(RestNodeConfig):
    """Configuration for the pf400 node module."""

    pf400_ip: Optional[str] = None
    """IP Address for the PF400 to control"""
    pf400_port: int = 10100
    """Port to connect to the PF400 robot, default is 10100"""
    pf400_status_port: int = 10000
    """Port to connect to the PF400 status server, default is 10000"""


class PF400Node(RestNode):
    """A Rest Node object to control PF400 robots"""

    pf400_interface: PF400 = None
    config: PF400NodeConfig = PF400NodeConfig()
    config_model = PF400NodeConfig

    def startup_handler(self) -> None:
        """Called to (re)initialize the node. Should be used to open connections to devices or initialize any other resources."""

        if self.resource_client:
            self.resource_owner = OwnershipInfo(node_id=self.node_definition.node_id)
            
            gripper_slot = Slot(
                resource_name="pf400_gripper",
                resource_class="PF400Gripper",
                capacity=1,
                attributes={
                    "gripper_type": "finger",
                    "payload_kg": 0.5,
                    "payload_lb": 1.1,
                    "max_grip_force_newton": 23.0,
                    "grip_width_range": [80.0, 140.0],
                    "description": "PF400 robot gripper slot"
                }
            )
            
            gripper_template = self.resource_client.init_template(
                resource=gripper_slot,
                template_name="pf400_gripper_slot",
                description="Template for PF400 robot gripper slot. Used to track what the robot is currently holding.",
                required_overrides=["resource_name"],
                tags=["pf400", "gripper", "slot"],
                created_by=self.node_definition.node_id,
                version="1.0.0"
            )
            
            self.gripper_resource = self.resource_client.create_resource_from_template(
                template_name="pf400_gripper_slot",
                resource_name="pf400_gripper",
                overrides={"resource_name_prefix": "123243134124"},
                add_to_database=True
            )
            self.logger.log(f"Initialized gripper resource from template: {self.gripper_resource.resource_id}")
        else:
            self.resource_client = None
            self.gripper_resource = None

        if self.config.pf400_ip is None:
            raise ValueError("PF400 IP address is not set in the configuration.")
        self.pf400_interface = PF400(
            host=self.config.pf400_ip,
            port=self.config.pf400_port,
            status_port=self.config.pf400_status_port,
            resource_client=self.resource_client,
            gripper_resource_id=self.gripper_resource.resource_id
            if self.gripper_resource
            else None,
        )
        self.pf400_interface.initialize_robot()

    def shutdown_handler(self) -> None:
        """Called to shutdown the node. Should be used to close connections to devices or release any other resources."""
        try:
            self.pf400_interface.disconnect()
            del self.pf400_interface
            self.pf400_interface = None
        except Exception as err:
            self.logger.log_error(f"Error shutting down the PF400 Node: {err}")
            raise err

    def state_handler(self) -> None:
        """Periodically called to update the current state of the node."""
        if self.pf400_interface is not None:
            # Getting robot state
            robot_state = self.pf400_interface.movement_state
            current_location = self.pf400_interface.get_joint_states()
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
    ) -> None:
        """Transfer a plate from `source` to `target`, optionally using intermediate `approach` positions and target rotations."""
        if self.resource_client:
            source_resource = self.resource_client.get_resource(source.resource_id)
            target_resource = self.resource_client.get_resource(target.resource_id)
            if source_resource.quantity == 0:
                raise Exception("Resource manager: Plate does not exist at source!"
                )
            if (
                target_resource.quantity != 0
                and target_resource.resource_id != source_resource.resource_id
            ):
                raise Exception("Resource manager: Target is occupied by another plate!"
                )

        self.pf400_interface.transfer(
            source=source,
            target=target,
            source_approach=source_approach if source_approach else None,
            target_approach=target_approach if target_approach else None,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        ) 

    @action(name="pick_plate", description="Pick a plate from a source location")
    def pick_plate(
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        source_approach: Annotated[
            Optional[LocationArgument], "Location to approach from"
        ] = None,
    ) -> None:
        """Picks a plate from `source`, optionally moving first to `source_approach`."""
        if self.resource_client:
            source_resource = self.resource_client.get_resource(source.resource_id)
            if source_resource.quantity == 0:
                raise Exception("Resource manager: Plate does not exist at source!"
                )

        pick_result = self.pf400_interface.pick_plate(
            source=source,
            source_approach=source_approach if source_approach else None,
        )
        if not pick_result:
            raise Exception(f"Failed to pick plate from location {source}.")

        

    @action(
        name="place_plate",
        description="Place a plate in a target location, optionally moving first to target_approach",
    )
    def place_plate(
        self,
        target: Annotated[LocationArgument, "Location to place a plate to"],
        target_approach: Annotated[
            Optional[LocationArgument], "Location to approach from"
        ] = None,
    ) -> None:
        """Place a plate in the `target` location, optionally moving first to `target_approach`."""
        if self.resource_client:
            target_resource = self.resource_client.get_resource(target.resource_id)
            if target_resource.quantity != 0:
                raise Exception(
                    "Resource manager: Target is occupied by another plate!"
                )
        self.pf400_interface.place_plate(
            target=target,
            target_approach=target_approach if target_approach else None,
        )

        

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
    ) -> None:
        """Remove a lid from a plate located at location ."""

        if self.resource_client:
            source_resource = self.resource_client.get_resource(source.resource_id)
            target_resource = self.resource_client.get_resource(target.resource_id)
            if source_resource.quantity == 0:
               raise Exception("Resource manager: Plate does not exist at source!"
                )
            if target_resource.quantity != 0:
                raise Exception(
                    "Resource manager: Target is occupied by another plate!"
                )

            lid_resource = self.resource_client.init_resource(
                SlotResourceDefinition(
                    resource_name="pf400_lid_slot",
                    owner=self.resource_owner,
                )
            )
            lid = self.resource_client.init_resource(
                AssetResourceDefinition(resource_name="Lid", owner=self.resource_owner)
            )
            lid_resource = self.resource_client.push(resource=lid_resource, child=lid)
            source.resource_id = lid_resource.resource_id

        self.pf400_interface.remove_lid(
            source=source,
            target=target,
            lid_height=lid_height,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )

        

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
    ) -> None:
        """A doc string, but not the actual description of the action."""
        if self.resource_client:
            source_resource = self.resource_client.get_resource(source.resource_id)
            target_resource = self.resource_client.get_resource(target.resource_id)
            if source_resource.quantity == 0:
                raise Exception(
                    "Resource manager: Lid does not exist at source!"
                )
            if target_resource.quantity == 0:
                raise Exception("Resource manager: No plate on target!")

            lid_resource = self.resource_client.init_resource(
                SlotResourceDefinition(
                    resource_name="pf400_lid_slot",
                    owner=self.resource_owner,
                )
            )
            target.resource_id = lid_resource.resource_id
        self.pf400_interface.replace_lid(
            source=source,
            target=target,
            lid_height=lid_height,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )
        if self.resource_client:
            self.resource_client.remove_resource(lid_resource.resource_id)

        

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
