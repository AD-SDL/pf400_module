#! /usr/bin/env python3
"""The server for the PF400 robot that takes incoming WEI flow requests from the experiment application"""

from typing import Annotated, Optional

from madsci.common.types.action_types import ActionFailed
from madsci.common.types.location_types import LocationArgument
from madsci.common.types.node_types import RestNodeConfig
from madsci.common.types.resource_types import Asset, Slot
from madsci.node_module.helpers import action
from madsci.node_module.rest_node_module import RestNode

from pf400_interface.pf400 import PF400
from pf400_interface.resource_helpers import LidResult, LidSlotResult


class PF400NodeConfig(RestNodeConfig):
    """Configuration for the pf400 node module."""

    pf400_ip: Optional[str] = None
    """IP Address for the PF400 to control"""
    pf400_port: int = 10100
    """Port to connect to the PF400 robot, default is 10100"""
    pf400_status_port: int = 10000
    """Port to connect to the PF400 status server, default is 10000"""
    rate_limit_requests: int = 500
    """Rate limit for requests to the PF400 robot, default is 100 ms"""


class PF400Node(RestNode):
    """A Rest Node object to control PF400 robots"""

    pf400_interface: PF400 = None
    config: PF400NodeConfig = PF400NodeConfig()
    config_model = PF400NodeConfig

    def startup_handler(self) -> None:
        """Called to (re)initialize the node. Should be used to open connections to devices or initialize any other resources."""

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
                "description": "PF400 robot gripper slot",
            },
        )

        self.resource_client.init_template(
            resource=gripper_slot,
            template_name="pf400_gripper",
            description="Template for PF400 robot gripper slot. Used to track what the robot is currently holding.",
            required_overrides=["resource_name"],
            tags=["pf400", "gripper", "slot"],
            created_by=self.node_definition.node_id,
            version="1.0.0",
        )

        self.gripper_resource = self.resource_client.create_resource_from_template(
            template_name="pf400_gripper",
            resource_name=f"{self.node_definition.node_name}.gripper",
            add_to_database=True,
        )
        self.logger.log_info(
            f"Initialized gripper resource from template: {self.gripper_resource.resource_id}"
        )

        # Create lid slot template for temporary lid storage
        lid_slot = Slot(
            resource_name="pf400_lid_slot",
            resource_class="PF400LidSlot",
            capacity=1,
            attributes={
                "slot_type": "lid_holder",
                "description": "Temporary slot for holding plate lids during lid operations",
            },
        )

        self.resource_client.init_template(
            resource=lid_slot,
            template_name="pf400_lid_slot",
            description="Template for temporary lid storage slot. Used when removing/replacing lids from plates.",
            required_overrides=["resource_name"],
            tags=["pf400", "lid", "slot", "temporary"],
            created_by=self.node_definition.node_id,
            version="1.0.0",
        )

        # Create plate lid asset template
        plate_lid = Asset(
            resource_name="Lid",
            resource_class="PlateLid",
            attributes={
                "lid_type": "microplate",
                "compatible_with": ["96-well"],
                "material": "plastic",
                "description": "Standard plate lid",
            },
        )

        self.resource_client.init_template(
            resource=plate_lid,
            template_name="plate_lid",
            description="Template for plate lids. Used to track lids during lid operations.",
            required_overrides=["resource_name"],
            tags=["lid", "plate", "asset"],
            created_by=self.node_definition.node_id,
            version="1.0.0",
        )

        if self.config.pf400_ip is None:
            raise ValueError("PF400 IP address is not set in the configuration.")
        self.pf400_interface = PF400(
            host=self.config.pf400_ip,
            port=self.config.pf400_port,
            status_port=self.config.pf400_status_port,
            resource_client=self.resource_client,
            gripper_resource_id=self.gripper_resource.resource_id,
        )
        self.pf400_interface.initialize_robot()
        self.logger.log_info("PF400 Node initialized.")

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
        rotation_deck: Optional[LocationArgument] = None,
        grab_offset: Optional[Annotated[float, "Add grab height offset"]] = None,
        source_approach_height_offset: Optional[
            Annotated[float, "Add source approach height offset"]
        ] = None,
        target_approach_height_offset: Optional[
            Annotated[float, "Add target approach height offset"]
        ] = None,
    ) -> Optional[ActionFailed]:
        """Transfer a plate from `source` to `target`, optionally using intermediate `approach` positions and target rotations."""

        if source.resource_id:
            source_resource = self.resource_client.get_resource(source.resource_id)
            if source_resource.quantity == 0:
                return ActionFailed(
                    errors=[
                        f"Plate does not exist at source location! Resource_id:{source.resource_id}."
                    ]
                )
        if target.resource_id:
            target_resource = self.resource_client.get_resource(target.resource_id)
            if (
                target_resource.quantity != 0
                and target_resource.resource_id != source_resource.resource_id
            ):
                return ActionFailed(
                    errors=[
                        f"Target is occupied by another plate! Resource_id:{target.resource_id}."
                    ]
                )

        transfer_result = self.pf400_interface.transfer(
            source=source,
            target=target,
            source_approach=source_approach if source_approach else None,
            target_approach=target_approach if target_approach else None,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
            rotation_deck=rotation_deck if rotation_deck else None,
            grab_offset=grab_offset,
            source_approach_height_offset=source_approach_height_offset,
            target_approach_height_offset=target_approach_height_offset,
        )
        if not transfer_result:
            return ActionFailed(
                errors=[f"Failed to transfer plate from {source} to {target}."]
            )

        return None

    @action(name="pick_plate", description="Pick a plate from a source location")
    def pick_plate(
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        source_approach: Annotated[
            Optional[LocationArgument], "Location to approach from"
        ] = None,
        source_plate_rotation: Annotated[
            str, "Orientation of the plate at the source, wide or narrow"
        ] = "",
        grab_offset: Optional[Annotated[float, "Add grab height offset"]] = None,
        approach_height_offset: Optional[
            Annotated[float, "Add approach height offset"]
        ] = None,
    ) -> Optional[ActionFailed]:
        """Picks a plate from `source`, optionally moving first to `source_approach`."""
        if source.resource_id:
            source_resource = self.resource_client.get_resource(source.resource_id)
            if source_resource.quantity == 0:
                return ActionFailed(
                    errors=[
                        f"Resource manager: Plate does not exist at source! Resource_id:{source.resource_id}."
                    ]
                )

        # set plate width for source
        if source_plate_rotation.lower() == "wide":
            plate_source_rotation = 90
            self.pf400_interface.grip_wide = True
        elif source_plate_rotation.lower() == "narrow" or source_plate_rotation == "":
            plate_source_rotation = 0
            self.pf400_interface.grip_wide = False
        else:
            return ActionFailed(
                errors=[
                    f"Invalid source plate rotation: {source_plate_rotation}. "
                    "Expected 'wide', 'narrow', or ''."
                ]
            )

        source.representation = self.pf400_interface.check_incorrect_plate_orientation(
            source.representation, plate_source_rotation
        )

        pick_result = self.pf400_interface.pick_plate(
            source=source,
            source_approach=source_approach if source_approach else None,
            grab_offset=grab_offset,
            approach_height_offset=approach_height_offset,
        )
        if not pick_result:
            return ActionFailed(
                errors=[f"Failed to pick plate from location {source}."]
            )
        return None

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
        target_plate_rotation: Annotated[
            str, "Final orientation of the plate at the target, wide or narrow"
        ] = "",
        grab_offset: Optional[Annotated[float, "Add grab height offset"]] = None,
        approach_height_offset: Optional[
            Annotated[float, "Add approach height offset"]
        ] = None,
    ) -> Optional[ActionFailed]:
        """Place a plate in the `target` location, optionally moving first to `target_approach`."""

        if target.resource_id:
            target_resource = self.resource_client.get_resource(target.resource_id)
            if target_resource.quantity != 0:
                return ActionFailed(
                    errors=[
                        f"Resource manager: Target is occupied by another plate! Resource_id:{target.resource_id}."
                    ]
                )

        if target_plate_rotation.lower() == "wide":
            plate_target_rotation = 90
            self.pf400_interface.grip_wide = True
        elif target_plate_rotation.lower() == "narrow" or target_plate_rotation == "":
            plate_target_rotation = 0
            self.pf400_interface.grip_wide = False
        else:
            return ActionFailed(
                errors=[
                    f"Invalid target plate rotation: {target_plate_rotation}. "
                    "Expected 'wide', 'narrow', or ''."
                ]
            )

        target.representation = self.pf400_interface.check_incorrect_plate_orientation(
            target.representation, plate_target_rotation
        )

        place_result = self.pf400_interface.place_plate(
            target=target,
            target_approach=target_approach if target_approach else None,
            grab_offset=grab_offset,
            approach_height_offset=approach_height_offset,
        )
        if not place_result:
            return ActionFailed(
                errors=["Transfer failed: plate not released properly."]
            )

        return None

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
        grab_offset: Optional[Annotated[float, "Add grab height offset"]] = None,
        source_approach_height_offset: Optional[
            Annotated[float, "Add source approach height offset"]
        ] = None,
        target_approach_height_offset: Optional[
            Annotated[float, "Add target approach height offset"]
        ] = None,
    ) -> Optional[ActionFailed]:
        """Remove a lid from a plate located at location ."""

        if source.resource_id:
            source_resource = self.resource_client.get_resource(source.resource_id)
            if source_resource.quantity == 0:
                return ActionFailed(
                    errors=[
                        f"Resource manager: Plate does not exist at source! Resource_id:{source.resource_id}."
                    ]
                )
        if target.resource_id:
            target_resource = self.resource_client.get_resource(target.resource_id)
            if target_resource.quantity != 0:
                return ActionFailed(
                    errors=[
                        f"Resource manager: Target is occupied by another plate! Resource_id:{target.resource_id}."
                    ]
                )

        # Extract id of plate resource at source
        source_resource = self.resource_client.get_resource(source.resource_id)
        plate_resource = source_resource.child
        plate_resource_id = plate_resource.resource_id

        # Check for plate resource format compliance
        # Is there a lid slot? Is there a lid in that lid slot?
        lid_resource = None
        lid_slot_resource = None
        if "lid_slot" in plate_resource.children:
            lid_slot_child_value = plate_resource.children["lid_slot"]
            if isinstance(lid_slot_child_value, Slot):
                if len(lid_slot_child_value.children) == 1:
                    lid_slot_resource = lid_slot_child_value
                    lid_resource = lid_slot_child_value.child  # collect lid resource
                    self.logger.log_info(
                        f"Identified lid resource {lid_resource.resource_id} for removal."
                    )
                else:
                    return ActionFailed(
                        errors=[
                            f"No lid resource exists in the lid slot. {lid_slot_child_value}"
                        ]
                    )
            else:
                return ActionFailed(
                    errors=[
                        f"Lid slot child value is not of type Slot. {lid_slot_child_value=}"
                    ]
                )
        else:
            # The plate resource does not have a compliant format. Create a new lid from scratch
            #  Create temporary lid slot from template
            self.logger.log_warning(
                "No lid_slot exists on plate resource at source indicating a non-compliant labware resource definition was used. Creating a lid from scratch."
            )
            lid_slot_resource = self.resource_client.create_resource_from_template(
                template_name="pf400_lid_slot",
                resource_name="pf400_lid_slot",
                add_to_database=True,
            )
            # Create lid asset from template
            lid_resource = self.resource_client.create_resource_from_template(
                template_name="plate_lid",
                resource_name=f"Lid_from_{plate_resource_id}",
                add_to_database=True,
            )
            # put newly created lid in newly created lid slot
            self.resource_client.push(resource=lid_slot_resource, child=lid_resource)

        if lid_slot_resource is not None and target_resource is not None:
            source.resource_id = lid_slot_resource.resource_id  # critical!
            self.pf400_interface.remove_lid(
                source=source,
                target=target,
                lid_height=lid_height,
                source_approach=source_approach,
                target_approach=target_approach,
                source_plate_rotation=source_plate_rotation,
                target_plate_rotation=target_plate_rotation,
                grab_offset=grab_offset,
                source_approach_height_offset=source_approach_height_offset,
                target_approach_height_offset=target_approach_height_offset,
            )

        else:
            return ActionFailed(
                errors=[
                    f"lid_resource or target_resource is None. \n{lid_resource=} \n{target_resource=}"
                ]
            )

        return None

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
        grab_offset: Optional[Annotated[float, "Add grab height offset"]] = None,
        source_approach_height_offset: Optional[
            Annotated[float, "Add source approach height offset"]
        ] = None,
        target_approach_height_offset: Optional[
            Annotated[float, "Add target approach height offset"]
        ] = None,
    ) -> Optional[ActionFailed]:
        """A doc string, but not the actual description of the action."""

        lid_resource = None
        lid_slot_resource = None

        # Collect lid from source, checking for lid resource conformity to standard.
        lid_resource, source_ok, err = self._get_lid_from_source(source=source)
        if err:
            return err

        # Collect lid slot resource from target, checking for plate resource conformity to standard.
        lid_slot_resource, target_ok, target_resource, err = (
            self._get_lid_slot_from_target(target)
        )
        if err:
            return err

        conforms_to_standard = source_ok and target_ok

        # Return action failed if we failed to collect any variables.
        if not (lid_resource and lid_slot_resource and target_resource):
            return ActionFailed(
                errors=[f"{lid_slot_resource=}, {lid_resource=}, {target_resource=}"]
            )

        target.resource_id = lid_slot_resource.resource_id  # critical!

        self.pf400_interface.replace_lid(
            source=source,
            target=target,
            lid_height=lid_height,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
            grab_offset=grab_offset,
            source_approach_height_offset=source_approach_height_offset,
            target_approach_height_offset=target_approach_height_offset,
        )

        if not conforms_to_standard:
            self.resource_client.remove_resource(lid_resource.resource_id)

        return None

    def _get_lid_from_source(self, source: LocationArgument) -> LidResult:
        """Retreives the lid from the source location. Called from replace_lid."""

        if not source.resource_id:
            return None, True, None
        source_resource = self.resource_client.get_resource(source.resource_id)
        if not source_resource.children:
            return (
                None,
                True,
                ActionFailed(errors=["No lid resource exists at source location."]),
            )
        child = source_resource.children[-1]
        if "lid" not in child.attributes:
            self.logger.log_warning(
                f"Resource Manager: Lid resource found does not conform to standard. {child}"
            )
            return child, False, None
        if not child.attributes["lid"]:
            return (
                None,
                True,
                ActionFailed(errors=['Resource Manager: "lid" attribute is False.']),
            )
        return LidResult(child, True, None)

    def _get_lid_slot_from_target(self, target: LocationArgument) -> LidSlotResult:
        """Retreives the lid slot from the plate on the target location. Called from replace_lid."""

        if not target.resource_id:
            return None, True, None, None
        target_resource = self.resource_client.get_resource(target.resource_id)
        if not target_resource.children:
            return (
                None,
                True,
                target_resource,
                ActionFailed(
                    errors=[
                        f"No plate resource exists at the target location {target.name}."
                    ]
                ),
            )
        plate = target_resource.children[-1]
        if "lid_slot" not in plate.children:
            self.logger.log_warning("Target plate has no lid slot. Creating one.")
            lid_slot = self.resource_client.create_resource_from_template(
                template_name="pf400_lid_slot",
                resource_name="pf400_lid_slot",
                add_to_database=True,
            )
            return lid_slot, False, target_resource, None
        lid_slot = plate.children["lid_slot"]
        if lid_slot.children:
            return (
                None,
                True,
                target_resource,
                ActionFailed(errors=["A lid already exists on the target plate."]),
            )
        return LidSlotResult(lid_slot, True, target_resource, None)

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
