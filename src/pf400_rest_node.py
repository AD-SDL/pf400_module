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

    def _parse_location_representation(
        self, location: LocationArgument
    ) -> tuple[
        LocationArgument,
        Optional[LocationArgument],
        Optional[str],
        Optional[float],
        Optional[float],
    ]:
        """
        Parse a LocationArgument that may have a dictionary representation.

        Expected dictionary structure:
        {
            "location": 6-digit list,
            "approach": single or multiple approach locations,
            "plate_rotation": "wide" or "narrow"  # optional plate rotation
            "approach_height_offset": float  # optional approach height offset
            "height_limit": float  # optional height limit for validation
        }

        Returns:
            tuple: (location_arg_with_list_repr, approach_location_arg or None, plate_rotation or None, approach_height_offset or None, height_limit or None)
        """
        if not isinstance(location.representation, dict):
            return location, None, None, None, None

        repr_dict = location.representation

        if "location" not in repr_dict:
            raise ValueError(
                "LocationArgument representation dictionary must contain 'location' key"
            )

        location_repr = repr_dict["location"]
        approach_repr = repr_dict.get("approach", None)
        plate_rotation = repr_dict.get("plate_rotation", None)
        approach_height_offset = repr_dict.get("approach_height_offset", None)
        height_limit = repr_dict.get("height_limit", None)
        press_depth = repr_dict.get("press_depth", None)

        parsed_location = LocationArgument(
            representation=location_repr,
            resource_id=location.resource_id,
            location_name=location.location_name,
            reservation=location.reservation,
        )

        parsed_approach = None
        if approach_repr is not None:
            parsed_approach = LocationArgument(
                representation=approach_repr,
                resource_id=None,
                location_name=None,
            )

        return (
            parsed_location,
            parsed_approach,
            plate_rotation,
            approach_height_offset,
            height_limit,
            press_depth,
        )

    @action(
        name="transfer", description="Transfer a plate from one location to another"
    )
    def transfer(
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        target: Annotated[LocationArgument, "Location to place a plate to"],
        rotation_deck: Optional[LocationArgument] = None,
    ) -> Optional[ActionFailed]:
        """Transfer a plate from `source` to `target`, optionally using intermediate `approach` positions and target rotations."""

        grab_height_offset = None

        if source.resource_id:
            source_resource = self.resource_client.get_resource(source.resource_id)
            if source_resource.quantity == 0:
                return ActionFailed(
                    errors=[
                        f"Plate does not exist at source location! Resource_id:{source.resource_id}."
                    ]
                )
            if source_resource.children:
                plate_resource = source_resource.children[-1]
                if plate_resource.attributes:
                    grab_height_offset = plate_resource.attributes.get(
                        "grab_height_offset", None
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
        try:
            (
                parsed_source,
                source_approach,
                source_rotation_from_dict,
                source_approach_height_offset,
                source_height_limit,
                source_press_depth,
            ) = self._parse_location_representation(source)
            (
                parsed_target,
                target_approach,
                target_rotation_from_dict,
                target_approach_height_offset,
                target_height_limit,
                target_press_depth,
            ) = self._parse_location_representation(target)
        except Exception as e:
            return ActionFailed(
                errors=[f"Failed to parse location representation: {e}"]
            )

        transfer_result = self.pf400_interface.transfer(
            source=parsed_source,
            target=parsed_target,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_rotation_from_dict,
            target_plate_rotation=target_rotation_from_dict,
            rotation_deck=rotation_deck,
            grab_offset=grab_height_offset,
            source_approach_height_offset=source_approach_height_offset,
            target_approach_height_offset=target_approach_height_offset,
            source_height_limit=source_height_limit,
            target_height_limit=target_height_limit,
            source_press_depth=source_press_depth,
            target_press_depth=target_press_depth,
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
    ) -> Optional[ActionFailed]:
        """Picks a plate from `source`, optionally moving first to `source_approach`."""
        grab_height_offset = None

        if source.resource_id:
            source_resource = self.resource_client.get_resource(source.resource_id)
            if source_resource.quantity == 0:
                return ActionFailed(
                    errors=[
                        f"Resource manager: Plate does not exist at source! Resource_id:{source.resource_id}."
                    ]
                )
            if source_resource.children:
                plate_resource = source_resource.children[-1]
                if plate_resource.attributes:
                    grab_height_offset = plate_resource.attributes.get(
                        "grab_height_offset", None
                    )

        try:
            (
                parsed_source,
                source_approach,
                source_rotation_from_dict,
                source_approach_height_offset,
                source_height_limit,
                press_depth,
            ) = self._parse_location_representation(source)
        except Exception as e:
            return ActionFailed(
                errors=[f"Failed to parse location representation: {e}"]
            )

        plate_source_rotation = (
            90
            if source_rotation_from_dict and source_rotation_from_dict.lower() == "wide"
            else 0
        )
        self.grip_wide = (
            source_rotation_from_dict and source_rotation_from_dict.lower() == "wide"
        )

        parsed_source.representation = (
            self.pf400_interface.check_incorrect_plate_orientation(
                parsed_source.representation, plate_source_rotation
            )
        )

        pick_result = self.pf400_interface.pick_plate(
            source=parsed_source,
            source_approach=source_approach,
            grab_offset=grab_height_offset,
            approach_height_offset=source_approach_height_offset,
            height_limit=source_height_limit,
            press_depth=press_depth,
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
    ) -> Optional[ActionFailed]:
        """Place a plate in the `target` location, optionally moving first to `target_approach`."""

        grab_height_offset = None

        if target.resource_id:
            target_resource = self.resource_client.get_resource(target.resource_id)
            if target_resource.quantity != 0:
                return ActionFailed(
                    errors=[
                        f"Resource manager: Target is occupied by another plate! Resource_id:{target.resource_id}."
                    ]
                )
        if self.gripper_resource.resource_id:
            gripper_resource = self.resource_client.get_resource(
                self.gripper_resource.resource_id
            )
            if gripper_resource.quantity > 0 and gripper_resource.children:
                plate_in_gripper = gripper_resource.children[-1]
                if plate_in_gripper.attributes:
                    grab_height_offset = plate_in_gripper.attributes.get(
                        "grab_height_offset", None
                    )

        try:
            (
                parsed_target,
                target_approach,
                target_rotation_from_dict,
                target_approach_height_offset,
                target_height_limit,
                press_depth,
            ) = self._parse_location_representation(target)
        except Exception as e:
            return ActionFailed(
                errors=[f"Failed to parse location representation: {e}"]
            )

        plate_target_rotation = (
            90
            if target_rotation_from_dict and target_rotation_from_dict.lower() == "wide"
            else 0
        )
        self.grip_wide = (
            target_rotation_from_dict and target_rotation_from_dict.lower() == "wide"
        )

        parsed_target.representation = (
            self.pf400_interface.check_incorrect_plate_orientation(
                parsed_target.representation, plate_target_rotation
            )
        )

        place_result = self.pf400_interface.place_plate(
            target=parsed_target,
            target_approach=target_approach,
            grab_offset=grab_height_offset,
            approach_height_offset=target_approach_height_offset,
            height_limit=target_height_limit,
            press_depth=press_depth,
        )
        if not place_result:
            return ActionFailed(
                errors=["Transfer failed: plate not released properly."]
            )

        return None

    @action(name="remove_lid", description="Remove a lid from a plate")
    def remove_lid(  # noqa: C901
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        target: Annotated[LocationArgument, "Location to place a plate to"],
    ) -> Optional[ActionFailed]:
        """Remove a lid from a plate located at location."""

        grab_height_offset = None
        resource_lid_height = None
        plate_resource = None

        if source.resource_id:
            source_resource = self.resource_client.get_resource(source.resource_id)
            if source_resource.quantity == 0:
                return ActionFailed(
                    errors=[
                        f"Resource manager: Plate does not exist at source! Resource_id:{source.resource_id}."
                    ]
                )

            if source_resource.children:
                plate_resource = source_resource.children[-1]
                if plate_resource.attributes:
                    has_lid = plate_resource.attributes.get("has_lid", None)

                    if has_lid is None:
                        self.logger.log_warning(
                            "Continuing without resource validation for lids - 'has_lid' attribute not found in resource"
                        )
                    elif has_lid is False:
                        return ActionFailed(
                            errors=[
                                f"Resource manager: Plate at source does not have a lid! Resource_id:{source.resource_id}."
                            ]
                        )

                    grab_height_offset = plate_resource.attributes.get(
                        "grab_height_offset", None
                    )
                    resource_lid_height = plate_resource.attributes.get(
                        "lid_height", None
                    )

        if target.resource_id:
            target_resource = self.resource_client.get_resource(target.resource_id)
            if target_resource.quantity != 0:
                return ActionFailed(
                    errors=[
                        f"Resource manager: Target is occupied by another plate! Resource_id:{target.resource_id}."
                    ]
                )

        lid_resource = self.resource_client.create_resource_from_template(
            template_name="pf400_lid_slot",
            resource_name="pf400_lid_slot",
            add_to_database=True,
        )

        lid = self.resource_client.create_resource_from_template(
            template_name="plate_lid",
            resource_name=f"Lid_from_{plate_resource.resource_id}",
            add_to_database=True,
        )

        lid_resource = self.resource_client.push(resource=lid_resource, child=lid)

        try:
            (
                parsed_source,
                source_approach,
                source_rotation_from_dict,
                source_approach_height_offset,
                source_height_limit,
                source_press_depth,
            ) = self._parse_location_representation(source)
            (
                parsed_target,
                target_approach,
                target_rotation_from_dict,
                target_approach_height_offset,
                target_height_limit,
                target_press_depth,
            ) = self._parse_location_representation(target)
        except Exception as e:
            return ActionFailed(
                errors=[f"Failed to parse location representation: {e}"]
            )

        parsed_source.resource_id = lid_resource.resource_id

        remove_lid_result = self.pf400_interface.remove_lid(
            source=parsed_source,
            target=parsed_target,
            lid_height=resource_lid_height,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_rotation_from_dict,
            target_plate_rotation=target_rotation_from_dict,
            grab_offset=grab_height_offset,
            source_approach_height_offset=source_approach_height_offset,
            target_approach_height_offset=target_approach_height_offset,
            source_height_limit=source_height_limit,
            target_height_limit=target_height_limit,
            source_press_depth=source_press_depth,
            target_press_depth=target_press_depth,
        )

        if not remove_lid_result:
            return ActionFailed(errors=["Failed to remove lid."])

        if plate_resource and plate_resource.attributes:
            plate_resource.attributes["has_lid"] = False
            self.resource_client.update_resource(plate_resource)

        return None

    @action(name="replace_lid", description="Replace a lid on a plate")
    def replace_lid(  # noqa: C901
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        target: Annotated[LocationArgument, "Location to place a plate to"],
    ) -> Optional[ActionFailed]:
        """A doc string, but not the actual description of the action."""
        grab_height_offset = None
        resource_lid_height = None

        if source.resource_id:
            source_resource = self.resource_client.get_resource(source.resource_id)
            if source_resource.quantity == 0:
                return ActionFailed(
                    "Resource manager: Lid does not exist at source! Resource_id:{source.resource_id}."
                )
            if source_resource.children:
                lid_resource_child = source_resource.children[-1]
                if lid_resource_child.attributes:
                    grab_height_offset = lid_resource_child.attributes.get(
                        "grab_height_offset", None
                    )
                    resource_lid_height = lid_resource_child.attributes.get(
                        "lid_height", None
                    )

        if target.resource_id:
            target_resource = self.resource_client.get_resource(target.resource_id)
            if target_resource.quantity == 0:
                return ActionFailed(
                    f"Resource manager: No plate on target! Resource_id:{target.resource_id}."
                )

        # Create temporary lid slot from template
        lid_resource = self.resource_client.create_resource_from_template(
            template_name="pf400_lid_slot",
            resource_name="pf400_lid_slot",
            add_to_database=True,
        )

        try:
            (
                parsed_source,
                source_approach,
                source_rotation_from_dict,
                source_approach_height_offset,
                source_height_limit,
                source_press_depth,
            ) = self._parse_location_representation(source)
            (
                parsed_target,
                target_approach,
                target_rotation_from_dict,
                target_approach_height_offset,
                target_height_limit,
                target_press_depth,
            ) = self._parse_location_representation(target)
        except Exception as e:
            return ActionFailed(
                errors=[f"Failed to parse location representation: {e}"]
            )

        parsed_target.resource_id = lid_resource.resource_id

        replace_lid_result = self.pf400_interface.replace_lid(
            source=parsed_source,
            target=parsed_target,
            lid_height=resource_lid_height,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_rotation_from_dict,
            target_plate_rotation=target_rotation_from_dict,
            grab_offset=grab_height_offset,
            source_approach_height_offset=source_approach_height_offset,
            target_approach_height_offset=target_approach_height_offset,
            source_height_limit=source_height_limit,
            target_height_limit=target_height_limit,
            source_press_depth=source_press_depth,
            target_press_depth=target_press_depth,
        )
        if not replace_lid_result:
            return ActionFailed(errors=["Failed to replace lid."])

        self.resource_client.remove_resource(lid_resource.resource_id)

        if target.resource_id and target_resource.children:
            plate_resource = target_resource.children[-1]
            if plate_resource.attributes:
                plate_resource.attributes["has_lid"] = True
                self.resource_client.update_resource(plate_resource)

        return None

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
