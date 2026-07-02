#! /usr/bin/env python3
"""The server for the PF400 robot that takes incoming WEI flow requests from the experiment application"""

from typing import Annotated, ClassVar, Optional

from madsci.common.types.action_types import ActionFailed
from madsci.common.types.location_types import LocationArgument
from madsci.common.types.node_types import (
    NodeRepresentationTemplateDefinition,
    RestNodeConfig,
)
from madsci.common.types.resource_types import Asset, Resource, Slot
from madsci.node_module.helpers import action
from madsci.node_module.rest_node_module import RestNode

from pf400_interface.pf400 import PF400
from pf400_interface.resource_types import PF400Plate


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

    # location templates
    location_representation_templates: ClassVar[
        list[NodeRepresentationTemplateDefinition]
    ] = [
        NodeRepresentationTemplateDefinition(
            template_name="pf400_deck_location_template",
            default_values={"gripper_config": "standard"},
            schema_def={
                "type": "object",
                "properties": {
                    "location": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 6,
                        "maxItems": 6,
                        "description": "6-digit list of joint angles associated with the PF400 location.",
                    },
                    "approach": {
                        "type": "array",
                        "description": "Array of one or more 6-digit PF400 location arrays assocoated with the PF400's safe travel path to the location.",
                    },
                    "plate_rotation": {
                        "type": "string",
                        "enum": ["wide", "narrow"],
                        "description": "Gripper orientation on the ANSI/SLAS compatible labware. wide = landscape, narrow = portrait.",
                    },
                    "approach_height_offset": {
                        "type": "number",  # or float?
                        "description": "Offset for height at which to approach location. Good for approaching taller labware safely.",
                    },
                    "height_limit": {
                        "type": "number",  # or float?
                        "description": "Height limit on approach height. Used to ensure PF400 does not hit barriers above the location.",
                    },
                    "gripper_height_offset": {
                        "type": "number",
                        "minimum": 0,
                        "description": "Vertical offset (mm, >= 0) added to the pick/place Z so the gripper stays above obstacles around the location (e.g. an OT2 slot frame). Applied at both pick and place so the plate-bottom still lands on the calibrated surface.",
                    },
                },
                "required": ["location", "plate_rotation"],
            },
            required_overrides=["location", "plate_rotation"],
            tags=["pf400", "deck"],
            version="1.1.0",
            description="PF400 deck access representation with joint values",
        ),
    ]

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
                "gripper_offset_applied": 0.0,
                "description": "PF400 robot gripper slot",
            },
        )

        self.resource_client.init_template(
            resource=gripper_slot,
            template_name="pf400_gripper",
            description="Template for PF400 robot gripper slot. Used to track what the robot is currently holding.",
            required_overrides=["resource_name"],
            tags=["pf400", "gripper", "slot"],
            created_by=self.node_info.node_id,
            version="1.0.0",
        )

        self.gripper_resource = self.resource_client.create_resource_from_template(
            template_name="pf400_gripper",
            resource_name=f"{self.node_info.node_name}.gripper",
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
            created_by=self.node_info.node_id,
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
            created_by=self.node_info.node_id,
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
            "gripper_height_offset": float  # optional pick/place clearance offset (>= 0)
        }

        Returns:
            tuple: (location_arg_with_list_repr, approach_location_arg or None,
                    plate_rotation or None, approach_height_offset or None,
                    height_limit or None, gripper_height_offset or None)
        """

        if not isinstance(location.representation, dict):
            return location, None, None, None, None, None

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
        gripper_height_offset = repr_dict.get("gripper_height_offset", None)

        if gripper_height_offset is not None and gripper_height_offset < 0:
            raise ValueError(
                f"gripper_height_offset must be >= 0, got {gripper_height_offset}. "
                "Locations are calibrated to the surface; descending lower is not supported via this field."
            )

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
            gripper_height_offset,
        )

    def _get_gripper_offset_applied(self) -> float:
        """Read the location-induced offset that was applied when the currently-held plate was picked.

        Returns 0.0 if the gripper is empty or no offset was recorded.
        """
        gripper = self.resource_client.get_resource(self.gripper_resource.resource_id)
        if not gripper.attributes:
            return 0.0
        value = gripper.attributes.get("gripper_offset_applied", 0.0)
        return float(value) if value is not None else 0.0

    def _set_gripper_offset_applied(self, value: float) -> None:
        """Persist the location-induced offset used at pick (or 0.0 to clear after place)."""
        gripper = self.resource_client.get_resource(self.gripper_resource.resource_id)
        if gripper.attributes is None:
            gripper.attributes = {}
        gripper.attributes["gripper_offset_applied"] = float(value)
        self.resource_client.update_resource(gripper)

    def _validate_lid_clearance(
        self,
        plate_resource: Resource,
        effective_grab_offset: float,
    ) -> Optional[str]:
        """Reject offsets large enough to drive the gripper fingers into the lid.

        Returns an error message if the plate has a lid and the offset exceeds the
        lid height; returns None otherwise. When ``has_lid`` is True but no
        ``lid_grip_height`` is recorded, the check is skipped with a warning since the
        precise bound can't be computed.  lid_height
        """
        if plate_resource is None:
            return None
        if not plate_resource.has_lid:
            return None
        if plate_resource.lid_grip_height is None:
            self.logger.log_warning(
                "Plate has_lid=True but no lid_grip_height recorded; skipping grab-offset/lid clearance check."
            )
            return None
        if effective_grab_offset > float(plate_resource.lid_grip_height):
            return (
                f"Effective grab offset ({effective_grab_offset:.2f} mm) exceeds lid height "
                f"({float(plate_resource.lid_grip_height):.2f} mm); gripper fingers would reach the lid instead of the plate body."
            )
        return None

    @action(
        name="transfer", description="Transfer a plate from one location to another"
    )
    def transfer(  # noqa: C901, PLR0911
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        target: Annotated[LocationArgument, "Location to place a plate to"],
        rotation_deck: Optional[LocationArgument] = None,
    ) -> Optional[ActionFailed]:
        """Transfer a plate from `source` to `target`, optionally using intermediate `approach` positions and target rotations."""

        plate_resource = None

        try:
            if source.resource_id:
                source_resource = self.resource_client.get_resource(source.resource_id)
                if source_resource.quantity == 0:
                    return ActionFailed(
                        errors=[
                            f"Plate does not exist at source location! Resource_id:{source.resource_id}."
                        ]
                    )
                if source_resource.children:
                    # Parse plate resource and attributes
                    try:
                        plate_resource = PF400Plate.from_resource(
                            source_resource.children[-1]
                        )
                    except Exception as e:
                        self.logger.log_error(
                            f"Plate resource at source does not match PF400 plate attributes requirements. \n{source_resource.children[-1]} \n{e}"
                        )
                        return ActionFailed(
                            errors=[
                                f"Plate resource at source does not match PF400 plate attributes requirements. \n{source_resource.children[-1]} \n{e}"
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
        except Exception as e:
            return ActionFailed(
                errors=[f"Resource manager error during transfer validation: {e}"]
            )
        try:
            (
                parsed_source,
                source_approach,
                source_rotation_from_dict,
                source_approach_height_offset,
                source_height_limit,
                source_gripper_height_offset,
            ) = self._parse_location_representation(source)
            (
                parsed_target,
                target_approach,
                target_rotation_from_dict,
                target_approach_height_offset,
                target_height_limit,
                target_gripper_height_offset,
            ) = self._parse_location_representation(target)
            if rotation_deck is not None:
                (
                    parsed_rotation,
                    _,
                    _,
                    _,
                    _,
                    _,
                ) = self._parse_location_representation(rotation_deck)
            else:
                parsed_rotation = None
        except Exception as e:
            return ActionFailed(
                errors=[f"Failed to parse location representation: {e}"]
            )

        location_offset = max(
            source_gripper_height_offset or 0.0,
            target_gripper_height_offset or 0.0,
        )
        effective_grab_offset = (
            plate_resource.grab_height_offset or 0.0
        ) + location_offset

        lid_error = self._validate_lid_clearance(plate_resource, effective_grab_offset)
        if lid_error:
            return ActionFailed(errors=[lid_error])

        transfer_result = self.pf400_interface.transfer(
            source=parsed_source,
            target=parsed_target,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_rotation_from_dict,
            target_plate_rotation=target_rotation_from_dict,
            rotation_deck=parsed_rotation,
            grab_offset=effective_grab_offset or None,
            source_approach_height_offset=source_approach_height_offset,
            target_approach_height_offset=target_approach_height_offset,
            source_height_limit=source_height_limit,
            target_height_limit=target_height_limit,
        )
        if not transfer_result:
            return ActionFailed(
                errors=[f"Failed to transfer plate from {source} to {target}."]
            )

        self._set_gripper_offset_applied(0.0)
        return None

    @action(name="pick_plate", description="Pick a plate from a source location")
    def pick_plate(
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
    ) -> Optional[ActionFailed]:
        """Picks a plate from `source`, optionally moving first to `source_approach`."""

        plate_resource = None
        failure: Optional[ActionFailed] = None

        try:
            # Ensure plate resource exists at source.
            if source.resource_id:
                source_resource = self.resource_client.get_resource(source.resource_id)
                if source_resource.quantity == 0:
                    failure = ActionFailed(
                        errors=[
                            f"Resource manager: Plate does not exist at source! Resource_id:{source.resource_id}."
                        ]
                    )
                elif source_resource.children:
                    # Parse plate resource and attributes
                    try:
                        plate_resource = PF400Plate.from_resource(
                            source_resource.children[-1]
                        )
                    except Exception as e:
                        error_message = (
                            "Plate resource at source does not match PF400 plate "
                            f"attributes requirements.\n"
                            f"{source_resource.children[-1]}\n{e}"
                        )
                        self.logger.log_error(error_message)
                        failure = ActionFailed(errors=[error_message])

        except Exception as e:
            failure = ActionFailed(
                errors=[f"Resource manager error during pick validation: {e}"]
            )

        if failure is not None:
            return failure

        try:
            (
                parsed_source,
                source_approach,
                source_rotation_from_dict,
                source_approach_height_offset,
                source_height_limit,
                source_gripper_height_offset,
            ) = self._parse_location_representation(source)
        except Exception as e:
            return ActionFailed(
                errors=[f"Failed to parse location representation: {e}"]
            )

        self.pf400_interface.grip_wide = (
            source_rotation_from_dict and source_rotation_from_dict.lower() == "wide"
        )

        location_offset = source_gripper_height_offset or 0.0
        effective_grab_offset = (
            plate_resource.grab_height_offset or 0.0
        ) + location_offset

        lid_error = self._validate_lid_clearance(plate_resource, effective_grab_offset)
        if lid_error:
            return ActionFailed(errors=[lid_error])

        pick_result = self.pf400_interface.pick_plate(
            source=parsed_source,
            source_approach=source_approach,
            grab_offset=effective_grab_offset or None,
            approach_height_offset=source_approach_height_offset,
            height_limit=source_height_limit,
        )
        if not pick_result:
            return ActionFailed(
                errors=[f"Failed to pick plate from location {source}."]
            )

        self._set_gripper_offset_applied(location_offset)
        return None

    @action(
        name="place_plate",
        description="Place a plate in a target location, optionally moving first to target_approach",
    )
    def place_plate(  # noqa: C901
        self,
        target: Annotated[LocationArgument, "Location to place a plate to"],
    ) -> Optional[ActionFailed]:
        """Place a plate in the `target` location, optionally moving first to `target_approach`."""

        gripper_offset_applied = 0.0
        plate_resource = None
        failure: Optional[ActionFailed] = None

        try:
            if target.resource_id:
                target_resource = self.resource_client.get_resource(target.resource_id)
                if target_resource.quantity != 0:
                    failure = ActionFailed(
                        errors=[
                            f"Resource manager: Target is occupied by another plate! Resource_id:{target.resource_id}."
                        ]
                    )

            if failure is None and self.gripper_resource.resource_id:
                gripper_resource = self.resource_client.get_resource(
                    self.gripper_resource.resource_id
                )

                if gripper_resource.attributes:
                    gripper_offset_applied = float(
                        gripper_resource.attributes.get("gripper_offset_applied", 0.0)
                        or 0.0
                    )

                if gripper_resource.quantity > 0 and gripper_resource.children:
                    plate_in_gripper = gripper_resource.children[-1]
                    try:
                        plate_resource = PF400Plate.from_resource(plate_in_gripper)
                    except Exception as e:
                        error_message = (
                            "Plate resource in gripper does not match PF400 plate "
                            f"attributes requirements.\n{plate_in_gripper}\n{e}"
                        )
                        self.logger.log_error(error_message)
                        failure = ActionFailed(errors=[error_message])

        except Exception as e:
            failure = ActionFailed(
                errors=[f"Resource manager error during place validation: {e}"]
            )

        if failure is not None:
            return failure

        try:
            (
                parsed_target,
                target_approach,
                target_rotation_from_dict,
                target_approach_height_offset,
                target_height_limit,
                target_gripper_height_offset,
            ) = self._parse_location_representation(target)
        except Exception as e:
            return ActionFailed(
                errors=[f"Failed to parse location representation: {e}"]
            )

        target_loc_offset = target_gripper_height_offset or 0.0
        if target_loc_offset > gripper_offset_applied:
            return ActionFailed(
                errors=[
                    f"Target requires {target_loc_offset:.2f} mm gripper clearance but plate "
                    f"was picked with only {gripper_offset_applied:.2f} mm offset. Re-pick the "
                    "plate with adequate offset, or reduce gripper_height_offset on the target."
                ]
            )

        self.pf400_interface.grip_wide = (
            target_rotation_from_dict and target_rotation_from_dict.lower() == "wide"
        )

        effective_grab_offset = (plate_resource.grab_height_offset or 0.0) + max(
            gripper_offset_applied, target_loc_offset
        )

        place_result = self.pf400_interface.place_plate(
            target=parsed_target,
            target_approach=target_approach,
            grab_offset=effective_grab_offset or None,
            approach_height_offset=target_approach_height_offset,
            height_limit=target_height_limit,
        )
        if not place_result:
            return ActionFailed(
                errors=["Transfer failed: plate not released properly."]
            )

        self._set_gripper_offset_applied(0.0)
        return None

    @action(
        name="move_to_location",
        description="Move to a location for testing/calibration (gripper open, no grip)",
    )
    def move_to_location(
        self,
        target: Annotated[LocationArgument, "Location to move to"],
    ) -> None:
        """Move to a location using the same approach/descend sequence as pick/place but with gripper open and no grip/release. Stays at the target for inspection. Use move_neutral to retract."""

        try:
            (
                parsed_target,
                target_approach,
                _target_rotation_from_dict,
                target_approach_height_offset,
                _target_height_limit,
                target_gripper_height_offset,
            ) = self._parse_location_representation(target)
        except Exception as e:
            return ActionFailed(
                errors=[f"Failed to parse location representation: {e}"]
            )

        self.pf400_interface.move_to_location(
            target=parsed_target,
            target_approach=target_approach or None,
            grab_offset=target_gripper_height_offset,
            approach_height_offset=target_approach_height_offset,
        )
        return None

    @action(
        name="move_neutral",
        description="Retract the arm to neutral position",
    )
    def move_neutral(
        self,
        height_offset: Optional[
            Annotated[
                float,
                "Height to retract before moving to neutral (defaults to default_approach_height)",
            ]
        ] = None,
    ) -> None:
        """Retract upward and move to neutral position. Use after move_to_location to retract the arm."""
        self.pf400_interface.move_neutral(height_offset=height_offset)

    @action(name="remove_lid", description="Remove a lid from a plate")
    def remove_lid(  # noqa: C901, PLR0911
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        target: Annotated[LocationArgument, "Location to place a plate to"],
    ) -> Optional[ActionFailed]:
        """Remove a lid from a plate located at location."""

        # TODO: Add option to ignore resource checks...

        plate_resource = None
        try:
            if source.resource_id:
                source_resource = self.resource_client.get_resource(source.resource_id)
                if source_resource.quantity == 0:
                    return ActionFailed(
                        errors=[
                            f"Resource manager: Plate does not exist at source! Resource_id:{source.resource_id}."
                        ]
                    )

                if source_resource.children:
                    try:
                        plate_resource = PF400Plate.from_resource(
                            source_resource.children[-1]
                        )
                    except Exception as e:
                        self.logger.log_error(
                            f"Plate resource at source does not match PF400 plate attributes requirements. \n{source_resource.children[-1]} \n{e}"
                        )
                        return ActionFailed(
                            errors=[
                                f"Plate resource at source does not match PF400 plate attributes requirements. \n{source_resource.children[-1]} \n{e}"
                            ]
                        )

                    if plate_resource.has_lid is False:
                        return ActionFailed(
                            errors=[
                                f"Resource manager: Plate at source does not have a lid! Resource_id:{source.resource_id}."
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

        except Exception as e:
            return ActionFailed(
                errors=[f"Resource manager error during remove lid validation: {e}"]
            )

        try:
            (
                parsed_source,
                source_approach,
                source_rotation_from_dict,
                source_approach_height_offset,
                source_height_limit,
                source_gripper_height_offset,
            ) = self._parse_location_representation(source)
            (
                parsed_target,
                target_approach,
                target_rotation_from_dict,
                target_approach_height_offset,
                target_height_limit,
                target_gripper_height_offset,
            ) = self._parse_location_representation(target)
        except Exception as e:
            return ActionFailed(
                errors=[f"Failed to parse location representation: {e}"]
            )

        # Set source resource id to the lid slot resource id
        parsed_source.resource_id = plate_resource.lid_slot_resource.resource_id

        location_offset = max(
            source_gripper_height_offset or 0.0,
            target_gripper_height_offset or 0.0,
        )
        effective_grab_offset = (
            plate_resource.grab_height_offset or 0.0
        ) + location_offset

        remove_lid_result = self.pf400_interface.remove_lid(
            source=parsed_source,
            target=parsed_target,
            lid_height=plate_resource.lid_grip_height,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_rotation_from_dict,
            target_plate_rotation=target_rotation_from_dict,
            grab_offset=effective_grab_offset or None,
            source_approach_height_offset=source_approach_height_offset,
            target_approach_height_offset=target_approach_height_offset,
            source_height_limit=source_height_limit,
            target_height_limit=target_height_limit,
        )

        if not remove_lid_result:
            return ActionFailed(errors=["Failed to remove lid."])

        self._set_gripper_offset_applied(0.0)

        return None

    @action(name="replace_lid", description="Replace a lid on a plate")
    def replace_lid(  # noqa: C901
        self,
        source: Annotated[LocationArgument, "Location to pick a plate from"],
        target: Annotated[LocationArgument, "Location to place a plate to"],
    ) -> Optional[ActionFailed]:
        """Replace a lid on the plate at the target location."""

        plate_resource = None
        lid_resource = None
        failure: Optional[ActionFailed] = None

        try:
            if source.resource_id:
                source_resource = self.resource_client.get_resource(source.resource_id)
                if source_resource.quantity == 0:
                    failure = ActionFailed(
                        errors=[
                            f"Resource manager: Lid does not exist at source! Resource_id:{source.resource_id}."
                        ]
                    )
                elif source_resource.children:
                    lid_resource = source_resource.children[-1]

                    if not lid_resource.attributes.get("lid", False):
                        failure = ActionFailed(
                            errors=[
                                "Expected source resources child to be a lid, but 'lid' attribute is missing or False."
                            ]
                        )

            if failure is None and target.resource_id:
                target_resource = self.resource_client.get_resource(target.resource_id)
                if target_resource.quantity == 0:
                    failure = ActionFailed(
                        errors=[
                            f"Resource manager: No plate on target! Resource_id:{target.resource_id}."
                        ]
                    )

                if target_resource.children:
                    try:
                        plate_resource = PF400Plate.from_resource(
                            target_resource.children[-1]
                        )
                    except Exception as e:
                        error_message = (
                            "Plate resource at target does not match PF400 plate "
                            f"attributes requirements.\n"
                            f"{target_resource.children[-1]}\n{e}"
                        )
                        self.logger.log_error(error_message)
                        failure = ActionFailed(errors=[error_message])

        except Exception as e:
            failure = ActionFailed(
                errors=[f"Resource manager error during replace lid validation: {e}"]
            )

        if failure is not None:
            return failure

        try:
            (
                parsed_source,
                source_approach,
                source_rotation_from_dict,
                source_approach_height_offset,
                source_height_limit,
                source_gripper_height_offset,
            ) = self._parse_location_representation(source)
            (
                parsed_target,
                target_approach,
                target_rotation_from_dict,
                target_approach_height_offset,
                target_height_limit,
                target_gripper_height_offset,
            ) = self._parse_location_representation(target)
        except Exception as e:
            return ActionFailed(
                errors=[f"Failed to parse location representation: {e}"]
            )

        parsed_target.resource_id = lid_resource.resource_id

        location_offset = max(
            source_gripper_height_offset or 0.0,
            target_gripper_height_offset or 0.0,
        )

        effective_grab_offset = (
            plate_resource.grab_height_offset or 0.0
        ) + location_offset

        replace_lid_result = self.pf400_interface.replace_lid(
            source=parsed_source,
            target=parsed_target,
            lid_height=plate_resource.lid_grip_height,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_rotation_from_dict,
            target_plate_rotation=target_rotation_from_dict,
            grab_offset=effective_grab_offset or None,
            source_approach_height_offset=source_approach_height_offset,
            target_approach_height_offset=target_approach_height_offset,
            source_height_limit=source_height_limit,
            target_height_limit=target_height_limit,
        )
        if not replace_lid_result:
            return ActionFailed(errors=["Failed to replace lid."])

        self._set_gripper_offset_applied(0.0)

        self.resource_client.remove_resource(lid_resource.resource_id)

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


if __name__ == "__main__":
    pf400_node = PF400Node()
    pf400_node.start_node()
