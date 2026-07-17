"""This module contains the Pydantic models for the PF400 resource types"""

from typing import Self

from madsci.common.types.resource_types import Resource
from pydantic import BaseModel


class PF400Plate(BaseModel):
    """PF400-specific plate properties."""

    grab_height_offset: float
    lid_only_grip_height: float | None = None
    lid_removal_grip_height: float | None = None
    has_lid: bool
    lid_resource: Resource | None = None
    lid_slot_resource: Resource | None = None

    @classmethod
    def from_resource(cls, resource: Resource) -> Self:
        """Extract the PF400-specific properties from a plate resource."""

        attrs = resource.attributes

        lid_resource = None
        lid_slot = resource.children.get("lid_slot")

        if lid_slot is not None and lid_slot.children:
            if len(lid_slot.children) > 1:
                raise ValueError("lid_slot contains more than one child.")

            lid_resource = lid_slot.children[0]
            if not lid_resource.attributes.get("lid", False):
                raise ValueError("Child in lid_slot is not marked as a lid.")

        return cls(
            grab_height_offset=attrs["pf400_grip_height"],
            lid_only_grip_height=attrs.get("pf400_lid_only_grip_height"),
            lid_removal_grip_height=attrs.get("pf400_lid_removal_grip_height"),
            has_lid=lid_resource is not None,
            lid_resource=lid_resource,
            lid_slot_resource=lid_slot,
        )
