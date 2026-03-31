"""Helpers for resource management in the PF400 REST node."""

from typing import Optional

from madsci.common.types.action_types import ActionFailed
from madsci.common.types.resource_types import Resource
from pydantic import BaseModel


class LidResult(BaseModel):
    """Pydantic model for defining the response from pf400 REST node _get_lid_from_slot function."""

    lid: Optional[Resource] = None
    conforms: bool
    error: Optional[ActionFailed] = None


class LidSlotResult(BaseModel):
    """Pydantic model for defining the response from pf400 REST node _get_lid_slot_from_target function."""

    lid_slot: Optional[Resource] = None
    target_resource: Optional[Resource] = None
    conforms: bool
    error: Optional[ActionFailed] = None
