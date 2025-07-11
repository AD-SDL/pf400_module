"""This module contains the custom exceptions for the pf400_driver package."""

from pf400_interface.pf400_constants import ERROR_CODES


class Pf400ConnectionError(Exception):
    """Exception raised for errors in the connection."""

    def __init__(self, err_message: str = "error") -> None:
        """Constructor for the Pf400ConnectionError class."""
        # Call the base class constructor with the parameters it needs
        super().__init__("Could not establish connection! Error type: " + err_message)


class Pf400CommandError(Exception):
    """Exception raised for errors in the command."""

    def __init__(self, err_message: str = "error") -> None:
        """Constructor for the Pf400CommandError class."""
        super().__init__(
            "Invalid command! Check if communication is open. Error type: "
            + err_message
        )


class Pf400ResponseError(Exception):
    """Error response during command execution.."""

    @staticmethod
    def from_error_code(error: str) -> "Pf400ResponseError":
        """Create an ErrorResponse from an error code."""
        if error not in ERROR_CODES:
            return Pf400ResponseError(f"Unknown error code: {error}")
        return Pf400ResponseError(ERROR_CODES[error])
