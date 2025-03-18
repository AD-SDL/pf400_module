"""This module contains the custom exceptions for the pf400_driver package."""

from pf400_interface.pf400_constants import ERROR_CODES


class ConnectionException(Exception):
    """Exception raised for errors in the connection."""

    def __init__(self, err_message="error"):
        """Constructor for the ConnectionException class."""
        # Call the base class constructor with the parameters it needs
        super(ConnectionException, self).__init__(
            "Could not establish connection! Error type: " + err_message
        )


class CommandException(Exception):
    """Exception raised for errors in the command."""

    def __init__(self, err_message="error"):
        """Constructor for the CommandException class."""
        super(CommandException, self).__init__(
            "Invalid command! Check if communication is open. Error type: "
            + err_message
        )


class ErrorResponse(Exception):
    """Error during command execution.."""

    @staticmethod
    def from_error_code(error: str):
        """Create an ErrorResponse from an error code."""
        if error not in ERROR_CODES:
            return ErrorResponse(f"Unknown error code: {error}")
        return ErrorResponse(ERROR_CODES[error])
