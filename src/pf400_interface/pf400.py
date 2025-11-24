#!/usr/bin/env python3
"""Driver code for the PF400 robot arm."""

import copy
import telnetlib
import time
import typing
from operator import add
from threading import Lock
from time import sleep
from typing import Optional

from madsci.client.event_client import EventClient
from madsci.client.resource_client import ResourceClient
from madsci.common.types.location_types import LocationArgument

from pf400_interface.pf400_constants import ERROR_CODES, MOTION_PROFILES, OUTPUT_CODES
from pf400_interface.pf400_errors import (
    Pf400CommandError,
    Pf400ConnectionError,
    Pf400ResponseError,
)
from pf400_interface.pf400_kinematics import KINEMATICS


class PF400(KINEMATICS):
    """Main Driver Class for the PF400 Robot Arm."""

    slow_motion_profile = 1
    fast_motion_profile = 2

    gripper_open_wide = 130
    gripper_open_narrow = 90
    gripper_close_wide = 127
    gripper_close_narrow = 85

    grip_wide = False

    safe_left_boundary = -350.0
    safe_right_boundary = 350.0

    default_approach_height = 15.0
    default_approach_vector: typing.ClassVar[list] = [
        default_approach_height,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    movement_state = 0

    robot_connection = None
    status_connection = None

    # Robot State
    power_state = "0"
    attach_state = "0"
    home_state = "0"
    system_state = "0"

    def __init__(
        self,
        host: str = "146.137.240.35",
        port: int = 10100,
        status_port: int = 10000,
        robot_id: int = 1,
        mode: int = 0,
        resource_client: ResourceClient = None,
        gripper_resource_id: Optional[str] = None,
        logger: Optional[EventClient] = None,
    ) -> None:
        """
        Description:
            - Python interface that allows remote commands to be executed using simple string messages over Telnet socket on PF400.
            - PF400 is the main object that will be used for operations such as remote connection as well as sending movement commands.
            - Programs are sent to the 10x00 port (first robot port: 10100).
            - A program sent to robot will be executed immediately unless there is a prior operation running on the robot.
            - If a second motion command is sent while the referenced robot is moving, the second command is blocked and will not reply until the first motion is complete.

        """
        super().__init__()  # PF400 kinematics
        self.logger = logger or EventClient()
        self.host = host
        self.port = port
        self.status_port = status_port
        self.mode = mode
        self.robot_id = robot_id
        self.resource_client = resource_client
        self.gripper_resource_id = gripper_resource_id
        self.command_lock = Lock()
        self.status_lock = Lock()
        # Initialize robot
        self.connect()
        self.configure_robot()
        # Plate variables

        # Initialize neutral_joints as an instance attribute
        self.neutral_joints = [
            400.0,
            1.400,
            177.101,
            537.107,
            self.gripper_close_narrow,
            0.0,
        ]

        self.set_gripper_open()
        self.set_gripper_close()
        self.logger.warn(
            "HARD CODED ROTATION LOCATION IS DEPRECATED, USE rotation_deck WITH TRANSFER METHOD INSTEAD"
        )

    def connect(self) -> None:
        """
        Description: Create a streaming socket to send string commands to the robot using telnetlib3.
        """
        try:
            self.robot_connection = telnetlib.Telnet(self.host, self.port, 5)  # noqa: S312
            self.status_connection = telnetlib.Telnet(self.host, self.status_port, 5)  # noqa: S312
        except Exception as e:
            raise Pf400ConnectionError(
                err_message=f"Failed to connect using telnetlib: {e}"
            ) from e

    def configure_robot(self) -> None:
        """Configures the robot by setting the mode and selecting the robot ID."""
        self.send_robot_command(f"mode {self.mode}")
        self.send_status_command(f"mode {self.mode}")
        self.send_robot_command(f"selectRobot {self.robot_id}")
        self.send_status_command(f"selectRobot {self.robot_id}")

    def disconnect(self) -> None:
        """Disconnects from the robot."""
        if self.robot_connection:
            self.robot_connection.close()
            self.robot_connection = None
        if self.status_connection:
            self.status_connection.close()
            self.status_connection = None

    def send_robot_command(self, command: str) -> str:
        """
        Sends a command to the robot and return the response.

        This method ensures that only one command is sent at a time by acquiring a lock.
        It checks and establishes a connection to the robot if necessary, waits for the robot
        to be in a ready state (movement_state <= 1), sends the command, and reads the response.
        Handles error and output codes appropriately.

        Args:
            command (str): The command string to send to the robot.

        Returns:
            str: The response received from the robot.


                    Raises:
            Pf400ConnectionError: If no connection to the robot can be established.
            Pf400CommandError: If an AttributeError occurs during command execution.
        """
        with self.command_lock:
            try:
                if not self.robot_connection:
                    self.connect()
                self.robot_connection.write((command + "\n").encode("ascii"))
                response = (
                    self.robot_connection.read_until(b"\r\n")
                    .decode("ascii")
                    .rstrip("\r\n")
                )
                if response != "" and response in ERROR_CODES:
                    self.handle_error_output(response)
                if response in OUTPUT_CODES:
                    self.logger.log_debug(response)
                self.await_movement_completion()
                return response
            except AttributeError as e:
                raise Pf400CommandError(err_message="Attribute Error") from e

    def send_status_command(self, command: str) -> str:
        """
        Sends a status command to the PF400 device and returns the response.

        This method ensures thread-safe access using a lock, establishes a connection if needed,
        writes the command to the status writer, and reads the response. It handles error and output
        codes appropriately, logging or raising exceptions as necessary.

        Args:
            command (str): The command string to send to the PF400 device.

        Returns:
            str: The response received from the PF400 device.

        Raises:
            Pf400ConnectionError: If no connection is established and the command cannot be sent.
            Pf400CommandError: If an AttributeError occurs during command processing.
        """
        with self.status_lock:
            try:
                if not self.status_connection:
                    self.connect()
                self.status_connection.write((command + "\n").encode("ascii"))
                response = (
                    self.status_connection.read_until(b"\r\n")
                    .decode("ascii")
                    .rstrip("\r\n")
                )
                if response != "" and response in ERROR_CODES:
                    self.handle_error_output(response)
                if response in OUTPUT_CODES:
                    self.logger.log_debug(response)
                return response
            except AttributeError as e:
                raise Pf400CommandError(err_message="Attribute Error") from e

    def handle_error_output(self, output: str) -> None:
        """
        Description: Handles the error message output
        """
        response = Pf400ResponseError.from_error_code(output)
        self.logger.log_error(response)
        raise response

    def enable_power(self) -> str:
        """
        Description: Enables the power on the robot
        """
        return self.send_robot_command("hp 1 -1")

    def disable_power(self) -> str:
        """
        Description: Disables the power on the robot
        """
        return self.send_robot_command("hp 0")

    def split_response(self, response: str) -> list[str]:
        """
        Description: Splits the response string into a list of strings.
        Parameters:
            - response: The response string to be split.
        Returns: A list of strings.
        """
        return response.split(" ") if response else []

    def check_powered(self) -> bool:
        """
        Description: Checks whether the robot power is on or off.
        Returns: bool indicating whether the robot is powered on.
        """
        self.power_state = self.split_response(self.send_status_command("hp"))[1]
        return self.power_state == "1"

    def check_attached(self) -> bool:
        """
        Description: Checks whether the robot is attached or not.
        Returns: bool indicating whether the robot is attached.
        """
        self.attach_state = self.split_response(self.send_robot_command("attach"))[1]
        return self.attach_state == "1"

    def check_homed(self) -> bool:
        """
        Description: Checks whether the robot is homed or not.
        Returns: bool indicating whether the robot is homed.
        """
        self.home_state = self.split_response(self.send_status_command("pd 2800"))[1]
        return self.home_state == "1"

    def check_system_state(self) -> str:
        """
        Description: Checks the global system state code
        Returns: The system state code as a string.
        """
        self.system_state = self.send_robot_command("sysState")
        return self.system_state

    def attach_robot(self) -> str:
        """
        Description: Attach to the robot to enable motion commands.
        """
        return self.send_robot_command("attach 1")

    def detach_robot(self) -> str:
        """
        Description: Detach from the robot to disable motion commands.
        """
        return self.send_robot_command("attach 0")

    def home_robot(self) -> str:
        """
        Description: Homes robot joints. Homing takes around 15 seconds.
        """

        return self.send_robot_command("home")

    def initialize_robot(self) -> None:
        """
        Description: Initializes the robot by calling enable_power, attach_robot, home_robot, set_profile functions and checks the robot state to find out if the initialization was successful
        """

        self.check_state()
        retry_count = 0
        while self.power_state != "1" and retry_count < 5:
            self.enable_power()
            self.check_powered()
            retry_count += 1
        if retry_count == 5:
            raise Exception("Failed to power on the robot after 5 attempts.")
        retry_count = 0
        while self.attach_state != "1" and retry_count < 5:
            self.attach_robot()
            self.check_attached()
            retry_count += 1
        if retry_count == 5:
            raise Exception("Failed to attach the robot after 5 attempts.")
        retry_count = 0
        while self.home_state != "1" and retry_count < 5:
            self.home_state = self.home_robot()
            self.check_homed()
            retry_count += 1
        if retry_count == 5:
            raise Exception("Failed to home the robot after 5 attempts.")
        self.set_profile()
        self.get_robot_movement_state()

    def get_robot_movement_state(self) -> int:
        """Checks the movement state of the robot
        States: 0 = Power off
                1 = Stopped
                2 = Acceleration
                3 = Deceleration
        """
        movement_state = self.send_status_command("state")
        self.movement_state = int(float(movement_state.split(" ")[1]))
        return self.movement_state

    def await_movement_completion(self) -> None:
        """Waits until the robot has finished moving"""
        while True:
            if self.get_robot_movement_state() <= 1:
                return
            time.sleep(0.1)

    def check_state(self) -> int:
        """
        Description: Checks the various state values of the robot and returns False if any of the states are not initialized correctly.
        """

        try:
            is_powered = self.check_powered()
            is_attached = self.check_attached()
            is_homed = self.check_homed()
            system_state = self.check_system_state()
            system_state_ok = self.split_response(system_state)[1] == "21"
            return is_powered and is_attached and is_homed and system_state_ok
        except Exception as e:
            self.logger.log_info(f"Exception during state check: {e}")
            return False

    def get_joint_states(self) -> list[float]:
        """
        Description: Locates the robot and returns the joint locations for all 6 joints.
        """
        states = self.send_robot_command("wherej")
        joints = states.split(" ")
        joints = joints[1:]
        return [float(x) for x in joints]

    def get_cartesian_coordinates(self) -> list[float]:
        """
        Description: This function finds the current cartesian coordinates and angles of the robot.
                Return: A float array with x/y/z yaw/pitch/roll
        """
        coordinates = self.send_robot_command("whereC")
        coordinates_list = coordinates.split(" ")
        coordinates_list = coordinates_list[1:-1]
        return [float(x) for x in coordinates_list]

    def get_gripper_length(self) -> float:
        """Returns the current length of the gripper."""
        joint_angles = self.get_joint_states()
        return joint_angles[4]

    def set_profile(self, profile_dict: dict = {"0": 0}) -> str:
        """
        Description: Sets and saves the motion profiles (defined in robot data) to the robot.
                                If user defines a custom profile, this profile will saved onto motion profile 3 on the robot
        Parameters:
                        - profile_dict: Custom motion profile
        """
        if len(profile_dict) == 1:
            profile1 = "Profile 1"
            for value in MOTION_PROFILES[0].values():
                profile1 += " " + str(value)
            profile2 = "Profile 2"
            for value in MOTION_PROFILES[1].values():
                profile2 += " " + str(value)
            out_msg = self.send_robot_command(profile1)
            self.send_robot_command(profile2)
        elif len(profile_dict) == 8:
            profile3 = "Profile 3"
            for value in profile_dict.values():
                profile3 += " " + str(value)
            out_msg = self.send_robot_command(profile3)
        else:
            raise Exception(
                f"Motion profile takes 8 arguments, {len(profile_dict)} where given"
            )
        return out_msg

    @property
    def gripper_open(self) -> int:
        """Returns the current gripper open length based on the grip_wide setting."""
        return self.gripper_open_wide if self.grip_wide else self.gripper_open_narrow

    @property
    def gripper_close(self) -> int:
        """Returns the current gripper close length based on the grip_wide setting."""
        return self.gripper_close_wide if self.grip_wide else self.gripper_close_narrow

    def set_gripper_open(self, gripper_length: Optional[int] = None) -> None:
        """Configure the definition of gripper open."""
        self.send_robot_command(f"GripOpenPos {gripper_length or self.gripper_open}")

    def set_gripper_close(self, gripper_length: Optional[int] = None) -> None:
        """Configure the definition of gripper close."""
        self.send_robot_command(f"GripClosePos {gripper_length or self.gripper_close}")

    def grab_plate(
        self, width: Optional[int] = None, speed: int = 100, force: int = 10
    ) -> bool:
        """
        Description:
                Grabs the plate by applying additional force
        Parameters:
            - width: Plate width, in mm. Should be accurate to within about 1 mm.
            - speed: Percent speed to open fingers.  1 to 100.
            - Force: Maximum gripper squeeze force, in Nt.
                A positive value indicates the fingers must close to grasp.
                A negative value indicates the fingers must open to grasp.
        Returns:
            True if the plate was successfully grabbed, False otherwise.
        """
        if width is None:
            width = self.gripper_close
        grab_plate_status = self.send_robot_command(
            f"GraspPlate {width} {speed} {force}"
        ).split(" ")

        if grab_plate_status[1] == "0":
            return False
        if grab_plate_status[1] == "-1":
            return True
        self.logger.log_error(
            f"Unexpected response from GraspPlate: {grab_plate_status[1]}"
        )
        raise Pf400ResponseError(
            f"Unexpected response from GraspPlate command: {grab_plate_status[1]}."
        )

    def release_plate(self, width: int = 130, speed: int = 100) -> str:
        """
        Description:
                Release the plate
        Parameters:
                - width: Open width, in mm. Larger than the widest corners of the plates.
                - speed: Percent speed to open fingers.  1 to 100.
        Returns:
            A string response from the robot indicating the result of the release command.
        """
        if width is None:
            width = self.gripper_open

        return self.send_robot_command("ReleasePlate " + str(width) + " " + str(speed))

    def open_gripper(self, gripper_length: Optional[int] = None) -> float:
        """Opens the gripper"""
        self.set_gripper_open(gripper_length=gripper_length)
        self.send_robot_command("gripper 1")
        return self.get_gripper_length()

    def close_gripper(self, gripper_length: Optional[int] = None) -> float:
        """Closes the gripper"""
        self.set_gripper_close(gripper_length=gripper_length)
        self.send_robot_command("gripper 2")
        return self.get_gripper_length()

    def set_plate_rotation(
        self, joint_states: list[float], rotation_degree: float = 0
    ) -> list[float]:
        """
        Description:
        Parameters:
                - joint_states:
                - rotation_degree:
        Note: If the rotation requires changing the "Quadrant" on the coordinate plane,
                        inverse kinematics calculation will be calculated wrong!
        """
        cartesian_coordinates, phi_angle, rail_pos = self.forward_kinematics(
            joint_states
        )
        # Fixing the orientation offset here
        if rotation_degree == -90:  # Yaw 90 to 0 degrees:
            cartesian_coordinates[1] += 4
            cartesian_coordinates[0] += 29
        elif rotation_degree == 90:
            cartesian_coordinates[1] -= 4
            cartesian_coordinates[0] -= 29

        if cartesian_coordinates[1] < 0:
            # Location is on the right side of the robot
            cartesian_coordinates[3] += rotation_degree
        elif cartesian_coordinates[1] > 0 and joint_states[1]:
            cartesian_coordinates[3] -= rotation_degree

        return self.inverse_kinematics(cartesian_coordinates, phi_angle, rail_pos)

    def check_incorrect_plate_orientation(
        self, goal_location: list[float], goal_rotation: list[float]
    ) -> list[float]:
        """
        Description: Fixes plate rotation on the goal location if it was recorded with an incorrect orientation.
        Parameters:
            - goal_location
            - goal_rotation
        Return:
            goal_location:
                - New goal location if the incorrect orientation was found.
                - Same goal location if there orientation was correct.
        """
        # This will fix plate rotation on the goal location if it was recorded with an incorrect orientation
        cartesian_goal, _phi_source, _rail_source = self.forward_kinematics(
            goal_location
        )
        # Checking yaw angle
        if goal_rotation != 0 and cartesian_goal[3] > -10 and cartesian_goal[3] < 10:
            goal_location = self.set_plate_rotation(goal_location, -goal_rotation)

        return goal_location

    def move_joint(
        self,
        target_joint_angles: list[float],
        profile: int = 1,
        gripper_close: bool = False,
        gripper_open: bool = False,
    ) -> str:
        """
        Description: Creates the movement commands with the given robot_location, profile, gripper closed and gripper open info
        Parameters:
                        - target: Which location the PF400 will move.
                        - profile: Motion profile ID.
                        - gripper_close: If set to TRUE, gripper is closed. If set to FALSE, gripper position will remain same as the previous location.
                        - gripper_open: If set to TRUE, gripper is opened. If set to FALSE, gripper position will remain same as the previous location.
        Return: Returns the created movement command in string format
        """

        # Checking unpermitted gripper command
        # add check gripper here and remove gripper open/close from state
        if gripper_close and gripper_open:
            raise Exception("Gripper cannot be open and closed at the same time!")

        # Setting the gripper location to open or close. If there is no gripper position passed in, target_joint_angles will be used.
        if gripper_close:
            target_joint_angles[4] = self.gripper_close
        elif gripper_open:
            target_joint_angles[4] = self.gripper_open
        else:
            target_joint_angles[4] = self.get_gripper_length()

        move_command = (
            "movej" + " " + str(profile) + " " + " ".join(map(str, target_joint_angles))
        )

        return self.send_robot_command(move_command)

    def move_cartesian(
        self, target_cartesian_coordinates: list[float], profile: int = 2
    ) -> str:
        """Move the arm to a target location in cartesian coordinates."""
        move_command = (
            "MoveC"
            + " "
            + str(profile)
            + " "
            + " ".join(map(str, target_cartesian_coordinates))
        )

        return self.send_robot_command(move_command)

    def move_in_one_axis(
        self, profile: int = 1, axis_x: int = 0, axis_y: int = 0, axis_z: int = 0
    ) -> str:
        """
        Description: Moves the end effector on single axis with a goal movement in millimeters.
        Parameters:
                - axis_x : Goal movement on x axis in mm
                - axis_y : Goal movement on y axis in mm
                - axis_z : Goal movement on z axis in mm
        Returns: A string response from the robot indicating the result of the move command.
        """

        # Find the cartesian coordinates of the target joint states
        cartesian_coordinates = self.get_cartesian_coordinates()

        # Move end effector on the single axis
        cartesian_coordinates[0] += axis_x
        cartesian_coordinates[1] += axis_y
        cartesian_coordinates[2] += axis_z

        move_command = (
            "MoveC"
            + " "
            + str(profile)
            + " "
            + " ".join(map(str, cartesian_coordinates))
        )
        return self.send_robot_command(move_command)

    def move_gripper_safe_zone(self) -> None:
        """
        Description: Check if end effector is outside the safe boundaries. If it is, move it on the y axis first to prevent collisions with the module frames.
        """

        current_cartesian_coordinates = self.get_cartesian_coordinates()

        if current_cartesian_coordinates[1] <= self.safe_left_boundary:
            y_distance = self.safe_left_boundary - current_cartesian_coordinates[1]
            self.move_in_one_axis(profile=self.slow_motion_profile, axis_y=y_distance)
        elif current_cartesian_coordinates[1] >= self.safe_right_boundary:
            y_distance = self.safe_right_boundary - current_cartesian_coordinates[1]
            self.move_in_one_axis(profile=self.slow_motion_profile, axis_y=y_distance)

    def move_gripper_neutral(self) -> None:
        """
        Description: Move end effector to neutral position
        """

        self.move_gripper_safe_zone()
        gripper_neutral = self.get_joint_states()
        gripper_neutral[3] = self.neutral_joints[3]

        self.move_joint(gripper_neutral, self.slow_motion_profile)

    def move_arm_neutral(self) -> None:
        """
        Description: Move arm to neutral position
        """
        arm_neutral = self.neutral_joints
        current_location = self.get_joint_states()
        arm_neutral[0] = current_location[0]
        arm_neutral[5] = current_location[5]

        self.move_joint(arm_neutral, self.slow_motion_profile)

    def move_rails_neutral(
        self, v_rail: Optional[float] = None, h_rail: Optional[float] = None
    ) -> None:
        """Setting the target location's linear rail position for pf400_neutral"""

        current_location = self.get_joint_states()

        if not v_rail:
            v_rail = current_location[0]  # Keep the vertical rail same
        if not h_rail:
            h_rail = current_location[5]  # Keep the horizontal rail same

        self.neutral_joints[5] = h_rail
        self.move_joint(self.neutral_joints, self.fast_motion_profile)
        self.neutral_joints[0] = v_rail + self.default_approach_height
        self.move_joint(self.neutral_joints, self.slow_motion_profile)

    def move_all_joints_neutral(self, target: Optional[list[float]] = None) -> None:
        """
        Description: Move all joints to neutral position
        """
        if target is None:
            target = self.get_joint_states()
        # First move end effector to it's nuetral position
        self.move_gripper_neutral()
        # Setting an arm neutral position without moving the horizontal & vertical rails
        self.move_arm_neutral()
        # Setting the target location's linear rail position for pf400_neutral
        self.move_rails_neutral(target[0], target[5])

    def remove_lid(
        self,
        source: LocationArgument,
        target: LocationArgument,
        lid_height: float = 7.0,
        source_approach: LocationArgument = None,
        target_approach: LocationArgument = None,
        source_plate_rotation: str = "",
        target_plate_rotation: str = "",
    ) -> None:
        """Remove the lid from the plate"""
        source.representation = copy.deepcopy(source.representation)
        source.representation[0] += lid_height

        self.transfer(
            source=source,
            target=target,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )

    def replace_lid(
        self,
        source: LocationArgument,
        target: LocationArgument,
        lid_height: float = 7.0,
        source_approach: LocationArgument = None,
        target_approach: LocationArgument = None,
        source_plate_rotation: str = "",
        target_plate_rotation: str = "",
    ) -> None:
        """Replace the lid on the plate"""
        target.representation = copy.deepcopy(target.representation)
        target.representation[0] += lid_height

        self.transfer(
            source=source,
            target=target,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )

    def rotate_plate_on_deck(
        self, rotation_degree: int, rotation_deck: Optional[LocationArgument] = None
    ) -> None:
        """
        Description: Uses the rotation deck to rotate the plate between two transfers
        Parameters: - rotation_degree: Rotation degree.
        """
        if not rotation_deck:
            raise ValueError("Rotation deck location must be provided.")
        target = rotation_deck.location

        # Fixing the offset on the z axis
        if rotation_degree == -90:
            target = self.set_plate_rotation(target, -rotation_degree)

        above_position = list(map(add, target, self.default_approach_vector))

        self.move_all_joints_neutral(target)
        self.move_joint(above_position, self.slow_motion_profile)
        self.move_joint(target, self.slow_motion_profile)
        self.release_plate()

        try:
            if self.resource_client:
                popped_plate, _updated_resource = self.resource_client.pop(
                    resource=self.gripper_resource_id
                )
                self.resource_client.push(
                    resource=rotation_deck.resource_id, child=popped_plate
                )
        except Exception as e:
            self.logger.log_error(f"Error during plate rotation: {e}")
            raise e

        self.move_in_one_axis(
            profile=self.slow_motion_profile, axis_z=self.default_approach_height
        )
        self.open_gripper(self.gripper_open_wide)

        # Rotating gripper to grab the plate from other rotation
        target = self.set_plate_rotation(target, rotation_degree)
        above_position = list(map(add, target, self.default_approach_vector))
        self.move_joint(
            target_joint_angles=above_position, profile=self.slow_motion_profile
        )
        self.move_joint(
            target_joint_angles=target,
            profile=self.slow_motion_profile,
            gripper_open=True,
        )
        self.grab_plate(speed=100, force=10)

        try:
            if self.resource_client:
                popped_plate, _updated_resource = self.resource_client.pop(
                    resource=rotation_deck.resource_id
                )
                self.resource_client.push(
                    resource=self.gripper_resource_id, child=popped_plate
                )
        except Exception as e:
            self.logger.log_error(f"Error during plate rotation: {e}")
            raise e

        self.move_in_one_axis(
            profile=self.slow_motion_profile, axis_z=self.default_approach_height
        )
        self.move_all_joints_neutral(target)

    def pick_plate(
        self,
        source: LocationArgument,
        source_approach: LocationArgument = None,
        grip_width: Optional[int] = None,
    ) -> bool:
        """
        Pick a plate from the source location, optionally using an approach location.

        Returns True if the plate was successfully grabbed, False otherwise.
        """

        above_position = list(
            map(add, source.representation, self.default_approach_vector)
        )
        self.open_gripper()
        if source_approach:
            if isinstance(source_approach.location[0], list):
                # Multiple approach locations provided
                self.move_all_joints_neutral(source_approach.location[0])
                for location in source_approach.location:
                    self.move_joint(
                        target_joint_angles=location,
                        profile=self.fast_motion_profile,
                    )
            else:
                # Single approach location provided
                self.move_all_joints_neutral(source_approach.location)
                self.move_joint(
                    target_joint_angles=source_approach.location,
                    profile=self.fast_motion_profile,
                )
        else:
            self.move_all_joints_neutral(source.representation)

        self.move_joint(
            target_joint_angles=above_position, profile=self.fast_motion_profile
        )
        self.move_joint(
            target_joint_angles=source.representation,
            profile=self.fast_motion_profile,
            gripper_open=True,
        )
        grab_succeeded = self.grab_plate(width=grip_width, speed=100, force=10)

        if self.resource_client and grab_succeeded and source.resource_id:
            popped_plate, _updated_resource = self.resource_client.pop(
                resource=source.resource_id
            )
            self.resource_client.push(
                resource=self.gripper_resource_id, child=popped_plate
            )

        self.move_in_one_axis(
            profile=self.slow_motion_profile, axis_z=self.default_approach_height
        )

        if source_approach:
            if isinstance(source_approach.location[0], list):
                for location in reversed(source_approach.location):
                    self.move_joint(
                        target_joint_angles=location,
                        profile=self.fast_motion_profile,
                    )
                self.move_all_joints_neutral(location)

            else:
                self.move_joint(
                    target_joint_angles=source_approach.location,
                    profile=self.fast_motion_profile,
                )
                self.move_all_joints_neutral(source_approach.location)
        else:
            self.move_all_joints_neutral(source.representation)
        return grab_succeeded

    def place_plate(
        self,
        target: LocationArgument,
        target_approach: LocationArgument = None,
        open_width: Optional[int] = None,
    ) -> None:
        """
        Place a plate in the target location
        """
        above_position = list(
            map(add, target.representation, self.default_approach_vector)
        )
        if target_approach:
            if isinstance(target_approach.location[0], list):
                # Multiple approach locations provided
                self.move_all_joints_neutral(target_approach.location[0])
                for location in target_approach.location:
                    self.move_joint(
                        target_joint_angles=location,
                        profile=self.fast_motion_profile,
                    )
            else:
                # Single approach location provided
                self.move_all_joints_neutral(target_approach.location)
                self.move_joint(
                    target_joint_angles=target_approach.location,
                    profile=self.fast_motion_profile,
                )
        else:
            self.move_all_joints_neutral(target.representation)

        self.move_joint(above_position, self.slow_motion_profile)
        self.move_joint(target.representation, self.slow_motion_profile)
        self.release_plate(width=open_width)
        if (
            self.resource_client
            and len(
                self.resource_client.get_resource(self.gripper_resource_id).children
            )
            > 0
        ):
            popped_plate, _updated_resource = self.resource_client.pop(
                resource=self.gripper_resource_id
            )
            if target.resource_id:
                self.resource_client.push(
                    resource=target.resource_id, child=popped_plate
                )

        self.move_in_one_axis(
            profile=self.slow_motion_profile, axis_z=self.default_approach_height
        )
        if target_approach:
            if isinstance(target_approach.location[0], list):
                for location in reversed(target_approach.location):
                    self.move_joint(
                        target_joint_angles=location,
                        profile=self.fast_motion_profile,
                    )
                self.move_all_joints_neutral(location)

            else:
                self.move_joint(
                    target_joint_angles=target_approach.location,
                    profile=self.fast_motion_profile,
                )
                self.move_all_joints_neutral(target_approach.location)

        else:
            self.move_all_joints_neutral(target.representation)

    def transfer(
        self,
        source: LocationArgument,
        target: LocationArgument,
        source_approach: LocationArgument = None,
        target_approach: LocationArgument = None,
        source_plate_rotation: str = "",
        target_plate_rotation: str = "",
        rotation_deck: Optional[LocationArgument] = None,
    ) -> None:
        """
        Description: Plate transfer function that performs series of movements to pick and place the plates
                Parameters:
                        - source: Source location
                        - target: Target location
                        - source_plate_rotation: narrow or wide
                        - target_plate_rotation: narrow or wide
                        - rotation_deck: Location for plate rotation deck
                Note: Plate rotation defines the rotation of the plate on the deck, not the grabbing angle.
        """
        source = copy.deepcopy(source)
        target = copy.deepcopy(target)
        for rotation_arg in [source_plate_rotation, target_plate_rotation]:
            if rotation_arg.lower() not in ["wide", "narrow", ""]:
                raise ValueError(
                    f"Invalid plate rotation argument: {rotation_arg}. "
                    "Expected 'wide', 'narrow', or ''."
                )

        # set plate width for source
        if source_plate_rotation.lower() == "wide":
            plate_source_rotation = 90
            self.grip_wide = True

        elif source_plate_rotation.lower() == "narrow" or source_plate_rotation == "":
            plate_source_rotation = 0
            self.grip_wide = False

        source.representation = self.check_incorrect_plate_orientation(
            source.representation, plate_source_rotation
        )

        pick_result = self.pick_plate(source=source, source_approach=source_approach)

        if not pick_result:
            self.move_all_joints_neutral()
            sleep(5)
            raise Exception("Transfer failed: no plate detected after picking.")

        # set plate width for target
        if target_plate_rotation.lower() == "wide":
            plate_target_rotation = 90
            self.grip_wide = True

        elif target_plate_rotation.lower() == "narrow" or target_plate_rotation == "":
            plate_target_rotation = 0
            self.grip_wide = False

        target.representation = self.check_incorrect_plate_orientation(
            target.representation, plate_target_rotation
        )

        if plate_source_rotation == 90 and plate_target_rotation == 0:
            # Need a transition from 90 degree to 0 degree
            self.rotate_plate_on_deck(
                rotation_degree=-plate_source_rotation, rotation_deck=rotation_deck
            )

        elif plate_source_rotation == 0 and plate_target_rotation == 90:
            # Need a transition from 0 degree to 90 degree
            self.rotate_plate_on_deck(
                rotation_degree=plate_target_rotation, rotation_deck=rotation_deck
            )

        self.place_plate(target=target, target_approach=target_approach)
