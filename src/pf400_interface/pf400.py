#!/usr/bin/env python3
"""Interface code for the PF400 robot arm."""

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


class PF400:
    """Main Interface Class for the PF400 Robot Arm."""

    slow_motion_profile = 1
    fast_motion_profile = 2
    straight_motion_profile = 3

    gripper_open_wide = 130
    gripper_open_narrow = 90
    gripper_close_wide = 127
    gripper_close_narrow = 85

    grip_wide = False

    safe_left_boundary = -350.0
    safe_right_boundary = 350.0

    default_bias_torque_pct: int = 50
    gripper_clearance_height = 110.0
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
        self.connect()
        self._configure_robot()

        self.neutral_joints = [
            400.0,
            1.650,
            177.662,
            -179.494,
            self.gripper_close_narrow,
            0.0,
        ]

        self.set_gripper_open()
        self.set_gripper_close()

    def connect(self) -> None:
        """Create a streaming socket to send string commands to the robot using telnetlib."""
        try:
            self.robot_connection = telnetlib.Telnet(self.host, self.port, 5)  # noqa: S312
            self.status_connection = telnetlib.Telnet(self.host, self.status_port, 5)  # noqa: S312
        except Exception as e:
            raise Pf400ConnectionError(
                err_message=f"Failed to connect using telnetlib: {e}"
            ) from e

    def _configure_robot(self) -> None:
        """Configures the robot by setting the mode and selecting the robot ID."""
        self.send_command(f"mode {self.mode}")
        self.send_status_command(f"mode {self.mode}")
        self.send_command(f"selectRobot {self.robot_id}")
        self.send_status_command(f"selectRobot {self.robot_id}")

    def disconnect(self) -> None:
        """Disconnects from the robot."""
        if self.robot_connection:
            self.robot_connection.close()
            self.robot_connection = None
        if self.status_connection:
            self.status_connection.close()
            self.status_connection = None

    def send_command(self, command: str) -> str:
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
                    self._handle_error_output(response)
                if response in OUTPUT_CODES:
                    self.logger.log_debug(response)
                self._await_movement_completion()
                return response
            except AttributeError as e:
                raise Pf400CommandError(err_message="Attribute Error") from e

    def send_status_command(self, command: str) -> str:
        """
        Sends a status command to the PF400 device and returns the response.

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
                    self._handle_error_output(response)
                if response in OUTPUT_CODES:
                    self.logger.log_debug(response)
                return response
            except AttributeError as e:
                raise Pf400CommandError(err_message="Attribute Error") from e

    def _parse_response(self, response: str) -> list[float]:
        """Parse a TCS response string into a list of floats, stripping the leading status code."""
        parts = response.split(" ")
        return [float(x) for x in parts[1:]]

    def _handle_error_output(self, output: str) -> None:
        """Handles the error message output."""
        response = Pf400ResponseError.from_error_code(output)
        self.logger.log_error(response)
        raise response

    def enable_power(self) -> str:
        """Enables the power on the robot."""
        return self.send_command("hp 1 -1")

    def disable_power(self) -> str:
        """Disables the power on the robot."""
        return self.send_command("hp 0")

    def _split_response(self, response: str) -> list[str]:
        """Splits the response string into a list of strings."""
        return response.split(" ") if response else []

    def check_powered(self) -> bool:
        """Checks whether the robot power is on or off."""
        self.power_state = self._split_response(self.send_status_command("hp"))[1]
        return self.power_state == "1"

    def check_attached(self) -> bool:
        """Checks whether the robot is attached or not."""
        self.attach_state = self._split_response(self.send_command("attach"))[1]
        return self.attach_state == "1"

    def check_homed(self) -> bool:
        """Checks whether the robot is homed or not."""
        self.home_state = self._split_response(self.send_status_command("pd 2800"))[1]
        return self.home_state == "1"

    def check_system_state(self) -> str:
        """Checks the global system state code."""
        self.system_state = self.send_command("sysState")
        return self.system_state

    def attach_robot(self) -> str:
        """Attach to the robot to enable motion commands."""
        return self.send_command("attach 1")

    def detach_robot(self) -> str:
        """Detach from the robot to disable motion commands."""
        return self.send_command("attach 0")

    def home_robot(self) -> str:
        """Homes robot joints. Homing takes around 15 seconds."""
        return self.send_command("home")

    def initialize_robot(self) -> None:
        """Initializes the robot by calling enable_power, attach_robot, home_robot, set_profile functions."""
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
        """Checks the movement state of the robot.

        States: 0 = Power off, 1 = Stopped, 2 = Acceleration, 3 = Deceleration
        """
        movement_state = self.send_status_command("state")
        self.movement_state = int(float(movement_state.split(" ")[1]))
        return self.movement_state

    def _await_movement_completion(self) -> None:
        """Waits until the robot has finished moving."""
        while True:
            if self.get_robot_movement_state() <= 1:
                return
            time.sleep(0.1)

    def check_state(self) -> int:
        """Checks the various state values of the robot."""
        try:
            is_powered = self.check_powered()
            is_attached = self.check_attached()
            is_homed = self.check_homed()
            system_state = self.check_system_state()
            system_state_ok = self._split_response(system_state)[1] == "21"
            return is_powered and is_attached and is_homed and system_state_ok
        except Exception as e:
            self.logger.log_info(f"Exception during state check: {e}")
            return False

    def get_joint_states(self) -> list[float]:
        """Locates the robot and returns the joint locations for all 6 joints."""
        states = self.send_command("wherej")
        joints = states.split(" ")
        joints = joints[1:]
        return [float(x) for x in joints]

    def get_cartesian_coordinates(self) -> list[float]:
        """Returns the current Cartesian coordinates of the robot as [X, Y, Z, yaw, pitch, roll]."""
        coordinates = self.send_command("whereC")
        coordinates_list = coordinates.split(" ")
        coordinates_list = coordinates_list[1:-1]
        return [float(x) for x in coordinates_list]

    def get_gripper_state(self) -> float:
        """Returns the current position of the gripper."""
        joint_angles = self.get_joint_states()
        return joint_angles[4]

    def set_profile(self, profile_dict: Optional[dict] = None) -> str:
        """Sets and saves the motion profiles to the robot."""
        if profile_dict is None:
            profile1 = "Profile 1"
            for value in MOTION_PROFILES[0].values():
                profile1 += " " + str(value)
            profile2 = "Profile 2"
            for value in MOTION_PROFILES[1].values():
                profile2 += " " + str(value)
            profile3 = "Profile 3"
            for value in MOTION_PROFILES[2].values():
                profile3 += " " + str(value)
            self.send_command(profile1)
            self.send_command(profile2)
            out_msg = self.send_command(profile3)
        elif len(profile_dict) == 8:
            profile4 = "Profile 4"
            for value in profile_dict.values():
                profile4 += " " + str(value)
            out_msg = self.send_command(profile4)
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
        self.send_command(f"GripOpenPos {gripper_length or self.gripper_open}")

    def set_gripper_close(self, gripper_length: Optional[int] = None) -> None:
        """Configure the definition of gripper close."""
        self.send_command(f"GripClosePos {gripper_length or self.gripper_close}")

    def grab_plate(
        self, width: Optional[int] = None, speed: int = 100, force: int = 10
    ) -> bool:
        """Grabs the plate by applying additional force."""
        if width is None:
            width = self.gripper_close
        grab_plate_status = self.send_command(
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

    def release_plate(self, width: Optional[int] = None, speed: int = 100) -> bool:
        """Release the plate."""
        if width is None:
            width = self.gripper_open
        release_plate_status = self.send_command(f"ReleasePlate {width} {speed}").split(
            " "
        )
        if release_plate_status[0] != "0":
            self.logger.log_error(
                f"Unexpected response from ReleasePlate: {release_plate_status[0]}"
            )
            raise Pf400ResponseError(
                f"Unexpected response from ReleasePlate command: {release_plate_status[0]}."
            )
        current_gripper_position = self.get_gripper_state()
        if abs(current_gripper_position - width) <= 5:
            return True
        self.logger.log_error(
            f"Gripper failed to open to target width. Expected: {width}, Got: {current_gripper_position}"
        )
        return False

    def open_gripper(self, gripper_length: Optional[int] = None) -> float:
        """Opens the gripper."""
        self.set_gripper_open(gripper_length=gripper_length)
        self.send_command("gripper 1")
        return self.get_gripper_state()

    def close_gripper(self, gripper_length: Optional[int] = None) -> float:
        """Closes the gripper."""
        self.set_gripper_close(gripper_length=gripper_length)
        self.send_command("gripper 2")
        return self.get_gripper_state()

    # -------------------------------------------------------------------------
    # Kinematics -- implemented via custom TCS server commands (Custom.gpl)
    # -------------------------------------------------------------------------

    def joint_to_cart(self, joint_states: list[float]) -> list[float]:
        """Forward kinematics (FK): convert joint angles to Cartesian coordinates.

        Calls the JointToCart custom TCS command which uses the robot's internal
        KineSol method. The rail offset is handled automatically inside the command.

        Args:
            joint_states: 6 joint values [j1, j2, j3, j4, j5, rail]

        Returns:
            Cartesian coordinates as [X, Y, Z, yaw, pitch, roll]
        """
        j1, j2, j3, j4, j5, rail = joint_states
        response = self.send_command(f"JointToCart {j1} {j2} {j3} {j4} {j5} {rail}")
        return self._parse_response(response)

    def cart_to_joint(
        self, cartesian_coordinates: list[float], rail: float
    ) -> list[float]:
        """Inverse kinematics (IK): convert Cartesian coordinates to joint angles.

        Calls the CartToJoint custom TCS command which uses the robot's internal
        KineSol method. The rail position must be passed explicitly so the command
        can subtract it from X before running IK, then return it as j6.

        Args:
            cartesian_coordinates: [X, Y, Z, yaw, pitch, roll] in world coordinates
            rail: Rail position in mm (j6 from wherej)

        Returns:
            Joint angles as [j1, j2, j3, j4, j5, rail]
        """
        x, y, z, yaw, pitch, roll = cartesian_coordinates
        response = self.send_command(
            f"CartToJoint {x} {y} {z} {yaw} {pitch} {roll} {rail}"
        )
        return self._parse_response(response)

    def rotate_yaw(self, joint_states: list[float], rotation_deg: float) -> list[float]:
        """Rotate the end effector yaw at a given joint location.

        Calls the RotateLoc custom TCS command which internally runs FK, applies
        the yaw rotation, then runs IK to return the new joint angles. Use this
        to switch between narrow and wide microplate orientations without saving
        duplicate locations.

        Args:
            joint_states: 6 joint values [j1, j2, j3, j4, j5, rail]
            rotation_deg: Yaw rotation to apply in degrees, typically 90 or -90

        Returns:
            New joint angles as [j1, j2, j3, j4, j5, rail]
        """
        j1, j2, j3, j4, j5, rail = joint_states
        response = self.send_command(
            f"RotateLoc {j1} {j2} {j3} {j4} {j5} {rail} {rotation_deg}"
        )
        return self._parse_response(response)

    def move_with_rotation(
        self,
        joint_states: list[float],
        rotation_deg: float,
        profile: int = 2,
    ) -> str:
        """Move the end effector to a location with a yaw rotation applied.

        Computes the rotated Cartesian location using FK and the given rotation,
        then moves to it using MoveC (straight line Cartesian motion). This avoids
        IK ambiguity by letting the robot's internal motion controller handle the
        joint configuration along the straight line path.

        Args:
            joint_states: 6 joint values [j1, j2, j3, j4, j5, rail]
            rotation_deg: Yaw rotation to apply in degrees, typically 90 or -90
            profile: Motion profile index, defaults to fast profile (2)

        Returns:
            Robot response string
        """
        cart = self.joint_to_cart(joint_states)
        cart[3] += rotation_deg
        # Normalize yaw to -180 to 180
        if cart[3] > 180:
            cart[3] -= 360
        elif cart[3] < -180:
            cart[3] += 360
        return self.move_cartesian(cart, profile=profile)

    # -------------------------------------------------------------------------
    # Force Compliance -- implemented via custom TCS server commands (Custom.gpl)
    # Requires XY Compliance license on the controller.
    # -------------------------------------------------------------------------

    def enable_compliance(self) -> str:
        """Enable horizontal force compliance on the robot joints.

        Allows the horizontal arm axes to float and comply to reaction forces
        while other axes continue to be driven normally. Use before descending
        into a pick or place location where the plate may be slightly misaligned
        or stuck. Always call disable_compliance() after the operation.

        Args:
            bias_torque_pct: Bias torque as a percentage of last used position control
                torque (0-100). 0 = fully free (maximum compliance), 100 = full
                holding torque (no compliance). Typical values: 0-20 for most
                pick/place operations.

        Returns:
            Robot response string
        """
        return self.send_command(f"EnableCompliance {self.default_bias_torque_pct}")

    def disable_compliance(self) -> str:
        """Disable horizontal force compliance and return to normal position control.

        Always call this after enable_compliance() once the pick or place
        operation is complete.

        Returns:
            Robot response string
        """
        return self.send_command("DisableCompliance")

    # -------------------------------------------------------------------------
    # Height Detection -- implemented via TCS PARobot Auto Center module
    # Requires Z Height Detection license on the controller.
    # -------------------------------------------------------------------------

    def height_detect(
        self,
        search_limit_mm: float = -500,
        max_force_n: float = -15,
        thorough: bool = True,
    ) -> float:
        """Detect the height of a surface below the gripper using motor force sensing.

        The gripper must be positioned at least 10-20mm above the surface before
        calling. The robot will descend until it detects contact or reaches the
        search limit.

        Args:
            search_limit_mm: Maximum downward search distance in mm, must be negative
            max_force_n: Maximum contact force in Newtons before stopping, must be negative
            thorough: If True uses thorough mode (0.3mm accuracy, ~4s slower),
                else quick mode (0.5mm accuracy, faster)

        Returns:
            Detected Z height in mm (world coordinates)
        """
        mode = 2 if thorough else 1
        response = self.send_command(
            f"HeightDetect {mode} {search_limit_mm} {max_force_n}"
        )
        return float(response.split(" ")[1])

    # -------------------------------------------------------------------------
    # Motion
    # -------------------------------------------------------------------------

    def move_joint(
        self,
        target_joint_angles: list[float],
        profile: int = 1,
        gripper_close: bool = False,
        gripper_open: bool = False,
    ) -> str:
        """Move the robot to a joint angle location.

        Args:
            target_joint_angles: Target joint angles [j1, j2, j3, j4, j5, rail]
            profile: Motion profile ID
            gripper_close: If True, gripper is closed before moving
            gripper_open: If True, gripper is opened before moving
        """
        if gripper_close and gripper_open:
            raise Exception("Gripper cannot be open and closed at the same time!")
        if gripper_close:
            target_joint_angles[4] = self.gripper_close
        elif gripper_open:
            target_joint_angles[4] = self.gripper_open
        else:
            target_joint_angles[4] = self.get_gripper_state()
        move_command = (
            "movej" + " " + str(profile) + " " + " ".join(map(str, target_joint_angles))
        )
        return self.send_command(move_command)

    def move_cartesian(
        self, target_cartesian_coordinates: list[float], profile: int = 2
    ) -> str:
        """Move the arm to a target location in Cartesian coordinates."""
        move_command = (
            "MoveC"
            + " "
            + str(profile)
            + " "
            + " ".join(map(str, target_cartesian_coordinates))
        )
        return self.send_command(move_command)

    def move_in_one_axis(
        self, profile: int = 1, axis_x: int = 0, axis_y: int = 0, axis_z: int = 0
    ) -> str:
        """Move the end effector on a single axis by a given distance in mm."""
        cartesian_coordinates = self.get_cartesian_coordinates()
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
        return self.send_command(move_command)

    def move_gripper_safe_zone(self) -> None:
        """Check if end effector is outside safe boundaries and move it in if needed."""
        current_cartesian_coordinates = self.get_cartesian_coordinates()
        if current_cartesian_coordinates[1] <= self.safe_left_boundary:
            y_distance = self.safe_left_boundary - current_cartesian_coordinates[1]
            self.move_in_one_axis(profile=self.slow_motion_profile, axis_y=y_distance)
        elif current_cartesian_coordinates[1] >= self.safe_right_boundary:
            y_distance = self.safe_right_boundary - current_cartesian_coordinates[1]
            self.move_in_one_axis(profile=self.slow_motion_profile, axis_y=y_distance)

    def move_gripper_neutral(self) -> None:
        """Move end effector to neutral position."""
        self.move_gripper_safe_zone()
        gripper_neutral = self.get_joint_states()
        gripper_neutral[3] = self.neutral_joints[3]
        self.move_joint(gripper_neutral, self.slow_motion_profile)

    def move_arm_neutral(self) -> None:
        """Move arm to neutral position."""
        arm_neutral = self.neutral_joints
        current_location = self.get_joint_states()
        arm_neutral[0] = current_location[0]
        arm_neutral[5] = current_location[5]
        self.move_joint(arm_neutral, self.slow_motion_profile)

    def move_rails_neutral(
        self, v_rail: Optional[float] = None, h_rail: Optional[float] = None
    ) -> None:
        """Move rails to neutral position."""
        current_location = self.get_joint_states()
        if not v_rail:
            v_rail = current_location[0]
        if not h_rail:
            h_rail = current_location[5]
        self.neutral_joints[5] = h_rail
        self.move_joint(self.neutral_joints, self.fast_motion_profile)
        self.neutral_joints[0] = v_rail + self.default_approach_height
        self.move_joint(self.neutral_joints, self.slow_motion_profile)

    def move_all_joints_neutral(self, target: Optional[list[float]] = None) -> None:
        """Move all joints to neutral position."""
        if target is None:
            target = self.get_joint_states()
        self.move_gripper_neutral()
        self.move_arm_neutral()
        self.move_rails_neutral(target[0], target[5])

    def remove_lid(
        self,
        source: LocationArgument,
        target: LocationArgument,
        lid_removal_grip_height: Optional[float] = None,
        lid_only_grip_height: Optional[float] = None,
        source_approach: LocationArgument = None,
        target_approach: LocationArgument = None,
        source_plate_rotation: Optional[str] = None,
        target_plate_rotation: Optional[str] = None,
        grab_offset: Optional[float] = None,
        source_approach_height_offset: Optional[float] = None,
        target_approach_height_offset: Optional[float] = None,
        source_height_limit: Optional[float] = None,
        target_height_limit: Optional[float] = None,
    ) -> bool:
        """Remove the lid from the plate"""

        source.representation = copy.deepcopy(source.representation)
        source.representation[0] += lid_removal_grip_height

        target.representation = copy.deepcopy(target.representation)
        target.representation[0] += lid_only_grip_height

        return self.transfer(
            source=source,
            target=target,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
            grab_offset=grab_offset,
            source_approach_height_offset=source_approach_height_offset,  # NONE
            target_approach_height_offset=target_approach_height_offset,  # NONE
            source_height_limit=source_height_limit,  # NONE
            target_height_limit=target_height_limit,  # NONE
        )

    def replace_lid(
        self,
        source: LocationArgument,
        target: LocationArgument,
        lid_removal_grip_height: Optional[float] = None,
        lid_only_grip_height: Optional[float] = None,
        source_approach: LocationArgument = None,
        target_approach: LocationArgument = None,
        source_plate_rotation: Optional[str] = None,
        target_plate_rotation: Optional[str] = None,
        grab_offset: Optional[float] = None,
        source_approach_height_offset: Optional[float] = None,
        target_approach_height_offset: Optional[float] = None,
        source_height_limit: Optional[float] = None,
        target_height_limit: Optional[float] = None,
    ) -> bool:
        """Replace the lid on the plate"""

        source.representation = copy.deepcopy(source.representation)
        source.representation[0] += lid_only_grip_height

        target.representation = copy.deepcopy(target.representation)
        target.representation[0] += lid_removal_grip_height

        return self.transfer(
            source=source,
            target=target,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
            grab_offset=grab_offset,
            source_approach_height_offset=source_approach_height_offset,
            target_approach_height_offset=target_approach_height_offset,
            source_height_limit=source_height_limit,
            target_height_limit=target_height_limit,
        )

    def rotate_plate_on_deck(
        self, rotation_degree: int, rotation_deck: Optional[LocationArgument] = None
    ) -> None:
        """Use the rotation deck to rotate the plate between two transfers."""
        if not rotation_deck:
            raise ValueError("Rotation deck location must be provided.")
        target = rotation_deck.representation

        if rotation_degree == -90:
            target = self.rotate_yaw(target, rotation_degree)

        above_position = list(map(add, target, self.default_approach_vector))

        self.move_all_joints_neutral(target)
        self.move_joint(above_position, self.slow_motion_profile)
        target_position_above_compliance = copy.deepcopy(target)
        target_position_above_compliance[0] += 1.0
        self.move_joint(target_position_above_compliance, self.slow_motion_profile)
        # self.enable_compliance() # noqa: ERA001
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
        # self.disable_compliance()  # noqa: ERA001
        self.open_gripper(self.gripper_open_wide)

        target = self.rotate_yaw(target, rotation_degree)
        above_position = list(map(add, target, self.default_approach_vector))
        self.move_joint(
            target_joint_angles=above_position, profile=self.slow_motion_profile
        )
        # self.enable_compliance()  # noqa: ERA001
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
        # self.disable_compliance()  # noqa: ERA001
        self.move_all_joints_neutral(target)

    def _handle_approach_location(self, approach: LocationArgument) -> None:
        """Handle moving to an approach location, whether single or multiple."""
        if isinstance(approach.representation[0], list):
            self.move_all_joints_neutral(approach.representation[0])
            for location in approach.representation:
                self.move_joint(
                    target_joint_angles=location,
                    profile=self.fast_motion_profile,
                )
        else:
            self.move_all_joints_neutral(approach.representation)
            self.move_joint(
                target_joint_angles=approach.representation,
                profile=self.fast_motion_profile,
            )

    def _handle_approach_return(
        self, approach: LocationArgument, default_motion: Optional[str] = None
    ) -> None:
        """
        Handle returning from an approach location, whether single or multiple.
        Uses straight motion profile for the first approach location (closest to target),
        and fast motion profile for remaining approach locations.
        """
        if isinstance(approach.representation[0], list):
            for index, location in enumerate(reversed(approach.representation)):
                if index == 0:
                    motion_profile = self.straight_motion_profile
                else:
                    motion_profile = (
                        default_motion
                        if default_motion is not None
                        else self.fast_motion_profile
                    )
                self.move_joint(
                    target_joint_angles=location,
                    profile=motion_profile,
                )
            self.move_all_joints_neutral(location)
        else:
            self.move_joint(
                target_joint_angles=approach.representation,
                profile=self.straight_motion_profile,
            )
            self.move_all_joints_neutral(approach.representation)

    def _calculate_above_position(
        self,
        position: list,
        approach_height_offset: Optional[float] = None,
        grab_height_offset: Optional[float] = None,
    ) -> list:
        """
        Calculate the position above a target with optional height offset.
        """
        above_offset = (
            [self.default_approach_height, 0, 0, 0, 0, 0]
            if approach_height_offset is None
            else [approach_height_offset, 0, 0, 0, 0, 0]
        )

        if grab_height_offset:
            above_offset[0] += grab_height_offset
        return list(map(add, position, above_offset))

    def _apply_grab_offset(self, position: list, grab_offset: float) -> list:
        """Apply grab offset to a position."""
        position = copy.deepcopy(position)
        position[0] += grab_offset
        return position

    def pick_plate(
        self,
        source: LocationArgument,
        source_approach: LocationArgument = None,
        grab_offset: Optional[float] = None,
        approach_height_offset: Optional[float] = None,
        height_limit: Optional[float] = None,
        grip_width: Optional[int] = None,
    ) -> bool:
        """
        Pick a plate from the source location, optionally using an approach location.

        Returns True if the plate was successfully grabbed, False otherwise.
        """

        above_position = self._calculate_above_position(
            source.representation, approach_height_offset, grab_offset
        )
        if height_limit is not None:
            calculated_height = (
                above_position[0]
                + self.gripper_clearance_height
                - source.representation[0]
            )
            if calculated_height >= height_limit:
                self.logger.log_error(
                    f"Height limit validation failed: calculated above position "
                    f"({calculated_height}) exceeds height limit ({height_limit})"
                )
                return False

        self.open_gripper()

        if source_approach:
            self._handle_approach_location(source_approach)
            approach_motion_profile = self.straight_motion_profile
        else:
            self.move_all_joints_neutral(source.representation)
            approach_motion_profile = self.fast_motion_profile
        self.move_joint(
            target_joint_angles=above_position, profile=approach_motion_profile
        )

        target_position = (
            self._apply_grab_offset(source.representation, grab_offset)
            if grab_offset
            else source.representation
        )
        self.move_joint(
            target_joint_angles=target_position,
            profile=approach_motion_profile,
            gripper_open=True,
        )
        # self.enable_compliance()  # noqa: ERA001
        grab_succeeded = self.grab_plate(width=grip_width, speed=100, force=10)

        if self.resource_client and grab_succeeded and source.resource_id:
            popped_plate, _updated_resource = self.resource_client.pop(
                resource=source.resource_id
            )
            self.resource_client.push(
                resource=self.gripper_resource_id, child=popped_plate
            )

        self.move_in_one_axis(
            profile=self.slow_motion_profile,
            axis_z=self.default_approach_height + approach_height_offset
            if approach_height_offset
            else self.default_approach_height,
        )
        # self.disable_compliance()  # noqa: ERA001

        if source_approach:
            self._handle_approach_return(
                approach=source_approach, default_motion=self.slow_motion_profile
            )
        else:
            self.move_all_joints_neutral(source.representation)

        return grab_succeeded

    def place_plate(
        self,
        target: LocationArgument,
        target_approach: LocationArgument = None,
        grab_offset: Optional[float] = None,
        approach_height_offset: Optional[float] = None,
        height_limit: Optional[float] = None,
        open_width: Optional[int] = None,
    ) -> bool:
        """
        Place a plate in the target location
        """
        above_position = self._calculate_above_position(
            target.representation, approach_height_offset, grab_offset
        )
        if height_limit is not None:
            calculated_height = (
                above_position[0]
                + self.gripper_clearance_height
                - target.representation[0]
            )
            if calculated_height >= height_limit:
                self.logger.log_error(
                    f"Height limit validation failed: calculated above position "
                    f"({calculated_height}) exceeds height limit ({height_limit})"
                )
                return False

        if target_approach:
            self._handle_approach_location(target_approach)
            approach_motion_profile = self.straight_motion_profile
        else:
            self.move_all_joints_neutral(target.representation)
            approach_motion_profile = self.slow_motion_profile

        self.move_joint(above_position, approach_motion_profile)

        target_position = (
            self._apply_grab_offset(target.representation, grab_offset)
            if grab_offset
            else target.representation
        )

        target_position_above_compliance = copy.deepcopy(target_position)
        target_position_above_compliance[0] += 2.0
        self.move_joint(target_position_above_compliance)
        # self.enable_compliance()  # noqa: ERA001
        self.move_joint(target_position, approach_motion_profile)
        # self.disable_compliance()  # noqa: ERA001
        release_succeeded = self.release_plate(width=open_width)

        if (
            self.resource_client
            and release_succeeded
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
            profile=self.fast_motion_profile,
            axis_z=self.default_approach_height + approach_height_offset
            if approach_height_offset
            else self.default_approach_height,
        )

        if target_approach:
            self._handle_approach_return(
                approach=target_approach, default_motion=self.fast_motion_profile
            )
        else:
            self.move_all_joints_neutral(target.representation)

        return release_succeeded

    def move_to_location(
        self,
        target: LocationArgument,
        target_approach: LocationArgument = None,
        grab_offset: Optional[float] = None,
        approach_height_offset: Optional[float] = None,
    ) -> None:
        """Move to a target location for testing/calibration purposes."""
        above_position = self._calculate_above_position(
            target.representation, approach_height_offset, grab_offset
        )

        holding_plate = (
            self.resource_client
            and len(
                self.resource_client.get_resource(self.gripper_resource_id).children
            )
            > 0
        )

        if not holding_plate:
            self.open_gripper()

        if target_approach:
            self._handle_approach_location(target_approach)
            approach_motion_profile = self.straight_motion_profile
        else:
            self.move_all_joints_neutral(target.representation)
            approach_motion_profile = self.fast_motion_profile

        self.move_joint(
            target_joint_angles=above_position, profile=approach_motion_profile
        )

        target_position = (
            self._apply_grab_offset(target.representation, grab_offset)
            if grab_offset
            else target.representation
        )
        self.move_joint(
            target_joint_angles=target_position,
            profile=approach_motion_profile,
            gripper_open=not holding_plate,
        )

    def move_neutral(self, height_offset: Optional[float] = None) -> None:
        """Retract upward and move to neutral position."""
        retract_height = (
            height_offset if height_offset is not None else self.default_approach_height
        )
        self.move_in_one_axis(
            profile=self.slow_motion_profile,
            axis_z=retract_height,
        )
        self.move_all_joints_neutral()

    def transfer(
        self,
        source: LocationArgument,
        target: LocationArgument,
        source_approach: LocationArgument = None,
        target_approach: LocationArgument = None,
        source_plate_rotation: str = "",
        target_plate_rotation: str = "",
        rotation_deck: Optional[LocationArgument] = None,
        grab_offset: Optional[float] = None,
        source_approach_height_offset: Optional[float] = None,
        target_approach_height_offset: Optional[float] = None,
        source_height_limit: Optional[float] = None,
        target_height_limit: Optional[float] = None,
    ) -> bool:
        """
        Description: Plate transfer function that performs series of movements to pick and place the plates
                Parameters:
                        - source: Source location
                        - target: Target location
                        - source_approach: Approach location for source
                        - target_approach: Approach location for target
                        - source_plate_rotation: narrow or wide
                        - target_plate_rotation: narrow or wide
                        - rotation_deck: Location for plate rotation deck
                        - grab_offset: Add grab height offset (applied identically at pick and place)
                        - source_approach_height_offset: Add source approach height offset
                        - target_approach_height_offset: Add target approach height offset
                        - source_height_limit: Maximum height limit for source pick
                        - target_height_limit: Maximum height limit for target place
                Returns:
                        True if transfer was successful, False otherwise.

                Note: Plate rotation defines the rotation of the plate on the deck, not the grabbing angle.
        """
        source = copy.deepcopy(source)
        target = copy.deepcopy(target)

        for rotation_arg in [source_plate_rotation, target_plate_rotation]:
            if rotation_arg is not None and rotation_arg.lower() not in [
                "wide",
                "narrow",
            ]:
                raise ValueError(
                    f"Invalid plate rotation argument: {rotation_arg}. "
                    "Expected None, 'wide', or 'narrow'."
                )

        # Determine source rotation (0 or 90 degrees)
        plate_source_rotation = (
            90
            if source_plate_rotation and source_plate_rotation.lower() == "wide"
            else 0
        )
        self.grip_wide = (
            source_plate_rotation and source_plate_rotation.lower() == "wide"
        )

        # Determine target rotation (0 or 90 degrees)
        plate_target_rotation = (
            90
            if target_plate_rotation and target_plate_rotation.lower() == "wide"
            else 0
        )

        rotation_needed = plate_target_rotation - plate_source_rotation
        if rotation_needed != 0 and rotation_deck is None:
            self.logger.log_error(
                f"Rotation required ({rotation_needed} degrees) but rotation_deck was not provided."
            )
            return False

        pick_result = self.pick_plate(
            source=source,
            source_approach=source_approach,
            grab_offset=grab_offset,
            approach_height_offset=source_approach_height_offset,
            height_limit=source_height_limit,
        )

        if not pick_result:
            self.move_all_joints_neutral()
            sleep(5)
            self.logger.error("Transfer failed: no plate detected after picking.")
            return False

        self.grip_wide = (
            target_plate_rotation and target_plate_rotation.lower() == "wide"
        )

        # Rotate plate if needed
        if rotation_needed != 0:
            self.rotate_plate_on_deck(
                rotation_degree=rotation_needed, rotation_deck=rotation_deck
            )

        place_result = self.place_plate(
            target=target,
            target_approach=target_approach,
            grab_offset=grab_offset,
            approach_height_offset=target_approach_height_offset,
            height_limit=target_height_limit,
        )
        if not place_result:
            self.logger.error("Transfer failed: plate not released properly.")
            return False

        return True
