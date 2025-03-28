#!/usr/bin/env python3
"""Driver code for the PF400 robot arm."""

import copy
import math
import telnetlib
import threading
from operator import add
from time import sleep

from madsci.client.resource_client import ResourceClient
from madsci.common.types.location_types import LocationArgument

from pf400_interface.pf400_constants import ERROR_CODES, MOTION_PROFILES, OUTPUT_CODES
from pf400_interface.pf400_errors import (
    CommandException,
    ConnectionException,
    ErrorResponse,
)
from pf400_interface.pf400_kinematics import KINEMATICS


class PF400(KINEMATICS):
    """Main Driver Class for the PF400 Robot Arm."""

    commandLock = threading.Lock()

    def __init__(
        self,
        host="146.137.240.35",
        port=10100,
        mode=0,
        resource_client: ResourceClient = None,
        gripper_resource_id: str = None,
    ):
        """
        Description:
                        - Python interface that allows remote commands to be executed using simple string messages over Telnet socket on PF400.
                        - PF400 is the main object that will be used for operations such as remote connection as well as sending movement commands.
                        - Programs are sent to the 10x00 port (first robot port: 10100).
                        - A program sent to robot will be executed immediately unless there is a prior operation running on the robot.
                        - If a second motion command is sent while the referenced robot is moving, the second command is blocked and will not reply until the first motion is complete.

        """
        super().__init__()  # PF400 kinematics

        print("Initializing connection...")
        self.host = host
        self.port = port
        self.mode = mode
        self.connection = None
        self.resource_client = resource_client
        self.gripper_resource_id = gripper_resource_id
        # Error code list of the PF400
        self.error_codes = ERROR_CODES

        # Default Motion Profile Parameters. Using two profiles for faster and slower movements
        self.motion_profiles = MOTION_PROFILES
        self.slow_motion_profile = 1
        self.fast_motion_profile = 2

        # Output code list of the PF400
        self.output_codes = OUTPUT_CODES

        # Robot State
        self.power_state = "0"
        self.attach_state = "0"
        self.home_state = "1"
        self.initialization_state = "0"

        # Initialize robot
        self.connect()
        self.init_connection_mode()
        if port == 10100:
            self.force_initialize_robot()
        elif port == 10000:
            self.status_port_initialization()

        sleep(2)
        self.movement_state = self.get_robot_movement_state()
        self.robot_state = "Normal"
        self.robot_error_msg = ""
        self.robot_warning = ""

        # Gripper variables
        self.gripper_open_wide = 130.0
        self.gripper_open_narrow = 90.0
        self.gripper_close_value = 77.0
        self.gripper_safe_height = 10.0

        # Arm variables
        self.joint_state_position = [0, 0, 0, 0, 0, 0, 0]
        self.neutral_joints = [
            400.0,
            1.400,
            177.101,
            537.107,
            self.gripper_close_value,
            0.0,
        ]
        self.module_left_dist = -350.0
        self.module_right_dist = 350.0

        # Sample variables
        self.sample_above_height = 17.0  # was 15 TESTING
        self.above = [self.sample_above_height, 0, 0, 0, 0, 0]
        self.y_recoil = 300.0

        # Plate variables
        self.plate_state = 0
        self.plate_width = self.gripper_open_wide
        self.plate_source_rotation = 0  # 90 to rotate 90 degrees
        self.plate_target_rotation = 0  # 90 to rotate 90 degrees
        self.plate_rotation_deck = [146.5, -33.811, 107.957, 643.401, 82.122, 995.051]
        self.plate_rotation_deck_narrow = [
            631.014,
            56.297,
            273.181,
            449.556,
            78.458,
            822.268,
        ]
        self.plate_rotation_deck_wide = [
            628.122,
            4.926,
            70.372,
            615.655,
            122.046,
            806.669,
        ]

        self.plate_lid_deck = [145.0, -26.352, 114.149, 629.002, 82.081, 995.105]
        self.plate_camera_deck = [90.597, 26.416, 66.422, 714.811, 81.916, 995.074]
        self.trash_bin = [259.847, -36.810, 69.090, 687.466, 81.002, 995.035]

        self.set_gripper_open()
        self.set_gripper_close()
        self.gripper_state = self.get_gripper_state()

    def connect(self):
        """
        Description: Create a streaming socket to send string commands to the robot.
        """
        try:
            self.connection = telnetlib.Telnet(self.host, self.port, 5)
        except TimeoutError as e:
            raise ConnectionException(err_message="Timed out error") from e

    def disconnect(self):
        """ """
        self.connection.close()

    def send_command(self, command):
        """
                Description: Sends the commands to the robot over the socket client
        Parameters:
                - command: Command itself in string format
        """

        self.commandLock.acquire()

        try:
            if not self.connection:
                self.connect()

            self.get_robot_movement_state()
            if self.movement_state > 1:
                while self.movement_state > 1:
                    self.get_robot_movement_state()

            self.connection.write((command.encode("ascii") + b"\n"))
            response = self.connection.read_until(b"\r\n").rstrip().decode("ascii")

            if response != "" and response in self.error_codes:
                self.robot_state = "ERROR"
                self.handle_error_output(response)
                return self.robot_error_msg
            else:
                # CAUSING TOO MANY MESSAGES TO BE PRINTED. UNCOMMENT IF NEEDED
                if response in self.output_codes:
                    # print("<< " + self.output_codes[response])
                    pass
                else:
                    # print("<< "+ response)
                    pass

                self.robot_state = "NORMAL"
                self.robot_error_msg = ""

            return response

        except AttributeError as e:
            raise CommandException(err_message="Attribute Error") from e

        finally:
            self.commandLock.release()

    def init_connection_mode(self):
        """ """
        if not self.connection:
            self.connect()

        if self.mode == 1:
            # Set TCS to verbose
            self.send_command("mode 1")
            self.send_command("selectrobot 1")
            print("Setting connection mode to 1")

        else:
            # Set TCS to non-verbose
            self.send_command("mode 0")
            print("Setting connection mode to 0")

    def handle_error_output(self, output):
        """
        Description: Handles the error message output
        """
        response = ErrorResponse.from_error_code(output)
        print(response)
        self.robot_error_msg = response

    def check_robot_state(self, wait: int = 0.1):
        """
        Description: Checks the robot state
        """

        out_msg = self.send_command("sysState")
        if "0 21" in out_msg:
            out_msg = "Robot initialized and in ready state"
        return out_msg

    def enable_power(self, wait: int = 0.1):
        """
        Description: Enables the power on the robot
        """

        out_msg = self.send_command("hp 1")
        return out_msg

    def disable_power(self, wait: int = 0.1):
        """
        Description: Disables the power on the robot
        """
        out_msg = self.send_command("hp 0")
        return out_msg

    def attach_robot(self, robot_id: str = "1", wait: int = 0.1):
        """
        Description: If there are multiple PF400 robots, chooses which robot will be programed attaches to the software.
                                If robot ID is not given it will attach the first robot.
        Parameters:
                        - robot_id: ID number of the robot
        """
        out_msg = self.send_command("attach " + robot_id)
        return out_msg

    def home_robot(self, wait: int = 0.1):
        """
        Description: Homes robot joints. Homing takes around 15 seconds.
        """
        cmd = "home"

        out_msg = self.send_command(cmd)
        sleep(10)

        return out_msg

    def initialize_robot(self):
        """
        Description: Initializes the robot by calling enable_power, attach_robot, home_robot, set_profile functions and
                                checks the robot state to find out if the initialization was successful
        """

        self.get_overall_state()

        if self.power_state == "-1":
            self.power_state = self.enable_power()
            sleep(6)

        if self.attach_state == "-1":
            self.attach_state = self.attach_robot()
            sleep(6)

        if self.home_state == "-1":
            self.home_robot()
            sleep(6)

        profile = self.set_profile()

        if (
            self.power_state[0].find("-") == -1
            and self.attach_state[0].find("-") == -1
            and profile[0].find("-") == -1
        ):
            print("Robot initialization successfull")
        else:
            print("Robot initialization failed")

    def force_initialize_robot(self):
        """
        Description: Repeats the initialization until there are no errors and the robot is initialized.
        """
        # Check robot state & initialize
        if self.get_overall_state() == -1:
            print("Robot is not initialized! Initializing now...")
            self.initialize_robot()
            self.force_initialize_robot()

    def status_port_initialization(self):
        """Initializes the robot on the status port"""
        self.send_command("selectRobot 1")
        self.enable_power()

    def refresh_joint_state(self):
        """
        Description:
        """

        try:
            self.connection.write(("wherej".encode("ascii") + b"\n"))
            joint_array = self.connection.read_until(b"\r\n").rstrip().decode("ascii")
        except AttributeError as e:
            raise CommandException(err_message="Attribute Error") from e

        if joint_array != "" and joint_array in self.error_codes:
            self.handle_error_output(joint_array)

        joint_array = joint_array.split(" ")
        self.joint_state_position[0] = float(joint_array[1]) * 0.001  # J1, Tower
        self.joint_state_position[1] = (
            float(joint_array[2]) * math.pi / 180
        )  # J2, shoulder
        self.joint_state_position[2] = (
            float(joint_array[3]) * math.pi / 180
        )  # J3, elbow
        self.joint_state_position[3] = (
            float(joint_array[4]) * math.pi / 180
        )  # J4, wrist
        self.joint_state_position[4] = (
            float(joint_array[5]) * 0.0005
        )  # J5, gripper (urdf is 1/2 scale)
        self.joint_state_position[5] = (
            float(joint_array[5]) * 0.0005
        )  # J5, gripper (urdf is 1/2 scale)
        self.joint_state_position[6] = float(joint_array[6]) * 0.001  # J6, rail
        return self.joint_state_position

    def get_robot_movement_state(self):
        """Checks the movement state of the robot
        States: 0 = Power off
                        1 = Stopping
                        2 = Acceleration
                        3 = Deceleration
        """
        try:
            self.connection.write(("state".encode("ascii") + b"\n"))
            movement_state = (
                self.connection.read_until(b"\r\n").rstrip().decode("ascii")
            )
        except AttributeError as e:
            raise CommandException(err_message="Attribute Error") from e

        try:
            if movement_state != "" and movement_state in self.error_codes:
                self.handle_error_output(movement_state)
            else:
                self.movement_state = int(float(movement_state.split(" ")[1]))
        except UnboundLocalError as e:
            raise CommandException(err_message="UnboundLocalError") from e

    def get_overall_state(self):
        """
        Description: Checks general state
        """

        self.connection.write(("hp".encode("ascii") + b"\n"))
        power_msg = (
            self.connection.read_until(b"\r\n").rstrip().decode("ascii").split(" ")
        )

        self.connection.write(("attach".encode("ascii") + b"\n"))
        attach_msg = (
            self.connection.read_until(b"\r\n").rstrip().decode("ascii").split(" ")
        )

        self.connection.write(("pd 2800".encode("ascii") + b"\n"))
        home_msg = (
            self.connection.read_until(b"\r\n").rstrip().decode("ascii").split(" ")
        )

        self.connection.write(("sysState".encode("ascii") + b"\n"))
        state_msg = (
            self.connection.read_until(b"\r\n").rstrip().decode("ascii").split(" ")
        )

        if len(power_msg) == 1 or power_msg[0].find("-") != -1 or power_msg[1] == "0":
            self.power_state = "-1"
        else:
            self.power_state = power_msg[1]

        if (
            attach_msg[1].find("0") != -1
            or attach_msg[0].find("-") != -1
            or attach_msg[1] == "0"
        ):
            self.attach_state = "-1"
        else:
            self.attach_state = attach_msg[1]

        if (
            home_msg[1].find("0") != -1
            or home_msg[0].find("-") != -1
            or home_msg[1] == "0"
        ):
            self.home_state = "-1"
        else:
            self.home_state = home_msg[1]

        if state_msg[1].find("7") != -1 or state_msg[0].find("-") != -1:
            self.initialization_state = "-1"
        else:
            self.initialization_state = state_msg[1]

        if (
            self.power_state == "-1"
            or self.attach_state == "-1"
            or self.home_state == "-1"
            or self.initialization_state == "-1"
        ):
            return -1
        else:
            return 0

    def get_joint_states(self):
        """
        Description: Locates the robot and returns the joint locations for all 6 joints.
        """
        states = self.send_command("wherej")
        joints = states.split(" ")
        joints = joints[1:]
        return [float(x) for x in joints]

    def get_cartesian_coordinates(self):
        """
        Description: This function finds the current cartesian coordinates and angles of the robot.
                Return: A float array with x/y/z yaw/pitch/roll
        """
        coordinates = self.send_command("whereC")
        coordinates_list = coordinates.split(" ")
        coordinates_list = coordinates_list[1:-1]
        return [float(x) for x in coordinates_list]

    def get_gripper_length(self):
        """Returns the current length of the gripper."""
        joint_angles = self.get_joint_states()
        return joint_angles[4]

    def get_gripper_state(self):
        """Returns the current state of the gripper."""

        if self.get_gripper_length() > self.gripper_close_value + 1.0:
            self.gripper_state = "open"
        else:
            self.gripper_state = "closed"
        return self.gripper_state

    def set_profile(self, wait: int = 0.1, profile_dict: dict = {"0": 0}):
        """
        Description: Sets and saves the motion profiles (defined in robot data) to the robot.
                                If user defines a custom profile, this profile will saved onto motion profile 3 on the robot
        Parameters:
                        - profile_dict: Custom motion profile
        """
        if len(profile_dict) == 1:
            profile1 = "Profile 1"
            for value in self.motion_profiles[0].values():
                profile1 += " " + str(value)
            profile2 = "Profile 2"
            for value in self.motion_profiles[1].values():
                profile2 += " " + str(value)

            out_msg = self.send_command(profile1)
            self.send_command(profile2)

        elif len(profile_dict) == 8:
            profile3 = "Profile 3"
            for value in profile_dict.values():
                profile3 += " " + str(value)

            out_msg = self.send_command(profile3)

        else:
            raise Exception(
                "Motion profile takes 8 arguments, {} where given".format(
                    len(profile_dict)
                )
            )

        return out_msg

    def set_gripper_open(self):
        """Configure the definition of gripper open."""
        self.send_command(f"GripOpenPos {self.plate_width}")

    def set_gripper_close(self):
        """Configure the definition of gripper close."""
        self.send_command(f"GripClosePos {self.gripper_close_value}")

    def grab_plate(self, width: int = 123, speed: int = 100, force: int = 10):
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
                - 1: Plate grabbed
                - 0: Plate is not grabbed
        """
        grab_plate_status = self.send_command(
            "GraspPlate " + str(width) + " " + str(speed) + " " + str(force)
        ).split(" ")

        if len(grab_plate_status) < 2:
            return

        elif grab_plate_status[1] == "-1":
            self.plate_state = 1

        elif grab_plate_status[1] == "0" and width > 80:  # Do not try smaller width
            width -= 1
            self.grab_plate(width, speed, force)

        elif width <= 80:
            print("PLATE WAS NOT FOUND!")
            self.robot_warning = "Missing Plate"
            self.plate_state = -1

        return grab_plate_status

    def release_plate(self, width: int = 130, speed: int = 100):
        """
        Description:
                Release the plate
        Parameters:
                - width: Open width, in mm. Larger than the widest corners of the plates.
                - speed: Percent speed to open fingers.  1 to 100.
        Returns:
                - release_plate_status == "0" -> Plate released
                - release_plate_status == "1" -> Plate is not released
        """

        release_plate_status = self.send_command(
            "ReleasePlate " + str(width) + " " + str(speed)
        ).split(" ")

        if release_plate_status[0] == "1":
            print("Plate is not released")
        elif release_plate_status[0] == "0":
            self.plate_state = 0

        return release_plate_status

    def gripper_open(self):
        """Opens the gripper"""
        self.set_gripper_open()
        self.send_command("gripper 1")
        return self.get_gripper_state()

    def gripper_close(self):
        """Closes the gripper"""
        self.set_gripper_close()
        self.send_command("gripper 2")
        return self.get_gripper_state()

    def set_plate_rotation(self, joint_states, rotation_degree=0):
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

        new_joint_angles = self.inverse_kinematics(
            cartesian_coordinates, phi_angle, rail_pos
        )

        return new_joint_angles

    def check_incorrect_plate_orientation(self, goal_location, goal_rotation):
        """
        Description: Fixes plate rotation on the goal location if it was recorded with an incorrect orientation.
        Parameters: - goal_location
                                - goal_rotation
        Return:
                goal_location: - New goal location if the incorrect orientation was found.
                                           - Same goal location if there orientation was correct.
        """
        # This will fix plate rotation on the goal location if it was recorded with an incorrect orientation
        cartesian_goal, phi_source, rail_source = self.forward_kinematics(goal_location)
        # Checking yaw angle
        if goal_rotation != 0 and cartesian_goal[3] > -10 and cartesian_goal[3] < 10:
            goal_location = self.set_plate_rotation(goal_location, -goal_rotation)

        return goal_location

    def move_joint(
        self,
        target_joint_angles,
        profile: int = 1,
        gripper_close: bool = False,
        gripper_open: bool = False,
    ):
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
            raise Exception("Gripper cannot be open and close at the same time!")

        # Setting the gripper location to open or close. If there is no gripper position passed in, target_joint_angles will be used.
        if gripper_close:
            target_joint_angles[4] = self.gripper_close_value
        elif gripper_open:
            target_joint_angles[4] = self.plate_width
        else:
            target_joint_angles[4] = self.get_gripper_length()

        move_command = (
            "movej" + " " + str(profile) + " " + " ".join(map(str, target_joint_angles))
        )

        return self.send_command(move_command)

    def move_cartesian(self, target_cartesian_coordinates, profile: int = 2):
        """Move the arm to a target location in cartesian coordinates."""
        move_command = (
            "MoveC"
            + " "
            + str(profile)
            + " "
            + " ".join(map(str, target_cartesian_coordinates))
        )

        return self.send_command(move_command)

    def move_in_one_axis_from_target(
        self,
        target,
        profile: int = 1,
        axis_x: int = 0,
        axis_y: int = 0,
        axis_z: int = 0,
    ):
        """
        TODO: FIX THIS FUNCTION

        Description: Moves the end effector on single axis with a goal movement in millimeters.
        Parameters:
                - target : Joint states of the target location
                - axis_x : Goal movement on x axis in mm
                - axis_y : Goal movement on y axis in mm
                - axis_z : Goal movement on z axis in mm
        """
        # First move robot on linear rail
        current_joint_state = self.get_joint_states()
        current_joint_state[5] = target[5]
        self.move_joint(current_joint_state)

        # Find the cartesian coordinates of the target joint states
        cartesian_coordinates = self.forward_kinematics(target)

        # Move en effector on the single axis
        cartesian_coordinates[0] += axis_x
        cartesian_coordinates[1] += axis_y
        cartesian_coordinates[2] += axis_z

        move_command = (
            "MoveC "
            + " "
            + str(profile)
            + " "
            + "".join(map(str, cartesian_coordinates))
        )
        self.send_command(move_command)

    def move_in_one_axis(
        self, profile: int = 1, axis_x: int = 0, axis_y: int = 0, axis_z: int = 0
    ):
        """
        Description: Moves the end effector on single axis with a goal movement in millimeters.
        Parameters:
                - axis_x : Goal movement on x axis in mm
                - axis_y : Goal movement on y axis in mm
                - axis_z : Goal movement on z axis in mm
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
        return self.send_command(move_command)

    def move_one_joint(self, joint_num, target, move_profile):
        """
        Description: Moves single joint to a target
        Parameters:
                                - joint_num: Joint number that will be moved between 6 joints
                                - target: Target location to move the sigle joint

        """
        return self.send_command(
            "moveoneaxis "
            + str(joint_num)
            + " "
            + str(target)
            + " "
            + str(move_profile)
        )

    def move_multiple_joint(self, target1, target2):
        """Moves extra two joints to their targets"""
        self.send_command("moveextraaxis " + str(target1) + " " + str(target2))
        pass

    def move_gripper_safe_zone(self):
        """
        Description: Check if end effector is inside a module. If it is, move it on the y axis first to prevent collisions with the module frames.
        """

        current_cartesian_coordinates = self.get_cartesian_coordinates()

        if current_cartesian_coordinates[1] <= self.module_left_dist:
            y_distance = self.module_left_dist - current_cartesian_coordinates[1]
            self.move_in_one_axis(1, 0, y_distance, 0)
        elif current_cartesian_coordinates[1] >= self.module_right_dist:
            y_distance = self.module_right_dist - current_cartesian_coordinates[1]
            self.move_in_one_axis(1, 0, y_distance, 0)

    def move_gripper_neutral(self):
        """
        Description: Move end effector to neutral position
        """

        # Create a new function to move the gripper into safe zone
        self.move_gripper_safe_zone()
        gripper_neutral = self.get_joint_states()
        gripper_neutral[3] = self.neutral_joints[3]

        self.move_joint(gripper_neutral, self.slow_motion_profile)

    def move_arm_neutral(self):
        """
        Description: Move arm to neutral position
        """
        arm_neutral = self.neutral_joints
        current_location = self.get_joint_states()
        arm_neutral[0] = current_location[0]
        arm_neutral[5] = current_location[5]

        self.move_joint(arm_neutral, self.slow_motion_profile)

    def move_rails_neutral(self, v_rail: float = None, h_rail: float = None):
        """Setting the target location's linear rail position for pf400_neutral"""

        current_location = self.get_joint_states()

        if not v_rail:
            v_rail = current_location[0]  # Keep the vertical rail same
        if not h_rail:
            h_rail = current_location[5]  # Keep the horizontal rail same

        self.neutral_joints[5] = h_rail
        self.move_joint(self.neutral_joints, self.fast_motion_profile)
        self.neutral_joints[0] = v_rail + self.sample_above_height
        self.move_joint(self.neutral_joints, self.slow_motion_profile)

    def move_all_joints_neutral(self, target=None):
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
    ):
        """Remove the lid from the plate"""
        source.location = copy.deepcopy(source.location)
        source.location[0] += lid_height

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
    ):
        """Replace the lid on the plate"""
        target.location = copy.deepcopy(target.location)
        target.location[0] += lid_height

        self.transfer(
            source=source,
            target=target,
            source_approach=source_approach,
            target_approach=target_approach,
            source_plate_rotation=source_plate_rotation,
            target_plate_rotation=target_plate_rotation,
        )

    def rotate_plate_on_deck(self, rotation_degree: int):
        """
        Description: Uses the rotation deck to rotate the plate between two transfers
        Parameters: - rotation_degree: Rotation degree.
        """
        target = self.plate_rotation_deck

        # Fixing the offset on the z axis
        if rotation_degree == -90:
            target = self.set_plate_rotation(target, -rotation_degree)
            target[0] += 5  # Setting vertical rail 5 mm higher

        abovePos = list(map(add, target, self.above))

        self.move_all_joints_neutral(target)
        self.move_joint(abovePos, self.slow_motion_profile)
        self.move_joint(target, self.slow_motion_profile)
        self.release_plate()
        self.move_in_one_axis(
            profile=1, axis_x=0, axis_y=0, axis_z=self.sample_above_height
        )
        self.gripper_open()

        # Fixing the offset on the z axis
        if rotation_degree == -90:
            target[0] -= 5  # Setting vertical rail 5 mm lower

        # Rotating gripper to grab the plate from other rotation
        target = self.set_plate_rotation(target, rotation_degree)
        # print(target)
        abovePos = list(map(add, target, self.above))
        self.move_joint(target_joint_angles=abovePos, profile=self.slow_motion_profile)
        self.move_joint(
            target_joint_angles=target,
            profile=self.slow_motion_profile,
            gripper_open=True,
        )
        self.grab_plate(self.plate_width, 100, 10)
        if self.plate_state == -1:
            self.robot_warning = "MISSING PLATE"
            print("Rotation cannot be completed, missing plate!")
        self.move_in_one_axis(
            profile=1, axis_x=0, axis_y=0, axis_z=self.sample_above_height
        )
        self.move_all_joints_neutral(target)

    def pick_plate(
        self, source: LocationArgument, source_approach: LocationArgument = None
    ) -> None:
        """
        Pick a plate from the source location
        """

        abovePos = list(map(add, source.location, self.above))
        self.gripper_open()
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
            self.move_all_joints_neutral(source.location)

        self.move_joint(target_joint_angles=abovePos, profile=self.fast_motion_profile)
        self.move_joint(
            target_joint_angles=source.location,
            profile=self.fast_motion_profile,
            gripper_open=True,
        )
        self.grab_plate(width=self.plate_width, speed=100, force=10)

        if self.resource_client:
            popped_plate, updated_resource = self.resource_client.pop(
                resource=source.resource_id
            )
            self.resource_client.push(
                resource=self.gripper_resource_id, child=popped_plate
            )

        self.move_in_one_axis(
            profile=1, axis_x=0, axis_y=0, axis_z=self.sample_above_height
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
            self.move_all_joints_neutral(source.location)

    def place_plate(
        self, target: LocationArgument, target_approach: LocationArgument = None
    ) -> None:
        """
        Plate a plate to the target location
        """
        abovePos = list(map(add, target.location, self.above))
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
            self.move_all_joints_neutral(target.location)

        self.move_joint(abovePos, self.slow_motion_profile)
        self.move_joint(target.location, self.slow_motion_profile)
        self.release_plate(width=self.plate_width)
        if self.resource_client:
            popped_plate, updated_resource = self.resource_client.pop(
                resource=self.gripper_resource_id
            )
            self.resource_client.push(resource=target.resource_id, child=popped_plate)

        self.move_in_one_axis(
            profile=1, axis_x=0, axis_y=0, axis_z=self.sample_above_height
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
            self.move_all_joints_neutral(target.location)

    def transfer(
        self,
        source: LocationArgument,
        target: LocationArgument,
        source_approach: LocationArgument = None,
        target_approach: LocationArgument = None,
        source_plate_rotation: str = "",
        target_plate_rotation: str = "",
    ) -> None:
        """
        Description: Plate transfer function that performs series of movements to pick and place the plates
                Parameters:
                        - source: Source location
                        - target: Target location
                        - source_plate_rotation: narrow or wide
                        - target_plate_rotation: narrow or wide

                Note: Plate rotation defines the rotation of the plate on the deck, not the grabing angle.

        """
        source = copy.deepcopy(source)
        target = copy.deepcopy(target)

        self.robot_warning = "CLEAR"

        # set plate width for source
        if source_plate_rotation.lower() == "wide":
            plate_source_rotation = 90
            self.plate_width = self.gripper_open_wide
            self.set_gripper_open()

        elif source_plate_rotation.lower() == "narrow" or source_plate_rotation == "":
            plate_source_rotation = 0
            self.plate_width = self.gripper_open_narrow
            self.set_gripper_open()

        source.location = self.check_incorrect_plate_orientation(
            source.location, plate_source_rotation
        )

        self.force_initialize_robot()
        self.pick_plate(source=source, source_approach=source_approach)

        if self.plate_state == -1:
            self.robot_warning = "MISSING PLATE"
            print("Transfer cannot be completed, missing plate!")
            self.move_all_joints_neutral()
            sleep(5)
            raise Exception("Transfer failed: no plate detected after picking.")

        # set plate width for target
        if target_plate_rotation.lower() == "wide":
            plate_target_rotation = 90
            self.plate_width = self.gripper_open_wide
            print(f"Setting wide plate width {self.plate_width}")
            self.set_gripper_open()

        elif target_plate_rotation.lower() == "narrow" or target_plate_rotation == "":
            plate_target_rotation = 0
            self.plate_width = self.gripper_open_narrow
            print(f"Setting narrow plate width {self.plate_width}")
            self.set_gripper_open()

        target.location = self.check_incorrect_plate_orientation(
            target.location, plate_target_rotation
        )

        if plate_source_rotation == 90 and plate_target_rotation == 0:
            # Need a transition from 90 degree to 0 degree
            self.rotate_plate_on_deck(-plate_source_rotation)

        elif plate_source_rotation == 0 and plate_target_rotation == 90:
            # Need a transition from 0 degree to 90 degree
            self.rotate_plate_on_deck(plate_target_rotation)

        self.place_plate(target=target, target_approach=target_approach)


if __name__ == "__main__":
    # from pf400_driver.pf400_driver import PF400
    robot = PF400("146.137.240.33")

    # sciclops = [223.0, -38.068, 335.876, 325.434, 79.923, 995.062]
    # sealer = [201.128, -2.814, 264.373, 365.863, 79.144, 411.553]
    # peeler = [225.521, -24.846, 244.836, 406.623, 80.967, 398.778]
    # thermocycler = [247.0, 40.698, 38.294, 728.332, 123.077, 301.082]
    # gamma = [161.481, 60.986, 88.774, 657.358, 124.091, -951.510]

    # ot2bioalpha_deck1= [708.132, -49.277, 318.588, 448.364, 78.476, -207.488]
    # ot2biobeta_deck1= [707.171, -18.952, 267.011, 381.434, 122.040, 688.614]
    # ot2biobeta_deck3= [706.990, -35.793, 256.701, 408.513, 122.046, 576.262]
    # hidex_geraldine_high_nest = [695.710, 34.270, 90.468, 682.956, 78.417, -455.409]
    # hidex_geraldine_low_nest = [688.105, 34.164, 90.435, 683.614, 82.034, -455.416]
    # hidex_geraldine_above_nest= [710.698, 34.164, 90.435, 683.630, 78.540, -455.418]
    # bio_biometra3_default= [775.356, 66.949, 67.620, 758.671, 77.462, 735.973]
    # bio_peeler_default= [606.489, -38.393, 231.287, 433.080, 77.591, -764.287]
    # bio_sealer_default= [583.320, -69.616, 233.645, 462.783, 77.802, -368.290]
    # exchange_deck_high_narrow= [638.532, -19.079, 65.561, 732.286, 78.587, 752.820]
    # exchange_deck_low_narrow= [631.616, -19.079, 65.561, 732.286, 78.587, 752.820]
    # exchange_deck_high_wide= [638.243, 4.798, 93.345, 592.047, 122.134, 918.216]
    # exchange_deck_low_wide= [631.616, 4.798, 93.345, 592.047, 122.134, 918.216]
    # bmg_reader_nest= [611.277, 8.035, 107.426, 691.801, 82.004, 917.639]
    # otflex_deckA= [802.022, 23.010, 280.050, 325.021, 82.051, 999.490]
    # otflex_deckB= [801.032, 5.962, 321.462, 300.193, 82.180, 999.476]
    # tekmatic_incubator_nest= [665.511, 48.877, 76.559, 681.782, 82.028, -104.668]
    # safe_path_tekmatic= [788.160, 28.604, 123.021, 655.281, 70.527, -103.591]
    # safe_path_flexA= [[787.882, -2.133, 175.682, 542.847, 78.581, 999.516], [783.291, 62.017, 175.055, 541.806, 78.587, 999.516], [826.490, 53.122, 275.371, 299.588, 70.515, 999.523]]
    # safe_path_flexB= [[787.882, -2.133, 175.682, 542.847, 78.581, 999.516], [783.291, 62.017, 175.055, 541.806, 78.587, 999.516], [826.748, 33.789, 311.985, 281.840, 89.976, 999.451]]
    # tower_deck1 = [655.606, 39.980, 89.753, 676.393, 82.040, -717.316]
    # tower_deck2 = [745.458, 40.302, 89.187, 677.089, 82.016, -717.307]
    # tower_deck3= [840.092, 40.374, 89.051, 677.195, 82.010, -717.307]
    # tower_deck4= [935.222, 40.442, 88.917, 677.262, 82.051, -717.282]
    # tower_deck5= [1016.476, 40.560, 88.680, 677.378, 82.010, -717.309]
    # safe_path_tower_deck1= [672.247, 17.260, 134.825, 653.927, 82.221, -717.291]
    # safe_path_tower_deck2= [760.688, 18.372, 133.883, 653.418, 82.186, -716.786]
    # safe_path_tower_deck3= [854.408, 16.788, 134.302, 655.330, 82.215, -717.111]
    # safe_path_tower_deck4= [950.484, 15.527, 136.545, 654.549, 82.169, -715.834]
    # safe_path_tower_deck5= [1030.837, 18.536, 134.023, 653.530, 82.174, -712.890]
    # lidnest_1_wide= [374.017, 23.944, 115.144, 671.159, 125.123, -611.851]
    # lidnest_2_wide= [376.183, 20.493, 112.279, 678.140, 125.006, -410.865]
    # lidnest_3_narrow = [376.649, 24.595, 107.940, 675.546, 81.389, -194.121]
    # safe_path_lidnest_1= [404.584, 6.928, 143.517, 658.529, 125.193, -608.622]
    # safe_path_lidnest_2= [404.942, 4.432, 142.130, 662.712, 125.164, -389.370]
    # safe_path_lidnest_3= [400.040, -4.149, 147.508, 664.154, 81.629, -194.110]
    # safe_home= [989.443, -2.007, 174.876, 542.366, 70.574, -843.897]
    # safe_exchange_height= [767.770, 0.265, 175.445, 540.110, 70.521, 543.010]
    # safe_exchange_above= [756.457, 3.342, 90.607, 586.714, 70.556, 918.234]
    # safe_path_exchange= [[767.770, 0.265, 175.445, 540.110, 70.521, 543.010], [756.457, 3.342, 90.607, 586.714, 70.556, 918.234]]
    # safe_path_bmg= [[749.014, -1.224, 175.218, 542.970, 82.151, 702.229], [837.705, 5.468, 111.581, 687.601, 70.486, 917.949]]
    # safe_path_hidex = [749.110, -2.968, 141.991, 667.548, 78.523, -455.415]

    # robot.move_joint([767.770, 0.265, 175.445, 540.110, 70.521, 543.010])
    # robot.move_joint([756.457, 3.342, 90.607, 586.714, 70.556, 918.234])
    # robot.move_joint([756.457, 3.342, 90.607, 586.714, 70.556, 918.234])
    # [783.291, 62.017, 175.055, 541.806, 78.587, 999.5
    #
    # 16]
    # robot.move_joint([787.882, -2.133, 175.682, 542.847, 78.581, 999.516])
    # robot.move_joint([783.291, 62.017, 175.055, 541.806, 78.587, 999.516])
    # robot.move_joint([826.484, 50.573, 279.716, 297.790, 89.976, 999.490])
    # robot.move_joint([802.022, 23.010, 280.050, 325.021, 82.051, 999.490]) # flex A
    # robot.move_joint(safe_path_hidex)
    # robot.move_joint(hidex_geraldine_above_nest)

    # robot.move_joint([787.882, -2.133, 175.682, 542.847, 78.581, 999.516])
    # robot.move_joint([783.291, 62.017, 175.055, 541.806, 78.587, 999.516])
    # robot.move_joint([826.742, 27.667, 323.533, 276.413, 89.976, 999.464])  # safe path to flex b

    # robot.move_joint([849.858, 32.559, 329.015, 266.497, 78.487, 999.523])
    # robot.move_joint([801.032, 5.962, 321.462, 300.193, 82.180, 999.476])
    # robot.pick_plate(exchange_deck_high_narrow)

    # robot.transfer(source=exchange_deck_high_narrow, target=hidex_geraldine_high_nest, source_approach=safe_path_exchange, target_approach=safe_path_hidex, source_plate_rotation="narrow", target_plate_rotation="narrow")
    # robot.transfer(source=hidex_geraldine_high_nest, target=exchange_deck_high_narrow, source_approach=safe_path_hidex, target_approach=safe_path_exchange, source_plate_rotation="narrow", target_plate_rotation="narrow")

    # robot.transfer(source=otflex_deckB, target=exchange_deck_low_narrow, target_approach=safe_path_exchange, source_approach=safe_path_flexB, target_plate_rotation="narrow", source_plate_rotation="narrow")

    # robot.transfer(source=ot)
#
# safe_path_flexA= [[787.882, -2.133, 175.682, 542.847, 78.581, 999.516], [783.291, 62.017, 175.055, 541.806, 78.587, 999.516], [849.853, 58.209, 283.568, 290.376, 78.575, 999.516]]


# robot.pick_plate(exchange_deck_low_narrow)
