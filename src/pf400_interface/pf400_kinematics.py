"""Handles the kinematics of the PF400 robot arm"""

import math


class KINEMATICS:
    """Class for calculating the forward and inverse kinematics of the PF400 robot arm."""

    """
    Joint coordinates, list of length 6 for PF400 with horizontal rail,
      rail positions units are mm, angles in degrees increasing CCW looking down (-z):
        0  vertical rail position in mm, +z up, range [ 0, 1500 ] mm (for the 1.5m v_rail)
        1  angle of shoulder joint (v_rail to upper_arm), 0 deg perp v_rail, range: [-93, 93] deg
        2  angle of elbow joint (upper_arm to forearm), 0 deg under upper_arm, range: [-168, 168] deg
        3  angle of wrist joint (forearm to end_effector), 0 deg extends forearm, range: [-960, 960] deg
        4  the separation between plate-handling gripper fingers, range: [ ???, ??? ] mm
        5  horizontal rail position in mm, +x perp v_rail, range: [-1000,1000] (for the 2m h_rail)

    Cartesian coordinates, list of length 6:
        0  X       increasing away from the face of the v_rail, parallel to the h_rail
        1  Y       increasing to the left when facing in the +x direction
        2  Z       increasing up along the v_rail
        3  yaw     angle of gripper fingers to +x axis
        4  pitch   fixed at  90 deg
        5  roll    fixed at 180 deg
    """

    def __init__(self) -> None:
        """Constructor for the KINEMATICS class."""
        # Robot arm segment lengths (in millimeters)
        self.shoulder_length = (
            302  # mm    (perhaps rename to upper_arm, as shoulder is a joint)
        )
        self.elbow_length = 289  # mm    (perhaps rename to forearm, as elbo is a joint)
        self.end_effector_length = (
            162  # mm    (???does this length terminate at the Tool Center Point?)
        )

    def forward_kinematics(
        self, joint_states: list[float]
    ) -> tuple[list[float], float, float]:
        """
        Description: Calculates the forward kinematics for a given array of joint_states.
        Parameters:
            - joint_states : 6 joint states of the target location
        Return:
            - cartesian_coordinates: Returns the calculated cartesian coordinates of the given joint states (in millimeters)
            - phi: Phi angle in degrees to be used for inverse kinematics
            - joint_state[5]: The rail length. Needs to be subtracted from x axis if calculated coordinates will be fed into inverse kinematics
        """

        if joint_states[2] > 180:
            adjusted_angle_j3 = (
                joint_states[2] - 360
            )  # Fixing the quadrant on the third joint. Joint 3 range is 10 to 350 instead of -180 to 180
        else:
            adjusted_angle_j3 = joint_states[2]

        # Convert angles to radians
        shoulder_angle = math.radians(joint_states[1])  # Joint 2
        elbow_angle = math.radians(joint_states[2])  # Joint 3
        gripper_angle = math.radians(joint_states[3])  # Joint 4

        x = (
            self.shoulder_length * math.cos(shoulder_angle)
            + self.elbow_length * math.cos(shoulder_angle + elbow_angle)
            + self.end_effector_length
            * math.cos(shoulder_angle + elbow_angle + gripper_angle)
        )
        y = (
            self.shoulder_length * math.sin(shoulder_angle)
            + self.elbow_length * math.sin(shoulder_angle + elbow_angle)
            + self.end_effector_length
            * math.sin(shoulder_angle + elbow_angle + gripper_angle)
        )
        z = joint_states[0]

        phi = (
            math.degrees(shoulder_angle)
            + adjusted_angle_j3
            + math.degrees(gripper_angle)
        )

        if phi > 0 and phi < 540:
            yaw = phi % 360
        elif phi > 540 and phi < 720:
            yaw = phi % 360 - 360
        elif phi > 720 and phi < 900:
            yaw = phi % 720
        elif phi > 900 and phi < 1080:
            yaw = phi % 720 - 720

        cartesian_coordinates = self.get_cartesian_coordinates()

        cartesian_coordinates[0] = round(x, 3) + joint_states[5]
        cartesian_coordinates[1] = round(y, 3)
        cartesian_coordinates[2] = round(z, 3)
        cartesian_coordinates[3] = round(yaw, 3)

        return cartesian_coordinates, round(phi, 3), joint_states[5]

    def inverse_kinematics(
        self,
        cartesian_coordinates: list,
        phi: float,
        rail: float = 0.0,
        get_gripper_length: float = 123.0,
    ) -> list[float]:
        """
        Description: Calculates the inverse kinematics for a given array of cartesian coordinates.
        Parameters:
            - cartesian_coordinates: X/Y/Z Yaw/Pitch/Roll cartesian coordinates.
                                        X axis has to be subtracted from the rail length before feeding into this function!
            - Phi: Phi angle. Phi = Joint_2_angle + Joint_3_angle + Joint_4_angle
            - Rail: Rail length (optional). If provided it will be subtracted from X axis.
        Return:
            - Joint angles: Calculated 6 new joint angles.
        """

        joint1 = cartesian_coordinates[2]
        xe = cartesian_coordinates[0] - rail
        ye = cartesian_coordinates[1]

        if phi < 360:
            phi = cartesian_coordinates[3]
        elif phi > 360 and phi < 540:
            phi = cartesian_coordinates[3] + 360
        elif (phi > 540 and phi < 720) or (phi > 720 and phi < 900):
            phi = cartesian_coordinates[3] + 720
        elif phi > 900 and phi < 1080:
            phi = cartesian_coordinates[3] + 1440

        phi_e = math.radians(phi)

        x_second_joint = xe - self.end_effector_length * math.cos(phi_e)
        y_second_joint = ye - self.end_effector_length * math.sin(phi_e)

        radius = math.sqrt(x_second_joint**2 + y_second_joint**2)
        gamma = math.acos(
            (
                radius * radius
                + self.shoulder_length * self.shoulder_length
                - self.elbow_length * self.elbow_length
            )
            / (2 * radius * self.shoulder_length)
        )

        theta2 = math.pi - math.acos(
            (
                self.shoulder_length * self.shoulder_length
                + self.elbow_length * self.elbow_length
                - radius * radius
            )
            / (2 * self.shoulder_length * self.elbow_length)
        )
        theta1 = math.atan2(y_second_joint, x_second_joint) - gamma
        theta3 = phi_e - theta1 - theta2

        if cartesian_coordinates[1] > 0 or (
            cartesian_coordinates[1] < 0
            and math.degrees(theta1) < 0
            and abs(math.degrees(theta1)) < abs(math.degrees(theta1 + 2 * gamma))
        ):
            # Robot is in the First Quadrant on the coordinate plane (x:+ , y:+)
            joint2 = math.degrees(theta1)
            joint3 = math.degrees(
                theta2
            )  # Adding 360 degrees to Joint 3 to fix the pose.
            joint4 = math.degrees(theta3)

        elif cartesian_coordinates[1] < 0:
            # Robot is in the Forth Quadrant on the coordinate plane (x:+ , y:-)
            # Use the joint angles for Forth Quadrant
            joint2 = math.degrees(theta1 + 2 * gamma)
            joint3 = (
                math.degrees(theta2 * -1) + 360
            )  # Adding 360 degrees to Joint 3 to fix the pose.
            joint4 = math.degrees(theta3 + 2 * (theta2 - gamma))

        return [joint1, joint2, joint3, joint4, get_gripper_length, rail]
