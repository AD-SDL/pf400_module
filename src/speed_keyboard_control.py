"""Test script to move pf400 using keyboard"""

import argparse

from pynput import keyboard
from pynput.keyboard import Key

from pf400_interface.pf400 import PF400

parser = argparse.ArgumentParser(
    prog="UR keyboard",
    description="control ur arm using keyboard",
)
parser.add_argument("-u", "--url", default="146.137.240.38")
parser.add_argument("-l", "--lin_speed", default=0.01)
parser.add_argument("-r", "--rad_speed", default=0.01)
args = parser.parse_args()
robot = PF400()
move_speed = float(args.lin_speed)
move_speed_radial = float(args.rad_speed)
# print(robot.ur_connection.getl())
velocity = [0, 0, 0]
robot.acceleration = 0.8
flag = False
flag2 = False


def move_robot(direction, sign):
    """Move robot with keyboard"""
    global flag, velocity
    flag = True
    velocity[direction] = sign * move_speed
    # robot.ur_connection.speedl_tool(velocity, 1, 0.1)
    flag = False


def move_robot_j(direction, sign):
    """Move robot joints with keyboard"""
    global flag2
    flag2 = True
    pos = robot.ur_connection.getj()
    print(pos)
    pos[direction] += sign * move_speed_radial
    robot.ur_connection.movej(pos, 1, 1)
    flag2 = False


exit = False


def on_press(key):
    """Press key on keyboard"""

    global flag, flag2
    # print(robot.ur_connection.get_orientation())
    # return
    print("key detected: " + str(key))
    if not flag:
        if key == Key.up:
            move_robot(1, 1)
        elif key == Key.down:
            move_robot(1, -1)
        elif key == Key.right:
            move_robot(0, -1)
        elif key == Key.left:
            move_robot(0, 1)
        elif key.char == "w":
            move_robot(2, 1)
        elif key.char == "s":
            move_robot(2, -1)
        elif key.char == "nn":
            flag2 = True
            # pos = robot.ur_connection.getj()
            # pos[5] = 0
            # robot.ur_connection.movej(pos, 0.8, 0.8)
            robot.move_all_joints_neutral()
            flag2 = False


def on_release(key):
    """Release key on keyboard"""

    global flag
    # print(robot.ur_connection.get_orientation())
    # return
    print("key detected: " + str(key))
    if not flag:
        if key == Key.up:
            move_robot(1, 0)
        elif key == Key.down:
            move_robot(1, 0)
        elif key == Key.right:
            move_robot(0, 0)
        elif key == Key.left:
            move_robot(0, 0)
        elif key.char == "w":
            move_robot(2, 0)
        elif key.char == "s":
            move_robot(2, 0)


listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.daemon = True
listener.start()
while not exit:
    if not flag2:
        robot.move_in_one_axis(
            axis_x=velocity[0], axis_y=velocity[1], axis_z=velocity[2]
        )
