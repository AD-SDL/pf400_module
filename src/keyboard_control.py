"""Test script to move pf400 using keyboard"""

import argparse
import math

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
print(args.url)
robot = PF400(args.url)
move_speed = float(args.lin_speed)
move_speed_radial = float(args.rad_speed)
# print(robot.ur_connection.getl())
robot.velocity = 0.8
robot.acceleration = 0.8
flag = False


def move_robot(direction, sign):
    """Move robot with keyboard"""
    global flag
    flag = True
    pos = [0, 0, 0]
    pos[direction] = move_speed * sign
    robot.move_in_one_axis(axis_x=pos[0], axis_y=pos[1], axis_z=pos[2])
    flag = False


def move_robot_j(direction, sign):
    """Move robot joints with keyboard"""
    global flag
    flag = True
    pos = robot.ur_connection.getj()
    print(pos)
    pos[direction] += sign * move_speed_radial
    robot.ur_connection.movej(pos, 1, 1)
    flag = False


exit = False


def on_press(key):
    """Press key on keyboard"""

    global flag
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
        elif key.char == "8":
            pos = robot.get_cartesian_coordinates()
            pos[3] = 90
            robot.move_cartesian(pos)
        elif key.char == "2":
            pos = robot.get_cartesian_coordinates()
            print(pos)
            pos[3] = 270
            robot.move_cartesian(pos)
        elif key.char == "4":
            pos = robot.get_cartesian_coordinates()
            print(pos)
            pos[3] = 0
            robot.move_cartesian(pos)
        elif key.char == "6":
            pos = robot.get_cartesian_coordinates()
            print(pos)
            pos[3] = 180
            robot.move_cartesian(pos)
        # elif key.char == "7":
        #   move_robot(5, 1)
        # elif key.char == "9":
        #   move_robot(5, -1)
        elif key.char == "l":
            # pos = robot.ur_connection.getj()
            # pos[5] = 0
            # robot.ur_connection.movej(pos, 0.8, 0.8)
            pos = robot.ur_connection.getl()

            pos[3] = math.pi / 2
            pos[4] = 0
            pos[5] = 0

            robot.ur_connection.movel(pos, acc=1, vel=1)
        elif key.char == "f":
            # pos = robot.ur_connection.getj()
            # pos[5] = 0
            # robot.ur_connection.movej(pos, 0.8, 0.8)
            pos = robot.ur_connection.set_freedrive(True, 600)


listener = keyboard.Listener(on_press=on_press)
listener.daemon = False
listener.start()
# while not exit:
#   pos = robot.ur_connection.getl()
#   print(dirs)
#   pos = numpy.array(pos) + numpy.array(dirs)*0.01
#   robot.ur_connection.movel(pos.tolist())
