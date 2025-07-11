"""Test script to move pf400 using keyboard"""

import argparse
import sys
import threading

from pynput import keyboard
from pynput.keyboard import Key

from pf400_interface.pf400 import PF400

parser = argparse.ArgumentParser(
    prog="PF400 Keyboard Control",
    description="Control a PF400 arm using keyboard",
)
parser.add_argument("-h", "--host", default="146.137.240.38")
parser.add_argument(
    "-p", "--port", default=10100, type=int, help="Port to connect to the PF400"
)
parser.add_argument("-s", "--speed", default=0.01)
args = parser.parse_args()
robot = PF400(host=args.host, port=args.port)
move_speed = args.speed
velocity = [0, 0, 0]
lock = threading.Lock()
inner_lock = threading.Lock()


def move_robot(direction: int, sign: int) -> None:
    """Move robot with keyboard"""
    with lock:
        velocity[direction] = sign * move_speed


def on_press(key: Key) -> None:
    """Press key on keyboard"""

    with lock:
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
        elif key.char == "n":
            with inner_lock:
                robot.move_all_joints_neutral()
        elif key.char == "q":
            listener.stop()
            sys.exit()


def on_release(key: Key) -> None:
    """Release key on keyboard"""

    with lock:
        if key in (Key.up, Key.down):
            move_robot(1, 0)
        elif key in (Key.right, Key.left):
            move_robot(0, 0)
        elif key.char in {"w", "s"}:
            move_robot(2, 0)


listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.daemon = True
listener.start()
while True:
    with inner_lock:
        robot.move_in_one_axis(
            axis_x=velocity[0], axis_y=velocity[1], axis_z=velocity[2]
        )
