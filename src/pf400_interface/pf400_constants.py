"""Constants for the PF400 driver."""

OUTPUT_CODES = {
    "0": "Success",
    "0 7": "Power off - waiting for power request TRUE",
    "0 20": "Power on - ready to have GPL attach robot",
    "0 21": "21 GPL project attached to robot",
}

MOTION_PROFILES = [
    {
        "speed": 30,
        "speed2": 0,
        "acceleration": 50,
        "deceleration": 50,
        "accelramp": 0.1,
        "decelramp": 0.1,
        "inrange": 0,
        "straight": 0,
    },
    {
        "speed": 120,  # Max 150
        "speed2": 0,
        "acceleration": 100,
        "deceleration": 100,
        "accelramp": 0.1,
        "decelramp": 0.1,
        "inrange": 0,
        "straight": 0,
    },
    {
        "speed": 70,
        "speed2": 0,
        "acceleration": 50,
        "deceleration": 50,
        "accelramp": 0.1,
        "decelramp": 0.1,
        "inrange": 0,
        "straight": -1,
    },
]

ERROR_CODES = {
    "-1009": "*No robot attached*",
    "-1012": "*Joint out-of-range* Set robot joints within their range",
    "-1039": "*Position too close* Robot 1",
    "-1040": "*Position too far* Robot 1",
    "-1042": "*Can't change robot config* Robot 1",
    "-1046": "*Power not enabled*",
    "-1600": "*Power off requested*",
    "-2800": "*Warning Parameter Mismatch*",
    "-2801": "*Warning No Parameters",
    "-2802": "*Warning Illegal move command*",
    "-2803": "*Warning Invalid joint angles*",
    "-2804": "*Warning: Invalid Cartesian coordinate values*",
    "-2805": "*Unknown command*  ",
    "-2806": "*Command Exception*",
    "-2807": "*Warning cannot set Input states*",
    "-2808": "*Not allowed by this thread*",
    "-2809": "*Invalid robot type*",
    "-2810": "*Invalid serial command*",
    "-2811": "*Invalid robot number*",
    "-2812": "*Robot already selected*",
    "-2813": "*Module not initialized*",
    "-2814": "*Invalid location index*",
    "-2816": "*Undefined location*",
    "-2817": "*Undefined profile*",
    "-2818": "*Undefined pallet*",
    "-2819": "*Pallet not supported*",
    "-2820": "*Invalid station index*",
    "-2821": "*Undefined station*",
    "-2822": "*Not a pallet*",
    "-2823": "*Not at pallet origin*",
    "-3122": "*Soft envelope error* Robot 1: 1",
}


# Joint soft stop limits, ordered [z, shoulder, elbow, wrist, gripper, rail].
#
# Read from the controller's own parameter database: 16077 is the maximum soft stop
# and 16078 the minimum. These are physical properties of the arm, set once when the
# robot is commissioned, so they live here beside ERROR_CODES rather than in node
# configuration. Widening them does not give the arm more reach, it just moves where
# the failure happens from a readable refusal to error -1012 after the command has
# already gone out.
#
# The hard stops (16075 and 16076) sit just outside these and are not used for
# checking, so a violation is caught before the arm is anywhere near them.
JOINT_NAMES = ("z", "shoulder", "elbow", "wrist", "gripper", "rail")
JOINT_UNITS = ("mm", "deg", "deg", "deg", "mm", "mm")
JOINT_SOFT_LIMIT_MIN = (1.5, -93.0, 12.0, -960.0, 69.0, -1000.0)
JOINT_SOFT_LIMIT_MAX = (1161.5, 93.0, 348.0, 960.0, 134.0, 1000.0)
