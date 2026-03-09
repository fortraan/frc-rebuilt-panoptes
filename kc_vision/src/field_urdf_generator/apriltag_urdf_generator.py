#!/usr/bin/python3
import json
from sys import argv, stderr
from string import Template
from os import linesep
from math import atan2, asin, sin, cos, radians

if len(argv) != 2:
    print("Usage: apriltag_urdf_generator.py <json path>", file=stderr)
    exit(1)

with open(argv[1], "r") as json_file:
    apriltag_data = json.load(json_file)

template = Template(linesep.join((
    '<link name="apriltag_$ID"/>',
    '<joint name="field_to_apriltag_$ID" type="fixed">',
    '    <parent link="field"/>',
    '    <child link="apriltag_$ID"/>',
    '    <origin xyz="$X $Y $Z" rpy="$ROLL $PITCH $YAW"/>',
    '</joint>'
)))

def quat_mult(q0, q1):
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    x = w0 * x1 + x0 * w1 - y0 * z1 + z0 * y1
    y = w0 * y1 + x0 * z1 + y0 * w1 - z0 * x1
    z = w0 * z1 - x0 * y1 + y0 * x1 + z0 * w1
    return w, x, y, z

def quat_to_rpy(q):
    q_w, q_x, q_y, q_z = q
    # source: https://discuss.luxonis.com/d/5453-how-to-convert-quaternions-to-pitchrollyaw/2
    # Roll (x-axis rotation)
    roll = atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))

    # Pitch (y-axis rotation)
    pitch = asin(2 * (q_w * q_y - q_z * q_x))

    # Yaw (z-axis rotation)
    yaw = atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))

    return roll, pitch, yaw

def rpy_to_quat(roll, pitch, yaw):
    w0 = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    x0 = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    y0 = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    z0 = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    return w0, x0, y0, z0

for apriltag in apriltag_data["tags"]:
    quat = (
        apriltag["pose"]["rotation"]["quaternion"]["W"],
        apriltag["pose"]["rotation"]["quaternion"]["X"],
        apriltag["pose"]["rotation"]["quaternion"]["Y"],
        apriltag["pose"]["rotation"]["quaternion"]["Z"]
    )

    # rotate frame of reference of tags
    quat = quat_mult(rpy_to_quat(radians(90), 0, radians(90)), quat)

    roll, pitch, yaw = quat_to_rpy(quat)

    mapping = {
        "ID": apriltag["ID"],
        "X": apriltag["pose"]["translation"]["x"],
        "Y": apriltag["pose"]["translation"]["y"],
        "Z": apriltag["pose"]["translation"]["z"],
        "ROLL": roll,
        "PITCH": pitch,
        "YAW": yaw
    }

    print(template.substitute(mapping))