
# Modified by Ofir Dvantman – June 2025
# Based on the original abb.py from https://github.com/milistu/open_abb/tree/fuerte-devel
# Part of my final project: Automated Food Serving with ABB IRB 1200 (simulation)
#
# Key changes:
# - Added `execute_predefined_route_repeatedly` for box-filling sequence
# - Added `move_in_circle` for circular stirring inside pot
# - Added `rotate_quaternion` for orientation correction
# - Introduced `predefined_route` with food-serving coordinates
# - Updated `__main__` to run full simulation behavior

import socket
import json
import time
import inspect
from threading import Thread
from collections import deque
import logging
import math

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())

class Robot:
    predefined_route = [
        [[-153.24, -446.46, 600], [0.086, -0.8, -0.56, -0.13]],
        [[-153.24, -446.46, 360], [0.086, -0.8, -0.56, -0.13]],
        [[-153.24, -371.71, 346.55], [0.086, 0.81, 0.56, -0.11]],
        [[-153.24, -300.71, 346.55], [0.127,0.8, 0.55, -0.2]],
        [[-153.24, -300.71, 450.6], [0.127,0.8, 0.55, -0.2]],
        [[-104.72, -347.71, 471.6], [0.232,0.698, 0.57, -0.36]],
        [[180,-583, 285.3], [0.296, 0.686, 0.59, -0.3]],
        [[180, -583, 285.3], [0.157, 0.267,-0.866,0.39]]
    ]

    def __init__(self, ip="192.168.125.1", port_motion=5000, port_logger=5001):
        self.delay = 0.08
        self.connect_motion((ip, port_motion))
        self.ip = ip
        self.port_motion = port_motion
        self.set_units("millimeters", "degrees")
        self.set_tool()
        self.set_workobject()
        self.set_speed()
        self.set_zone()

    def connect_motion(self, remote):
        log.info("Attempting to connect to robot motion server at %s", str(remote))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.5)
        self.sock.connect(remote)
        self.sock.settimeout(None)
        log.info("Connected to robot motion server at %s", str(remote))

    def connect_logger(self, remote, maxlen=None):
        self.pose = deque(maxlen=maxlen)
        self.joints = deque(maxlen=maxlen)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(remote)
        s.setblocking(1)
        try:
            while True:
                data = map(float, s.recv(4096).split())
                if int(data[1]) == 0:
                    self.pose.append([data[2:5], data[5:]])
        finally:
            s.shutdown(socket.SHUT_RDWR)

    def set_units(self, linear, angular):
        units_l = {"millimeters": 1.0, "meters": 1000.0, "inches": 25.4}
        units_a = {"degrees": 1.0, "radians": 57.2957795}
        self.scale_linear = units_l[linear]
        self.scale_angle = units_a[angular]

    def set_cartesian(self, pose):
        msg = "01 " + self.format_pose(pose)
        return self.send(msg)

    def set_joints(self, joints):
        if len(joints) != 6:
            return False
        msg = "02 "
        for joint in joints:
            msg += format(joint * self.scale_angle, "+08.2f") + " "
        msg += "#"
        return self.send(msg)

    def get_cartesian(self):
        msg = "03 #"
        data = self.send(msg).split()
        r = [float(s) for s in data]
        return [r[2:5], r[5:9]]

    def get_joints(self):
        msg = "04 #"
        data = self.send(msg).split()
        return [float(s) / self.scale_angle for s in data[2:8]]

    def get_robotinfo(self):
        msg = "98 #"
        data = str(self.send(msg))[5:].split("*")
        return data

    def set_tool(self, tool=[[0, 0, 0], [1, 0, 0, 0]]):
        msg = "06 " + self.format_pose(tool)
        self.send(msg)
        self.tool = tool

    def set_workobject(self, work_obj=[[0, 0, 0], [1, 0, 0, 0]]):
        msg = "07 " + self.format_pose(work_obj)
        self.send(msg)

    def set_speed(self, speed=[100, 50, 50, 50]):
        if len(speed) != 4:
            return False
        msg = "08 "
        msg += format(speed[0], "+08.1f") + " "
        msg += format(speed[1], "+08.2f") + " "
        msg += format(speed[2], "+08.1f") + " "
        msg += format(speed[3], "+08.2f") + " #"
        self.send(msg)

    def set_zone(self, zone_key="z0", point_motion=False, manual_zone=[]):
        zone_dict = {
            "z0": [0.3, 0.3, 0.03],
            "z1": [1, 1, 0.1],
            "z5": [5, 8, 0.8],
            "z10": [10, 15, 1.5],
            "z15": [15, 23, 2.3],
            "z20": [20, 30, 3],
            "z30": [30, 45, 4.5],
            "z50": [50, 75, 7.5],
            "z100": [100, 150, 15],
            "z200": [200, 300, 30],
        }
        if point_motion:
            zone = [0, 0, 0]
        elif len(manual_zone) == 3:
            zone = manual_zone
        elif zone_key in zone_dict.keys():
            zone = zone_dict[zone_key]
        else:
            return False
        msg = "09 "
        msg += str(int(point_motion)) + " "
        msg += format(zone[0], "+08.4f") + " "
        msg += format(zone[1], "+08.4f") + " "
        msg += format(zone[2], "+08.4f") + " #"
        self.send(msg)

    def buffer_add(self, pose):
        msg = "30 " + self.format_pose(pose)
        self.send(msg)

    def clear_buffer(self):
        msg = "31 #"
        return self.send(msg)

    def buffer_len(self):
        msg = "32 #"
        data = self.send(msg).split()
        return int(float(data[2]))

    def buffer_set(self, pose_list):
        self.clear_buffer()
        for pose in pose_list:
            self.buffer_add(pose)

    def buffer_execute(self):
        msg = "33 #"
        return self.send(msg)

    def send(self, message, wait_for_response=True):
        caller = inspect.stack()[1][3]
        log.debug("%-14s sending: %s", caller, message)
        self.sock.sendto(message.encode(), (self.ip, self.port_motion))
        time.sleep(self.delay)
        if not wait_for_response:
            return
        data = self.sock.recv(4096)
        log.debug("%-14s received: %s", caller, data)
        return data

    def format_pose(self, pose):
        pose = check_coordinates(pose)
        msg = ""
        for cartesian in pose[0]:
            msg += format(cartesian * self.scale_linear, "+08.1f") + " "
        for quaternion in pose[1]:
            msg += format(quaternion, "+08.5f") + " "
        msg += "#"
        return msg

    def rotate_quaternion(self, quaternion, angle):
        angle = -angle
        rotation = [
            math.cos(angle / 2),
            0,
            0,
            math.sin(angle / 2)
        ]
        w1, x1, y1, z1 = quaternion
        w2, x2, y2, z2 = rotation
        return [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ]

def check_coordinates(coordinates):
    if (
        (len(coordinates) == 2)
        and (len(coordinates[0]) == 3)
        and (len(coordinates[1]) == 4)
    ):
        return coordinates
    elif len(coordinates) == 7:
        return [coordinates[0:3], coordinates[3:7]]
    raise NameError("Malformed coordinate!")

if __name__ == "__main__":
    formatter = logging.Formatter("[%(asctime)s] %(levelname)-7s (%(filename)s:%(lineno)3s) %(message)s", "%Y-%m-%d %H:%M:%S")
    handler_stream = logging.StreamHandler()
    handler_stream.setFormatter(formatter)
    handler_stream.setLevel(logging.DEBUG)
    log = logging.getLogger("abb")
    log.setLevel(logging.DEBUG)
    log.addHandler(handler_stream)

    robot = Robot(ip="192.168.125.1", port_motion=5000)
    try:
        robot.execute_predefined_route_repeatedly(box_count=3)
        robot.move_in_circle()
    finally:
        robot.close()
