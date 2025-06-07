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
    #predefined route for the first food box
    predefined_route = [
        [[-153.24, -446.46, 600], [0.086, -0.8, -0.56, -0.13]],#starting point
        [[-153.24, -446.46, 360], [0.086, -0.8, -0.56, -0.13]],#go inside the pot
        [[-153.24, -371.71, 346.55], [0.086, 0.81, 0.56, -0.11]],#grasp food
        [[-153.24, -300.71, 346.55], [0.127,0.8, 0.55, -0.2]],#rotate
        [[-153.24, -300.71, 450.6], [0.127,0.8, 0.55, -0.2]],# go up
        [[-104.72, -347.71, 471.6], [0.232,0.698, 0.57, -0.36]],#rotate
        [[180,-583, 285.3], [0.296, 0.686, 0.59, -0.3]],#go into food box
        [[180, -583, 285.3], [0.157, 0.267,-0.866,0.39]]#rotate
        
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
                # elif int(data[1]) == 1: self.joints.append([a[2:5], a[5:]])
        finally:
            s.shutdown(socket.SHUT_RDWR)

    def set_units(self, linear, angular):
        units_l = {"millimeters": 1.0, "meters": 1000.0, "inches": 25.4}
        units_a = {"degrees": 1.0, "radians": 57.2957795}
        self.scale_linear = units_l[linear]
        self.scale_angle = units_a[angular]

    def set_cartesian(self, pose):
        """
        Executes a move immediately from the current pose,
        to 'pose', with units of millimeters.
        """
        msg = "01 " + self.format_pose(pose)
        return self.send(msg)

    def set_joints(self, joints):
        """
        Executes a move immediately, from current joint angles,
        to 'joints', in degrees.
        """
        if len(joints) != 6:
            return False
        msg = "02 "
        for joint in joints:
            msg += format(joint * self.scale_angle, "+08.2f") + " "
        msg += "#"
        return self.send(msg)

    def get_cartesian(self):
        """
        Returns the current pose of the robot, in millimeters
        """
        msg = "03 #"
        data = self.send(msg).split()
        r = [float(s) for s in data]
        return [r[2:5], r[5:9]]

    def get_joints(self):
        """
        Returns the current angles of the robots joints, in degrees.
        """
        msg = "04 #"
        data = self.send(msg).split()
        return [float(s) / self.scale_angle for s in data[2:8]]

    def get_external_axis(self):
        """
        If you have an external axis connected to your robot controller
        (such as a FlexLifter 600, google it), this returns the joint angles
        """
        msg = "05 #"
        data = self.send(msg).split()
        return [float(s) for s in data[2:8]]

    def get_robotinfo(self):
        """
        Returns a robot-unique string, with things such as the
        robot's model number.
        Example output from an IRB 2400:
        ['24-53243', 'ROBOTWARE_5.12.1021.01', '2400/16 Type B']
        """
        msg = "98 #"
        data = str(self.send(msg))[5:].split("*")
        log.debug("get_robotinfo result: %s", str(data))
        return data

    def set_tool(self, tool=[[0, 0, 0], [1, 0, 0, 0]]):
        """
        Sets the tool centerpoint (TCP) of the robot.
        When you command a cartesian move,
        it aligns the TCP frame with the requested frame.

        Offsets are from tool0, which is defined at the intersection of the
        tool flange center axis and the flange face.
        """
        msg = "06 " + self.format_pose(tool)
        self.send(msg)
        self.tool = tool

    def load_json_tool(self, file_obj):
        if file_obj.__class__.__name__ == "str":
            file_obj = open(file_obj, "rb")
        tool = check_coordinates(json.load(file_obj))
        self.set_tool(tool)

    def get_tool(self):
        log.debug("get_tool returning: %s", str(self.tool))
        return self.tool

    def set_workobject(self, work_obj=[[0, 0, 0], [1, 0, 0, 0]]):
        """
        The workobject is a local coordinate frame you can define on the robot,
        then subsequent cartesian moves will be in this coordinate frame.
        """
        msg = "07 " + self.format_pose(work_obj)
        self.send(msg)

    def set_speed(self, speed=[100, 50, 50, 50]):
        """
        speed: [robot TCP linear speed (mm/s), TCP orientation speed (deg/s),
                external axis linear, external axis orientation]
        """

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
        """
        Sets the motion zone of the robot. This can also be thought of as
        the flyby zone, AKA if the robot is going from point A -> B -> C,
        how close do we have to pass by B to get to C

        zone_key: uses values from RAPID handbook (stored here in zone_dict)
        with keys 'z*', you should probably use these

        point_motion: go to point exactly, and stop briefly before moving on

        manual_zone = [pzone_tcp, pzone_ori, zone_ori]
        pzone_tcp: mm, radius from goal where robot tool centerpoint 
                   is not rigidly constrained
        pzone_ori: mm, radius from goal where robot tool orientation 
                   is not rigidly constrained
        zone_ori: degrees, zone size for the tool reorientation
        """

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
        """
        Appends single pose to the remote buffer
        Move will execute at current speed (which you can change between buffer_add calls)
        """
        msg = "30 " + self.format_pose(pose)
        self.send(msg)

    def buffer_set(self, pose_list):
        """
        Adds every pose in pose_list to the remote buffer
        """
        self.clear_buffer()
        for pose in pose_list:
            self.buffer_add(pose)
        if self.buffer_len() == len(pose_list):
            log.debug("Successfully added %i poses to remote buffer", len(pose_list))
            return True
        else:
            log.warn("Failed to add poses to remote buffer!")
            self.clear_buffer()
            return False

    def clear_buffer(self):
        msg = "31 #"
        data = self.send(msg)
        if self.buffer_len() != 0:
            log.warn("clear_buffer failed! buffer_len: %i", self.buffer_len())
            raise NameError("clear_buffer failed!")
        return data

    def buffer_len(self):
        """
        Returns the length (number of poses stored) of the remote buffer
        """
        msg = "32 #"
        data = self.send(msg).split()
        return int(float(data[2]))

    def buffer_execute(self):
        """
        Immediately execute linear moves to every pose in the remote buffer.
        """
        msg = "33 #"
        return self.send(msg)

    def set_external_axis(self, axis_unscaled=[-550, 0, 0, 0, 0, 0]):
        if len(axis_unscaled) != 6:
            return False
        msg = "34 "
        for axis in axis_unscaled:
            msg += format(axis, "+08.2f") + " "
        msg += "#"
        return self.send(msg)

    def move_circular(self, pose_onarc, pose_end):
        """
        Executes a movement in a circular path from current position,
        through pose_onarc, to pose_end
        """
        msg_0 = "35 " + self.format_pose(pose_onarc)
        msg_1 = "36 " + self.format_pose(pose_end)

        data = self.send(msg_0).split()
        if data[1] != "1":
            log.warn("move_circular incorrect response, bailing!")
            return False
        return self.send(msg_1)

    def set_dio(self, value, id=0):
        """
        A function to set a physical DIO line on the robot.
        For this to work you're going to need to edit the RAPID function
        and fill in the DIO you want this to switch.
        """
        msg = "97 " + str(int(bool(value))) + " #"
        return
        # return self.send(msg)

    def send(self, message, wait_for_response=True):
        """
        Send a formatted message to the robot socket.
        if wait_for_response, we wait for the response and return it
        """
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

    def close(self):
        self.send("99 #", False)
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        log.info("Disconnected from ABB robot.")

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def execute_predefined_route_repeatedly(self, box_count):
       
        print("Filling {} boxes with 2 repetitions per box.".format(box_count))
        base_route = self.predefined_route.copy()  # Make a copy of the original route
        cumulative_z_offset = 0

        for box_index in range(box_count):  # Loop through each box
            print("Filling box {} of {}.".format(box_index + 1, box_count))
            log.info("Starting box %d of %d", box_index + 1, box_count)

            # Perform stirring motion if there is more than one box and this is not the first box
            if box_index > 0:  # Stir only before moving to the next box, not the first one
                print("Performing stirring motion before taking food for the next box.")
                self.set_cartesian([[-171.24, -454, 600], [0, 0, -1, 0]]) #going above the pot near its center
                self.move_in_circle()  # Call the stirring motion function
            
            for repetition in range(3):  # Ensure 3 repetitions per box
                print("Repetition {} of 3 for box {}.".format(repetition + 1, box_index + 1))
                log.info("Starting repetition %d of 3 for box %d", repetition + 1, box_index + 1)

                # Modify the route for the current repetition
                adjusted_route = []
                for point_index, point in enumerate(base_route):
                    adjusted_point = [point[0][:], point[1][:]]

                    # Adjust Z values for points 2,3 and 4 (inside the pot, fix rot and grasping)
                    if point_index == 1 or point_index == 2 or point_index ==3:  
                        adjusted_point[0][2] -= cumulative_z_offset  # Apply cumulative Z offset

                    # Adjust Y values for points 6 and 7 (container approach and rotation)
                    if point_index == 6 or point_index == 7:  # Points 7 and 8
                        adjusted_point[0][1] += 210 * box_index  # Offset Y by +210 mm for each box

                    adjusted_route.append(adjusted_point)

                # Send adjusted route to the robot
                self.clear_buffer()  # Clear the buffer at the start of each repetition
                self.buffer_set(adjusted_route)  # Add the adjusted route to the buffer
                self.buffer_execute()  # Execute the entire buffer

                # Log the movement
                for index, pose in enumerate(adjusted_route, start=1):
                    log.info("Point %d reached: Cartesian: %s, Orientation: %s", index, pose[0], pose[1])
                    print("Point {} reached: Cartesian: {}, Orientation: {}.".format(index, pose[0], pose[1]))

                # Update the cumulative Z offset after each repetition
                cumulative_z_offset += 3  # Increment Z offset for the next repetition

            log.info("Completed box %d of %d", box_index + 1, box_count)
            print("Completed filling box {}.".format(box_index + 1))
    
    def rotate_quaternion(self, quaternion, angle):
        """
        Rotate a quaternion around the Z-axis by a specified angle.
        :param quaternion: List [q1, q2, q3, q4] representing the initial quaternion.
        :param angle: Angle (in radians) to rotate.
        :return: List [q1, q2, q3, q4] representing the rotated quaternion.
        """
        angle = -angle # Reverse the rotation direction

        # Quaternion representing the rotation around the Z-axis
        rotation = [
            math.cos(angle / 2),  # w
            0,                   # x
            0,                   # y
            math.sin(angle / 2)  # z
        ]

        # Multiply the two quaternions
        w1, x1, y1, z1 = quaternion
        w2, x2, y2, z2 = rotation
        rotated_quaternion = [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,  # w
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,  # x
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,  # y
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2   # z
        ]

        return rotated_quaternion

    def move_in_circle(self, center=[-150.24, -480, 450], radius=80, num_points=36):
        """
        Moves the robot arm in a perfect circle while ensuring it always faces the center,
        while respecting the robot's 360° rotation limit.
        
        Parameters:
            center (list): [x, y, z] coordinates of the circle's center.
            radius (float): Radius of the circle in mm.
            num_points (int): Number of points to define the circular path.
        """
        # Move to the starting point first
        start_angle = math.pi / 2  # Define the initial angle as 0 (starting point)
        self.set_cartesian([center, [0, 0, -1, 0]])
        start_x = center[0] + radius * math.cos(start_angle)
        start_y = center[1] + radius * math.sin(start_angle)
        start_z = center[2]  # starting stirring inside the pot

        # Calculate initial orientation facing the center
        dx = center[0] - start_x
        dy = center[1] - start_y
        facing_angle = math.atan2(dy, dx)
        base_quaternion = [0, 0, -1, 0]  # Assumes downward orientation by default
        start_quaternion = self.rotate_quaternion(base_quaternion, facing_angle)

        # Move to the starting point
        self.set_cartesian([[start_x, start_y, start_z], start_quaternion])
        print(f"Moved to starting point: {[start_x, start_y, start_z]}")

        # Generate the circle points
        circle_points = []
        prev_angle = math.pi / 2  # Track the previous angle to ensure smooth rotations
        for i in range(num_points):
            angle = (-2 * math.pi * i / num_points) + math.pi / 2  # Divide the circle into equal angles
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            z = center[2] -100  # Maintain constant z-level

            # Calculate orientation to face the center
            dx = center[0] - x
            dy = center[1] - y
            facing_angle = math.atan2(dy, dx)  # Angle to face the center

            # Ensure orientation stays within 360° limits
            delta_angle = facing_angle - prev_angle
            if delta_angle > math.pi:  # Correct for overshoot
                delta_angle -= 2 * math.pi
            elif delta_angle < -math.pi:  # Correct for undershoot
                delta_angle += 2 * math.pi
            limited_facing_angle = prev_angle + delta_angle
            prev_angle = limited_facing_angle  # Update for the next iteration

            # Adjust the quaternion to face downward
            facing_quaternion = self.rotate_quaternion(base_quaternion, limited_facing_angle)  # Adjust for center-facing angle

            circle_points.append([[x, y, z], facing_quaternion])

        # Move the robot along the circular path
        self.clear_buffer()
        for point in circle_points:
            self.buffer_add(point)  # Add each point to the robot's motion buffer
        self.buffer_execute()  # Execute the buffered motion
        print("Circular motion executed.")

        self.set_cartesian([[-171.24, -454, 500], [0, 0, -1, 0]]) # go near th pot's center

        # reset the sixth joint to 0 degrees to avoid stagnation
        current_joints = self.get_joints()
        current_joints[5] = 0  
        self.set_joints(current_joints)

def check_coordinates(coordinates):
    if (
        (len(coordinates) == 2)
        and (len(coordinates[0]) == 3)
        and (len(coordinates[1]) == 4)
    ):
        return coordinates
    elif len(coordinates) == 7:
        return [coordinates[0:3], coordinates[3:7]]
    log.warn("Received malformed coordinate: %s", str(coordinates))
    raise NameError("Malformed coordinate!")

if __name__ == "__main__":
    formatter = logging.Formatter(
        "[%(asctime)s] %(levelname)-7s (%(filename)s:%(lineno)3s) %(message)s",
        "%Y-%m-%d %H:%M:%S",
    )
    handler_stream = logging.StreamHandler()
    handler_stream.setFormatter(formatter)
    handler_stream.setLevel(logging.DEBUG)
    log = logging.getLogger("abb")
    log.setLevel(logging.DEBUG)
    log.addHandler(handler_stream)

    robot = Robot(ip="192.168.125.1", port_motion=5000)
    try:
        # Execute predefined route repeatedly
        robot.execute_predefined_route_repeatedly()  # Example repetitions
        print("Starting continuous circular motion...")
        robot.perform_continuous_circle_motion()
    finally:
        robot.close()
