#!/usr/bin/env python3.8
# license removed for brevity
import rospy
from std_msgs.msg import String
import os
from potential_field import APF
import math
import numpy as np
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# os.environ['LD_LIBRARY_PATH'] = "/usr/local/webots/lib/controller"
# os.environ['PYTHONPATH'] = "/usr/local/webots/lib/controller/python38"

from controller import Robot, Camera, Lidar, GPS, Compass, Motor, Keyboard

class ControllerProgram():
    """
    This is the object that provides feedback about the 
    robot traversing world 
    """
    def __init__(self):
        # Variables
        self.DISTANCE_TOLERANCE = .5
        self.TARGET_POINTS_SIZE = 13
        self.speeds = [0.0, 0.0]
        self.MAX_SPEED = 7.0
        self.TURN_COEFFICIENT = 4.0

        # Misc variables
        self.AUTOPILOT = True
        self.OLD_AUTOPILOT = True
        self.old_key = -1
        self.traverse_path =[ (-4.209318, 9.147717),   (0.946812, 9.404304),
                            (0.175989, -1.784311), (-2.805353, -8.829694),  
                            (-3.846730, -15.602851),(-4.394915, -24.550777),
                            (-1.701877, -33.617226), (-4.394915, -24.550777),
                            (-3.846730, -15.602851), (-2.805353, -8.829694),
                            (0.175989, -1.784311),   (0.946812, 9.404304),
                            (-7.930821, 6.421292)]
 

        # creating instance of devices
        self.robot = Robot()
        self.front_cam = Camera("camera")
        self.lidar = Lidar("lidar")
        self.gps = GPS("gps")
        self.compass = Compass("compass")
        self.keyboard = Keyboard()
        self.motors = []
        self.motor_names = ["left motor 1",  "left motor 2",  "left motor 3",  
                            "left motor 4", "right motor 1", "right motor 2",
                            "right motor 3", "right motor 4"]

        for m in range(8):
            self.motors.append(self.robot.getDevice(self.motor_names[m]))

        # get the time step of the current world.
        self.timestep = int(self.robot.getBasicTimeStep())

        # Enable all devices
        self.front_cam.enable(self.timestep)
        self.lidar.enable(self.timestep)
        self.gps.enable(self.timestep)
        self.compass.enable(self.timestep)
        self.keyboard.enable(self.timestep)
    
    def robot_set_speed(self, left, right):
        i = 0
        for i in range(4):
            self.motors[i + 0].setPosition(float("inf"))
            self.motors[i + 4].setPosition(float("inf"))
            self.motors[i + 0].setVelocity(left)
            self.motors[i + 4].setVelocity(right)

    # v = v1-v2
    def minus(self, v1, v2):
        v1 = np.array(v1)
        v2 = np.array(v2)
        return v1-v2

    def modulus_double(self, a, m):
        div = int(a/m)
        r = a - div * m
        if r < 0.0:
            r += m
        return r

    def autopilot_mode(self):
        rospy.loginfo("Autopilot mode started")
        self.speeds = [0.0, 0.0]
        position_3d = self.gps.getValues()
        north_3d = self.compass.getValues()
        position = [position_3d[0], position_3d[1]]
        
        #compute the direction and the distance to the target
        direction = self.minus(self.traverse_path[current_target_index], position)
        distance = np.linalg.norm(direction)
        direction /= distance
        
        # compute the error angle
        robot_angle = math.atan2(north_3d[0], north_3d[1])
        target_angle = math.atan2(direction[1], direction[0])
        beta = self.modulus_double(target_angle - robot_angle, 2.0 * math.pi) - math.pi
        
        
        # move singularity
        if beta > 0:
            beta = math.pi - beta
        else:
            beta = -beta - math.pi
        
        # a target position has been reached
        if distance < self.DISTANCE_TOLERANCE:
            index_char = "th"
            if current_target_index == 0:
                index_char = "st"
            elif current_target_index == 1:
                index_char = "nd"
            elif current_target_index == 2:
                index_char = "rd"
            rospy.loginfo(f"{current_target_index + 1}{index_char} target reached. {current_target_index}\n")
            current_target_index += 1
            current_target_index %= self.TARGET_POINTS_SIZE
        else:
            self.speeds[0] = self.MAX_SPEED - math.pi + self.TURN_COEFFICIENT * beta
            self.speeds[1] = self.MAX_SPEED - math.pi - self.TURN_COEFFICIENT * beta
            
        
        self.robot_set_speed(self.speeds[0], self.speeds[1])
        if current_target_index > len(self.traverse_path)-1:
            self.robot_set_speed(0, 0)
            self.AUTOPILOT = False
        pass
    
    def check_keyboard(self):
        LEFT = 0
        RIGHT = 1
        speeds = [0.0, 0.0]
        key = self.keyboard.getKey()

        if key >= 0:
            if key == self.keyboard.UP:
                speeds[LEFT] = self.MAX_SPEED
                speeds[RIGHT] = self.MAX_SPEED
                self.AUTOPILOT = False

            elif key == self.keyboard.DOWN:
                speeds[LEFT] = -self.MAX_SPEED
                speeds[RIGHT] = -self.MAX_SPEED
                self.AUTOPILOT = False

            elif key == self.keyboard.RIGHT:
                speeds[LEFT] = self.MAX_SPEED
                speeds[RIGHT] = -self.MAX_SPEED
                self.AUTOPILOT = False

            elif key == self.keyboard.LEFT:
                speeds[LEFT] = -self.MAX_SPEED;
                speeds[RIGHT] = self.MAX_SPEED;
                self.AUTOPILOT = False

            elif key == 'A':
                if key != self.old_key:  # perform this action just once
                    self.AUTOPILOT = not self.AUTOPILOT
        
        if self.AUTOPILOT != self.OLD_AUTOPILOT:
            self.OLD_AUTOPILOT = self.AUTOPILOT
            if self.AUTOPILOT:
                print("auto control\n")
            else:
                print("manual control\n")

        self.robot_set_speed(speeds[LEFT], speeds[RIGHT])
        self.old_key = key

    def start(self):
        while True:
            self.check_keyboard()
            if self.AUTOPILOT:
                self.autopilot_mode()


class ROSInterface():
    def __init__(self, rostopic, rosnode):
        self.pub = rospy.Publisher(rostopic, String, queue_size=10)
        rospy.init_node(rosnode, anonymous=False)
        self.rate = rospy.Rate(10) #10hz
    
    def start_node(self):
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.pub.publish(hello_str)
            self.rate.sleep()


try:
    talker = ROSInterface("chatter", "talker")
    talker.start_node()
    robot_controller = ControllerProgram()
except rospy.ROSInterruptException:
    pass