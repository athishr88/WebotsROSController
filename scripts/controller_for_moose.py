#!/usr/bin/env python3.8

from controller import Robot, Camera, Lidar, GPS, Compass, Motor
from potential_field import APF
import math
import numpy as np

# Variables
DISTANCE_TOLERANCE = .5
TARGET_POINTS_SIZE = 13
speeds = [0.0, 0.0]
MAX_SPEED = 7.0
TURN_COEFFICIENT = 4.0

# creating instance of devices
robot = Robot()
front_cam = Camera("camera")
lidar = Lidar("lidar")
gps = GPS("gps")
compass = Compass("compass")
motors = []
motor_names = ["left motor 1",  "left motor 2",  "left motor 3",  "left motor 4",
                          "right motor 1", "right motor 2", "right motor 3", "right motor 4"]

for m in range(8):
    motors.append(robot.getDevice(motor_names[m]))

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
front_cam.enable(timestep)
lidar.enable(timestep)
gps.enable(timestep)
compass.enable(timestep)
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Potential field application
k_att, k_rep = 1.0, 500.0
rr = 3
step_size, max_iters, goal_threashold = .2, 500, .2
step_size_ = 10

start, goal = (14.2, -5.6), (-2.8, 13.2)
obs = [[14.9, 3.77], [11.6, 3.96], [7.66, 4.21], [3.17, 4.15]]

apf = APF(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold)
apf.path_plan()

if apf.is_path_plan_success:
    path = apf.path
    path_ = []
    i = int(step_size_ / step_size)
    while (i < len(path)):
        path_.append(path[i])
        i += int(step_size_ / step_size)

    if path_[-1] != path[-1]:
        path_.append(path[-1])
    print('planed path points:{}'.format(path_))
    print('path plan success')
else:
    print('path plan failed')

print("Path", path_)
# Misc functions
# set left and right motor speed [rad/s]
def robot_set_speed(left, right):
    i = 0
    for i in range(4):
        motors[i + 0].setPosition(float("inf"))
        motors[i + 4].setPosition(float("inf"))
        motors[i + 0].setVelocity(left)
        motors[i + 4].setVelocity(right)

# v = v1-v2
def minus(v1, v2):
    v1 = np.array(v1)
    v2 = np.array(v2)
    return v1-v2

def modulus_double(a, m):
    div = int(a/m)
    r = a - div * m
    if r < 0.0:
        r += m
    return r
    
    
current_target_index = 0        
while robot.step(timestep) != -1:
        # compute the 2D position of the robot and its orientation
    speeds = [0.0, 0.0]
    position_3d = gps.getValues()
    north_3d = compass.getValues()
    position = [position_3d[0], position_3d[1]]
    
    #compute the direction and the distance to the target
    direction = minus(path_[current_target_index], position)
    distance = np.linalg.norm(direction)
    direction /= distance
    
    # compute the error angle
    robot_angle = math.atan2(north_3d[0], north_3d[1])
    target_angle = math.atan2(direction[1], direction[0])
    beta = modulus_double(target_angle - robot_angle, 2.0 * math.pi) - math.pi
    
    
    # move singularity
    if beta > 0:
        beta = math.pi - beta
    else:
        beta = -beta - math.pi
    
    # a target position has been reached
    if distance < DISTANCE_TOLERANCE:
        index_char = "th"
        if current_target_index == 0:
            index_char = "st"
        elif current_target_index == 1:
            index_char = "nd"
        elif current_target_index == 2:
            index_char = "rd"
        print(f"{current_target_index + 1}{index_char} target reached. {current_target_index}\n")
        current_target_index += 1
        current_target_index %= TARGET_POINTS_SIZE
    else:
        speeds[0] = MAX_SPEED - math.pi + TURN_COEFFICIENT * beta
        speeds[1] = MAX_SPEED - math.pi - TURN_COEFFICIENT * beta
        
    
    robot_set_speed(speeds[0], speeds[1])
    if current_target_index > len(path_)-1:
        robot_set_speed(0, 0)
        break
    
    pass
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    

# Enter here exit cleanup code.
