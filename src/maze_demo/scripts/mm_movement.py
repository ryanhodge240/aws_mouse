#!/usr/bin/env python
'''A node for the navigation of the MicroMouse'''

import sys
import rospy
import numpy as np
from math import cos, sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from img_recognition.msg import Prediction
from tf.transformations import euler_from_quaternion, quaternion_from_euler

np.set_printoptions(precision=2)
foundHeading = False
target_complete = True
target_angle = 0
is_turning = False

current_heading = 0
orbit = 0
laser_sensors = {'l': 0, 'fl': 0, 'f': 0, 'fr': 0, 'r': 0}

linear_vel = 0.2
angular_vel = 0.2
wall_distance = 0.2
wall_distance_forward = 0.30
wall_distance_side = 0.15
rotation_imu =0

inf = float('inf')

left = -1
going_left = -2
right = 1
going_right = 2

# Minimum Safety Range to Wall
wall_distance = 0.2

# Distance where you should be able to turn
turn_distance = 0.5

# Allowable Distance to Stop Moving Forward
wall_distance_forward = 0.15

# Allowable Distance to Stop Moving in Other Directions
wall_distance_side = 0.1

def is_within_tolerance(num1, num2, tolerance):
    '''Check if the first number is within a specified tolerance of another'''
    return abs(num1 - num2) < tolerance

def calculate_lasers_range(data):
    '''Dynamic range intervals'''
    global laser_sensors
    half_pi = np.pi / 2
    initial_angle = 0  
    final_angle = 0   
    if data.angle_min < -half_pi:
        default_min_angle = half_pi / data.angle_increment
        robot_initial_angle = -data.angle_min / data.angle_increment
        initial_angle = robot_initial_angle - default_min_angle
    if data.angle_max > np.pi / 2:
        default_max_angle = half_pi / data.angle_increment
        robot_final_angle = data.angle_max / data.angle_increment
        final_angle = robot_final_angle - default_max_angle

    laser_interval = (len(data.ranges) - initial_angle - final_angle) / 2
    half_laser_interval = laser_interval / 4
    initial_angle = final_angle - 4*laser_interval
#    laser_interval = 45
#    half_laser_interval = 22.5
#    rospy.loginfo("\n\n=------> ref : %s %s  %s\n\n",final_angle, laser_interval,initial_angle)

    interval = [None] * 5
    interval[0] = np.mean(data.ranges[int(initial_angle- half_laser_interval):int(initial_angle+half_laser_interval)])
    interval[0] = min(interval[0], 8.0)
 #   rospy.loginfo("\n\n=------> len : %s  %s\n\n",int(initial_angle- half_laser_interval),int(initial_angle+half_laser_interval))

    for i in range(1, 5):
        dirty_values = data.ranges[int(
            initial_angle + i * laser_interval - half_laser_interval
        ):int(initial_angle + i * laser_interval + half_laser_interval) + 1]
#        rospy.loginfo("=------> lser : %s  %s\n",int(initial_angle + i * laser_interval - half_laser_interval),int(initial_angle + i * laser_interval + half_laser_interval) )
        interval[i] = min(np.mean(np.nan_to_num(dirty_values)), 8.0)

    laser_sensors['r'] = interval[0]
    laser_sensors['fr'] = interval[1]
    laser_sensors['f'] = interval[2]
    laser_sensors['fl'] = interval[3]
    laser_sensors['l'] = interval[4]

def get_velocity_message(turn_left, turn_right, move_forward, turn_around):
    '''Take in instructions for the type of velocity to put on the robot'''
    global laser_sensors, rotation_imu, foundHeading, target_complete, target_angle, is_turning
    target_angle = 0
    linear_velocity = 0
    turn =""
    target_complete = True
    is_turning = True

    if turn_left:
    	#turn left based on current oritenation
        target_angle = rotation_imu + 1.57
        turn += "Turn Left | "
        target_complete = False
    if turn_right:
        target_angle = rotation_imu - 1.57
        turn += "Turn Right | "
        target_complete = False
    if turn_around:
        target_angle = rotation_imu + 3.14
        turn += "Turn Around | "
        target_complete = False
    if move_forward:
        is_turning = False
        linear_velocity = linear_vel
        target_angle = rotation_imu
        turn += "Move Forward   "

    vel_msg = Twist()
    vel_msg.linear.x = linear_velocity
    vel_msg.angular.z = target_angle

    rospy.loginfo("Movement: linear %s angular %s - Direction %s ", linear_velocity, target_angle, turn)

    return vel_msg

def laser_callback(laser_msg):
    '''Callback function for receiving laser messages to publish velocity messages'''
    global laser_sensors, target_complete, is_turning
    turn_left = False
    turn_right = False
    move_forward = False
    turn_around = False

    calculate_lasers_range(laser_msg)

    if is_turning and is_within_tolerance(target_angle, rotation_imu, 0.1):
        target_complete = True
        is_turning = False

    if target_complete:
        print('Target complete')

        if laser_sensors['r'] > turn_distance:
            turn_right = True
        elif laser_sensors['f'] > wall_distance_forward:
            move_forward = True
        elif laser_sensors['l'] > turn_distance:
            turn_left = True
        else:
            turn_around = True

        vel_msg = get_velocity_message(turn_left, turn_right, move_forward, turn_around)

        # Publish to the velocity msg
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        vel_pub.publish(vel_msg)


def imu_callback(imu_msg):
    '''Callback used for the IMU sensor'''
    global rotation_imu, current_heading, foundHeading
    global roll_imu, pitch_imu, yaw_imu
    orientation_q = imu_msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll_imu, pitch_imu, yaw_imu) = euler_from_quaternion(orientation_list)
    if not foundHeading:
        current_heading = yaw_imu
        foundHeading = True
    Kp=38
    rotation_imu =  Kp*(current_heading-yaw_imu)
    print(" \n IMU yaw: ", yaw_imu)

def prediction_callback(prediction):
    '''Callback used for the prediction algorithm'''
    print('Prediction')

def listeners():
    '''Setup all the listeners then wait for ROS to die'''
    # subscribe and read scan message, check laser_callback function
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    # subscribe imu message (oritenation of the robot)
    rospy.Subscriber('/imu', Imu, imu_callback)
    #subscribe to predication topic
    rospy.Subscriber("/prediction", Prediction, prediction_callback)
    rospy.spin()


# Run the MazeRunner
if __name__ == "__main__" :
    rospy.init_node('avoid_wall_1', anonymous=True)
    listeners()
