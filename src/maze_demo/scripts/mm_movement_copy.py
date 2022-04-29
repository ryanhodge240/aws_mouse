#!/usr/bin/env python
'''A node for the navigation of the MicroMouse'''

from __future__ import division
import sys
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from img_recognition.msg import Prediction
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from mouse_common.utils import Micromouse_Node

np.set_printoptions(precision = 2)

# Minimum distance from walls on sides and in front
WALL_DIST = 0.14
FORWARD_WALL_DIST = 0.2

# Normal linear and angular velocity speed
LIN_VEL = 0.3
ANG_VEL = 0.3

# The tolerance for turning (degrees)
TOL = 4

def is_within_tolerance(num1, num2, tolerance):
    '''Check if the first number is within a specified tolerance of another'''
    return abs(num1 - num2) < tolerance

class MazeRunner(object):
    '''MazeRunner python class object for running the MicroMouse'''

    def __init__(self):
        # Start Mouse Node
        self.found_heading = False
        self.current_heading = 0
        self.laser_sensors = {'l': 0, 'fl': 0, 'f': 0, 'fr': 0, 'r': 0}
        self.rotation_imu = 0

        # Initialize publisher
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize subscribers
        self.listeners()

        # Require ctrl+c to cancel process
        rospy.spin()

    def listeners(self):
        '''Setup all the listeners then wait for ROS to die'''
        # subscribe and read scan message, check laser_callback function
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        # subscribe imu message (oritenation of the robot)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        #subscribe to predication topic
        rospy.Subscriber("/prediction", Prediction, self.prediction_callback)

    def imu_callback(self, imu_msg):
        '''Callback used for the IMU sensor'''
        orientation_q = imu_msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw_imu) = euler_from_quaternion(orientation_list)
        if not self.found_heading:
            self.current_heading = yaw_imu
            self.found_heading = True
        k_p = 38
        self.rotation_imu = k_p * (self.current_heading-yaw_imu)
        # print(" \n IMU yaw: ", yaw_imu)

    def prediction_callback(self, prediction):
        '''Callback used for the prediction algorithm'''
        print('Prediction')

    def laser_callback(self, laser_msg):
        '''Callback function for receiving laser messages to publish velocity messages'''
        self.calculate_lasers_range(laser_msg)

        if self.laser_sensors['f'] >= FORWARD_WALL_DIST:
            self.forward()
        elif self.laser_sensors['l'] >= WALL_DIST:
            self.left()

    def calculate_lasers_range(self, data):
        '''Dynamic range intervals'''
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
        initial_angle = final_angle - 4 * laser_interval

        interval = [None] * 5
        interval[0] = np.mean(data.ranges[int(initial_angle- half_laser_interval):int(initial_angle+half_laser_interval)])
        interval[0] = min(interval[0], 8.0)

        for i in range(1, 5):
            dirty_values = data.ranges[int(
                initial_angle + i * laser_interval - half_laser_interval
            ):int(initial_angle + i * laser_interval + half_laser_interval) + 1]
            interval[i] = min(np.mean(np.nan_to_num(dirty_values)), 8.0)

        self.laser_sensors['r'] = interval[0]
        self.laser_sensors['fr'] = interval[1]
        self.laser_sensors['f'] = interval[2]
        self.laser_sensors['fl'] = interval[3]
        self.laser_sensors['l'] = interval[4]

    def left(self):
        '''Turn the robot left'''
        print('Turn left!!!')
        start_angle = self.rotation_imu

        # Send out desired velocity
        vel_msg = Twist()
        vel_msg.angular.z = ANG_VEL
        self.vel_publisher.publish(vel_msg)

        # Wait for robot to reach the position
        while True:
            print('Target Angle: ', start_angle - 90)
            print('Angle: ', self.rotation_imu)
            if is_within_tolerance(start_angle - 90, self.rotation_imu, TOL):
                print('test turn complete\n\n\n\n')
                self.vel_publisher.publish(Twist())
                break

    def right(self):
        '''Turn the robot right'''

    def back(self):
        '''Turn the robot around'''

    def forward(self):
        '''Move the robot forward'''
        # Send out desired velocity
        self.vel_publisher.publish(self.follow_right_wall())

    def follow_right_wall(self, mv_forward = True, desired_dist = 0.14, k_p = 111):
        '''Follow the right wall to keep straight *MAGIC*'''
        # pylint: disable=invalid-name
        theta = 45
        sign = 1

        if mv_forward is True:
            ac = LIN_VEL
        else:
            ac = -LIN_VEL
            k_p=k_p/10
            sign=-1

        a = self.laser_sensors['fr']
        b = self.laser_sensors['r']
        swing = math.radians(theta)
        ab_angle = sign*math.atan2( a * math.cos(swing) - b , a * math.sin(swing))
        ab = b * math.cos(ab_angle)

        cd = ab + ac * math.sin(ab_angle)
        error = cd - desired_dist

        msg = Twist()
        output = -sign * k_p * error
        msg.linear.x = ac
        msg.angular.z =  output

        return msg

# Run the MazeRunner
if __name__ == "__main__" :
    rospy.init_node('mouse_node', anonymous=True)
    mr = MazeRunner()
