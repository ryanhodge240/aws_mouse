#!/usr/bin/env python

import sys
import rospy
import numpy as np
from math import cos, sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

np.set_printoptions(precision=2)
foundHeading = False
current_heading =0
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

'''
NO NEED to revise the code
this function gives you the distance to front, front right, front left and right to the wall
'''
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


def log_info():
    '''Initial orbit state'''
    global orbit, laser_sensors
    orbit_values = {-2: 'Going Left', -1: 'Left', 0: 'Undefined', 1: 'Right', 2: 'Going Right'}
    rospy.loginfo("Orbit: %s, Left  : %s, Front Left: %s, Front : %s, Front Right: %s, Right : %s", orbit_values[orbit],
                  laser_sensors['l'], laser_sensors['fl'], laser_sensors['f'],
                  laser_sensors['fr'], laser_sensors['r'])


def create_velocity_message(turn_left, turn_right, forward):
    global laser_sensors, rotation_imu, foundHeading
    angular = 0
    linear = 0

    if (turn_left):
    	#turn left based on current oritenation
        angular = rotation_imu + 1.57
        foundHeading = False  # need to update heading
        #linear = linear_vel*0.2
    if (turn_right):
        angular = rotation_imu -1.57
       # linear = linear_vel*0.2
    if (forward):
        linear = linear_vel
        angular = rotation_imu
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    
    turn =""
    if (turn_left):
        turn += "Turn Left| " 
    if (turn_right):
        turn += "Turn Right| " 
    if (forward):
        turn += "Forward  " 
     
    rospy.loginfo("Movement: linear %s angular %s - Direction %s ",linear, angular, turn)

    return vel_msg


def publish_velocity_message(vel_msg):
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel_pub.publish(vel_msg)

def clbk_imu(msg):
    global rotation_imu, current_heading, foundHeading
    global roll_imu, pitch_imu, yaw_imu
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll_imu, pitch_imu, yaw_imu) = euler_from_quaternion (orientation_list)
    if (not foundHeading):
        current_heading = yaw_imu
        foundHeading = True
    Kp=38
    rotation_imu =  Kp*(current_heading-yaw_imu)
    print(" \n IMU yaw: ",yaw_imu)

    
def laser_callback(data):
    global orbit, laser_sensors

    # find where the wall distance are
    # will detemine the values of laser_sensors['l'], laser_sensors['fl'], laser_sensors['f'],
    #              laser_sensors['fr'], laser_sensors['r']
    calculate_lasers_range(data)

    # begin the logic
    # put logic here to determine the direction of forward, turn_left or turn_right
    # based on the values laser_sensors['l'], laser_sensors['fl'], laser_sensors['f'],
    #              laser_sensors['fr'], laser_sensors['r']
    
    # end of the logic
    linear = 0
    angular = 0
    forward = True
    turn_left = False
    turn_right = False
    log_info()
    
    # create a moving message,whether it needs to turn  
    vel_msg = create_velocity_message(turn_left, turn_right, forward)
    
    # send the message out
    publish_velocity_message(vel_msg)

def sonar_callback(data):
    pass


def listeners():
    # subscribe and read scan message, check laser_callback function
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    # subscribe imu message (oritenation of the robot)
    rospy.Subscriber('/imu', Imu, clbk_imu)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('avoid_wall_1', anonymous=True)
    listeners()

