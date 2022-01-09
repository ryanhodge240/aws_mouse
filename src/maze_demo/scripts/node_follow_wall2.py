#!/usr/bin/env python

import sys
import rospy
import numpy as np
from math import cos, sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

np.set_printoptions(precision=2)

orbit = 0
laser_sensors = {'w': 0, 'nw': 0, 'n': 0, 'ne': 0, 'e': 0}

linear_vel = 0.2
angular_vel = 0.2
wall_distance = 0.2
wall_distance_forward = 0.30
wall_distance_side = 0.15

inf = float('inf')

left = -1
going_left = -2
right = 1
going_right = 2


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

    laser_sensors['e'] = interval[0]
    laser_sensors['ne'] = interval[1]
    laser_sensors['n'] = interval[2]
    laser_sensors['nw'] = interval[3]
    laser_sensors['w'] = interval[4]


def log_info():
    '''Initial orbit state'''
    global orbit, laser_sensors
    orbit_values = {-2: 'Going Left', -1: 'Left', 0: 'Undefined', 1: 'Right', 2: 'Going Right'}
    rospy.loginfo("Orbit: %s, W : %s, NW: %s, N : %s, NE: %s, E : %s", orbit_values[orbit],
                  laser_sensors['w'], laser_sensors['nw'], laser_sensors['n'],
                  laser_sensors['ne'], laser_sensors['e'])


def create_velocity_message(turn_left, turn_right, forward):
    global laser_sensors
    angular = 0
    linear = 0
    kp=15
    error = laser_sensors['e'] - laser_sensors['w']
    output = kp*error*angular_vel 
    if (turn_left):
        angular = output
        linear = linear_vel*0.2
    if (turn_right):
        angular = output
        linear = linear_vel*0.2
    if (forward):
        linear = linear_vel
        angular = output
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


def laser_callback(data):
    global orbit, laser_sensors

    calculate_lasers_range(data)



    linear = 0
    angular = 0
    forward = False
    turn_left = False
    turn_right = False
    if (laser_sensors['n'] < 0.15 or laser_sensors['nw'] < 0.15 or laser_sensors['ne'] < 0.15):
        print("\n \n Warning \n \n")
    if (orbit == 0):
       if (laser_sensors['n'] < wall_distance_forward):
            if (laser_sensors['w'] < wall_distance_side):
            	orbit = left
            	print("Case: 1")       
            elif (laser_sensors['e'] < wall_distance_side):
               orbit = right
               print("Case: 2")
            elif (laser_sensors['nw'] < wall_distance and laser_sensors['ne'] < wall_distance):
                if (laser_sensors['nw'] <laser_sensors['ne']):
                    orbit = going_left
                    print("Case: 3")
                else:
                    orbit = going_right
                    print("Case: 4")
            elif (laser_sensors['nw'] < wall_distance):
               orbit = going_left
               print("Case: 5")
            elif (laser_sensors['ne'] < wall_distance):
               orbit = going_right
               print("Case: 6")
       else:
           print("Case: 7")
           forward = True
           turn_right = False
           turn_left = False
    elif (orbit == going_left or orbit == going_right):
    	if (laser_sensors['n'] < wall_distance_forward):
    	    if (laser_sensors['ne'] < laser_sensors['nw']):
    	        turn_left = True
    	        turn_right = False
    	        print("Case: 8")
    	    else:
    	        turn_right = True
    	        turn_left = False
    	        print("Case: 9")	            
    	elif (laser_sensors['w'] < wall_distance_side):
            orbit = left
            print("Case: 10")
    	elif (laser_sensors['e'] < wall_distance_side):
            orbit = right
            print("Case: 11")
    	elif (orbit == going_left):
            turn_right = True
            turn_left = False
            print("Case: 12")
    	elif (orbit == going_right):
            turn_left = True
            turn_right = False
            print("Case: 13")
    elif (orbit == left):
        if (laser_sensors['n'] > wall_distance_forward
                and (laser_sensors['w'] > wall_distance_side
                     or laser_sensors['e'] > wall_distance_side)):
            forward = True
            print("Case: 14")
        elif (laser_sensors['w'] <= wall_distance_side
                and laser_sensors['e'] <= wall_distance_side):
            turn_right = True
            print("Case: 15")
        elif (laser_sensors['nw'] <= wall_distance
              or laser_sensors['ne'] <= wall_distance):
            turn_right = True
            print("Case: 16")
        else:
            if (laser_sensors['ne'] < wall_distance
                    or laser_sensors['nw'] < wall_distance
                    or laser_sensors['n'] < wall_distance_forward):
                turn_right = True
                print("Case: 17")
            else:
                turn_left = True
                print("Case: 18")               
    elif (orbit == right):
        if (laser_sensors['n'] > wall_distance_forward
                and (laser_sensors['w'] > wall_distance_side
                     or laser_sensors['e'] > wall_distance_side)):
            forward = True
            print("Case: 19")
        if (laser_sensors['w'] <= wall_distance_side
                and laser_sensors['e'] <= wall_distance_side):
            turn_left = True
            print("Case: 20")
        elif (laser_sensors['nw'] <= wall_distance
              or laser_sensors['ne'] <= wall_distance):
            turn_left = True
            print("Case: 21")
        else:
            if (laser_sensors['ne'] < wall_distance
                    or laser_sensors['nw'] < wall_distance
                    or laser_sensors['n'] < wall_distance_forward):
                turn_left = True
                print("Case: 22")
            else:
                turn_right = True
                print("Case: 23")

    log_info()
    vel_msg = create_velocity_message(turn_left, turn_right, forward)
    
    publish_velocity_message(vel_msg)

def sonar_callback(data):
    pass


def listeners():
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('avoid_wall_1', anonymous=True)
    listeners()

