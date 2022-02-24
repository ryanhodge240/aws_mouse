#!/usr/bin/env python
import sys
import os, sys, argparse, errno, yaml, time, datetime
import rospy
import numpy as np
from math import cos, sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

foundHeading = False
current_heading =0
total_step =0
orbit = 0


linear_vel = 0.2
same_cell = False
current_distance =0
angular_vel = 0.2
t0=0

wall_distance = 0.2
wall_distance_forward = 0.2
wall_distance_side = 0.15
rotation_imu =0
total_step = 0

inf = float('inf')

left = -1
going_left = -2
right = 1
going_right = 2

class Micromouse_Node(object):
    def __init__(self):
        self.package = "maze_demo"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.start = rospy.wait_for_message("/scan", LaserScan)
        rospy.loginfo("[{}]  Initializing micrmouse_node.py......".format(self.node_name))

        # configure subscriber
        self.first_sub = True
        self.sub_msg = rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=1)        
        self.sub_imumsg = rospy.Subscriber("/imu", Imu, self.clbk_imu, queue_size=1)

        # configure Publisher
        self.pub_msg = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sensors = {'l': 0, 'fl': 0, 'f': 0, 'fr': 0, 'r': 0}
    
    '''
    NO NEED to revise the code
    this function gives you the distance to front, front right, front left and right to the wall
    '''
    def calculate_lasers_range(self, data):
        '''Dynamic range intervals'''        

        half_pi = np.pi / 2
        laser_sensors = {'l': 0, 'fl': 0, 'f': 0, 'fr': 0, 'r': 0}
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

        interval = [None] * 5
        interval[0] = np.mean(data.ranges[int(initial_angle- half_laser_interval):int(initial_angle+half_laser_interval)])
        interval[0] = min(interval[0], 8.0)

        for i in range(1, 5):
            dirty_values = data.ranges[int(
                initial_angle + i * laser_interval - half_laser_interval
            ):int(initial_angle + i * laser_interval + half_laser_interval) + 1]
            interval[i] = min(np.mean(np.nan_to_num(dirty_values)), 8.0)

        laser_sensors['r'] = interval[0]
        laser_sensors['fr'] = interval[1]
        laser_sensors['f'] = interval[2]
        laser_sensors['fl'] = interval[3]
        laser_sensors['l'] = interval[4]

        self.laser_sensors = laser_sensors


    def clbk_imu(self, msg):
        global rotation_imu, current_heading, foundHeading
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll_imu, pitch_imu, yaw_imu) = euler_from_quaternion (orientation_list)
        if (not foundHeading):
            current_heading = yaw_imu
            foundHeading = True
        Kp=38
        rotation_imu =  Kp*(current_heading-yaw_imu)  

    def create_velocity_message(self, turn_left, turn_right, forward):
        global laser_sensors, rotation_imu, foundHeading
        angular = 0
        linear = 0

        if (turn_left):
            #turn left based on current oritenation
            angular = angular_vel
            linear = 0
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
        
    #    rospy.loginfo("Movement: linear %s angular %s - Direction %s ",linear, angular, turn)

        return vel_msg


 
    def read_param_from_file(self, file_name, file_folder):
        fname = self.getFilePath(name=file_name,folder=file_folder)
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                print(" YAML syntax  error. File: {}".format(fname))
        return yaml_dict

    def log_info(self, laser_sensors):
        rospy.loginfo("Left  : %s, Front Left: %s, Front : %s, Front Right: %s, Right : %s", 
	     laser_sensors['l'], laser_sensors['fl'], laser_sensors['f'],
		          laser_sensors['fr'], laser_sensors['r'])


    def turnleft(self, laser_sensors):   
    	global rotation_imu
    	import math 
    	forward = False
    	turn_left = True
    	turn_right = False
    	t0 = rospy.Time.now().to_sec()
    	current_angle = 0
    	rate = rospy.Rate(10)
    	current_rotation_imu = rotation_imu
    	while not rospy.is_shutdown():
            if self.laser_sensors is not None:
            	vel_msg = self.create_velocity_message(turn_left, turn_right, forward)
            	self.pub_msg.publish(vel_msg)
            	t1=rospy.Time.now().to_sec()
            	current_angle= angular_vel *(t1-t0)
            	if (self.laser_sensors['f']>wall_distance_forward):
            	    break
            	if (current_angle>1.57):
            	    break
            	rate.sleep()
    	print(current_angle)

    	vel_msg = Twist()
    	vel_msg.angular.z = 0
    	self.pub_msg.publish(vel_msg)
    	time.sleep(1)
    	
    	
    def moveaction(self, distance):    
#    	self.log_info(laser_sensors)
 
    	start_time = rospy.get_time()
    	if self.first_sub == True:
    	    rospy.loginfo("[{}] detemine action Please wait...".format(self.node_name))
        
    	rate = rospy.Rate(10)
    	current_distance = 0
    	forward = True
    	turn_left = False
    	turn_right = False
    	t0 = rospy.Time.now().to_sec()
      
    	while not rospy.is_shutdown():
            if self.laser_sensors is not None:
            	vel_msg = self.create_velocity_message(turn_left, turn_right, forward)
            	self.pub_msg.publish(vel_msg)
            	t1=rospy.Time.now().to_sec()
            	current_distance= linear_vel *(t1-t0)
            	if (current_distance>distance):
            	    break
            	rate.sleep()
    	print(current_distance)
    	vel_msg = Twist()
    	vel_msg.linear.x = 0
    	self.pub_msg.publish(vel_msg)
    	return current_distance



    def on_shutdown(self): 
        rospy.loginfo("[{}] Close.".format(self.node_name))
        rospy.loginfo("[{}] shutdown.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown=True

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def laser_callback(self, data):
    
    # find where the wall distance are
    # will detemine the values of laser_sensors['l'], laser_sensors['fl'], laser_sensors['f'],
    #              laser_sensors['fr'], laser_sensors['r']
        global total_step
        self.calculate_lasers_range(data)
        laser_sensors = self.laser_sensors
        time.sleep(0.5)
        if (laser_sensors['f']>wall_distance_forward):
            #move forward
            current_distance = self.moveaction(0.3)
            total_step = total_step+1
            print(total_step)
        elif (laser_sensors['l']>wall_distance_side):
            print('turn left {}'.format(laser_sensors['f']))
            self.turnleft(laser_sensors)

if __name__ == "__main__" :
    rospy.init_node("mouse_node", anonymous=False)
  
    micromouse_node = Micromouse_Node()
    rospy.on_shutdown(micromouse_node.on_shutdown)   
    rospy.spin()    
    
    
