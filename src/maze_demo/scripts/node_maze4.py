#!/usr/bin/env python
import sys
import os, sys, argparse, errno, yaml, time, datetime
import rospy
import math

import numpy as np
from math import cos, sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32,String
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

wall_distance = 0.15
wall_distance_forward = 0.15
wall_distance_side = 0.35
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
        self.start = rospy.wait_for_message("/imu", Imu)
        rospy.loginfo("[{}]  Initializing micrmouse_node.py......".format(self.node_name))

        # configure subscriber
        self.first_sub = True
        self.sub_msg = rospy.Subscriber("/scan", LaserScan, self.calculate_lasers_range, queue_size=1)        
        self.sub_imumsg = rospy.Subscriber("/imu", Imu, self.clbk_imu, queue_size=1)


        # configure Publisher
        self.pub_msg = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sensors = {'l': 0, 'fl': 0, 'f': 0, 'fr': 0, 'r': 0}
        self.runMaze()
    
    def runMaze(self):
        global total_step, long_error, zero_error, kp, ki, kd
        kp =  0.06
        ki = 0
        kd = 0
        rate = rospy.Rate(10)
        long_error = rospy.Publisher('/long_error',Float32, queue_size=1)
        zero_error = rospy.Publisher('/zero_error',Float32, queue_size=1)

        
        while not rospy.is_shutdown():
 #           print('current left and front distance {} ---  {}'.format(self.laser_sensors['l'], self.laser_sensors['f']))
            if (self.laser_sensors['f']>wall_distance_forward):
                #move forward    
                print("          follow the wall       ")  
                vel_msg=self.follow_right_wall()
                self.pub_msg.publish(vel_msg)
                total_step = total_step+1
                print(total_step)
            elif (self.laser_sensors['l']<wall_distance_side ):
                print("          need to fnd the wall       ")  
                vel_msg=self.follow_right_wall()
                self.pub_msg.publish(vel_msg)
                self.pub_msg.publish(vel_msg)
            elif (self.laser_sensors['l']>wall_distance_side):
                print('turn left {}'.format(self.laser_sensors['f']))
                self.turnleft()

            elif (self.laser_sensors['l']<wall_distance_side and self.laser_sensors['f']<wall_distance_forward and self.laser_sensors['f'] > 0):
                self.turnaroundright()
                print("need to turn around")
                
            rate.sleep()
#            self.turnleft()
    
    def find_wall(self):
        global regions_
        global kp,ki,kd
        global long_error, zero_error
        import math

        msg = Twist()
        integ = 0
        diff = 0
        dist = 0.15
        error = dist - self.laser_sensors['l']
        integ += error
        output = kp*error + ki*integ + kd*diff
        diff = error
        msg.linear.x = 0.0
        msg.angular.z = 0.0 - output
        long_error.publish(output)
        zero_error.publish(0.0)
        return msg
    
    def follow_right_wall(self):
        global kp,ki,kd
        global long_error, zero_error
        kp =  82
        ki = 0
        kd = 0

        theta = 45;
        desired_trajectory =0.11
        a = self.laser_sensors['fr']
        b = self.laser_sensors['r']
        swing = math.radians(theta)
        ABangle = math.atan2( a * math.cos(swing) - b , a * math.sin(swing))
        AB = b * math.cos(ABangle)
        AC = 0.2     # how much the car moves in one time shot or linear.x in twist message
        CD = AB + AC * math.sin(ABangle)
        error = CD - desired_trajectory

        msg = Twist()
        integ = 0
        diff = 0
        output = -kp*error # - ki*integ - kd*diff
        msg.linear.x = AC
        print("output->:{:.3f} and left distance->: {:.3f}, current right distance->: {:.3f}".format(output, self.laser_sensors['l'], self.laser_sensors['r']))
        msg.angular.z =  output
        long_error.publish(output)
        zero_error.publish(0.0)

        return msg

    def follow_left_wall(self):
        global kp,ki,kd
        global long_error, zero_error
        kp =  82
        ki = 0
        kd = 0

        theta = 45;
        desired_trajectory =0.2
        a = self.laser_sensors['fl']
        b = self.laser_sensors['l']
        swing = math.radians(theta)
        ABangle = math.atan2( a * math.cos(swing) - b , a * math.sin(swing))
        AB = b * math.cos(ABangle)
        AC = 0.2     # how much the car moves in one time shot or linear.x in twist message
        CD = AB + AC * math.sin(ABangle)
        error = CD - desired_trajectory

        msg = Twist()
        integ = 0
        diff = 0
        output = kp*error # - ki*integ - kd*diff
        msg.linear.x = AC
        print("output->:{:.3f} and left distance->: {:.3f}, current right distance->: {:.3f}".format(output, self.laser_sensors['l'], self.laser_sensors['r']))
        msg.angular.z =  output
        long_error.publish(output)
        zero_error.publish(0.0)

        return msg
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
        (self.roll_imu, self.pitch_imu, self.yaw_imu) = euler_from_quaternion (orientation_list)
        if (not foundHeading):
            current_heading = self.yaw_imu
            foundHeading = True
        Kp=38
        rotation_imu =  Kp*(current_heading-self.yaw_imu)  

    def create_velocity_message(self, turn_left, turn_right, forward):
        global laser_sensors, rotation_imu, foundHeading
        angular = 0
        linear = 0

        if (turn_left):
            #turn left based on current oritenation
            angular = angular_vel
            #linear = linear_vel*0.2
        elif (turn_right):
            angular = -angular_vel
        # linear = linear_vel*0.2
        elif (forward):
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


    def turnleft(self):   
    	global foundHeading, current_heading
    	import math 
    	forward = False
    	turn_left = True
    	turn_right = False
    	t0 = rospy.Time.now().to_sec()
    	current_angle = 0
    	rate = rospy.Rate(2)
    	
    	while not rospy.is_shutdown():
            vel_msg = self.create_velocity_message(turn_left, turn_right, forward)
            self.pub_msg.publish(vel_msg)
            t1=rospy.Time.now().to_sec()
            current_angle= angular_vel *(t1-t0)
            print('>>>>>>pose[{} ]>>>>>>turnnnnning left {}'.format(self.yaw_imu, self.laser_sensors['f']))

            if (self.yaw_imu-current_heading>1.57):
            	foundHeading = False   # updated heading after the rotation
            	break
            rate.sleep()
    	print(current_angle, self.laser_sensors['f'])

    	vel_msg = Twist()
    	vel_msg.angular.z = 0
    	self.pub_msg.publish(vel_msg)
    	rate.sleep()
    	
    def turnright(self):   
    	global foundHeading, current_heading
    	import math 
    	forward = False
    	turn_left = False
    	turn_right = True
    	t0 = rospy.Time.now().to_sec()
    	current_angle = 0
    	rate = rospy.Rate(2)
    	
    	while not rospy.is_shutdown():
            vel_msg = self.create_velocity_message(turn_left, turn_right, forward)
            self.pub_msg.publish(vel_msg)
            t1=rospy.Time.now().to_sec()
            current_angle= angular_vel *(t1-t0)
            print('>>>>>>pose[{} ]>>>>>>>>>>>>>>>>turnnnnning left {}'.format(self.yaw_imu, self.laser_sensors['f']))

            if (self.yaw_imu-current_heading<-1.57):
            	foundHeading = False   # updated heading after the rotation
            	break
            rate.sleep()
    	print(current_angle, self.laser_sensors['f'])

    	vel_msg = Twist()
    	vel_msg.angular.z = 0
    	self.pub_msg.publish(vel_msg)
    	rate.sleep()

    def turnaroundright(self):   
    	global foundHeading, current_heading
    	import math 
    	forward = False
    	turn_left = False
    	turn_right = True
    	t0 = rospy.Time.now().to_sec()
    	current_angle = 0
    	rate = rospy.Rate(2)
    	
    	while not rospy.is_shutdown():
            vel_msg = self.create_velocity_message(turn_left, turn_right, forward)
            self.pub_msg.publish(vel_msg)
            t1=rospy.Time.now().to_sec()
            current_angle= angular_vel *(t1-t0)
            print('>>>>>>pose[{} : {}]>>>>>>>>>>>>>>>>turnnnnning around {}'.format(self.yaw_imu, current_angle, self.laser_sensors['f']))

            if (self.yaw_imu-current_heading> 6.28):
            	foundHeading = False   # updated heading after the rotation
            	break
            rate.sleep()
    	print(current_angle, self.laser_sensors['f'])

    	vel_msg = Twist()
    	vel_msg.angular.z = 0
    	self.pub_msg.publish(vel_msg)
    	rate.sleep()

    	    	
    def moveaction(self, distance):    
#    	self.log_info(laser_sensors)
 
        
    	rate = rospy.Rate(50)
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
            	if (self.laser_sensors['f']<wall_distance_forward):
            	    break
            	if (current_distance>distance):
            	    break
            	rate.sleep()
    	print(current_distance)
    	vel_msg = Twist()
    	vel_msg.linear.x = 0
    	self.pub_msg.publish(vel_msg)
    	return current_distance

    def movecenter(self, distance):            
    	rate = rospy.Rate(50)
    	current_distance = 0
    	forward = True
    	turn_left = False
    	turn_right = False
    	
    	if (self.laser_sensors['l']<self.laser_sensors['r']):
    	    turn_left = True
    	else:
    	    turn_right = True
    	t0 = rospy.Time.now().to_sec()
      
    	while not rospy.is_shutdown():
            if self.laser_sensors is not None:
            	vel_msg = self.create_velocity_message(turn_left, turn_right, forward)
            	self.pub_msg.publish(vel_msg)
            	t1=rospy.Time.now().to_sec()
            	current_distance= linear_vel *(t1-t0)
            	if (self.laser_sensors['f']<wall_distance_forward):
            	    break
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


if __name__ == "__main__" :
    rospy.init_node("mouse_node", anonymous=False)
  
    micromouse_node = Micromouse_Node()
    rospy.on_shutdown(micromouse_node.on_shutdown)   
    rospy.spin()    
    
    
