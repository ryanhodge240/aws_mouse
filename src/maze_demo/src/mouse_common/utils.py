#!/usr/bin/env python
import sys
import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
import math
import numpy as np
from math import cos, sin
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32,String
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from img_recognition.msg import Prediction

foundHeading = False
current_heading =0
total_step =0
orbit = 0


linear_vel = 0.2
same_cell = False
current_distance =0
angular_vel = 0.3
t0=0

wall_distance = 0.2
wall_distance_forward = 0.15
wall_distance_side = 0.1
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
        self.laser_ready=False
        self.label = ''
        self.confidence = 0
        rospy.loginfo("[{}]  Initializing micrmouse_node.py......".format(self.node_name))

        # configure subscriber
        self.first_sub = True
        self.sub_msg = rospy.Subscriber("/scan", LaserScan, self.calculate_lasers_range, queue_size=1)        
        # self.sub_imumsg = rospy.Subscriber("/imu", Imu, self.clbk_imu, queue_size=1)
        self.sub_odommsg = rospy.Subscriber("/odom", Odometry, self.odom_callback)
    	#subscribe to predication topic
        rospy.Subscriber("/prediction", Prediction, self.clbk_prediction)

        self._mved_distance = Float64()
        self._mved_distance.data = 0.0    
        #self.get_init_position()    
        self._current_position = None


        # configure Publisher
        self.pub_msg = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sensors = {'left': 0, 'frontleft': 0, 'front': 0, 'frontright': 0, 'right': 0}

        while (not self.laser_ready):
            time.sleep(1)

    def getPredication(self):
        return self.label

    def move_onecell(self, distance=0.3, kp = 111, DEBUG = False):
    
    	rate = rospy.Rate(30)
    	
    	self._mved_distance.data = 0.0
    	self.get_init_position()
    	
    	if distance < 0.0:
    	    mv_forward = False
    	else:
    	    mv_forward = True
    	      
    	while not rospy.is_shutdown():
            if self.laser_sensors is not None:
            	vel_msg = self.follow_right_wall(mv_forward, desired_dist = 0.14, kp = 111)
            	#vel_msg = self.follow_both_wall(mv_forward, desired_dist = 0.14, kp=kp)
            	self.pub_msg.publish(vel_msg)
            	if (self.laser_sensors['front']<wall_distance_forward):
            	    break
            	if (self._mved_distance.data >abs(distance)):
            	    break
            	rate.sleep()
    	vel_msg = Twist()
    	vel_msg.linear.x = 0
    	self.pub_msg.publish(vel_msg)
    	if (DEBUG):
    	    print("distance travelled {:.3f}".format(self._mved_distance.data))
    	    
    	return self._mved_distance.data

    def clbk_prediction(self, data):
        while (not rospy.is_shutdown()) and (data is None):
            try:
                data = rospy.wait_for_message("/prediction", Prediction, timeout=1)
            except:
                rospy.loginfo("Current image prediction not ready yet, retrying for setting up image prediction firsts")
                
        if (not rospy.is_shutdown()):
            if (data is None):
                self.label = data.label
                self.confidence = data.confidence
                return data
            else:
                self.label = "no prediction publisher"
                return None
        else:
            self.label = "ROS is down"
            return None

    def follow_right_wall(self, mv_forward, desired_dist = 0.14, kp = 111):
        #kp =  111
        ki = 0
        kd = 0

        theta = 45;
        sign = 1
        
        if mv_forward is True:
            AC = 0.2
        else:
            AC = -0.2
            kp=kp/10
            sign=-1
        
        #desired_trajectory =0.14
        
        
        a = self.laser_sensors['frontright']
        #if self.laser_sensors['r'] > 0.3 and self.laser_sensors['fr'] < 0.3:
        #    print(" no wall left")
        #    b = a
        #else:
        #    b = self.laser_sensors['r']
        b = self.laser_sensors['right']
        swing = math.radians(theta)
        ABangle = sign*math.atan2( a * math.cos(swing) - b , a * math.sin(swing))
        AB = b * math.cos(ABangle)
        # AC = 0.2     # how much the car moves in one time shot or linear.x in twist message
        CD = AB + AC * math.sin(ABangle)
        error = CD - desired_dist

        msg = Twist()
        integ = 0
        diff = 0
        output = -sign*kp*error # - ki*integ - kd*diff
        msg.linear.x = AC
        #print("turning angle {:.3f}".format(output))

#        print("output->:{:.3f} and left distance->: {:.3f}, current right distance->: {:.3f}".format(output, self.laser_sensors['l'], self.laser_sensors['r']))

        msg.angular.z =  output


        return msg
        
        
    def follow_both_wall(self, mv_forward, desired_dist = 0.14, kp = 111):
        #kp =  111
        ki = 0
        kd = 0

        theta = 45;
        
        if mv_forward is True:
            AC = 0.2
        else:
            AC = -0.2
        
        #desired_trajectory =0.14
        
        
        a = self.laser_sensors['frontright']
        b = self.laser_sensors['right']
        c = self.laser_sensors['left']
        d = self.laser_sensors['frontleft']
        #if self.laser_sensors['r'] > 0.3 and self.laser_sensors['fr'] < 0.3:
        #    print(" no wall left")
        #    b = a
        #else:
        #    b = self.laser_sensors['r']
        swing = math.radians(theta)
        ABangle = math.atan2( a * math.cos(swing) - b , a * math.sin(swing))
        AB = b * math.cos(ABangle)
        # AC = 0.2     # how much the car moves in one time shot or linear.x in twist message
        CDr = b * math.cos(ABangle) + AC * math.sin(ABangle)
        CDl = c*math.cos(ABangle) - AC*math.sin(ABangle)
        if b < 0.3 and c < 0.3: 
            error = CDr - CDl
        elif c < 0.3:
            error = desired_dist - CDl
        elif b < 0.3:
            error = CDr - desired_dist
        else:
            error = 0
        

        msg = Twist()
        integ = 0
        diff = 0
        #if a > c:
        #    output = -kp*(a-c)
        #else:
        #    output = kp*(c-a)

        output = -kp*error # - ki*integ - kd*diff
        msg.linear.x = AC
#        print("output->:{:.3f} and left distance->: {:.3f}, current right distance->: {:.3f}".format(output, self.laser_sensors['l'], self.laser_sensors['r']))
        msg.angular.z =  output
 #       print("turning angle {:.3f}".format(output))

        return msg

    '''
    NO NEED to revise the code
    this function gives you the distance to front, front right, front left and right to the wall
    '''


    def odom_callback(self, msg):
        """Process odometry data sent by the subscriber."""
        global rotation_odom, current_heading, foundHeading
        # Get the position information from the odom message
        # See the structure of an /odom message in the `get_init_position` function
        NewPosition = msg.pose.pose.position
        NewOrientation = msg.pose.pose.orientation
        
        # Calculate the orientatiom from odom
        orientation_odom = [NewOrientation.x, NewOrientation.y, NewOrientation.z, NewOrientation.w]
        (self.roll_odom, self.pitch_odom, self.yaw_odom) = euler_from_quaternion (orientation_odom)
        self.yaw_odom = self.pi_to_pi(self.yaw_odom)
        
        # Update curent heading
        if (not foundHeading):
            current_heading = self.yaw_odom
            foundHeading = True
        Kp=38
        rotation_odom =  Kp*(current_heading-self.yaw_odom)

        # Calculate the new distance moved, and add it to _mved_distance and 
        if (self._current_position !=None):
            self._mved_distance.data += self.calculate_distance(NewPosition, self._current_position)
        
        # Update the current position of the robot so we have a new reference point
        # (The robot has moved and so we need a new reference for calculations)
            self.updatecurrent_positin(NewPosition)
        
    def updatecurrent_positin(self, new_position):
        """Update the current position of the robot."""
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y
        self._current_position.z = new_position.z

    def calculate_distance(self, new_position, old_position):
        """Calculate the distance between two Points (positions)."""
        x2 = new_position.x
        x1 = old_position.x
        y2 = new_position.y
        y1 = old_position.y
        dist = math.hypot(x2 - x1, y2 - y1)
        return dist
    
    
    
    def calculate_lasers_range(self, data):
        '''Dynamic range intervals'''        

        half_pi = np.pi / 2
        laser_sensors = {'left': 0, 'frontleft': 0, 'front': 0, 'frontright': 0, 'right': 0}
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

        laser_sensors['right'] = interval[0]
        laser_sensors['frontright'] = interval[1]
        laser_sensors['front'] = interval[2]
        laser_sensors['frontleft'] = interval[3]
        laser_sensors['left'] = interval[4]
        self.laser_ready=True

        self.laser_sensors = laser_sensors


    #def clbk_imu(self, msg):
        #global rotation_imu, current_heading, foundHeading
        #orientation_q = msg.orientation
        #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        #(self.roll_imu, self.pitch_imu, self.yaw_imu) = euler_from_quaternion (orientation_list)
        # self.yaw_imu = self.pi_to_pi(self.yaw_imu)
        
        #if (not foundHeading):
         #   current_heading = self.yaw_imu
          #  foundHeading = True
        #Kp=38
        #rotation_imu =  Kp*(current_heading-self.yaw_imu)
        
    def pi_to_pi(self, angle):
        angle = np.mod(angle, (2*np.pi))
        if angle>np.pi:
            angle = angle - (2*np.pi)
        elif angle<(-np.pi):
            angle = angle + (2*np.pi)
           
        return angle
        
    def get_init_position(self):
        data_odom = None
        # wait for a message from the odometry topic and store it in data_odom when available
        while (not rospy.is_shutdown()) and (data_odom is None):
            try:
                data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying for setting up init pose")
        
        # Store the received odometry "position" variable in a Point instance 
        self._current_position = Point()
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z
        

    def getFilePath(self,name ,folder="image"):
        rospack = rospkg.RosPack()
        return rospack.get_path(self.package) + "/" + folder + "/" + name  

    def read_param_from_file(self, file_name, file_folder):
        fname = self.getFilePath(name=file_name,folder=file_folder)
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                print(" YAML syntax  error. File: {}".format(fname))
        return yaml_dict

    def log_info(self):
        rospy.loginfo("Left  : %s, Front Left: %s, Front : %s, Front Right: %s, Right : %s", 
	     self.laser_sensors['left'], self.laser_sensors['frontleft'], self.laser_sensors['front'],
		          self.laser_sensors['frontright'], self.laser_sensors['right'])

    def turnangle(self, target = 90, DEBUG=False):  
        if (target > 90):
            q = np.divmod(target,90)
            for i in range(0, q[0]):
                self.turnleft(90, DEBUG)
            if (q[1]>0):
                self.turnleft(q[1], DEBUG)
        elif (target < -90):
            q = np.divmod(abs(target),90)
            q[1] = -q[1]
            for i in range(0, q[0]):
                self.turnleft(-90, DEBUG)
            if (q[1]<0):
                self.turnleft(q[1], DEBUG)
        else:
            self.turnleft(target, DEBUG)

    def turnleft(self, target=90, DEBUG=False):   
    	global foundHeading, current_heading
    	import math 
    	current_angle = 0
    	rate = rospy.Rate(20)
    	kp=0.5
    	
    	target_rad = self.pi_to_pi(np.radians(target)+current_heading)  # (target* math.pi/180)   # current heading add 90 degree
    	cnt=0
    	while not rospy.is_shutdown():
            if (abs(target_rad-self.yaw_odom)<0.01):
             	foundHeading = False   # updated heading after the rotation
             	break;
            if (self.laser_sensors['frontright']<0.08 or self.laser_sensors['frontleft']<0.08):
                rospy.loginfo('Too close to wall, cannot rotate anymore')
            else:
             	vel_msg = Twist()
             	vel_msg.angular.z = kp *self.pi_to_pi(target_rad -self.yaw_odom)
             	self.pub_msg.publish(vel_msg)
             	cnt = cnt + 1
             	if (DEBUG and cnt%100==0):             	
             	    print('>pose[{:.3f} ]>target_heading {:.3f}'.format(self.yaw_odom, target_rad))
             	
            #self.prev_yaw_imu=self.yaw_imu
            rate.sleep()
            
    	if (DEBUG):
    	    print(current_angle, self.laser_sensors['front'])

    	vel_msg = Twist()
    	vel_msg.angular.z = 0
    	self.pub_msg.publish(vel_msg)
    	rate.sleep()
    	

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
    
    
