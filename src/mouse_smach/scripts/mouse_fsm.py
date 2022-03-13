#!/usr/bin/env python
import sys
import os, sys, argparse, errno, yaml, time, datetime
import rospy
import math
import smach, smach_ros
from smach import State, StateMachine,  CBState
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
        rospy.loginfo("[{}]  Initializing micrmouse_node.py......".format(self.node_name))

        # configure subscriber
        self.first_sub = True
        self.sub_msg = rospy.Subscriber("/scan", LaserScan, self.calculate_lasers_range, queue_size=1)        
        self.sub_imumsg = rospy.Subscriber("/imu", Imu, self.clbk_imu, queue_size=1)
        self.sub_odommsg = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self._mved_distance = Float64()
        self._mved_distance.data = 0.0    
        #self.get_init_position()    
        self._current_position = None


        # configure Publisher
        self.pub_msg = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sensors = {'l': 0, 'fl': 0, 'f': 0, 'fr': 0, 'r': 0}
        while (not self.laser_ready):
            time.sleep(1)
    
        self.FSM()
        
        
    def FSM(self):
#    	rospy.init_node('fsm_py')

  	# Create a SMACH state machine container
    	sm = StateMachine(outcomes=['IDEL', 'succeeded', 'forward', 'left', 'right'])
 
  	# Set user data for the finite state machine
    	sm.userdata.lspeed = 0.2
  	#sm.userdata.sm_input = input("Enter 1 to Push or 0 to Insert a Coin: ")
     
  	# Open the state machine container. A state machine container holds a number of states.
    	with sm:
    	    StateMachine.add('IDEL', CBState(self.cb_idel, cb_args=[self]),
	    	{'succeeded':'IDEL','forward':'FORWARD', 'left':'LEFT', 'right':'RIGHT'})
    	    StateMachine.add('FORWARD', CBState(self.cb_forward, cb_args=[self]),{'succeeded':'IDEL'})
    	    StateMachine.add('LEFT', CBState(self.cb_left, cb_args=[self]), {'succeeded':'IDEL'})	
    	    StateMachine.add('RIGHT', CBState(self.cb_right, cb_args=[self]),{'succeeded':'IDEL'})
    	    StateMachine.add('DONE', CBState(self.cb_done, cb_args=[self]), {'succeeded':'IDEL'})	    		    	 
	  # View our state transitions using ROS by creating and starting the instrospection server
    	    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    	    sis.start()
	  # Execute the state machine 
    	    outcome = sm.execute()
 
	  # Wait for ctrl-c to stop the application
    	    rospy.spin()
    	    sis.stop()


    
    @smach.cb_interface(input_keys=['lspeed'], outcomes=['succeeded', 'IDEL', 'forward', 'left', 'right'])
    def cb_idel(user_data, self):
    	if (user_data.lspeed <0.3):
   	    return 'forward'
    	else:
   	    return 'IDEL'

    @smach.cb_interface(input_keys=['lspeed'], output_keys=[], outcomes=['succeeded'])
    def cb_forward(user_data, self):
    	if (user_data.lspeed <0.3):
    	    self.move_onecell(0.32)
    	    return 'succeeded'
   	    
    @smach.cb_interface(input_keys=[], output_keys=[], outcomes=['succeeded'])
    def cb_left(user_data, self):
        pass
    @smach.cb_interface(input_keys=[], output_keys=[], outcomes=['succeeded'])
    def cb_right(user_data, self):
        pass
    @smach.cb_interface(input_keys=[], output_keys=[], outcomes=['succeeded'])
    def cb_done(user_data, self):
        pass                
    def move_onecell(self, distance):
    	rate = rospy.Rate(10)
    	current_distance = 0
    	t0 = rospy.Time.now().to_sec()
    	self._mved_distance.data = 0.0
    	self.get_init_position()
    	
      
    	while not rospy.is_shutdown():
            if self.laser_sensors is not None:
            	vel_msg = self.follow_right_wall()
            	self.pub_msg.publish(vel_msg)
            	t1=rospy.Time.now().to_sec()
            	current_distance= linear_vel *(t1-t0)
            	if (self.laser_sensors['f']<wall_distance_forward):
            	    break
            	if (self._mved_distance.data >distance):
            	    break
            	rate.sleep()
    	print(self._mved_distance.data)
#    	self.prevleftTurn = self.leftTurn 
#    	self.leftTurn = False
#    	if (self.laser_sensors['f']<wall_distance_forward and self.laser_sensors['fl']<wall_distance_side):
#    	     self.prevleftTurn = self.leftTurn
#    	     self.leftTurn = True
    	vel_msg = Twist()
    	vel_msg.linear.x = 0
    	self.pub_msg.publish(vel_msg)
    	return current_distance
   
    def follow_right_wall(self):
        global kp,ki,kd
        kp =  111
        ki = 0
        kd = 0

        theta = 45;
        desired_trajectory =0.14
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
#        print("output->:{:.3f} and left distance->: {:.3f}, current right distance->: {:.3f}".format(output, self.laser_sensors['l'], self.laser_sensors['r']))
        msg.angular.z =  output


        return msg

    '''
    NO NEED to revise the code
    this function gives you the distance to front, front right, front left and right to the wall
    '''
    def odom_callback(self, msg):
        """Process odometry data sent by the subscriber."""
        # Get the position information from the odom message
        # See the structure of an /odom message in the `get_init_position` function
        NewPosition = msg.pose.pose.position

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
        self.laser_ready=True

        self.laser_sensors = laser_sensors


    def clbk_imu(self, msg):
        global rotation_imu, current_heading, foundHeading
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll_imu, self.pitch_imu, self.yaw_imu) = euler_from_quaternion (orientation_list)
        self.yaw_imu = self.pi_to_pi(self.yaw_imu)
        
        if (not foundHeading):
            current_heading = self.yaw_imu
            foundHeading = True
        Kp=38
        rotation_imu =  Kp*(current_heading-self.yaw_imu)
        
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
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying for setting up init pose")
        
        # Store the received odometry "position" variable in a Point instance 
        self._current_position = Point()
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z
        
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
    	kp=0.5
    	target = 90
    	target_rad = self.pi_to_pi(target* math.pi/180 + current_heading)   # current heading add 90 degree
    	self.prev_yaw_imu =target_rad  #90
  	
    	while not rospy.is_shutdown():
            if (abs(self.prev_yaw_imu-self.yaw_imu)<0.01):
             	foundHeading = False   # updated heading after the rotation
             	break;
    	
            vel_msg = Twist()
            vel_msg.angular.z = kp *(target_rad -self.yaw_imu)
            self.pub_msg.publish(vel_msg)

            print('>>>>>>pose[{:.3f} ]>>>>>>target_heading {:.3f}'.format(self.yaw_imu, target_rad))
            #self.prev_yaw_imu=self.yaw_imu


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
    	turn_left = True
    	turn_right = False
    	t0 = rospy.Time.now().to_sec()
    	current_angle = 0
    	rate = rospy.Rate(2)
    	kp=0.5
    	target = -90
    	target_rad = self.pi_to_pi(target* math.pi/180 + current_heading)   # current heading add 90 degree
    	self.prev_yaw_imu =target_rad  #-90
  	
    	while not rospy.is_shutdown():
            if (abs(self.prev_yaw_imu-self.yaw_imu)<0.01):
             	foundHeading = False   # updated heading after the rotation
             	break;
    	
            vel_msg = Twist()
            vel_msg.angular.z = kp *(target_rad -self.yaw_imu)
            self.pub_msg.publish(vel_msg)

            print('>>>>>>pose[{:.3f} ]>>>>>>target_heading {:.3f}'.format(self.yaw_imu, target_rad))
            #self.prev_yaw_imu=self.yaw_imu


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
    
    
