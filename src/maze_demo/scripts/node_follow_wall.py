#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32,String

import laser_geometry.laser_geometry as lg
import math

## initialize variables
active_ = False
pub_ = None
lp = None
pc_pub = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
        'back': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'turn right',
    4: 'turn around',
}


# funtion to calculate average
def avg(lst):
    return sum(lst)/len(lst)

# Service message
def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res
    
# funtion to convert laser scan to point cloud    
def scan_cb(msg):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # now we can do something with the PointCloud2 for example:
    # publish it
    pc_pub.publish(pc2_msg)

# funtion to calculate range from laser data
def clbk_laser(msg):
    global regions_
    #360/5 = 72
    merged_list = msg.ranges[0:2] + msg.ranges[357:359]
    regions_ = {
        'back':   min(avg(merged_list),8.0),
        'front':  min(avg(msg.ranges[178:182]),8.0),
        'right':  min(avg(msg.ranges[88:92]),8.0),
        'fleft': min(avg(msg.ranges[220:230]),8.0),
        'left':  min(avg(msg.ranges[268:272]),8.0),
        'fright':   min(avg(msg.ranges[130:140]),8.0),
    	}
    take_action()
    

# Twist msg according to the state determined by range
def change_state(state):
    global state_, state_dict_

    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

    if state_ == 0:
        msg = find_wall()
        #msg = test()
    elif state_ == 1:
        msg = turn_left()
        #msg = test()
    elif state_ == 2:
        msg = follow_the_wall()
        #msg = test()
    elif state_ == 3:
    	#msg = find_wall()
        msg = turn_right()
        #msg = test()
    elif state_ == 4:
        msg = turn_around()
        #msg = test()
    else:
        rospy.logerr('Unknown state!')

    pub_.publish(msg)

# according to range define the state
def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    # tuning parameters to decide the state depending on range
    # d => minimum range to go straight, e => minimum range to make a turn
    d = 0.4
    e = 0.3
    
    # F > d, L > e, R > e, F.R = ?, F.R = ?
    if regions['front'] > d and regions['left'] > e and regions['right'] > e:
        state_description = 'case 1 - nothing'
        change_state(2)
    # F < d, L > e, R > e
    elif regions['front'] < d and regions['left'] > e and regions['right'] > e:
    	if regions['fleft'] > d and regions['fright'] < d:
    		state_description = 'case 2 - left'
    		change_state(1)
    	elif regions['fleft'] < d and regions['fright'] > d: 
    		state_description = 'case 3 - right'
    		change_state(3)
    	else:
    		state_description = 'case 4 - left'
    		change_state(1) 		
   # F > d, L < e, R < e,
    elif regions['front'] > d and regions['left'] < e and regions['right'] < e:
          state_description = 'case 5 - front'
          change_state(2)
    # F > d, L > e, R < e,
    elif regions['front'] > d and regions['left'] > e and regions['right'] < e:
          state_description = 'case 6 - front'
          change_state(2)
    # F > d, L < e, R > e
    elif regions['front'] > d and regions['left'] < e and regions['right'] > e:
          state_description = 'case 7 - front'
          change_state(2)
    # F < d, L > e, R < e
    elif regions['front'] < d and regions['left'] > e and regions['right'] < e:
          state_description = 'case 8 - left'
          change_state(1)
    # F < d, L < e, R > e
    elif regions['front'] < d and regions['left'] < e and regions['right'] > e:
          state_description = 'case 9 - right'
          change_state(3)
    # F < d, L < e, R < e
    elif regions['front'] < d and regions['left'] < e and regions['right'] < e:
    	if regions['fleft'] > d and regions['fright'] < d:
    		state_description = 'case 10 - turn left'
    		change_state(0)
    	elif regions['fleft'] < d and regions['fright'] > d:
    		state_description = 'case 11 - turn right'
    		change_state(0)
    	else:
    		state_description = 'case 12 - turn around'
    		change_state(4)       
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
        rospy.logerr('Unknown state!')
    print(regions_)
    print(state_description)
    
def test():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    return msg

def find_wall():
    global regions_
    global kp,ki,kd
    global long_error, zero_error
    msg = Twist()
    integ = 0
    diff = 0
    dist = 0.1
#    error = dist - regions_['left']
    error = regions_['fright'] - regions_['fleft']
    integ += error
    output = kp*error + ki*integ + kd*diff
    diff = error
    msg.linear.x = 0.0
    msg.angular.z = 0.0 - output
    long_error.publish(output)
    zero_error.publish(0.0)
    return msg

def turn_left():
    global regions_
    global kp,ki,kd
    global long_error, zero_error
    msg = Twist()
    integ = 0
    diff = 0
    dist = 0.1
#    error = dist - regions_['left']
    error = regions_['right'] - regions_['left']
    integ += error
    output = kp*error + ki*integ + kd*diff
    diff = error
    msg.linear.x = 0.0
    msg.angular.z = 0.9 - output
    long_error.publish(output)
    zero_error.publish(0.0)

    return msg
    
    
def turn_right():
    global regions_
    global kp,ki,kd
    global long_error, zero_error
    msg = Twist()
    integ = 0
    diff = 0
    dist = 0.1
#    error = dist - regions_['left']
    error = regions_['right'] - regions_['left']
    integ += error
    output = kp*error + ki*integ + kd*diff
    diff = error
    msg.linear.x = 0.0
    msg.angular.z = -0.9 - output
    long_error.publish(output)
    zero_error.publish(0.0)
    
    return msg

def turn_around():
    global regions_
    global kp,ki,kd
    global long_error, zero_error
    msg = Twist()
    integ = 0
    diff = 0
    dist = 0.1
#    error = dist - regions_['left']
    error = regions_['right'] - regions_['left']
    integ += error
    output = kp*error + ki*integ + kd*diff
    diff = error
    msg.linear.x = 0.0
    msg.angular.z = 3.14 - output
    long_error.publish(output)
    zero_error.publish(0.0)
     
    return msg


    
def follow_the_wall():
    global regions_
    global kp,ki,kd
    global long_error, zero_error
    msg = Twist()
    integ = 0
    diff = 0
    dist = 0.1
#    error = dist - regions_['left']
    error = regions_['right'] - regions_['left']
    integ += error
    output = kp*error + ki*integ + kd*diff
    diff = error
    msg.linear.x = 0.1
    msg.angular.z = 0.0 - output
    long_error.publish(output)
    zero_error.publish(0.0)

    return msg


def wall_follow(alt):
    global kp,ki,kd
    kp = alt.Kp * 0.006
    ki = alt.Ki * 0.0008
    kd = alt.Kd * 0.03

def main():
    global pub_, active_
    global kp,ki,kd
    global long_error, zero_error
    global lp, pc_pub
    kp = 459 * 0.006
    ki = 0
    kd = 1288 *0.03
    rospy.init_node('reading_laser')
    
    lp = lg.LaserProjection()

    pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
    
    rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=1)
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rospy.Subscriber('/pid_tuning_altitude', PidTune, wall_follow)

    long_error = rospy.Publisher('/long_error',Float32, queue_size=1)
    zero_error = rospy.Publisher('/zero_error',Float32, queue_size=1)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        
        msg = Twist()
        '''
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
        elif state_ == 3:
            msg = turn_around()
        else:
            rospy.logerr('Unknown state!')
        '''

        pub_.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()
