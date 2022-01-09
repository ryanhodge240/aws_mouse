#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from tf import transformations
from std_srvs.srv import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32,String


rotation =0

def clbk_laser(msg):
    global rotation
    pose = msg.pose.pose
    pos_x=pose.position.x
    pos_y=pose.position.y
    Kp=40
    rotation = - Kp* pos_y
    print(pos_y)
    

def main():
    global rotation
    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/odom', Odometry, clbk_laser)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
        
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = rotation
        
        pub_.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
