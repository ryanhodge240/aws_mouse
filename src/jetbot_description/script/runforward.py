#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from tf import transformations
from std_srvs.srv import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32,String
from tf.transformations import euler_from_quaternion, quaternion_from_euler


rotation_odom =0
rotation_imu =0

def clbk_odom(msg):
    global rotation_odom
    global roll_odom, pitch_odom, yaw_odom
    pose = msg.pose.pose
    pos_x=pose.position.x
    pos_y=pose.position.y
    orientation_q = pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll_odom, pitch_odom, yaw_odom) = euler_from_quaternion (orientation_list)
    Kp=38
    rotation_odom = - Kp*yaw_odom
    print("Odometry yaw: ",yaw_odom)
    
    
def clbk_imu(msg):
    global rotation_imu
    global roll_imu, pitch_imu, yaw_imu
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll_imu, pitch_imu, yaw_imu) = euler_from_quaternion (orientation_list)
    Kp=38
    rotation_imu = - Kp*yaw_imu
    print(" \n IMU yaw: ",yaw_imu)
    

def main():
    global rotation_odom
    global rotation_imu
    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_imu = rospy.Subscriber('/imu', Imu, clbk_imu)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
        
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = rotation_imu
        
        pub_.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
