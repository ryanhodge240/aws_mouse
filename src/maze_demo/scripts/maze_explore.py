#!/usr/bin/env 
'''

'''

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf import transformations



# Global Variables
sensor_l, sensor_c, sensor_r = 0, 0, 0
pub = 0 


## get the range at front, left and right from laser scan
def clbk_laser():
    global sensor_l, sensor_c, sensor_r
    msg = LaserScan()
    regions = [
        round(100*min(min(msg.ranges[0:143]), 100)),
        round(100*min(min(msg.ranges[144:287]), 100)),
        round(100*min(min(msg.ranges[288:431]), 100)),
        round(100*min(min(msg.ranges[432:575]), 100)),
        round(100*min(min(msg.ranges[576:713]), 100)),
    ]
    
    sensor_l = regions[4]
    sensor_c = regions[2]
    sensor_r = regions[0]



## robot motions
# move forward
def motion_go_straight():
    global pub
    msg = Twist()
    # linear motion
    msg.linear.x = 0.08
    msg.linear.y = 0.00
    msg.linear.z = 0.00
    # angular motion
    msg.angular.x = 0.00
    msg.angular.y = 0.00
    msg.angular.z = 0.00
    pub.publish(msg)


# move backward
def motion_go_back():
    global pub
    msg = Twist()
    # linear motion
    msg.linear.x = -0.08
    msg.linear.y = 0.00
    msg.linear.z = 0.00
    # angular motion
    msg.angular.x = 0.00
    msg.angular.y = 0.00
    msg.angular.z = 0.00
    pub.publish(msg)


# turn right
def motion_turn_right():
    global pub
    msg = Twist()
    # linear motion
    msg.linear.x = 0.00
    msg.linear.y = 0.00
    msg.linear.z = 0.00
    # angular motion
    msg.angular.x = 0.00
    msg.angular.y = 0.00
    msg.angular.z = 1.57
    pub.publish(msg)


# turn left
def motion_turn_left():
    global pub
    msg = Twist()
    # linear motion
    msg.linear.x = 0.00
    msg.linear.y = 0.00
    msg.linear.z = 0.00
    # angular motion
    msg.angular.x = 0.00
    msg.angular.y = 0.00
    msg.angular.z = -1.57
    pub.publish(msg)


# stop
def motion_stop():
    global pub
    msg = Twist()
    # linear motion
    msg.linear.x = 0.00
    msg.linear.y = 0.00
    msg.linear.z = 0.00
    # angular motion
    msg.angular.x = 0.00
    msg.angular.y = 0.00
    msg.angular.z = 0.00
    pub.publish(msg)



## main loop
def main():
    
    global sensor_l, sensor_c, sensor_r
    global pub

    # intialize the node
    rospy.init_node('maze_explore')
    
    # publish clock msg
    rospy.Publisher('/clock', clock, queue_size=1).publish()

    # subscribe to laser scan and get the ranges from funtion clbk_laser
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser())

    # publish the twist msg
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        
        
        # front is open
        if(sensor_c > 10):
            motion_go_straight()
        # front is blocked, left is open
        elif((sensor_c < 10) and (sensor_l) > 10):
            motion_turn_left()
        # front is blocked, left is blocked, right is open
        elif((sensor_c < 10) and (sensor_l) < 10 and (sensor_r > 10)):
            motion_turn_right()
        else:
            motion_stop()

        rate.sleep()

if __name__ == '__main__':
    main()
   


''' 
## Subscriber
def sensor():
    rospy.init_node('sensor', anonymous=True)

    rospy.Subscriber('/scan', LaserScan)
    rospy.Subscriber('/camera/rgb/image_raw', Image)
    rospy.Subscriber('/odom', Odometry)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    sensor()



## Publisher
def data():
    pub_range = rospy.Publisher('/scan', LaserScan)
    pub_image = rospy.Publisher('/camera/rgb/image_raw', Image)
    pub_odometry = rospy.Publisher('/odom', Odometry)
    rospy.init_node('data', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub_range.publish()
        pub_image.publish()
        pub_odometry.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        data()
    except rospy.ROSInterruptException:
        pass


'''


