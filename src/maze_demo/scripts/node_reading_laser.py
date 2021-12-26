#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

def avg(lst):
    return sum(lst)/len(lst)

def clbk_laser(msg):
    # 720 / 5 = 144
    regions = {
        'right':  min(avg(msg.ranges[0:10]),10.0),
        'fright': min(avg(msg.ranges[30:40]),10.0),
        'front':  min(avg(msg.ranges[85:95]),10.0),
        'fleft':  min(avg(msg.ranges[140:150]),10.0),
        'left':   min(avg(msg.ranges[170:180]),10.0),
    	}
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rospy.spin()

if __name__ == '__main__':
    main()
