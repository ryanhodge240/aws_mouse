#!/usr/bin/env python
import sys
import rospy
from mouse_common.utils import Micromouse_Node

wall_distance = 0.2
wall_distance_forward = 0.15
wall_distance_side = 0.1

class MazeRunner(object):
    def __init__(self):
        rospy.init_node("mouse_node", anonymous=False)
        micromouse_node = Micromouse_Node()
        self.runMaze(micromouse_node)
        rospy.on_shutdown(micromouse_node.on_shutdown)   
        rospy.spin()    

    def runMaze(self, mn):
        # mn is the mouse node from MicroMouse_Node object
        rospy.loginfo('starting....')
        step = 0
        while not rospy.is_shutdown():
#            print(' left:front:right {:.3f}--{:.3f}--{:.3f}'.format(mn.laser_sensors['l'], mn.laser_sensors['f'], mn.laser_sensors['r']))
            # if all three side are great thant the threshold, should go forward
            if (mn.laser_sensors['l']>wall_distance_side):
                #move forward distance one cell is 0.31 meters    
                fdist=0.3
                if  (mn.laser_sensors['f']>fdist):
                    dist=mn.move_onecell(fdist)
                    step = step + 1
                    print("distance travelled in this cell is {:.4f}-{}".format(dist, step))
                    print("classfication is {} ".format(mn.getPredication()))
                else:
                    dist = mn.laser_sensors['f'] 
                    mn.move_onecell(dist)	


if __name__ == "__main__" :
    mr = MazeRunner()  
    
    
