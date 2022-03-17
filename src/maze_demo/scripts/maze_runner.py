#!/usr/bin/env python

# Importing packages will allow us to use specific functions to simplify our code
# sys: Allows us to interact more closely with the interpreter
import sys
# rospy: Allows for interaction with ROS nodes and topics 
import rospy
# Micromouse_Node: A Node to manage Micromouse behavior. More can be found on this in the maze_demo/src/mouse_common/utils.py file
from mouse_common.utils import Micromouse_Node


# Minimum Safety Range to Wall
wall_distance = 0.2

# Allowable Distance to Stop Moving Forward
wall_distance_forward = 0.15

# Allowable Distance to Stop Moving in Other Directions
wall_distance_side = 0.1

class MazeRunner(object):
    
    # A Python Class creates an object that we can provide with methods. These methods will allow our object to perform some tasks. 
    # MazeRunner is the name of our class, and we can instantiate a MazeRunner Object with the properties defined in the __init__
    
    def __init__(self):

        # Every Python class must have an __init__(self) method to provide every object of that type with a set of properties

        # Start Mouse Node 
        rospy.init_node("mouse_node", anonymous=False)

        # Instantiate a new copy of the Micromouse_Node object 
        micromouse_node = Micromouse_Node()

        # Rune Maze with out new object 
        self.runMaze(micromouse_node)
        
        # Shut down node when finished
        rospy.on_shutdown(micromouse_node.on_shutdown)   

        # Require ctrl+c to cancel process
        rospy.spin()    


    def runMaze(self, mn):
        # Inputs: 
            # self: MazeRunner Object
            # mn: Mouse Node from MazeRunner Object
        # Function:
            # While rospy is running this method will check the left and front laser scan topics
            # and decide to move forward one cell or rotate 

        # Log Info 
        rospy.loginfo('starting....')

        # Start Counting
        step = 0

        # While ROS is running
        while not rospy.is_shutdown():
            # if all three side are great thant the threshold, should go forward
            if (mn.laser_sensors['left']>wall_distance_side):

                # Set Distance Moved With Each Forward Step  
                fdist=0.3

                # This case statement will select what to do given sensor data 
                if  (mn.laser_sensors['front']>fdist):
                    # When there is room asdin front
                    # Use the move_oncecell function of our mn object
                    dist=mn.move_onecell(fdist)
                    # Add a count 
                    step = step + 1
                    # Log this info to screen
                    print("distance travelled in this cell is {:.4f}-{}".format(dist, step))
                    print("classfication is {} ".format(mn.getPredication()))

                # What can we do when the laser detects a wall in front of us?
                else:
                    dist = mn.laser_sensors['front'] 
                    mn.move_onecell(dist)	

# Run the MazeRunner
if __name__ == "__main__" :
    mr = MazeRunner()  
    
    
