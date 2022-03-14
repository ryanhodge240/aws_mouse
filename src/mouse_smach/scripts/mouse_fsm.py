#!/usr/bin/env python
from utils import Micromouse_Node
import smach, smach_ros, rospy
from smach import State, StateMachine,  CBState

wall_distance = 0.2
wall_distance_forward = 0.15
wall_distance_side = 0.1

class Mouse_FSM(object):
    def __init__(self):
        rospy.init_node("mouse_node", anonymous=False)
        self.mn = Micromouse_Node()
        rospy.on_shutdown(self.mn.on_shutdown)   


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
    	if (self.mn.laser_sensors['f']>wall_distance_forward):
   	    return 'forward'
    	else:
   	    return 'IDEL'

    @smach.cb_interface(input_keys=['lspeed'], output_keys=[], outcomes=['succeeded'])
    def cb_forward(user_data, self):
    	if (self.mn.laser_sensors['f']>wall_distance_forward):
    	    self.mn.move_onecell(0.32)
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

if __name__ == "__main__" :
  
    mf= Mouse_FSM()
    
    
