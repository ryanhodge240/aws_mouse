## For full information http://wiki.ros.org/catkin/CMakeLists.txt 

#############################################################################
## Set minimum required version of cmake, project name and compile options ##
#############################################################################
cmake_minimum_required(VERSION 3.0.2)
project(maze_demo)

###########################################################################
## Find catkin packages and libraries for catkin and system dependencies ##
###########################################################################
find_package(catkin REQUIRED COMPONENTS
   rospy
)

##########################################
## Setup for python modules and scripts ##
##########################################
catkin_python_setup()
################################################
## Declare ROS messages, services and actions ##
################################################
# No new message declared.

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
# No new parameters declared.

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS scripts/maze_runner.py scripts/test_runner.py scripts/mm_movement.py
       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY launch scripts
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
# No unit test performed.
