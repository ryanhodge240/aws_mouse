cmake_minimum_required(VERSION 2.8.3)
project(maze_world)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package(DEPENDS gazebo_ros)

install(DIRECTORY launch world models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
