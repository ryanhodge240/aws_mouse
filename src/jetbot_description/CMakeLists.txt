cmake_minimum_required(VERSION 3.0.2)

project(jetbot_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

catkin_install_python(PROGRAMS script/runforward.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY config launch meshes meshes-jetbot-ros urdf rviz script
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
)
