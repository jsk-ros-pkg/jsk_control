cmake_minimum_required(VERSION 2.8.3)
project(joy_mouse)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)
catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joy_mouse
  CATKIN_DEPENDS rospy sensor_msgs
#  DEPENDS system_lib
)

install(DIRECTORY
  scripts launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
