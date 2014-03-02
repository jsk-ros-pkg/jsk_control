cmake_minimum_required(VERSION 2.8.3)
project(jsk_footstep_controller)

find_package(catkin REQUIRED COMPONENTS
  jsk_footstep_msgs
  jsk_footstep_planner
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_footstep_controller
 CATKIN_DEPENDS jsk_footstep_msgs jsk_footstep_planner
#  DEPENDS system_lib
)
install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
)
