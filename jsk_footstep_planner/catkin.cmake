cmake_minimum_required(VERSION 2.8.3)
project(jsk_footstep_planner)

find_package(catkin REQUIRED COMPONENTS jsk_footstep_msgs)
catkin_package(
  DEPENDS
  CATKIN-DEPENDS  jsk_footstep_msgs
  INCLUDE_DIRS # TODO include
  LIBRARIES # TODO
  )

install(DIRECTORY euslisp launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
  )