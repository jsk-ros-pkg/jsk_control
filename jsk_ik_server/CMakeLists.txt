cmake_minimum_required(VERSION 2.8.3)
project(jsk_ik_server)

if($ENV{ROS_DISTRO} STREQUAL "indigo") # indigo requres to add find_pacakge(roseus) to compile roseus message
  set(ROSEUS_package roseus)
endif()

find_package(catkin REQUIRED COMPONENTS tf moveit_msgs rostest cmake_modules mk ${ROSEUS_package})

catkin_package(CATKIN_DEPENDS roseus tf moveit_msgs rostest cmake_modules mk ${ROSEUS_package})

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  )


