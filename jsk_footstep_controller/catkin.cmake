cmake_minimum_required(VERSION 2.8.3)
project(jsk_footstep_controller)

find_package(catkin REQUIRED COMPONENTS
  jsk_footstep_msgs
  jsk_footstep_planner
  actionlib_msgs
  jsk_pcl_ros
  tf_conversions
  message_generation genmsg message_filters sensor_msgs geometry_msgs tf jsk_topic_tools
)

add_service_files(FILES
  RequireLog.srv)

add_action_files(
  DIRECTORY action
  FILES LookAroundGround.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_footstep_controller
 CATKIN_DEPENDS jsk_footstep_msgs jsk_footstep_planner message_filters sensor_msgs geometry_msgs tf jsk_topic_tools
#  DEPENDS system_lib
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(footcoords src/footcoords.cpp)
target_link_libraries(footcoords ${catkin_LIBRARIES})

install(TARGETS footcoords
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY euslisp launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
)
