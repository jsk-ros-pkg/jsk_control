cmake_minimum_required(VERSION 2.8.3)
project(jsk_footstep_planner)

find_package(catkin REQUIRED COMPONENTS jsk_footstep_msgs jsk_pcl_ros
  message_generation jsk_topic_tools roscpp)

add_service_files(FILES
  ChangeSuccessor.srv)
generate_messages()
#catkin_python_setup()
catkin_package(
  DEPENDS
  CATKIN_DEPENDS  jsk_footstep_msgs jsk_pcl_ros
  INCLUDE_DIRS # TODO include
  LIBRARIES jsk_footstep_planner # TODO
  )

add_definitions("-g -O2")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(jsk_footstep_planner src/simple_neighbored_graph.cpp)

add_executable(sample_simple_graph demo/sample_simple_graph.cpp)
target_link_libraries(sample_simple_graph ${catkin_LIBRARIES} jsk_footstep_planner)
add_executable(sample_astar demo/sample_astar.cpp)
target_link_libraries(sample_astar ${catkin_LIBRARIES} jsk_footstep_planner)

install(DIRECTORY euslisp launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  )
install(TARGETS jsk_footstep_planner sample_astar sample_simple_graph
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})