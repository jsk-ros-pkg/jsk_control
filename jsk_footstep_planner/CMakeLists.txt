cmake_minimum_required(VERSION 2.8.3)
project(jsk_footstep_planner)

find_package(catkin REQUIRED COMPONENTS jsk_footstep_msgs jsk_pcl_ros
  message_generation jsk_topic_tools roscpp pcl_ros
  jsk_interactive_marker visualization_msgs)
find_package(PCL REQUIRED)
add_service_files(FILES
  ChangeSuccessor.srv)
generate_messages()
#catkin_python_setup()
catkin_package(
  DEPENDS
  CATKIN_DEPENDS  jsk_footstep_msgs jsk_pcl_ros jsk_interactive_marker
  INCLUDE_DIRS include # TODO include
  LIBRARIES jsk_footstep_planner # TODO
  )

add_definitions("-g -O2")
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_libraries(${pcl_ros_LIBRARIES} ${catkin_LIBRARIES})
add_library(jsk_footstep_planner
  src/simple_neighbored_graph.cpp
  src/footstep_state.cpp
  src/footstep_graph.cpp
  src/footstep_state_discrete_close_list.cpp)

add_executable(sample_simple_graph demo/sample_simple_graph.cpp)
target_link_libraries(sample_simple_graph jsk_footstep_planner)
add_executable(sample_astar demo/sample_astar.cpp)
target_link_libraries(sample_astar jsk_footstep_planner)
add_executable(footstep_planning_2d_demo demo/footstep_planning_2d_demo.cpp)
target_link_libraries(footstep_planning_2d_demo jsk_footstep_planner)
add_executable(footstep_planning_2d_interactive_demo demo/footstep_planning_2d_interactive_demo.cpp)
target_link_libraries(footstep_planning_2d_interactive_demo jsk_footstep_planner)
add_executable(footstep_planning_2d_perception_demo demo/footstep_planning_2d_perception_demo.cpp)
target_link_libraries(footstep_planning_2d_perception_demo jsk_footstep_planner)

add_executable(footstep_projection_demo demo/footstep_projection_demo.cpp)
target_link_libraries(footstep_projection_demo jsk_footstep_planner)

install(DIRECTORY euslisp launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  )
install(TARGETS jsk_footstep_planner sample_astar sample_simple_graph
  footstep_planning_2d_demo footstep_planning_2d_interactive_demo
  footstep_planning_2d_perception_demo
  footstep_projection_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
