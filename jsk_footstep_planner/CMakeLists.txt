cmake_minimum_required(VERSION 2.8.3)
project(jsk_footstep_planner)

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

find_package(catkin REQUIRED COMPONENTS jsk_footstep_msgs jsk_recognition_utils
  message_generation jsk_topic_tools roscpp pcl_ros geometry_msgs eigen_conversions
  jsk_interactive_marker visualization_msgs dynamic_reconfigure roseus rostest)
find_package(PCL REQUIRED)
add_service_files(FILES
  ChangeSuccessor.srv CollisionBoundingBoxInfo.srv ProjectFootstep.srv)
generate_dynamic_reconfigure_options(
  cfg/FootstepPlanner.cfg cfg/PointCloudModelGenerator.cfg cfg/FootstepMarker.cfg)
generate_messages(DEPENDENCIES geometry_msgs jsk_footstep_msgs)
#catkin_python_setup()
catkin_package(
  DEPENDS
  CATKIN_DEPENDS jsk_footstep_msgs jsk_recognition_utils jsk_interactive_marker
  INCLUDE_DIRS include # TODO include
  LIBRARIES jsk_footstep_planner # TODO
  )

# add_definitions("-g -O2 -DEIGEN_NO_DEBUG")
add_definitions("-g -O2")
find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_libraries(${pcl_ros_LIBRARIES} ${catkin_LIBRARIES})
add_library(jsk_footstep_planner
  src/footstep_conversions.cpp
  src/marker_array_publisher.cpp
  src/simple_neighbored_graph.cpp
  src/footstep_state.cpp
  src/footstep_graph.cpp
  src/footstep_state_discrete_close_list.cpp
  src/ann_grid.cpp
  src/pointcloud_model_generator.cpp
  src/transition_limit.cpp
  src/footstep_planner.cpp
  src/line2d.cpp)

add_executable(footstep_marker src/footstep_marker.cpp)
target_link_libraries(footstep_marker jsk_footstep_planner)

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
add_executable(bench_footstep_planner bench/bench_footstep_planner.cpp)
target_link_libraries(bench_footstep_planner jsk_footstep_planner)
add_executable(ann_grid_demo demo/ann_grid_demo.cpp)
target_link_libraries(ann_grid_demo jsk_footstep_planner)
add_executable(footstep_planner_node src/footstep_planner_node.cpp)
target_link_libraries(footstep_planner_node jsk_footstep_planner)
add_executable(footstep_projection_demo demo/footstep_projection_demo.cpp)
target_link_libraries(footstep_projection_demo jsk_footstep_planner)
add_executable(pointcloud_model_generator_node src/pointcloud_model_generator_node.cpp)
target_link_libraries(pointcloud_model_generator_node jsk_footstep_planner)

add_dependencies(jsk_footstep_planner ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
add_dependencies(footstep_planner_node ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
add_dependencies(pointcloud_model_generator_node ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

install(DIRECTORY euslisp launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  )
install(TARGETS jsk_footstep_planner sample_astar sample_simple_graph
  footstep_planning_2d_demo footstep_planning_2d_interactive_demo
  footstep_planning_2d_perception_demo
  footstep_projection_demo
  bench_footstep_planner
  ann_grid_demo
  footstep_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_rostest(test/test_footstep_planning_eus_client.test)