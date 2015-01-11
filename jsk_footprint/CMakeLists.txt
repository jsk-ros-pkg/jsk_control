cmake_minimum_required(VERSION 2.8.3)
project(jsk_footprint)

find_package(catkin REQUIRED COMPONENTS
  pcl_msgs
  pcl_ros
  roscpp
  roseus
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  dynamic_reconfigure
)
catkin_package(
  CATKIN_DEPENDS pcl_msgs pcl_ros roscpp roseus rospy sensor_msgs std_msgs tf tf2 dynamic_reconfigure
  DEPENDS pcl
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(costmap_footorint_reconfigure src/costmap_footorint_reconfigure.cpp)
add_dependencies(costmap_footorint_reconfigure jsk_footprint_generate_messages_cpp)
target_link_libraries(costmap_footorint_reconfigure
  ${catkin_LIBRARIES}
  )