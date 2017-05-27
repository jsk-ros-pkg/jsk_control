cmake_minimum_required(VERSION 2.8.3)
project(cmd_vel_smoother)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
  cfg/CmdVelSmoother.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(cmd_vel_smoother src/cmd_vel_smoother.cpp)
target_link_libraries(cmd_vel_smoother
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(cmd_vel_smoother ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_genmsg)

install(TARGETS cmd_vel_smoother
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
