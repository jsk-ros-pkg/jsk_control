cmake_minimum_required(VERSION 2.8.3)
project(jsk_kalman_filter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  roscpp
  pluginlib
)

find_package(Eigen3 REQUIRED)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")

generate_dynamic_reconfigure_options(
  cfg/KalmanFilter.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp pluginlib
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


## pluginlib_tutorials library
add_library(kf_pos_vel_acc_pluginlib src/kf_pos_vel_acc_plugin.cpp)
target_link_libraries(kf_pos_vel_acc_pluginlib ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(kf_pos_vel_acc_pluginlib ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS kf_pos_vel_acc_pluginlib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
