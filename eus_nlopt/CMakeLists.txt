if(NOT (USE_ROSBUILD OR "$ENV{BUILDER}" STREQUAL "rosbuild"))
  message("  eus_nlopt: use catkin_make")
  include(catkin.cmake)
  return()
endif()

project(eus_nlopt)
message("  eus_nlopt: use rosmake")
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
cmake_minimum_required(VERSION 2.4.6)

#rosbuild_init()
#rosbuild_find_ros_package(nlopt)

# Uses Eigen
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})
#include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include("./path-tricker/includes.txt")


find_package(NLopt REQUIRED)
if(NLOPT_FOUND)
  include_directories(${NLOPT_INCLUDE_DIR})
  MESSAGE("-- NLopt found (include: " ${NLOPT_INCLUDE_DIR} ", link: "  ${NLOPT_LIBRARY} ")")
else (NLOPT_FOUND)
  MESSAGE("-- NLopt missing")
endif(NLOPT_FOUND)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(nlopt_test src/test.cpp src/nlopt_solver.cpp)
add_library(nlopt_solver SHARED src/nlopt_solver.cpp)
add_library(nlopt_wrapper SHARED src/nlopt_wrapper.cpp src/nlopt_solver.cpp)
#add_executable(nlopt_wrapper_test src/nlopt_wrapper.cpp src/nlopt_solver.cpp)

#rosbuild_genmsg()

if(NLOPT_FOUND)
  TARGET_LINK_LIBRARIES(nlopt_test ${NLOPT_LIBRARY})
#  TARGET_LINK_LIBRARIES(nlopt_wrapper_test ${NLOPT_LIBRARY})
  TARGET_LINK_LIBRARIES(nlopt_wrapper ${NLOPT_LIBRARY})
endif(NLOPT_FOUND)