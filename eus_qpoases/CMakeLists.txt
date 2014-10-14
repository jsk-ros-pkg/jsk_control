if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f ${PROJECT_SOURCE_DIR}/Makefile.qpOASES
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of qpOASES failed")
endif(_make_failed)

link_directories(${PROJECT_SOURCE_DIR}/build/qpOASES/libs)
include_directories(${PROJECT_SOURCE_DIR}/build/qpOASES-source/include)

add_executable(example1  examples/example1.cpp)
target_link_libraries(example1 qpOASES)
add_library(eus_qpoases SHARED src/eus_qpoases.cpp)
target_link_libraries(eus_qpoases qpOASES)
