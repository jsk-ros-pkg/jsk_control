project(eus_caffe)

cmake_minimum_required(VERSION 2.4.6)

find_package(catkin COMPONENTS cmake_modules)
find_package(OMPL REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/src)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()
