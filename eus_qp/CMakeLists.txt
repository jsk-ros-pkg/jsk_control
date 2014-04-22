project(eus_qp)

cmake_minimum_required(VERSION 2.4.6)

find_package(catkin COMPONENTS cmake_modules)
find_package(Eigen REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)
include_directories(${PROJECT_SOURCE_DIR}/src)
include("./path-tricker/includes.txt")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(test src/example.cpp)
add_library(eus_qp SHARED src/qp_lib.cpp)
