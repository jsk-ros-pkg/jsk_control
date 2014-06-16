project(eus_nlopt)

cmake_minimum_required(VERSION 2.4.6)

find_package(catkin REQUIRED COMPONENTS nlopt)
find_package(Eigen REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include("./path-tricker/includes.txt")

## catkin
if(nlopt_FOUND_CATKIN_PROJECT)
  ##
  set(NLOPT_FOUND true)
  if(NOT ${nlopt_INCLUDE_DIRS} STREQUAL "")
    FIND_PATH(NLOPT_INCLUDE_DIR nlopt.h PATHS ${nlopt_INCLUDE_DIRS})
  else()
    FIND_PATH(NLOPT_INCLUDE_DIR nlopt.h PATHS ${nlopt_SOURCE_PREFIX}/include)
  endif()
  ##
  SET (NLOPT_NAMES nlopt nlopt_cxx)
  if(NOT ${nlopt_LIBRARY_DIRS} STREQUAL "")
    FIND_LIBRARY (NLOPT_LIBRARY NAMES ${NLOPT_NAMES} PATHS ${nlopt_LIBRARY_DIRS})
  else()
    FIND_LIBRARY (NLOPT_LIBRARY NAMES ${NLOPT_NAMES} PATHS ${nlopt_SOURCE_PREFIX}/lib)
  endif()
  SET (NLOPT_LIBRARIES ${NLOPT_LIBRARY})
  ##
  include_directories(${NLOPT_INCLUDE_DIR})
  MARK_AS_ADVANCED (NLOPT_LIBRARY NLOPT_INCLUDE_DIR)
  MESSAGE("-- NLopt found (include: " ${NLOPT_INCLUDE_DIR} ", link: "  ${NLOPT_LIBRARY} ")")
else ()
  MESSAGE("-- NLopt missing")
endif()

catkin_package(
  CATKIN_DEPENDS nlopt
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## add_executable(nlopt_test src/test.cpp src/nlopt_solver.cpp)
## add_library(nlopt_solver SHARED src/nlopt_solver.cpp)
add_library(nlopt_wrapper SHARED src/nlopt_wrapper.cpp src/nlopt_solver.cpp)
#add_executable(nlopt_wrapper_test src/nlopt_wrapper.cpp src/nlopt_solver.cpp)

#rosbuild_genmsg()

if(NLOPT_FOUND)
##  TARGET_LINK_LIBRARIES(nlopt_test ${NLOPT_LIBRARY})
##  TARGET_LINK_LIBRARIES(nlopt_wrapper_test ${NLOPT_LIBRARY})
  TARGET_LINK_LIBRARIES(nlopt_wrapper ${NLOPT_LIBRARY})
endif(NLOPT_FOUND)
