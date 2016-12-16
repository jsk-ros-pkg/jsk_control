cmake_minimum_required(VERSION 2.8.3)
project(eus_qpoases)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_package()

include(ExternalProject)

ExternalProject_Add(qpoases
  SVN_REPOSITORY https://projects.coin-or.org/svn/qpOASES/stable/3.0
  SVN_TRUST_CERT 1
  TIMEOUT 30
  PATCH_COMMAND patch -s -p0 < ${PROJECT_SOURCE_DIR}/patch/UseShareLibrary.patch
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
  BUILD_COMMAND make -j
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND ""
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
ExternalProject_Get_Property(qpoases source_dir)
link_directories(${source_dir}/libs)
include_directories(${source_dir}/include)

add_executable(example1 examples/example1.cpp)
set_target_properties(example1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_dependencies(example1 qpoases)
target_link_libraries(example1 qpOASES)
add_library(eus_qpoases SHARED src/eus_qpoases.cpp)
set_target_properties(eus_qpoases PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_dependencies(eus_qpoases qpoases)
target_link_libraries(eus_qpoases qpOASES)

add_rostest(test/eus_qpoases.test)

install(DIRECTORY ${source_dir}/libs/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY euslisp examples lib # lib/ is added here to install euslisp library
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY ${source_dir}/include/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
