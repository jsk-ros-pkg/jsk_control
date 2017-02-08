cmake_minimum_required(VERSION 2.8.3)
project(eus_qpoases)

find_package(catkin REQUIRED COMPONENTS rostest)

include(ExternalProject)

ExternalProject_Add(qpoases
  SVN_REPOSITORY https://projects.coin-or.org/svn/qpOASES/stable/3.0
  SVN_TRUST_CERT 1
  TIMEOUT 30
  PATCH_COMMAND patch -s -p0 < ${PROJECT_SOURCE_DIR}/patch/UseShareLibrary.patch
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
  BUILD_COMMAND make -j
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND sh -c "cp -r include/* ${PROJECT_SOURCE_DIR}/include/" && sh -c "rm -fr ${PROJECT_SOURCE_DIR}/include/qpOASES/.svn ${PROJECT_SOURCE_DIR}/include/qpOASES/extras/.svn" && cp libs/libqpOASES.so ${PROJECT_SOURCE_DIR}/lib/ && cp libs/libqpOASES.so.3.0 ${PROJECT_SOURCE_DIR}/lib/
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
ExternalProject_Get_Property(qpoases source_dir)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(include)
link_directories(lib)

add_executable(example1 examples/example1.cpp)
set_target_properties(example1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_dependencies(example1 qpoases)
target_link_libraries(example1 qpOASES)
add_library(eus_qpoases SHARED src/eus_qpoases.cpp)
set_target_properties(eus_qpoases PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_dependencies(eus_qpoases qpoases)
target_link_libraries(eus_qpoases qpOASES)

add_rostest(test/eus_qpoases.test)

install(DIRECTORY lib/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY euslisp examples
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS eus_qpoases # lib/ is added here to install euslisp library
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib)
install(DIRECTORY include/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
