cmake_minimum_required(VERSION 2.8.3)
project(eus_osqp)

find_package(catkin REQUIRED COMPONENTS rostest)

include(ExternalProject)

ExternalProject_Add(osqpproj
  GIT_REPOSITORY https://github.com/oxfordcontrol/osqp.git
  GIT_TAG v0.6.0
  TIMEOUT 30
  CMAKE_ARGS -DDFLOAT=OFF
  BUILD_COMMAND make -j
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND sh -c "cp -r include/* ${PROJECT_SOURCE_DIR}/include/" && sh -c "rm -fr ${PROJECT_SOURCE_DIR}/include/osqp/.gitignore" && cp out/libosqp.so ${PROJECT_SOURCE_DIR}/lib/
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
ExternalProject_Get_Property(osqpproj source_dir)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${Boost_INCLUDE_DIRS})
link_directories(lib)

add_library(eus_osqp SHARED src/eus_osqp.cpp)
set_target_properties(eus_osqp PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_dependencies(eus_osqp osqpproj)
target_link_libraries(eus_osqp osqp)

add_rostest(test/eus_osqp.test)
add_rostest(test/eus_osqp_contact_wrench_opt.test)
add_rostest(test/eus_osqp_model_predictive_control.test)

install(DIRECTORY lib/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY euslisp
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS eus_osqp # lib/ is added here to install euslisp library
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib)
install(DIRECTORY include/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
