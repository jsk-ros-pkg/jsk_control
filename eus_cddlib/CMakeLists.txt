project(eus_cddlib)

cmake_minimum_required(VERSION 2.4.6)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_package()

add_definitions(-DGMPRATIONAL)
add_library(eus_cddlib SHARED src/eus_cddlib.c)
set_target_properties(eus_cddlib PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
target_link_libraries(eus_cddlib cddgmp)
install(DIRECTORY lib/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY euslisp
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS eus_cddlib # lib/ is added here to install euslisp library
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib)
