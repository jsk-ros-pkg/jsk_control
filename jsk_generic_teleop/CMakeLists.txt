cmake_minimum_required(VERSION 2.8.3)
project(jsk_generic_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roseus
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  roseus
  sensor_msgs
  std_msgs
)

file(GLOB PYTHON_SCRIPTS scripts/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config euslisp launch sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
