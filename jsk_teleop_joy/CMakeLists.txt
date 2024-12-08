cmake_minimum_required(VERSION 2.8.3)
project(jsk_teleop_joy)

find_package(catkin REQUIRED) # pr2_controllers_msgs robot_monitor
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  
  )

file(GLOB PYTHON_SCRIPTS scripts/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch configs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  )

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


