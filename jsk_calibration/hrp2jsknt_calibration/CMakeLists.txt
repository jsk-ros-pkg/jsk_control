
cmake_minimum_required(VERSION 2.8.3)
project(hironx_calibration)

find_package(catkin)
catkin_package()

install(DIRECTORY capture_data estimate_params # view_results
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

