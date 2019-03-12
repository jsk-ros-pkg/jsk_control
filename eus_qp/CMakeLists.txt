project(eus_qp)

cmake_minimum_required(VERSION 2.4.6)

find_package(catkin COMPONENTS cmake_modules rostest)
find_package(Eigen REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)
include_directories(${PROJECT_SOURCE_DIR}/src)
include("./path-tricker/includes.txt")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()

add_executable(euq_qp_test src/example.cpp)
add_library(eus_qp SHARED src/qp_lib.cpp)
add_rostest(test/test_contact_wrench_opt.test)
add_rostest(test/test_model_predictive_control.test)
add_rostest(test/test_cfr_cwc_calculation.test)
add_rostest(optmotiongen/test/test-sample-inverse-kinematics.test)
add_rostest(optmotiongen/test/test-sample-inverse-kinematics-statics.test)
add_rostest(optmotiongen/test/test-samplerobot.test)
add_rostest(optmotiongen/test/test-sample-torque-gradient.test)
add_rostest(optmotiongen/test/test-load-euslisp-files.test)

install(DIRECTORY euslisp test
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS eus_qp
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY optmotiongen/euslisp optmotiongen/scripts optmotiongen/test
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/optmotiongen)
