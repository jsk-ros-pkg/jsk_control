project(eus_caffe)

cmake_minimum_required(VERSION 2.4.6)

find_package(catkin COMPONENTS cmake_modules)
## find_package(OMPL REQUIRED)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} bash make.sh)
set(CAFFE_ROOT_DIR ${PROJECT_SOURCE_DIR}/3rdparty/caffe)
# execute_process(COMMAND locate caffe
#   COMMAND grep -e "/caffe$"
#   COMMAND head -1
#   OUTPUT_VARIABLE CAFFE_ROOT_DIR
#   OUTPUT_STRIP_TRAILING_WHITESPACE
#   )
MESSAGE(-- find_caffe -- ${CAFFE_ROOT_DIR})

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${CAFFE_ROOT_DIR}/include)
include_directories(${CAFFE_ROOT_DIR}/.build_release/src)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "-DCPU_ONLY -std=c++0x -lglog")

catkin_package()

## add_executable(eus_caffe_test src/test.cpp)
## TARGET_LINK_LIBRARIES(eus_caffe_test /home/noda/prog/euslib/demo/s-noda/tmp-ros-package/caffe/.build_release/lib/libcaffe.so /usr/local/lib/libglog.so.0)

add_library(eus_caffe SHARED src/eus_caffe_lib.cpp)
target_link_libraries(eus_caffe ${CAFFE_ROOT_DIR}/.build_release/lib/libcaffe.so ${CAFFE_ROOT_DIR}/3rdparty/glog-0.3.3/.libs/libglog.so.0)

add_library(eus_caffe_db SHARED src/eus_caffe_db_lib.cpp)
target_link_libraries(eus_caffe_db ${CAFFE_ROOT_DIR}/.build_release/lib/libcaffe.so ${CAFFE_ROOT_DIR}/3rdparty/glog-0.3.3/.libs/libglog.so.0)

execute_process(COMMAND ln -s ${CAFFE_ROOT_DIR}/3rdparty/glog-0.3.3/.libs/libglog.so.0 ${CATKIN_DEVEL_PREFIX}/lib)
execute_process(COMMAND ln -s ${CAFFE_ROOT_DIR}/3rdparty/mdb/libraries/liblmdb/liblmdb.so ${CATKIN_DEVEL_PREFIX}/lib)
