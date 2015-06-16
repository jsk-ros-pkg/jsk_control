cmake_minimum_required(VERSION 2.8.3)
project(eus_qpoases)

find_package(catkin REQUIRED COMPONENTS mk)

execute_process(
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR}
  make -f ${PROJECT_SOURCE_DIR}/Makefile.qpOASES
  PATCH_DIR=${PROJECT_SOURCE_DIR}
  MK_DIR=${mk_PREFIX}/share/mk
  RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of qpOASES failed")
else (_make_failed)
  add_custom_target(qpOASES_built)
endif(_make_failed)

catkin_package()

link_directories(${CMAKE_CURRENT_BINARY_DIR}/build/qpOASES/libs)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/build/qpOASES-source/include)

add_executable(example1 ${PROJECT_SOURCE_DIR}/examples/example1.cpp)
set_target_properties(example1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_dependencies(example1 qpOASES_built)
target_link_libraries(example1 qpOASES)
add_library(eus_qpoases SHARED ${PROJECT_SOURCE_DIR}/src/eus_qpoases.cpp)
set_target_properties(eus_qpoases PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_dependencies(eus_qpoases qpOASES_built)
target_link_libraries(eus_qpoases qpOASES)


