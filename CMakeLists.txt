
cmake_minimum_required(VERSION 3.5)
project(vlfeat_sample)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/${ADDITIONAL_CMAKE_MODULE_DIR}")

FIND_PACKAGE(OpenCV)
FIND_PACKAGE(Glog)
FIND_PACKAGE(Gflags)
FIND_PACKAGE(Eigen3)
FIND_PACKAGE(OpenImageIO)

include_directories(${CMAKE_SOURCE_DIR})
link_directories(${CMAKE_SOURCE_DIR}/extern/lib)

set(TARGET "sift_test")
set(SRC "src/sift_test.cc")

# Generate Exe Files
add_executable(${TARGET} ${SRC})
target_link_libraries(${TARGET} ${GFLAGS_LIBRARIES})
target_link_libraries(${TARGET} ${GLOG_LIBRARIES})
target_link_libraries(${TARGET} ${OpenCV_LIBS})
target_link_libraries(${TARGET} vl)
target_link_libraries(${TARGET} ${OpenImageIO_LIBRARY})