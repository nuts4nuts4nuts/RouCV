cmake_minimum_required(VERSION 2.6)

project(Physics_Tracker)

include_directories(${CMAKE_SOURCE_DIR}/thirdparty/opencv/include)

link_directories(${CMAKE_SOURCE_DIR}/thirdparty/opencv/lib)

set(SOURCE source)

add_executable(Physics_Tracker ${SOURCE}/main.cpp)

set(LIBS opencv_world320d)

target_link_libraries(Physics_Tracker ${LIBS})