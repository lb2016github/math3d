cmake_minimum_required(VERSION 3.14)
set(PROJECT_NAME "math3d-client")
project(${PROJECT_NAME})

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_executable(${PROJECT_NAME} ${SOURCE_FILES} test/math3d-client.cpp)