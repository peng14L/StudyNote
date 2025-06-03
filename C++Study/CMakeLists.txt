cmake_minimum_required(VERSION 3.16)
project(yamlRead)

set(CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(yamlRead yamlRead.cpp)
message(${CMAKE_CURRENT_SOURCE_DIR})
message(${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(yamlRead PRIVATE Eigen3::Eigen pthread yaml-cpp)