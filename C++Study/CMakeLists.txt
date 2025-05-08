cmake_minimum_required(VERSION 3.16)
project(test)

set(CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)

add_executable(test test.cpp)

target_link_libraries(test PRIVATE Eigen3::Eigen pthread)