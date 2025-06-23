cmake_minimum_required(VERSION 3.10)
project(OpenCV_Example)

# 查找OpenCV包
find_package(OpenCV REQUIRED)

add_executable(OpenCV_Example practice.cpp)

target_link_libraries(OpenCV_Example PRIVATE ${OpenCV_LIBS})