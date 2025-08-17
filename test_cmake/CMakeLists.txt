cmake_minimum_required(VERSION 3.8)
project(test_example)

set(CMAKE_CXX_STANDARD 17)

# 使用 find_package 找到已安装的库
find_package(hardware_driver REQUIRED)

add_executable(test_example test_example.cpp)
target_link_libraries(test_example hardware_driver::arm_with_omnipicker_canfd)