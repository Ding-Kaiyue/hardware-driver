cmake_minimum_required(VERSION 3.16)
project(dm_tools)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 检测是否在ROS2环境中
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  # ROS2 编译模式
  message(STATUS "Building dm_tools with ROS2 support")
  
  # 查找ROS2依赖
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  
  # 查找 libusb-1.0 库
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
  
  # Include directories
  include_directories(${LIBUSB_INCLUDE_DIRS})
  
  # ROS2设备扫描器节点
  add_executable(device_scanner_node
    src/device_scanner_node.cpp
  )
  
  ament_target_dependencies(device_scanner_node
    rclcpp
    std_msgs
  )
  
  target_link_libraries(device_scanner_node
    ${LIBUSB_LIBRARIES}
  )
  
  # 独立的dev_sn工具
  add_executable(dev_sn
    src/dev_sn.cpp
  )
  
  target_link_libraries(dev_sn
    ${LIBUSB_LIBRARIES}
  )
  
  # 安装目标
  install(TARGETS
    device_scanner_node
    dev_sn
    DESTINATION lib/${PROJECT_NAME}
  )
  
  # 安装启动文件
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )
  
  ament_package()
  
else()
  # 独立编译模式
  message(STATUS "Building dm_tools in standalone mode")
  
  # 查找 libusb-1.0 库
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
  
  # Include directories
  include_directories(${LIBUSB_INCLUDE_DIRS})
  
  # Device serial number detector tool
  add_executable(dev_sn
    src/dev_sn.cpp
  )
  
  target_link_libraries(dev_sn
    ${LIBUSB_LIBRARIES}
  )
  
  # Install (optional)
  install(TARGETS dev_sn
    DESTINATION bin
  )
  
endif()