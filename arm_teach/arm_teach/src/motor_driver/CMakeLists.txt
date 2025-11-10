cmake_minimum_required(VERSION 3.16)
project(dm_motor)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 检测是否在ROS2环境中
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  # ROS2 编译模式
  message(STATUS "Building dm_motor with ROS2 support")
  
  # 查找ROS2依赖
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  
  # 查找系统依赖
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
  find_package(Threads REQUIRED)
  
  # Include directories
  include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LIBUSB_INCLUDE_DIRS}
  )
  
  # 编译封装DM_MOTOR库
  add_library(motor STATIC
      src/protocol/damiao.cpp 
  )
  
  target_include_directories(motor PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${LIBUSB_INCLUDE_DIRS}
  )
  
  target_link_libraries(motor
      Threads::Threads
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libu2canfd.a
      ${LIBUSB_LIBRARIES}
  )
  
  # ROS2机械臂控制节点
  add_executable(arm_controller_node
    src/arm_controller_node.cpp
  )
  
  target_link_libraries(arm_controller_node
    motor
  )
  
  ament_target_dependencies(arm_controller_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
  )
  
  # 独立可执行文件 (向后兼容)
  add_executable(dm_motor_main
      src/main.cpp
  )
  
  target_link_libraries(dm_motor_main
      motor
  )
  
  # 安装目标
  install(TARGETS 
    arm_controller_node
    dm_motor_main
    DESTINATION lib/${PROJECT_NAME}
  )
  
  # 安装启动文件
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
    OPTIONAL
  )
  
  # 安装配置文件
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
    OPTIONAL
  )
  
  ament_package()
  
else()
  # 独立编译模式
  message(STATUS "Building dm_motor in standalone mode")
  
  find_package(Threads REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
  
  # 编译封装DM_MOTOR库
  add_library(motor STATIC
      src/protocol/damiao.cpp 
  )
  
  target_include_directories(motor PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${LIBUSB_INCLUDE_DIRS}
  )
  
  target_link_libraries(motor PRIVATE
      Threads::Threads
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libu2canfd.a
      ${LIBUSB_LIBRARIES}
  )
  
  # 添加可执行文件
  add_executable(dm_motor_main
      src/main.cpp
  )
  
  target_link_libraries(dm_motor_main PRIVATE
      motor
  )
  
  # 安装目标 (可选)
  install(TARGETS dm_motor_main
    DESTINATION bin
  )

endif()















