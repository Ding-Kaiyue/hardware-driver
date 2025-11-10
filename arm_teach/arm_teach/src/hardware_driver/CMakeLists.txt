cmake_minimum_required(VERSION 3.16)
project(hardware_driver)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type and flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "Detected architecture: ${ARCH}")

if(ARCH STREQUAL "x86_64")
    set(PLATFORM "x86_64")
elseif(ARCH MATCHES "aarch64")
    set(PLATFORM "aarch64")
elseif(ARCH MATCHES "armv7" OR ARCH MATCHES "armv8" OR ARCH MATCHES "^arm")
    set(PLATFORM "arm")
elseif(ARCH MATCHES "i686" OR ARCH MATCHES "i386")
    set(PLATFORM "x86")
else()
    set(PLATFORM "unknown")
endif()

message(STATUS "Configured for platform: ${PLATFORM}")

if(PLATFORM STREQUAL "aarch64")
    set(LIB_PATH "arm")
elseif(PLATFORM STREQUAL "x86_64")
    set(LIB_PATH "x86")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O2 -Wall")

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIBUSB_INCLUDE_DIRS}
)

# ========== Protocol Libraries ==========

# Raytron Motor Protocol Library
add_library(raytron_lib STATIC
  src/protocol/raytron_motor_ctrl.cpp
  src/protocol/teaching_device.cpp
)

target_include_directories(raytron_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(raytron_lib
  Threads::Threads
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/${LIB_PATH}/libu2canfd.a
  ${LIBUSB_LIBRARIES}
)

# Gripper Protocol Library
add_library(gripper_lib STATIC
  src/protocol/gripper_ctrl.cpp
)

target_include_directories(gripper_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ========== Hardware Tools Libraries ==========

# USB Device Tools Library
add_library(usb_device_tools_lib STATIC
  src/tools/usb_device_finder.cpp
)

target_include_directories(usb_device_tools_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(usb_device_tools_lib
  ${LIBUSB_LIBRARIES}
)

# USB2CANFD Manager Library
add_library(usb2canfd_manager_lib STATIC
  src/hardware/usb2canfd_manager.cpp
)

target_include_directories(usb2canfd_manager_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(usb2canfd_manager_lib
  Threads::Threads
  usb_device_tools_lib
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/${LIB_PATH}/libu2canfd.a
  ${LIBUSB_LIBRARIES}
)

# ========== Executables ==========

# Hardware Driver ROS2 Node
add_executable(hardware_driver
  src/hardware_driver.cpp
)

ament_target_dependencies(hardware_driver
  rclcpp
  std_msgs
  sensor_msgs
)

target_include_directories(hardware_driver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(hardware_driver
  Threads::Threads
  raytron_lib
  gripper_lib
  usb2canfd_manager_lib
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/${LIB_PATH}/libu2canfd.a
  ${LIBUSB_LIBRARIES}
)

# Standalone CAN communication demo
add_executable(dm_main
  src/hardware_driver_node.cpp
)

target_include_directories(dm_main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(dm_main
  Threads::Threads
  raytron_lib
  gripper_lib
  usb2canfd_manager_lib
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/${LIB_PATH}/libu2canfd.a
  ${LIBUSB_LIBRARIES}
)

# Device list tool
add_executable(list_devices
  src/tools/list_devices.cpp
)

target_include_directories(list_devices PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(list_devices
  usb_device_tools_lib
  ${LIBUSB_LIBRARIES}
)

# ========== Installation ==========

# Install libraries
install(TARGETS
  raytron_lib
  gripper_lib
  usb_device_tools_lib
  usb2canfd_manager_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  hardware_driver
  dm_main
  list_devices
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ========== Package Configuration ==========

ament_package()
