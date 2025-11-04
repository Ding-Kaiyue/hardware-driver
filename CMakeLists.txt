cmake_minimum_required(VERSION 3.10...3.30)
project(hardware_driver)

# 防止in-source构建 - 强制使用build目录
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
  message(FATAL_ERROR "
禁止在源目录中进行构建。请使用以下方式构建：
  mkdir build
  cd build
  cmake ..
  make
")
endif()

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)

# 添加构建测试的选项，默认为 OFF
option(BUILD_TESTS "Build unit tests" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 只有在启用测试时才查找 GTest
if(BUILD_TESTS)
  find_package(GTest)
endif()
find_package(Threads REQUIRED)

# TBB库已移除，使用std::unordered_map + std::shared_mutex替代

# === canfd 组合 ===
add_library(hardware_driver_canfd SHARED
  src/bus/canfd_bus_impl.cpp
  src/driver/motor_driver_impl.cpp
  src/driver/gripper_driver_impl.cpp
  src/protocol/motor_protocol.cpp
  src/protocol/gripper_omnipicker_protocol.cpp
  src/protocol/iap_protocol.cpp
  src/interface/robot_hardware.cpp
)
# 设置链接库
set(HARDWARE_DRIVER_LIBS Threads::Threads)

target_link_libraries(hardware_driver_canfd ${HARDWARE_DRIVER_LIBS})
target_include_directories(hardware_driver_canfd
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE src
)

# 设置库的版本信息
set_target_properties(hardware_driver_canfd PROPERTIES
  VERSION 3.0.0
  SOVERSION 2
  OUTPUT_NAME "hardware_driver"
)

# 设置库的输出目录到lib文件夹
set_target_properties(hardware_driver_canfd PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# === ethercat 组合 ===
# add_library(hardware_driver_ethercat SHARED
#   src/bus/ethercat_bus_impl.cpp
#   src/driver/motor_driver_impl.cpp
#   src/driver/gripper_driver_impl.cpp
#   src/protocol/motor_protocol.cpp
#   src/protocol/gripper_omnipicker_protocol.cpp
#   src/interface/robot_hardware.cpp
# )
# target_link_libraries(hardware_driver_ethercat Threads::Threads)
# target_include_directories(hardware_driver_ethercat PRIVATE src include)
# # 设置库的版本信息
# set_target_properties(hardware_driver_ethercat PROPERTIES
#   VERSION 2.0.0
#   SOVERSION 2
#   OUTPUT_NAME "hardware_driver"
# )

# === 示例程序构建 ===
# 查找所有示例文件并构建到build/examples
file(GLOB EXAMPLE_SOURCES examples/*.cpp)
if(EXAMPLE_SOURCES)
  foreach(EXAMPLE_FILE ${EXAMPLE_SOURCES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
    
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
    set_target_properties(${EXAMPLE_NAME} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )
    target_link_libraries(${EXAMPLE_NAME}
      hardware_driver_canfd
      ${HARDWARE_DRIVER_LIBS}
    )
  endforeach()
endif()

# === 安装头文件和so ===
install(DIRECTORY include/hardware_driver DESTINATION include)

# 创建硬件驱动库的别名
add_library(hardware_driver ALIAS hardware_driver_canfd)

# 设置安装时的库名称
set_target_properties(hardware_driver_canfd PROPERTIES
  LIBRARY_OUTPUT_NAME "hardware_driver"
  ARCHIVE_OUTPUT_NAME "hardware_driver"
)

install(TARGETS
  hardware_driver_canfd
  # hardware_driver_ethercat
  EXPORT hardware_driverTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# 创建配置文件
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/hardware_driverConfigVersion.cmake"
  VERSION 3.0.0
  COMPATIBILITY AnyNewerVersion
)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/hardware_driverConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hardware_driverConfig.cmake"
  @ONLY
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/hardware_driverConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/hardware_driverConfigVersion.cmake"
  DESTINATION lib/cmake/hardware_driver
)

install(EXPORT hardware_driverTargets
  FILE hardware_driverTargets.cmake
  NAMESPACE hardware_driver::
  DESTINATION lib/cmake/hardware_driver
)

# === 单元测试构建 ===
if(BUILD_TESTS)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    enable_testing()
    message(STATUS "GTest found, building tests to build/tests/")
    
    # 查找所有测试文件并构建到build/tests
    file(GLOB TEST_SOURCES tests/*.cpp)
    if(TEST_SOURCES)
      foreach(TEST_FILE ${TEST_SOURCES})
        get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
        
        add_executable(${TEST_NAME} ${TEST_FILE})
        set_target_properties(${TEST_NAME} PROPERTIES
          RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
        )
        
        # 链接所需库
        target_link_libraries(${TEST_NAME}
          hardware_driver_canfd
          GTest::gtest
          GTest::gtest_main
          GTest::gmock
          GTest::gmock_main
          ${HARDWARE_DRIVER_LIBS}
        )
        
        # 添加到CTest
        add_test(NAME ${TEST_NAME} COMMAND $<TARGET_FILE:${TEST_NAME}>)
      endforeach()
    endif()
  else()
    message(WARNING "BUILD_TESTS=ON but GTest not found. Install GTest or use -DBUILD_TESTS=OFF")
  endif()
else()
  message(STATUS "BUILD_TESTS=OFF, skipping tests (use -DBUILD_TESTS=ON to enable)")
endif()

# === Finalize package ===
ament_package()
