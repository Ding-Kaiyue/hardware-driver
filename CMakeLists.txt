cmake_minimum_required(VERSION 3.8)
project(hardware_driver)

set(CMAKE_CXX_STANDARD 17)

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

# === canfd 组合 ===
add_library(arm_with_omnipicker_canfd SHARED
  src/bus/canfd_bus_impl.cpp
  src/driver/motor_driver_impl.cpp
  src/driver/gripper_driver_impl.cpp
  src/protocol/motor_protocol.cpp
  src/protocol/gripper_omnipicker_protocol.cpp
  src/interface/robot_hardware.cpp
)
target_link_libraries(arm_with_omnipicker_canfd Threads::Threads)
target_include_directories(arm_with_omnipicker_canfd PRIVATE src include)

# 设置库的版本信息
set_target_properties(arm_with_omnipicker_canfd PROPERTIES
  VERSION 1.0.0
  SOVERSION 1
  OUTPUT_NAME "hardware_driver"
)

# 设置库的输出目录到lib文件夹
set_target_properties(arm_with_omnipicker_canfd PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# === ethercat 组合 ===
# add_library(arm_with_omnipicker_ethercat SHARED
#   src/bus/ethercat_bus_impl.cpp
#   src/driver/motor_driver_impl.cpp
#   src/driver/gripper_driver_impl.cpp
#   src/protocol/motor_protocol.cpp
#   src/protocol/gripper_omnipicker_protocol.cpp
#   src/interface/robot_hardware.cpp
# )
# target_link_libraries(arm_with_omnipicker_ethercat Threads::Threads)
# target_include_directories(arm_with_omnipicker_ethercat PRIVATE src)

# 添加简单示例
add_executable(simple_example examples/simple_example.cpp)
target_link_libraries(simple_example
  arm_with_omnipicker_canfd
  Threads::Threads
)

# 添加简化版本测试
add_executable(test_simplified_version test_simplified.cpp)
target_link_libraries(test_simplified_version
  arm_with_omnipicker_canfd
  Threads::Threads
)

# === 安装头文件和so ===
install(DIRECTORY include/hardware_driver DESTINATION include)

# 创建硬件驱动库的别名
add_library(hardware_driver ALIAS arm_with_omnipicker_canfd)

# 设置安装时的库名称
set_target_properties(arm_with_omnipicker_canfd PROPERTIES
  LIBRARY_OUTPUT_NAME "hardware_driver"
  ARCHIVE_OUTPUT_NAME "hardware_driver"
)

install(TARGETS
  arm_with_omnipicker_canfd
  # arm_with_omnipicker_ethercat
  EXPORT hardware_driverTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# 创建配置文件
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/hardware_driverConfigVersion.cmake"
  VERSION 1.0.0
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

# === gtest集成 ===
# 只有在启用测试选项且找到 GTest 时才构建测试
if(BUILD_TESTS AND GTest_FOUND)
  message(STATUS "BUILD_TESTS is ON and GTest found, building tests")
  
  # 创建tests目录
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/tests)

  # 查找所有测试文件
  file(GLOB TEST_SOURCES tests/*.cpp)
  if(TEST_SOURCES)
    # 为每个测试文件创建单独的可执行文件
    foreach(TEST_FILE ${TEST_SOURCES})
      # 获取文件名（不含扩展名）
      get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
      
      # 创建测试可执行文件
      add_executable(${TEST_NAME} ${TEST_FILE})
      
      # 设置输出目录到build/tests
      set_target_properties(${TEST_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
      )
      
      # 链接库
      set(TEST_LIBS
        arm_with_omnipicker_canfd
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
      )
      
      # 如果 GMock 可用，则添加它
      if(TARGET GTest::gmock)
        list(APPEND TEST_LIBS GTest::gmock)
      endif()
      if(TARGET GTest::gmock_main)
        list(APPEND TEST_LIBS GTest::gmock_main)
      endif()
      
      target_link_libraries(${TEST_NAME} ${TEST_LIBS})
      
      # 添加到CTest
      add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endforeach()
  endif()
else()
  if(NOT BUILD_TESTS)
    message(STATUS "BUILD_TESTS is OFF, skipping tests (use -DBUILD_TESTS=ON to enable)")
  elseif(NOT GTest_FOUND)
    message(STATUS "GTest not found, skipping tests")
  endif()
endif()