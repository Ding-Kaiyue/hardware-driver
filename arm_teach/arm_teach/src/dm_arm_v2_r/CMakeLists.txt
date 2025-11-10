cmake_minimum_required(VERSION 3.5)
project(dm_arm_v2_r)

# 使用 ament_cmake 构建系统
find_package(ament_cmake REQUIRED)

# 安装资源文件夹
foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

# 声明该包是一个 ROS 2 包
ament_package()

