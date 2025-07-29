#!/bin/bash

echo "=== 硬件驱动编译脚本 ==="

# 检查是否在正确的目录
if [ ! -f "CMakeLists.txt" ]; then
    echo "错误: 请在项目根目录运行此脚本"
    exit 1
fi

BUILD_TYPE=${1:-Debug}

# 创建构建目录
echo "创建构建目录..."
mkdir -p build
cd build

# 清理之前的构建
echo "清理之前的构建..."
rm -rf *

# 配置项目
echo "配置项目..."
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
if [ $? -ne 0 ]; then
    echo "错误: CMake配置失败"
    exit 1
fi

# 编译项目
echo "编译项目..."
make -j$(nproc)
if [ $? -ne 0 ]; then
    echo "错误: 编译失败"
    exit 1
fi

echo "=== 编译成功！ ==="
echo "可执行文件位置: build/demo_main"
echo "库文件位置: build/lib/libhardware_driver.so"

# 显示文件信息
echo ""
echo "=== 生成的文件 ==="
ls -la demo_main
ls -la lib/libhardware_driver.so

# echo ""
# echo "=== 运行测试 ==="
# if [ -d tests ]; then
#   echo "==== 运行单元测试 ===="
#   # for test_file in tests/test_*; do
#   #   if [ -f "$test_file" ] && [ -x "$test_file" ]; then
#   #     echo "运行测试: $(basename "$test_file")"
#   #     ./"$test_file"
#   #     echo ""
#   #   fi
#   # done
#   echo "运行测试: test_robot_hardware"
#   ./tests/test_robot_hardware
#   echo ""
# fi