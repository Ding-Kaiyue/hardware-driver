#!/bin/bash

echo "=== 硬件驱动库安装脚本 ==="

# 检查是否为root用户
if [ "$EUID" -ne 0 ]; then
    echo "错误: 请使用sudo运行此脚本"
    exit 1
fi

# 检查是否在正确的目录
if [ ! -f "CMakeLists.txt" ]; then
    echo "错误: 请在项目根目录运行此脚本"
    exit 1
fi

# 检查依赖
echo "检查系统依赖..."
if ! command -v cmake &> /dev/null; then
    echo "安装cmake..."
    apt-get update
    apt-get install -y cmake build-essential
fi

if ! command -v make &> /dev/null; then
    echo "安装make..."
    apt-get install -y build-essential
fi

# 检查GTest依赖
if ! pkg-config --exists gtest; then
    echo "安装GTest..."
    apt-get install -y libgtest-dev libgmock-dev
fi

# 编译安装
echo "编译安装硬件驱动库..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install

# 更新动态库缓存
echo "更新动态库缓存..."
ldconfig

echo "=== 安装完成 ==="
echo "硬件驱动库已成功安装到系统中"
echo ""
echo "使用示例:"
echo "  #include <hardware_driver.hpp>"
echo "  # 编译时链接: -lhardware_driver -lpthread"
echo ""
echo "库文件位置:"
echo "  - 头文件: /usr/local/include/hardware_driver/"
echo "  - 库文件: /usr/local/lib/libhardware_driver.so"
echo "  - 配置文件: /usr/local/lib/cmake/hardware_driver/" 