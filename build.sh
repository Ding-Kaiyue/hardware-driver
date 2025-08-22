#!/bin/bash

echo "=== 硬件驱动编译脚本 ==="

# 检查是否在正确的目录
if [ ! -f "CMakeLists.txt" ]; then
    echo "错误: 请在项目根目录运行此脚本"
    exit 1
fi

BUILD_TYPE=${1:-Debug}
ENABLE_TESTS=${2:-OFF}

# 创建构建目录
echo "创建构建目录..."
mkdir -p build
cd build

# 清理之前的构建
echo "清理之前的构建..."
rm -rf *

# 配置项目
echo "配置项目..."
if [ "$ENABLE_TESTS" = "ON" ] || [ "$ENABLE_TESTS" = "on" ]; then
    echo "启用单元测试编译..."
    cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DBUILD_TESTS=ON
else
    cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DBUILD_TESTS=OFF
fi

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

# 显示库文件信息
echo ""
echo "=== 生成的库文件 ==="
if [ -f lib/libhardware_driver.so ]; then
    ls -la lib/libhardware_driver.so
else
    echo "警告: 库文件未找到"
fi

# 显示示例程序
echo ""
echo "=== 生成的示例程序 ==="
if [ -d examples ]; then
    echo "示例程序:"
    ls -la examples/
else
    echo "警告: 示例程序目录未找到"
fi

# 运行测试
if [ "$ENABLE_TESTS" = "ON" ] || [ "$ENABLE_TESTS" = "on" ]; then
    echo ""
    echo "=== 运行单元测试 ==="
    if [ -d tests ]; then
        echo "==== 运行单元测试 ===="
        for test_file in tests/test_*; do
            if [ -f "$test_file" ] && [ -x "$test_file" ]; then
                echo "运行测试: $(basename "$test_file")"
                ./"$test_file"
                if [ $? -eq 0 ]; then
                    echo "✅ $(basename "$test_file") 测试通过"
                else
                    echo "❌ $(basename "$test_file") 测试失败"
                fi
                echo ""
            fi
        done
    else
        echo "警告: tests目录未找到"
    fi
else
    echo ""
    echo "=== 跳过单元测试 ==="
    echo "要运行单元测试，请使用: $0 $BUILD_TYPE ON"
fi

echo ""
echo "=== 使用说明 ==="
echo "运行示例程序:"
echo "  cd build && ./examples/example_motor_zero_position"
echo "  cd build && ./examples/example_event_motor_bus"
echo "  cd build && ./examples/example_motor_callback_fb"
echo "  cd build && ./examples/example_motor_observer"
echo ""
echo "脚本参数:"
echo "  $0 [BUILD_TYPE] [ENABLE_TESTS]"
echo "  BUILD_TYPE: Debug(默认) | Release"
echo "  ENABLE_TESTS: OFF(默认) | ON"
echo ""
echo "示例:"
echo "  $0                    # Debug模式，不编译测试"
echo "  $0 Release            # Release模式，不编译测试"
echo "  $0 Debug ON           # Debug模式，编译并运行测试"
echo "  $0 Release ON         # Release模式，编译并运行测试"