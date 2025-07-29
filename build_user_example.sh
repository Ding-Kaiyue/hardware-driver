#!/bin/bash

echo "=== 编译用户示例 ==="

# 检查库文件是否存在
if [ ! -f "lib/libhardware_driver.so" ]; then
    echo "错误: 找不到 lib/libhardware_driver.so"
    echo "请先运行 make 编译库文件"
    exit 1
fi

# 编译用户示例
g++ -std=c++17 \
    -I./include \
    -L./lib \
    -Wl,-rpath,./lib \
    examples/user_example.cpp \
    -lhardware_driver \
    -lpthread \
    -o user_example

if [ $? -eq 0 ]; then
    echo "编译成功! 运行示例:"
    echo "./user_example"
else
    echo "编译失败!"
    exit 1
fi 