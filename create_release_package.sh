#!/bin/bash

echo "=== 创建硬件驱动发布包 ==="

# 创建发布目录
RELEASE_DIR="hardware_driver_release"
rm -rf $RELEASE_DIR
mkdir -p $RELEASE_DIR

# 复制必要的头文件
echo "复制头文件..."
mkdir -p $RELEASE_DIR/include/hardware_driver
cp include/hardware_driver.hpp $RELEASE_DIR/include/
cp -r include/hardware_driver/* $RELEASE_DIR/include/hardware_driver/

# 复制协议文件
echo "复制协议文件..."
mkdir -p $RELEASE_DIR/include/hardware_driver/protocol
cp src/protocol/motor_protocol.hpp $RELEASE_DIR/include/hardware_driver/protocol/
cp src/protocol/gripper_omnipicker_protocol.hpp $RELEASE_DIR/include/hardware_driver/protocol/

# 复制总线接口文件
echo "复制总线接口文件..."
mkdir -p $RELEASE_DIR/include/hardware_driver/bus
cp src/bus/canfd_bus_impl.hpp $RELEASE_DIR/include/hardware_driver/bus/
cp src/bus/ethercat_bus_impl.hpp $RELEASE_DIR/include/hardware_driver/bus/

# 复制驱动实现文件
echo "复制驱动实现文件..."
mkdir -p $RELEASE_DIR/include/hardware_driver/driver
cp src/driver/motor_driver_impl.hpp $RELEASE_DIR/include/hardware_driver/driver/
cp src/driver/gripper_driver_impl.hpp $RELEASE_DIR/include/hardware_driver/driver/

# 复制库文件
echo "复制库文件..."
mkdir -p $RELEASE_DIR/lib
cp build/lib/libhardware_driver.so* $RELEASE_DIR/lib/

# 复制示例
echo "复制示例..."
mkdir -p $RELEASE_DIR/examples
cp examples/simple_example.cpp $RELEASE_DIR/examples/

# 复制文档
echo "复制文档..."
cp README.md $RELEASE_DIR/README.md

# 创建CMake配置文件
echo "创建CMake配置文件..."
cat > $RELEASE_DIR/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(hardware_driver_user)

set(CMAKE_CXX_STANDARD 17)

# 查找硬件驱动库
find_library(HARDWARE_DRIVER_LIB hardware_driver PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib NO_DEFAULT_PATH)
if(NOT HARDWARE_DRIVER_LIB)
    message(FATAL_ERROR "hardware_driver library not found in ${CMAKE_CURRENT_SOURCE_DIR}/lib")
endif()

# 包含头文件目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 查找线程库
find_package(Threads REQUIRED)

# 创建示例可执行文件
add_executable(simple_example examples/simple_example.cpp)
target_link_libraries(simple_example ${HARDWARE_DRIVER_LIB} Threads::Threads)

# 设置RPATH
set_target_properties(simple_example PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)
EOF

# 获取版本号
VERSION=$(grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+' debian/changelog | head -1)
if [ -z "$VERSION" ]; then
    VERSION="1.0.0"
fi

# 创建pkg-config文件
echo "创建pkg-config文件..."
cat > $RELEASE_DIR/hardware_driver.pc << EOF
prefix=/usr/local
exec_prefix=\${prefix}
libdir=\${prefix}/lib
includedir=\${prefix}/include

Name: hardware_driver
Description: Hardware driver library for motor control
Version: $VERSION
Libs: -L\${libdir} -lhardware_driver -lpthread
Cflags: -I\${includedir}
Requires: 
EOF

# 创建安装脚本
echo "创建安装脚本..."
cat > $RELEASE_DIR/install.sh << 'EOF'
#!/bin/bash

echo "=== 硬件驱动库安装脚本 ==="

# 检查是否为root用户
if [ "$EUID" -ne 0 ]; then
    echo "请使用sudo运行此脚本"
    exit 1
fi

# 创建安装目录
INSTALL_PREFIX="/usr/local"
echo "安装到: $INSTALL_PREFIX"

# 安装头文件
echo "安装头文件..."
mkdir -p $INSTALL_PREFIX/include/hardware_driver
cp include/hardware_driver.hpp $INSTALL_PREFIX/include/
cp -r include/hardware_driver/* $INSTALL_PREFIX/include/hardware_driver/

# 安装库文件
echo "安装库文件..."
mkdir -p $INSTALL_PREFIX/lib
cp lib/libhardware_driver.so* $INSTALL_PREFIX/lib/

# 更新动态链接库缓存
echo "更新动态链接库缓存..."
ldconfig

# 安装pkg-config文件
echo "安装pkg-config文件..."
mkdir -p $INSTALL_PREFIX/lib/pkgconfig
cp hardware_driver.pc $INSTALL_PREFIX/lib/pkgconfig/

echo "安装完成！"
echo "使用示例:"
echo "  g++ -std=c++17 -lhardware_driver -lpthread your_program.cpp -o your_program"
EOF

chmod +x $RELEASE_DIR/install.sh

# 创建编译示例脚本
echo "创建编译示例脚本..."
cat > $RELEASE_DIR/build_example.sh << 'EOF'
#!/bin/bash

echo "=== 编译示例程序 ==="

# 检查库文件是否存在
if [ ! -f "lib/libhardware_driver.so" ]; then
    echo "错误: 库文件不存在，请先运行 ./install.sh"
    exit 1
fi

# 编译示例
echo "编译示例程序..."
g++ -std=c++17 \
    -I./include \
    -L./lib \
    -Wl,-rpath,./lib \
    examples/simple_example.cpp \
    -lhardware_driver \
    -lpthread \
    -o simple_example

if [ $? -eq 0 ]; then
    echo "编译成功！"
    echo "运行示例: ./simple_example"
else
    echo "编译失败！"
    exit 1
fi
EOF

chmod +x $RELEASE_DIR/build_example.sh

# 创建README
echo "创建README..."
cat > $RELEASE_DIR/README_INSTALL.md << 'EOF'
# 硬件驱动库 - 安装指南

## 快速安装

### 方法1: 使用安装脚本
```bash
sudo ./install.sh
```

### 方法2: 手动安装
```bash
# 安装头文件
sudo cp include/hardware_driver.hpp /usr/local/include/
sudo cp -r include/hardware_driver/ /usr/local/include/

# 安装库文件
sudo cp lib/libhardware_driver.so* /usr/local/lib/

# 更新动态链接库缓存
sudo ldconfig
```

## 编译示例

```bash
# 使用提供的脚本
./build_example.sh

# 或手动编译
g++ -std=c++17 -lhardware_driver -lpthread examples/simple_example.cpp -o simple_example
```

## 使用示例

```cpp
#include "hardware_driver.hpp"
#include <iostream>

int main() {
    // 配置硬件
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 2, 3, 4}}
    };
    
    try {
        // 创建驱动
        hardware_driver::HardwareDriver driver(interfaces, motor_config);
        
        // 使能电机
        driver.enable_motor("can0", 1, 4);
        
        // 控制电机
        driver.control_motor_in_velocity_mode("can0", 1, 5.0);
        
        // 获取状态
        auto status = driver.get_motor_status("can0", 1);
        std::cout << "位置: " << status.position << std::endl;
        
        // 失能电机
        driver.disable_motor("can0", 1);
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```

## 依赖项

- C++17 编译器
- pthread 库
- CAN 接口支持

## 故障排除

1. **库文件未找到**: 运行 `sudo ldconfig`
2. **编译错误**: 确保使用 C++17 标准
3. **运行时错误**: 检查 CAN 接口配置和权限
EOF

# 创建压缩包
echo "创建压缩包..."
tar -czf hardware_driver_v$VERSION.tar.gz $RELEASE_DIR/

echo "=== 发布包创建完成 ==="
echo "发布包: hardware_driver_v$VERSION.tar.gz"
echo "内容:"
ls -la $RELEASE_DIR/
echo ""
echo "用户可以使用以下命令安装:"
echo "  tar -xzf hardware_driver_v$VERSION.tar.gz"
echo "  cd hardware_driver_release"
echo "  sudo ./install.sh" 