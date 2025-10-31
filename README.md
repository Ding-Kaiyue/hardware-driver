# Hardware Driver Library

[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/Ding-Kaiyue/hardware-driver/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/Ding-Kaiyue/hardware-driver/actions/workflows/ci.yml)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/CMake-3.10+-blue.svg)](https://cmake.org/)
[![ROS 2 Compatible](https://img.shields.io/badge/ROS%202-Compatible-blue)](https://docs.ros.org/en/humble/)

一个现代化的、事件驱动的硬件驱动库，为机器人控制系统提供高性能、线程安全的电机驱动和状态监控。**完全独立于ROS，可直接在任何C++项目中使用，同时也完全兼容ROS 2生态系统。**

## 🚀 特性

- **事件驱动架构**: 观察者模式 + 事件总线系统，实时状态更新
- **高性能**: 微秒级延迟，支持数百个电机并发控制
- **线程安全**: 多线程优化，CPU亲和性绑定
- **CAN-FD支持**: 高速可靠的工业通信
- **IAP固件更新**: 内置 IAP 协议支持，支持在线固件更新
- **模块化设计**: 清晰的总线-驱动-接口分层

## 📦 安装

### 方法一：源码编译（推荐）
```bash
git clone https://github.com/Ding-Kaiyue/hardware-driver.git
cd hardware-driver
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 方法二：发布包安装
```bash
wget https://github.com/Ding-Kaiyue/hardware-driver/releases/download/v2.0.0-release/hardware_driver_v2.0.0.tar.gz
tar -xzvf hardware_driver_v2.0.0.tar.gz
cd hardware-driver
sudo ./install.sh
```

### 方法三：Debian包安装
```bash
wget https://github.com/Ding-Kaiyue/hardware-driver/releases/download/v2.0.0-release/libhardware-driver0_2.0.0-1_amd64.deb
wget https://github.com/Ding-Kaiyue/hardware-driver/releases/download/v2.0.0-release/libhardware-driver-dev_2.0.0-1_amd64.deb
sudo dpkg -i libhardware-driver0_2.0.0-1_amd64.deb
sudo dpkg -i libhardware-driver-dev_2.0.0-1_amd64.deb
sudo apt-get install -f
```

## 📦 卸载
```bash
# 删除头文件
  sudo rm -rf /usr/local/include/hardware_driver

  # 删除库文件
  sudo rm -f /usr/local/lib/libhardware_driver*
  sudo rm -f /usr/local/lib64/libhardware_driver*

  # 删除CMake配置文件
  sudo rm -rf /usr/local/lib/cmake/hardware_driver
  sudo rm -rf /usr/local/share/hardware_driver

  # 删除可执行文件（如果有的话）
  sudo rm -f /usr/local/bin/*hardware_driver*

  # 更新动态链接库缓存
  sudo ldconfig
```

## 🚀 快速开始

### 基本使用

```cpp
#include "hardware_driver/interface/robot_hardware.hpp"

// 配置硬件
std::vector<std::string> interfaces = {"can0"};
std::map<std::string, std::vector<uint32_t>> motor_config = {
    {"can0", {1, 2, 3}}
};

// 创建硬件栈
auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
RobotHardware robot(motor_driver, motor_config);

// 控制电机
robot.enable_motor("can0", 1, 4);
robot.control_motor_in_velocity_mode("can0", 1, 10.0f);  // 10 度/秒
robot.disable_motor("can0", 1);
```

### 观察者模式（实时组件推荐）

```cpp
class MyMotorObserver : public hardware_driver::motor_driver::MotorStatusObserver {
public:
    void on_motor_status_update(const std::string& interface, 
                               uint32_t motor_id, 
                               const hardware_driver::motor_driver::Motor_Status& status) override {
        std::cout << "电机 " << interface << ":" << motor_id 
                  << " 位置:" << status.position 
                  << " 速度:" << status.velocity << std::endl;
    }
};

// 添加观察者
auto observer = std::make_shared<MyMotorObserver>();
motor_driver->add_observer(observer);
```

### IAP固件更新（在线更新）

```cpp
#include "hardware_driver/interface/robot_hardware.hpp"

// 定义IAP状态反馈观察者
class IAPObserver : public hardware_driver::motor_driver::IAPStatusObserver {
public:
    void on_iap_status_feedback(const std::string& interface,
                                uint32_t motor_id,
                                const hardware_driver::iap_protocol::IAPStatusMessage& msg) override {
        using namespace hardware_driver::iap_protocol;
        std::cout << "[IAP] " << interface << ":" << motor_id << " -> "
                  << iap_status_to_string(msg) << std::endl;
    }
};

// 创建机器人硬件实例（带IAP观察者）
auto iap_observer = std::make_shared<IAPObserver>();
auto robot = std::make_shared<RobotHardware>(motor_driver, motor_config, iap_observer);

// 执行固件更新
robot->start_update("can0", 1, "./firmware/motor_v2.0.bin");
```

### 事件总线模式（非实时组件推荐）

```cpp
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"

// 创建事件总线并订阅事件
auto event_bus = std::make_shared<EventBus>();
auto handler = event_bus->subscribe<MotorStatusEvent>(
    [](const auto& event) {
        std::cout << "事件: 电机 " << event->get_interface() << ":"
                  << event->get_motor_id() << " 位置="
                  << event->get_status().position << std::endl;
    });
```

## 📋 API 参考

### 电机控制
```cpp
// 使能/失能
robot.enable_motor("can0", 1, 4);
robot.disable_motor("can0", 1);

// 运动控制
robot.control_motor_in_velocity_mode("can0", 1, 10.0f);    // 速度 (度/秒)
robot.control_motor_in_position_mode("can0", 1, 90.0f);    // 位置 (度)
robot.control_motor_in_effort_mode("can0", 1, 2.5f);       // 力矩 (Nm)
robot.control_motor_in_mit_mode("can0", 1, 45.0f, 5.0f, 1.0f); // MIT模式
```

### IAP固件更新
```cpp
// 固件更新
robot->start_update("can0", 1, "./firmware/motor_v2.0.bin");

// 注意：在更新过程中自动暂停电机反馈请求，以减少CAN总线干扰
// 更新完成后自动恢复反馈请求
```

### 配置
```cpp
// 硬件配置
std::vector<std::string> interfaces = {"can0", "can1"};
std::map<std::string, std::vector<uint32_t>> motor_config = {
    {"can0", {1, 2, 3, 4}},
    {"can1", {5, 6, 7, 8}}
};

// 时序配置
hardware_driver::motor_driver::TimingConfig timing;
timing.control_interval = std::chrono::microseconds(500);
timing.control_cpu_core = 4;
motor_driver->set_timing_config(timing);
```

## 📡 IAP协议说明

### 协议概述
IAP（In-Application Programming）允许在应用运行时对电机固件进行更新。驱动库提供了完整的 IAP 协议支持。

### 协议流程
1. **进入 IAP 模式**: 发送 `[0x01, 0x12]` 请求
2. **Bootloader 启动**: 接收 `BS00` 状态
3. **发送密钥**: 发送 `'k','e','y'` 进入 IAP 模式
4. **接收状态**: 依次收到 `BK01`, `BK02`, `BK03`
5. **传输固件数据**: 以 64 字节分块发送固件
6. **完成**: 接收 `BJ06` 和 `AS00` 状态

### 反馈消息
| 消息 | 含义 | CAN ID |
|-----|------|--------|
| AJ01 | APP 收到 IAP 指令 | 0xFF + motor_id |
| BS00 | Bootloader 启动 | 0xFF + motor_id |
| BK01 | 收到 Key，进入 IAP | 0xFF + motor_id |
| BK02 | 擦除 APP 程序 | 0xFF + motor_id |
| BK03 | 准备接收固件数据 | 0xFF + motor_id |
| BD04 | 接收数据中 | 0xFF + motor_id |
| BJ06 | 跳转准备（校验完成） | 0xFF + motor_id |
| AS00 | APP 启动成功 | 0xFF + motor_id |

## 🧪 测试

```bash
# 编译测试
cmake .. -DBUILD_TESTS=ON
make

# 运行测试
make test

# 运行示例
./examples/example_motor_observer

# 运行IAP固件更新示例
./examples/example_iap_update
```

## 📊 性能

基于Jetson Orin平台测试：

- **控制延迟**: < 200μs
- **状态更新频率**: 2.5kHz (高频) / 20Hz (低频)
- **CPU使用率**: < 5%
- **内存占用**: < 50MB

## 📁 项目结构

```
hardware_driver_lib/
├── include/hardware_driver/          # 公开头文件
│   ├── driver/                       # 驱动接口
│   ├── interface/                    # 高层接口
│   ├── bus/                          # 总线接口
│   └── event/                        # 事件系统
├── src/
│   ├── driver/                       # 驱动实现
│   ├── interface/                    # 接口实现
│   ├── bus/                          # CAN总线实现
│   ├── protocol/                     # IAP协议实现
│   └── event/                        # 事件总线实现
├── examples/                         # 使用示例
│   ├── example_motor_observer.cpp    # 观察者模式示例
│   ├── example_iap_update.cpp        # IAP固件更新示例
│   └── ...
├── tests/                            # 单元测试
├── docs/                             # 文档
└── CMakeLists.txt
```

## 🛠️ 系统要求

- Linux 4.4+ (CAN支持)
- GCC 7+ 或 Clang 6+ (C++17)
- CMake 3.8+
- SocketCAN兼容的CAN接口

## 🔍 故障排除

### CAN接口配置
```bash
# 配置CAN接口
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on

# 检查状态
ip link show can0
```

### 权限问题
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 或使用sudo运行
sudo ./your_program
```

## 📄 许可证

MIT License - 详见 [LICENSE](LICENSE) 文件

## 📞 联系方式

- **GitHub**: [Issues](https://github.com/Ding-Kaiyue/hardware-driver/issues)
- **Email**: kaiyue.ding@raysense.com

---

⭐ **如果这个项目对你有帮助，请给我们一个星标！**