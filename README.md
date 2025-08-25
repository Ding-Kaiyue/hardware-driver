# Hardware Driver Library

一个现代化的、事件驱动的硬件驱动库，为机器人控制系统提供高性能、线程安全的电机驱动和状态监控。

## 🚀 特性

- **事件驱动架构**: 观察者模式 + 事件总线系统，实时状态更新
- **高性能**: 微秒级延迟，支持数百个电机并发控制
- **线程安全**: 多线程优化，CPU亲和性绑定
- **CAN-FD支持**: 高速可靠的工业通信
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

## 🧪 测试

```bash
# 编译测试
cmake .. -DBUILD_TESTS=ON
make

# 运行测试
make test

# 运行示例
./examples/example_motor_observer
```

## 📊 性能

基于Jetson Orin平台测试：

- **控制延迟**: < 200μs
- **状态更新频率**: 2.5kHz (高频) / 20Hz (低频)
- **CPU使用率**: < 5%
- **内存占用**: < 50MB

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