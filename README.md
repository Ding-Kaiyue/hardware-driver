# 硬件驱动库 v2.0.0

一个现代化的、事件驱动的硬件驱动库，为机器人控制系统提供高性能、线程安全的电机驱动和状态监控。

## 🚀 v2.0.0 重大更新

### 全新的事件驱动架构
- **观察者模式**: 实时电机状态更新，无需轮询
- **事件总线系统**: 解耦的异步事件处理
- **混合架构**: 支持实时组件和非实时组件的不同数据流
- **线程安全**: 多线程优化的数据结构和算法

### 性能优化
- **多线程架构**: 分离控制、反馈和数据处理线程
- **CPU亲和性**: 自动绑定实时任务到高性能CPU核心
- **自适应频率**: 根据系统负载动态调整通信频率
- **零拷贝数据传输**: 最小化内存分配和拷贝

### 增强的API设计
- **模块化组件**: 清晰分离的总线、驱动和接口层
- **类型安全**: 强类型的事件和状态定义
- **异常安全**: 完整的错误处理和资源管理
- **扩展性**: 易于添加新的总线类型和设备

## 📦 快速安装

### 源码编译（推荐）
```bash
git clone https://github.com/Ding-Kaiyue/hardware_driver_lib.git
cd hardware_driver_lib
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 验证安装
```bash
# 运行测试
make test

# 运行示例程序
./examples/example_motor_observer
```

## 🏁 快速开始

### 方式1: 观察者模式 (推荐用于实时组件)

```cpp
#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>
#include <memory>

// 自定义状态观察者
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

int main() {
    // 配置硬件
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 2, 3}}
    };
    
    try {
        // 创建硬件栈
        auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
        auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
        
        // 添加观察者
        auto observer = std::make_shared<MyMotorObserver>();
        motor_driver->add_observer(observer);
        
        // 创建机器人接口
        RobotHardware robot(motor_driver, motor_config);
        
        // 控制电机
        robot.control_motor_in_velocity_mode("can0", 1, 10.0f);  // 10 度/秒
        
        // 让程序运行一段时间接收状态
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        // 停止电机
        robot.control_motor_in_velocity_mode("can0", 1, 0.0f);
        robot.disable_motor("can0", 1);
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```

### 方式2: 回调函数模式

```cpp
#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>

int main() {
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 2, 3}}
    };
    
    try {
        auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
        auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
        
        // 定义状态回调函数
        auto status_callback = [](const std::string& interface, uint32_t motor_id, 
                                 const hardware_driver::motor_driver::Motor_Status& status) {
            std::cout << "电机状态更新: " << interface << ":" << motor_id 
                      << " 位置=" << status.position << std::endl;
        };
        
        // 使用回调函数创建机器人接口
        RobotHardware robot(motor_driver, motor_config, status_callback);
        
        // 控制电机...
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```

### 方式3: 事件总线模式 (推荐用于非实时组件)

```cpp
#include "hardware_driver/interface/robot_hardware.hpp"
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include <iostream>

using namespace hardware_driver::event;

int main() {
    // 创建事件总线
    auto event_bus = std::make_shared<EventBus>();
    
    // 订阅电机状态事件
    auto handler = event_bus->subscribe<MotorStatusEvent>(
        [](const auto& event) {
            std::cout << "事件: 电机 " << event->get_interface() << ":" 
                      << event->get_motor_id() << " 位置=" 
                      << event->get_status().position << std::endl;
        });
    
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 2, 3}}
    };
    
    try {
        auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
        auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
        
        // 定义批量状态回调，发布到事件总线
        auto batch_callback = [event_bus](const std::string& interface, 
                                          const std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>& status_all) {
            event_bus->emit<MotorBatchStatusEvent>(interface, status_all);
        };
        
        RobotHardware robot(motor_driver, motor_config, batch_callback);
        
        // 控制和监控...
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```

## 🎯 主要特性

### 🔧 硬件支持
- **CAN-FD总线**: 高速、可靠的工业通信
- **多接口支持**: 同时支持多个CAN接口
- **热插拔**: 运行时检测和配置硬件
- **错误恢复**: 自动处理总线错误和重连

### ⚡ 高性能
- **实时性能**: 微秒级延迟的控制响应
- **高吞吐量**: 支持数百个电机的并发控制
- **自适应调度**: 根据系统负载优化线程调度
- **内存优化**: 零拷贝数据路径和内存池

### 🛡️ 可靠性
- **线程安全**: 所有API都是线程安全的
- **异常安全**: 强异常安全保证
- **资源管理**: RAII和智能指针
- **故障隔离**: 单个设备故障不影响整个系统

### 🧩 架构设计
- **分层架构**: 清晰的总线-驱动-接口分层
- **可扩展**: 插件式的总线和设备驱动
- **解耦设计**: 观察者模式和事件系统
- **配置驱动**: 声明式的硬件配置

## 📋 API 参考

### 基本电机控制
```cpp
// 使能/失能
robot.enable_motor("can0", 1, 4);   // 接口、电机ID、模式
robot.disable_motor("can0", 1);

// 运动控制
robot.control_motor_in_velocity_mode("can0", 1, 10.0f);      // 速度模式 (度/秒)
robot.control_motor_in_position_mode("can0", 1, 90.0f);      // 位置模式 (度)
robot.control_motor_in_effort_mode("can0", 1, 2.5f);         // 力矩模式 (Nm)

// MIT模式 (位置、速度、力矩混合控制)
robot.control_motor_in_mit_mode("can0", 1, 45.0f, 5.0f, 1.0f);
```

### 事件系统
```cpp
// 订阅单个电机状态事件
auto handler1 = event_bus->subscribe<MotorStatusEvent>(
    [](const auto& event) {
        // 处理单个电机状态更新
        auto& status = event->get_status();
        // ...
    });

// 订阅批量电机状态事件
auto handler2 = event_bus->subscribe<MotorBatchStatusEvent>(
    [](const auto& event) {
        // 处理批量电机状态更新
        for (const auto& [id, status] : event->get_status_all()) {
            // ...
        }
    });

// 取消订阅
event_bus->unsubscribe<MotorStatusEvent>(handler1);
```

### 观察者模式
```cpp
class MyObserver : public hardware_driver::motor_driver::MotorStatusObserver {
public:
    // 实现观察者接口
    void on_motor_status_update(const std::string& interface, uint32_t motor_id, 
                               const Motor_Status& status) override {
        // 处理状态更新
    }
    
    void on_motor_status_update(const std::string& interface,
                               const std::map<uint32_t, Motor_Status>& status_all) override {
        // 处理批量状态更新
    }
};

// 注册观察者
auto observer = std::make_shared<MyObserver>();
motor_driver->add_observer(observer);
motor_driver->remove_observer(observer);
```

## 🔧 配置说明

### 硬件配置
```cpp
// 基本配置
std::vector<std::string> interfaces = {"can0", "can1"};
std::map<std::string, std::vector<uint32_t>> motor_config = {
    {"can0", {1, 2, 3, 4}},    // can0上的电机ID
    {"can1", {5, 6, 7, 8}}     // can1上的电机ID
};

// 高级配置
hardware_driver::motor_driver::TimingConfig timing;
timing.control_interval = std::chrono::microseconds(500);      // 控制频率
timing.high_freq_feedback = std::chrono::microseconds(400);    // 高频反馈
timing.control_cpu_core = 4;                                   // CPU绑定

motor_driver->set_timing_config(timing);
```

### CAN总线配置
```cpp
// 创建CAN总线时指定波特率
auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(
    interfaces, 
    1000000,    // 仲裁位速率 1Mbps
    5000000     // 数据位速率 5Mbps
);

// 设置扩展帧格式
bus->set_extended_frame("can0", true);
bus->set_fd_mode("can0", true);
```

## 🧪 示例程序

项目包含多个示例程序，展示不同的使用场景：

- `example_motor_observer.cpp` - 观察者模式示例
- `example_motor_callback_fb.cpp` - 回调函数示例  
- `example_motor_event_bus.cpp` - 事件总线示例
- `example_motor_zero_position.cpp` - 电机零位设定示例

编译并运行示例：
```bash
cd build
make
./examples/example_motor_observer
```

## 🧪 测试

项目包含完整的单元测试和集成测试：

```bash
# 编译测试
cmake .. -DBUILD_TESTS=ON
make

# 运行所有测试
make test

# 运行特定测试
./tests/test_motor_driver_impl
./tests/test_event_bus_api
./tests/test_robot_hardware
```

## 📊 性能基准

基于Jetson Orin平台的测试结果：

- **控制延迟**: < 200μs (从命令到CAN发送)
- **状态更新频率**: 2.5kHz (高频模式) / 20Hz (低频模式)
- **CPU使用率**: < 5% (典型配置)
- **内存占用**: < 50MB (典型配置)

## 🛠️ 系统要求

### 最低要求
- Linux内核 4.4+ (CAN支持)
- GCC 7+ 或 Clang 6+ (C++17)
- CMake 3.8+
- pthread库

### 推荐环境
- Ubuntu 20.04+ 或类似发行版
- 实时内核 (PREEMPT_RT)
- 多核处理器 (4核+)
- CAN-FD硬件接口

### 硬件兼容性
- SocketCAN兼容的CAN接口
- USB-CAN适配器
- PCIe CAN卡
- 嵌入式CAN控制器

## 🔍 故障排除

### 常见问题

**1. CAN接口初始化失败**
```bash
# 检查接口状态
ip link show can0

# 配置CAN接口
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

**2. 权限不足**
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 或使用sudo运行程序
sudo ./your_program
```

**3. 实时性能问题**
```bash
# 设置实时优先级
sudo chrt -f 50 ./your_program

# 检查CPU绑定
taskset -c 2-5 ./your_program
```

**4. 队列溢出警告**
- 检查电机配置是否正确
- 确认只连接了配置的CAN接口
- 调整反馈频率设置

## 🏗️ 从v1.0迁移

v2.0引入了重大架构变更，主要迁移步骤：

### API变更
```cpp
// v1.0 (旧API)
hardware_driver::HardwareDriver driver(interfaces, motor_config);
auto status = driver.get_motor_status("can0", 1);

// v2.0 (新API)
auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
RobotHardware robot(motor_driver, motor_config);
```

### 状态获取方式
```cpp
// v1.0 - 轮询方式
auto status = driver.get_motor_status("can0", 1);

// v2.0 - 事件驱动方式
class MyObserver : public MotorStatusObserver {
    void on_motor_status_update(...) override {
        // 状态自动推送到这里
    }
};
```

## 📚 更多文档

- [开发者指南](docs/DEVELOPER.md) - 内部架构和开发指南
- [API文档](docs/DOCUMENTATION.md) - 详细API参考
- [事件系统指南](docs/EVENT_BUS_GUIDE.md) - 事件总线使用详解
- [发布说明](docs/RELEASE_NOTES.md) - 版本更新记录
- [安全指南](docs/SECURITY.md) - 安全最佳实践

## 🤝 贡献

我们欢迎社区贡献！请参阅 [CONTRIBUTING.md](CONTRIBUTING.md) 了解如何参与。

### 开发环境设置
```bash
# 安装开发工具
sudo apt install build-essential cmake ninja-build

# 克隆仓库
git clone --recursive https://github.com//hardware_driver_lib.git
cd hardware_driver_lib

# 编译开发版本
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON
make -j$(nproc)
```

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件。

## 📞 联系方式

- **GitHub**: [Issues](https://github.com/Ding-Kaiyue/hardware_driver_lib/issues)
- **Email**: kaiyue.ding@raysense.com
- **微信**: d18292819833

## 🌟 致谢

感谢所有贡献者和测试用户的支持！

---

**⭐ 如果这个项目对你有帮助，请给我们一个星标！**