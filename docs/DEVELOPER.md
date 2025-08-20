# 硬件驱动库 v2.0.0 - 开发者文档

## 项目结构

```
hardware_driver_lib/
├── include/hardware_driver/        # 公共头文件
│   ├── bus/                       # 总线接口
│   │   ├── bus_interface.hpp      # 总线基类
│   │   └── canfd_bus_impl.hpp     # CAN-FD总线实现
│   ├── driver/                    # 驱动接口
│   │   ├── motor_driver_interface.hpp  # 电机驱动接口
│   │   └── gripper_driver_interface.hpp # 夹爪驱动接口
│   ├── event/                     # 事件系统
│   │   ├── event_bus.hpp          # 事件总线
│   │   └── motor_events.hpp       # 电机事件定义
│   └── interface/                 # 硬件接口
│       └── robot_hardware.hpp     # 机器人硬件接口
├── src/                           # 源码实现
│   ├── bus/                       # 总线实现
│   ├── driver/                    # 驱动实现
│   ├── event/                     # 事件系统实现
│   ├── interface/                 # 硬件接口实现
│   └── protocol/                  # 协议实现
├── examples/                      # v2.0示例代码
│   ├── example_motor_observer.cpp # 观察者模式示例
│   ├── example_motor_event_bus.cpp# 事件总线示例
│   ├── example_motor_callback_fb.cpp # 回调模式示例
│   └── example_motor_zero_position.cpp # 零位设置示例
├── tests/                         # 单元测试
├── docs/                          # 完整文档
├── debian/                        # Debian包配置
├── build.sh                       # 构建脚本
├── create_debian_package.sh       # Debian包构建脚本
├── create_release_package.sh      # 源码包构建脚本
└── CMakeLists.txt                 # CMake配置
```

## 构建系统

### 依赖项
- CMake 3.8+
- C++17 编译器
- pthread 库
- GTest (用于测试)

### 构建步骤
```bash
# 克隆项目
git clone https://github.com/Ding-Kaiyue/hardware_driver.git
cd hardware_driver

# 构建库和示例
./build.sh

# 或手动构建
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON
make -j$(nproc)
```

### 测试
```bash
# 运行所有测试
cd build
ctest

# 或运行特定测试
./tests/test_robot_hardware
./tests/test_motor_driver_impl
./tests/test_canfd_bus_integration
./tests/test_event_bus_api
./tests/test_hybrid_architecture

# 运行示例程序
./examples/example_motor_observer
./examples/example_motor_event_bus
./examples/example_motor_callback_fb
```

## v2.0.0 架构设计

### 事件驱动架构概述

v2.0.0采用全新的事件驱动架构，提供三种数据获取模式：
- **观察者模式**：实时性能最佳，适用于控制回路
- **回调模式**：兼容v1.x的用法，提供简单接口
- **事件总线**：解耦架构，适用于监控、日志、诊断等

### 核心组件

1. **RobotHardware** - 主要用户接口类
   - 统一的电机控制API
   - 支持多种数据获取模式
   - 线程安全的接口设计

2. **MotorDriverImpl** - 电机驱动实现层
   - 实现观察者模式接口
   - 管理电机状态和控制
   - 支持事件总线集成
   - 多线程架构（控制线程、反馈线程、数据处理线程）

3. **CanFdBus** - CAN-FD总线实现
   - 高性能CAN通信
   - 自动队列管理
   - CPU亲和性优化
   - 错误恢复机制

4. **EventBus** - 事件总线系统
   - 类型安全的事件发布/订阅
   - 弱引用管理，防止内存泄漏
   - 线程安全的并发处理
   - 主题过滤和统计信息

5. **MotorStatusObserver** - 观察者接口
   - 实时状态更新通知
   - 支持单个和批量状态更新
   - 函数和参数操作结果回调

### v2.0.0 线程模型

```
用户线程 (控制调用)
    ↓
RobotHardware (主接口)
    ↓
MotorDriverImpl (驱动层)
    ↓
┌─────────────────┬─────────────────┬─────────────────┐
│   控制线程       │   反馈线程       │   数据处理线程    │
│ (异步控制命令)    │ (高频状态请求)    │ (事件分发处理)    │
│ CPU亲和性绑定    │ CPU亲和性绑定    │ 观察者通知       │
│                │ 高低频智能切换    │ 事件总线发布     │
└─────────────────┴─────────────────┴─────────────────┘
    ↓
CanFdBus (CAN-FD通信层)
    ↓
┌─────────────────┬─────────────────┐
│   发送线程       │   接收线程       │
│ (命令队列处理)    │ (状态数据接收)    │
│                │ 队列管理         │
└─────────────────┴─────────────────┘
    ↓
硬件 (CAN接口 - can0, can1, ...)

数据流向：
观察者模式: 硬件 → 接收线程 → 数据处理线程 → 直接回调 (最快)
事件总线: 硬件 → 接收线程 → 数据处理线程 → 事件总线 → 订阅者
```

## v2.0.0 开发指南

### 添加新的电机控制功能

1. **在RobotHardware中添加接口**
   ```cpp
   // include/hardware_driver/interface/robot_hardware.hpp
   class RobotHardware {
   public:
       void new_control_function(const std::string& interface, uint32_t motor_id, double param);
   };
   ```

2. **在实现文件中实现功能**
   ```cpp
   // src/interface/robot_hardware.cpp
   void RobotHardware::new_control_function(const std::string& interface, uint32_t motor_id, double param) {
       if (motor_driver_) {
           motor_driver_->new_control_function(interface, motor_id, param);
       }
   }
   ```

3. **在MotorDriverImpl中添加底层实现**
   ```cpp
   // src/driver/motor_driver_impl.cpp
   void MotorDriverImpl::new_control_function(const std::string& interface, uint32_t motor_id, double param) {
       // 创建控制命令并异步发送
       auto command = create_new_command(motor_id, param);
       send_command_async(interface, command);
   }
   ```

### 添加新协议

1. **创建协议文件**
   ```cpp
   // src/protocol/new_protocol.hpp
   class NewProtocol {
   public:
       static void encode_command(...);
       static void decode_response(...);
   };
   ```

2. **在驱动中使用**
   ```cpp
   // src/driver/motor_driver_impl.cpp
   #include "protocol/new_protocol.hpp"
   ```

### 添加新总线类型

1. **继承BusInterface**
   ```cpp
   // include/hardware_driver/bus/new_bus_impl.hpp
   class NewBusImpl : public BusInterface {
   public:
       NewBusImpl(const std::vector<std::string>& interfaces);
       
       // 实现基类虚函数
       bool send_frame(const std::string& interface, const CanFrame& frame) override;
       void set_batch_callback(BatchStatusCallback callback) override;
       void start() override;
       void stop() override;
   };
   ```

2. **在RobotHardware中使用**
   ```cpp
   // 使用新总线类型
   auto bus = std::make_shared<hardware_driver::bus::NewBusImpl>(interfaces);
   auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
   ```

### 添加新事件类型

1. **创建事件类**
```cpp
// include/hardware_driver/event/custom_events.hpp
class CustomEvent : public Event {
private:
    std::string data_;
    
public:
    CustomEvent(const std::string& data) : data_(data) {}
    
    std::string get_type_name() const override {
        return "CustomEvent";
    }
    
    std::string get_topic() const override {
        return "custom.device.data";
    }
    
    const std::string& get_data() const { return data_; }
};
```

2. **在事件总线中使用**
```cpp
// 发布自定义事件
event_bus->emit<CustomEvent>("custom data");

// 订阅自定义事件
event_bus->subscribe<CustomEvent>([](const auto& event) {
    std::cout << "收到自定义事件: " << event->get_data() << std::endl;
});
```

### 添加新的观察者

1. **实现观察者接口**
```cpp
class CustomObserver : public MotorStatusObserver {
public:
    void on_motor_status_update(const std::string& interface, uint32_t motor_id, 
                               const Motor_Status& status) override {
        // 处理电机状态更新
        process_motor_data(interface, motor_id, status);
    }
    
    void on_motor_batch_status_update(const std::string& interface, 
                                     const std::map<uint32_t, Motor_Status>& status_all) override {
        // 处理批量状态更新
        for (const auto& [motor_id, status] : status_all) {
            process_motor_data(interface, motor_id, status);
        }
    }
    
private:
    void process_motor_data(const std::string& interface, uint32_t motor_id, const Motor_Status& status) {
        // 自定义处理逻辑
    }
};
```

2. **注册观察者**
```cpp
auto observer = std::make_shared<CustomObserver>();
motor_driver->add_observer(observer);
```

## 发布流程

> **注意**：正式版本发布由项目维护者负责，开发者请勿直接发布。

### 发布检查清单
- [ ] 所有测试通过
- [ ] 文档更新
- [ ] 版本号更新
- [ ] 发布包创建
- [ ] Debian包构建
- [ ] GitHub发布

### 开发者贡献流程
1. Fork项目
2. 创建功能分支
3. 提交Pull Request
4. 等待代码审查
5. 维护者决定是否合并和发布

## 代码规范

### 命名规范
- **类名**: PascalCase (如 `HardwareDriver`)
- **函数名**: snake_case (如 `control_motor_in_velocity_mode`)
- **变量名**: snake_case (如 `motor_config`)
- **常量**: UPPER_SNAKE_CASE (如 `DEFAULT_TIMEOUT`)

### 文件组织
- **头文件**: 只包含声明，不包含实现
- **实现文件**: 包含完整的实现逻辑
- **测试文件**: 对应实现文件的测试

### 注释规范
```cpp
/**
 * @brief 函数功能描述
 * @param param1 参数1描述
 * @param param2 参数2描述
 * @return 返回值描述
 */
```

## 调试指南

### 常见问题

1. **编译错误**
   - 检查C++17标准
   - 检查依赖库
   - 检查头文件包含路径

2. **运行时错误**
   - 检查CAN接口配置
   - 检查权限设置
   - 检查电机连接

3. **性能问题**
   - 检查线程配置
   - 检查通信频率
   - 检查内存使用

### 调试工具

```bash
# 检查库依赖
ldd build/lib/libhardware_driver.so

# 检查符号表
nm build/lib/libhardware_driver.so | grep HardwareDriver

# 运行测试
cd build
./test_robot_hardware
./test_motor_driver_impl
./test_motor_protocol
./test_canfd_bus_integration

# 运行所有测试（如果有测试脚本）
./build.sh里面有运行所有测试的脚本

# 性能分析
valgrind --tool=callgrind ./simple_example

# 查看性能报告
# 使用kcachegrind图形界面查看
kcachegrind callgrind.out.*

# 或者使用命令行工具
callgrind_annotate callgrind.out.*

# 内存泄漏检查
# 检查内存泄漏
valgrind --tool=memcheck --leak-check=full ./simple_example

# 检查内存访问错误
valgrind --tool=memcheck ./simple_example

# 其他有用的选项
# 详细的内存检查
valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all ./simple_example

# 检查未初始化的变量
valgrind --tool=memcheck --track-origins=yes ./simple_example

# 生成详细报告
valgrind --tool=callgrind --callgrind-out-file=profile.out ./simple_example
```

## 版本管理

### 版本号格式
- **主版本号**: 不兼容的API修改
- **次版本号**: 向下兼容的功能性新增
- **修订号**: 向下兼容的问题修正

### 发布检查清单
- [ ] 所有测试通过
- [ ] 文档更新
- [ ] 版本号更新
- [ ] 发布包创建
- [ ] Debian包构建
- [ ] GitHub发布

## 贡献指南

1. **Fork项目**
2. **创建功能分支**
3. **编写代码和测试**
4. **提交Pull Request**
5. **代码审查**
6. **合并到主分支**

## 许可证

本项目采用开源许可证，具体许可证信息请查看项目根目录的 `LICENSE` 文件。

### 开源贡献
欢迎社区贡献代码！请遵循以下流程：
1. Fork项目
2. 创建功能分支
3. 提交Pull Request
4. 等待代码审查
5. 维护者决定是否合并 
