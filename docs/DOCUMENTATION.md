# 硬件驱动库 v2.0.0 - API文档

## 概述

v2.0.0版本采用全新的事件驱动架构，提供三种数据获取模式：观察者模式、回调模式和事件总线。支持高性能多线程处理，控制延迟降至微秒级，状态更新频率支持高低频智能切换。

## 快速开始

### 观察者模式示例（推荐用于实时控制）

```cpp
#include "hardware_driver/interface/robot_hardware.hpp"

// 创建电机状态观察者
class MotorStatusPrinter : public hardware_driver::motor_driver::MotorStatusObserver {
public:
    void on_motor_status_update(const std::string& interface, uint32_t motor_id, 
                               const hardware_driver::motor_driver::Motor_Status& status) override {
        std::cout << "电机 " << interface << ":" << motor_id 
                  << " | 位置:" << status.position 
                  << " | 速度:" << status.velocity << std::endl;
    }
};

int main() {
    // 配置硬件
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 9}}
    };
    
    // 创建CAN总线
    auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
    
    // 创建电机驱动
    auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
    
    // 添加观察者
    auto observer = std::make_shared<MotorStatusPrinter>();
    motor_driver->add_observer(observer);
    
    // 创建硬件接口
    RobotHardware robot(motor_driver, motor_config);
    
    // 控制电机
    robot.control_motor_in_velocity_mode("can0", 1, 5.0);
    
    return 0;
}
```

### 事件总线示例（推荐用于监控、日志等）

```cpp
#include "hardware_driver/interface/robot_hardware.hpp"
#include "hardware_driver/event/event_bus.hpp"

int main() {
    // 配置硬件
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 9}}
    };
    
    // 创建事件总线
    auto event_bus = std::make_shared<hardware_driver::event::EventBus>();
    
    // 创建CAN总线和电机驱动
    auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
    auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
    
    // 设置事件总线
    motor_driver->set_event_bus(event_bus);
    
    // 订阅电机状态事件
    event_bus->subscribe<hardware_driver::event::MotorStatusEvent>([](const auto& event) {
        std::cout << "Event Bus收到状态: " << event->get_status().position << std::endl;
    });
    
    // 创建硬件接口
    RobotHardware robot(motor_driver, motor_config);
    
    // 控制电机
    robot.control_motor_in_velocity_mode("can0", 1, 5.0);
    
    return 0;
}
```

### 回调模式示例（兼容v1.x用法）

```cpp
#include "hardware_driver/interface/robot_hardware.hpp"

int main() {
    // 配置硬件
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 9}}
    };
    
    // 创建CAN总线和电机驱动
    auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
    
    // 配置批量状态回调
    bus->set_batch_callback([](const std::string& interface, 
                              const std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>& status_all) {
        for (const auto& [motor_id, status] : status_all) {
            std::cout << "回调收到状态 " << interface << ":" << motor_id 
                      << " 位置:" << status.position << std::endl;
        }
    });
    
    auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
    
    // 创建硬件接口
    RobotHardware robot(motor_driver, motor_config);
    
    // 控制电机
    robot.control_motor_in_velocity_mode("can0", 1, 5.0);
    
    return 0;
}
```

## API参考

### RobotHardware类

v2.0.0的主要用户接口类，提供统一的电机控制API。

#### 构造函数

```cpp
RobotHardware(std::shared_ptr<motor_driver::MotorDriverInterface> motor_driver,
              const std::map<std::string, std::vector<uint32_t>>& motor_config);
```
- **功能**: 创建机器人硬件接口
- **参数**: 
  - `motor_driver`: 电机驱动实例
  - `motor_config`: 电机配置，格式: {{"can0", {1,9}}}

#### 电机控制

##### enable_motor
```cpp
void enable_motor(const std::string& interface, uint32_t motor_id, uint32_t mode);
```
- **功能**: 使能指定电机
- **参数**: 
  - `interface`: CAN接口名称 (如 "can0")
  - `motor_id`: 电机ID
  - `mode`: 控制模式 (1-4)

##### disable_motor
```cpp
void disable_motor(const std::string& interface, uint32_t motor_id);
```
- **功能**: 失能指定电机

##### control_motor_in_velocity_mode
```cpp
void control_motor_in_velocity_mode(const std::string& interface, uint32_t motor_id, double velocity);
```
- **功能**: 速度模式控制
- **参数**: `velocity`: 目标速度 (degrees/s)

##### control_motor_in_position_mode
```cpp
void control_motor_in_position_mode(const std::string& interface, uint32_t motor_id, double position);
```
- **功能**: 位置模式控制
- **参数**: `position`: 目标位置 （degrees）

##### control_motor_in_effort_mode
```cpp
void control_motor_in_effort_mode(const std::string& interface, uint32_t motor_id, double effort);
```
- **功能**: 力矩模式控制
- **参数**: `effort`: 目标力矩 (N⋅m)

##### control_motor_in_mit_mode
```cpp
void control_motor_in_mit_mode(const std::string& interface, uint32_t motor_id, 
                              double position, double velocity, double effort);
```
- **功能**: MIT模式控制 (位置+速度+力矩)
- **参数**: 
  - `position`: 目标位置 (degrees)
  - `velocity`: 目标速度 (degrees/s)
  - `effort`: 目标力矩 (N⋅m)

#### 参数操作

##### read_motor_param
```cpp
void read_motor_param(const std::string& interface, uint32_t motor_id, uint16_t address);
```
- **功能**: 读取电机参数
- **参数**: `address`: 参数地址

##### write_motor_param (int32_t)
```cpp
void write_motor_param(const std::string& interface, uint32_t motor_id, uint16_t address, int32_t value);
```
- **功能**: 写入整数参数

##### write_motor_param (float)
```cpp
void write_motor_param(const std::string& interface, uint32_t motor_id, uint16_t address, float value);
```
- **功能**: 写入浮点参数

#### 功能操作

##### motor_function_operation
```cpp
void motor_function_operation(const std::string& interface, uint32_t motor_id, uint8_t opcode);
```
- **功能**: 执行电机函数操作
- **参数**: `opcode`: 操作码

##### set_motor_zero_position
```cpp
void set_motor_zero_position(const std::string& interface, const std::vector<uint32_t>& motor_ids);
```
- **功能**: 设置电机零位
- **参数**: `motor_ids`: 电机ID列表

### MotorDriverImpl类

电机驱动实现类，提供观察者模式和事件总线支持。

#### 观察者管理

##### add_observer
```cpp
void add_observer(std::shared_ptr<MotorStatusObserver> observer);
```
- **功能**: 添加状态观察者

##### remove_observer
```cpp
void remove_observer(std::shared_ptr<MotorStatusObserver> observer);
```
- **功能**: 移除状态观察者

#### 事件总线集成

##### set_event_bus
```cpp
void set_event_bus(std::shared_ptr<event::EventBus> event_bus);
```
- **功能**: 设置事件总线

### CanFdBus类

CAN-FD总线实现类，提供高性能通信。

#### 构造函数
```cpp
CanFdBus(const std::vector<std::string>& interfaces);
```
- **功能**: 创建CAN-FD总线实例
- **参数**: `interfaces`: CAN接口列表

#### 回调设置

##### set_batch_callback
```cpp
void set_batch_callback(BatchStatusCallback callback);
```
- **功能**: 设置批量状态回调函数

### EventBus类

事件总线系统，提供类型安全的发布/订阅机制。

#### 事件订阅

##### subscribe
```cpp
template<typename EventType>
void subscribe(std::function<void(std::shared_ptr<const EventType>)> handler);
```
- **功能**: 按类型订阅事件

##### subscribe_topic
```cpp
template<typename EventType>
void subscribe_topic(const std::string& topic_pattern, 
                     std::function<void(std::shared_ptr<const EventType>)> handler);
```
- **功能**: 按主题订阅事件

#### 事件发布

##### emit
```cpp
template<typename EventType, typename... Args>
void emit(Args&&... args);
```
- **功能**: 发布事件

## 观察者接口

### MotorStatusObserver

电机状态观察者接口，用于接收实时状态更新。

```cpp
class MotorStatusObserver {
public:
    virtual void on_motor_status_update(const std::string& interface, uint32_t motor_id, 
                                       const Motor_Status& status) = 0;
    
    virtual void on_motor_batch_status_update(const std::string& interface, 
                                             const std::map<uint32_t, Motor_Status>& status_all) = 0;
    
    virtual void on_motor_function_result(const std::string& interface, uint32_t motor_id, 
                                         uint8_t opcode, bool success) = 0;
    
    virtual void on_motor_parameter_result(const std::string& interface, uint32_t motor_id, 
                                          uint16_t address, uint8_t data_type, int32_t value) = 0;
};
```

## 事件类型

### MotorStatusEvent
单个电机状态事件，包含位置、速度、力矩等信息。

### MotorBatchStatusEvent  
批量电机状态事件，包含一个接口上所有电机的状态。

### MotorFunctionResultEvent
电机函数操作结果事件，包含操作码和成功状态。

### MotorParameterResultEvent
电机参数读写结果事件，包含地址、数据类型和数值。

## 数据结构

### Motor_Status
```cpp
struct Motor_Status {
    uint32_t motor_id;
    float position;          // 当前位置 (degrees)
    float velocity;          // 当前速度 (degrees/s)  
    float effort;            // 当前力矩 (N⋅m)
    uint8_t enable_flag;     // 使能状态
    uint8_t motor_mode;      // 控制模式
    uint8_t limit_flag;      // 限位状态
    uint8_t temperature;     // 温度 (℃)
    uint16_t voltage;        // 电压 (V)
    uint16_t error_code;     // 错误码
};
```

## 错误处理

库在以下情况下会抛出 `std::runtime_error` 异常：

### 常见错误情况
- **CAN接口初始化失败**: 接口不存在或权限不足
- **电机驱动初始化失败**: 总线通信问题
- **消息发送失败**: 网络通信异常
- **配置错误**: 电机配置参数无效

### 错误处理示例
```cpp
try {
    // 创建硬件
    auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
    auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
    RobotHardware robot(motor_driver, motor_config);
    
    // 控制电机
    robot.enable_motor("can0", 1, 4);
    robot.control_motor_in_velocity_mode("can0", 1, 10.0);
    
} catch (const std::runtime_error& e) {
    std::cerr << "运行时错误: " << e.what() << std::endl;
} catch (const std::exception& e) {
    std::cerr << "未知错误: " << e.what() << std::endl;
}
```

## 线程安全

所有API都是线程安全的，可以在多线程环境中使用。v2.0.0采用多线程架构：
- **控制线程**: 处理异步控制命令
- **反馈线程**: 高低频状态请求 (智能切换)
- **数据处理线程**: 事件分发和观察者通知

## 性能说明

### v2.0.0性能指标
- **控制延迟**: < 200μs (显著提升)
- **状态更新频率**: 高低频智能切换 (有控制命令时2.5kHz，无控制命令时降频)
- **CPU使用率**: 降低60% (相同负载下)
- **内存效率**: 提高40%，减少内存碎片
- **并发电机数**: 支持16+电机 (受限于CAN-FD总线通信能力)

### 数据流性能对比
- **观察者模式**: 最佳性能，直接回调
- **事件总线**: 轻微性能开销，提供解耦优势  
- **回调模式**: 兼容v1.x，性能居中

## 最佳实践

### 选择合适的数据获取模式
1. **实时控制**: 使用观察者模式，获得最佳性能
2. **监控日志**: 使用事件总线，实现解耦架构
3. **简单应用**: 使用回调模式，兼容旧版本

### 资源管理
- 使用shared_ptr管理观察者生命周期
- 及时移除不需要的事件订阅
- 合理设置CPU亲和性

### 错误处理
- 在观察者中捕获异常
- 监控电机温度和错误码
- 实现重连和恢复机制

## 从v1.x迁移

### 主要变化
1. **API架构**: 从HardwareDriver类改为RobotHardware + MotorDriverImpl
2. **数据获取**: 从get_motor_status()轮询改为事件驱动
3. **构造方式**: 需要显式创建总线和驱动实例

### 迁移示例
```cpp
// v1.x 写法
hardware_driver::HardwareDriver driver(interfaces, motor_config);
auto status = driver.get_motor_status("can0", 1);

// v2.0.0 写法 (观察者模式)
auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
RobotHardware robot(motor_driver, motor_config);

// 添加观察者接收状态
auto observer = std::make_shared<MyObserver>();
motor_driver->add_observer(observer);
```

详细的迁移指南请参考 `docs/RELEASE_NOTES.md`。