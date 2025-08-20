# 事件总线系统使用指南

## 概述

这是一个通用的、类型安全的事件总线系统，支持发布-订阅模式，可以与现有的观察者模式共存。

## 核心组件

### 1. EventBus - 事件总线核心
```cpp
auto event_bus = std::make_shared<EventBus>();
```

### 2. Event - 事件基类
所有事件都继承自`Event`类，提供类型名称和主题信息。

### 3. EventBridge - 观察者模式桥接器
将现有的观察者模式转换为事件总线事件。

## 基本使用方法

### 创建和发布事件
```cpp
// 方法1：直接创建事件对象
auto event = std::make_shared<MotorStatusEvent>("can0", 1, motor_status);
event_bus->publish(event);

// 方法2：使用emit便捷方法
event_bus->emit<MotorStatusEvent>("can0", 1, motor_status);
```

### 订阅事件
```cpp
// 按类型订阅
event_bus->subscribe<MotorStatusEvent>([](const auto& event) {
    std::cout << "Motor status: " << event->get_status().position << std::endl;
});

// 按主题订阅
event_bus->subscribe_topic<MotorStatusEvent>("motor.can0.*.status", [](const auto& event) {
    // 只处理can0接口的状态事件
});
```

### 与现有观察者模式集成
```cpp
// 创建事件桥接器
auto event_bridge = std::make_shared<EventBridge>(event_bus);

// 注册为观察者
motor_driver.add_observer(event_bridge);

// 现在所有的电机事件都会自动发布到事件总线
```

## 事件类型

### MotorStatusEvent
电机状态更新事件，包含位置、速度、温度等信息。

### MotorFunctionResultEvent  
电机函数操作结果事件，包含操作码和成功状态。

### MotorParameterResultEvent
电机参数读写结果事件，包含地址、数据类型和数值。

## 主题命名规范

事件主题采用层次化命名：
- `motor.{interface}.{motor_id}.status` - 电机状态
- `motor.{interface}.{motor_id}.function` - 函数操作
- `motor.{interface}.{motor_id}.parameter` - 参数操作

## 线程安全

- 所有EventBus操作都是线程安全的
- 使用mutex保护内部数据结构
- 事件处理器中的异常不会影响其他处理器

## 性能特性

- 类型安全：编译时检查事件类型
- 零拷贝：使用shared_ptr传递事件
- 弱引用：自动清理失效的处理器
- 统计信息：提供性能监控数据

## 使用场景

### 实时控制
保持观察者模式用于实时控制，事件总线用于非实时功能。

### 日志记录
```cpp
class Logger {
    Logger(std::shared_ptr<EventBus> bus) {
        bus->subscribe<MotorStatusEvent>([](const auto& event) {
            log_to_file(event);
        });
    }
};
```

### 监控和诊断
```cpp
class Monitor {
    Monitor(std::shared_ptr<EventBus> bus) {
        bus->subscribe<MotorStatusEvent>([](const auto& event) {
            check_performance(event);
        });
    }
};
```

### 示教功能（未来扩展）
事件总线的设计为示教功能提供了基础：
- 事件记录：订阅所有状态事件并保存
- 时间戳：每个事件都有精确时间戳
- 回放：按时间顺序重新发布事件

## 最佳实践

1. **分层架构**：实时控制用观察者，其他功能用事件总线
2. **异常处理**：在事件处理器中捕获异常
3. **资源管理**：使用RAII管理事件处理器生命周期
4. **性能监控**：定期调用`get_statistics()`检查性能
5. **主题设计**：使用层次化主题便于过滤

## 扩展事件类型

创建新的事件类型：
```cpp
class CustomEvent : public Event {
public:
    CustomEvent(/* parameters */) { /* implementation */ }
    
    std::string get_type_name() const override {
        return "CustomEvent";
    }
    
    std::string get_topic() const override {
        return "custom.topic";
    }
    
    // 添加访问器方法
};
```

## 实际示例代码

### 观察者模式使用示例
参考 `examples/example_motor_observer.cpp` 查看直接观察者模式的完整使用示例：
```cpp
// 电机状态观察者 - 打印接收到的数据
class MotorStatusPrinter : public hardware_driver::motor_driver::MotorStatusObserver {
public:
    void on_motor_status_update(const std::string& interface, 
                               uint32_t motor_id, 
                               const hardware_driver::motor_driver::Motor_Status& status) override {
        std::cout << "[状态] 电机 " << interface << ":" << motor_id 
                  << " | 位置:" << status.position 
                  << " | 速度:" << status.velocity
                  << " | 力矩:" << status.effort << std::endl;
    }
};

// 使用方法
auto status_printer = std::make_shared<MotorStatusPrinter>();
motor_driver->add_observer(status_printer);
```

### 事件总线使用示例  
参考 `examples/example_motor_event_bus.cpp` 查看事件总线的完整使用示例：
```cpp
// 创建事件总线
auto event_bus = std::make_shared<hardware_driver::event::EventBus>();

// 创建电机驱动时启用事件总线
auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
motor_driver->set_event_bus(event_bus);

// 订阅事件
event_bus->subscribe<hardware_driver::event::MotorStatusEvent>([](const auto& event) {
    std::cout << "Event Bus收到电机状态: " << event->get_status().position << std::endl;
});
```

### 混合使用模式
您可以同时使用观察者模式和事件总线：
- **实时控制**：使用直接观察者获得最佳性能
- **数据记录、监控等**：使用事件总线实现解耦架构