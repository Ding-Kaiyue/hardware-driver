# 硬件驱动库 - API文档

## 概述

硬件驱动库提供了简单易用的API来控制电机和夹爪。支持多种控制模式和实时状态监控。

## 快速开始

```cpp
#include <hardware_driver.hpp>

int main() {
    hardware_driver::HardwareDriver driver;
    
    // 使能电机
    driver.enable_motor("can0", 1, 4);
    
    // 控制电机
    driver.control_motor_in_velocity_mode("can0", 1, 10.0);
    
    // 获取状态
    auto status = driver.get_motor_status("can0", 1);
    
    return 0;
}
```

## API参考

### HardwareDriver类

#### 构造函数
```cpp
HardwareDriver();
```

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

#### 状态查询

##### get_motor_status
```cpp
MotorStatus get_motor_status(const std::string& interface, uint32_t motor_id);
```
- **功能**: 获取单个电机状态
- **返回**: MotorStatus结构体

##### get_all_motor_status
```cpp
std::vector<MotorStatus> get_all_motor_status(const std::string& interface);
```
- **功能**: 获取接口上所有电机状态
- **返回**: MotorStatus向量

#### 夹爪控制

##### control_gripper
```cpp
void control_gripper(const std::string& interface, uint32_t gripper_id, double position);
```
- **功能**: 控制夹爪位置
- **参数**: `position`: 目标位置

## 数据结构

### MotorStatus
```cpp
struct MotorStatus {
    uint32_t motor_id;
    double position;      // 当前位置 (degrees)
    double velocity;      // 当前速度 (degrees/s)
    double effort;        // 当前力矩 (N⋅m)
    uint8_t enable_flag;         // 使能状态
    uint32_t motor_mode;        // 控制模式
    uint8_t limit_flag;         // 限位状态
    uint32_t temperature;       // 温度 (℃)
    uint32_t voltage;           // 电压 (V)
    uint32_t error_code;        // 错误码
};
```

## 错误处理

库在以下情况下会抛出 `std::runtime_error` 异常：

### 常见错误情况
- **CAN接口初始化失败**: 接口不存在或权限不足
- **CAN接口未找到**: 指定的接口名称不存在
- **消息格式错误**: 数据包大小超出限制
- **网络通信错误**: 发送或接收失败

### 错误处理示例
```cpp
try {
    // 初始化硬件驱动
    hardware_driver::HardwareDriver driver(interfaces, motor_config);
    
    // 控制电机
    driver.enable_motor("can0", 1, 4);
    driver.control_motor_in_velocity_mode("can0", 1, 10.0);
    
} catch (const std::runtime_error& e) {
    std::cerr << "运行时错误: " << e.what() << std::endl;
    // 处理错误，如重新初始化或退出程序
} catch (const std::exception& e) {
    std::cerr << "未知错误: " << e.what() << std::endl;
}
```

### 错误检查建议
- 在程序启动时检查CAN接口是否可用
- 定期检查电机连接状态
- 监控错误码和温度等状态信息

## 线程安全

所有API都是线程安全的，可以在多线程环境中使用。

## 性能说明

- 状态查询频率: 没有控制命令时，状态查询频率在 20Hz；有控制命令时，状态查询频率在 2.5kHz
- 控制命令频率: 对于高优先级的控制命令，频率在 5kHz，并且会优先执行，有重试机制；对于低优先级的控制命令，频率在 5kHz，没有重试机制
- 内存使用: < 10MB 