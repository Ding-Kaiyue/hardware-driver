# IAP固件更新使用指南

## 概述

IAP (In-Application Programming) 是一种应用程序内部编程技术，允许在不需要外部编程器的情况下更新设备的固件。本库提供了一个完整的IAP更新框架，支持多个电机的并行固件更新。

## 功能特性

- ✅ 支持加载.bin固件文件（如9NM.bin、51NM.bin等）
- ✅ 数据分片传输，自动处理大文件
- ✅ 实时进度反馈和回调通知
- ✅ 支持多电机并行更新
- ✅ 超时保护和错误处理
- ✅ 线程安全的设计

## 文件说明

### 核心组件

| 文件 | 说明 |
|------|------|
| `src/protocol/iap_protocol.hpp` | IAP协议定义和接口 |
| `src/protocol/iap_protocol.cpp` | IAP协议实现 |
| `src/driver/iap_manager.hpp` | IAP管理器头文件 |
| `src/driver/iap_manager.cpp` | IAP管理器实现 |

### 修改的文件

| 文件 | 修改内容 |
|------|---------|
| `src/driver/motor_driver_impl.hpp` | 添加IAP相关接口 |
| `src/driver/motor_driver_impl.cpp` | 实现IAP相关接口 |

## 基本使用

### 1. 简单的IAP更新示例

```cpp
#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>

using namespace hardware_driver;
using namespace hardware_driver::motor_driver;

// 定义IAP状态反馈观察者
class IAPObserver : public IAPStatusObserver {
public:
    void on_iap_status_feedback(const std::string& interface,
                                uint32_t motor_id,
                                const iap_protocol::IAPStatusMessage& msg) override {
        std::cout << "[IAP] " << interface << ":" << motor_id << " -> "
                  << iap_protocol::iap_status_to_string(msg) << std::endl;
    }
};

int main() {
    // 配置硬件
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1}}
    };

    // 创建电机驱动
    auto motor_driver = createCanFdMotorDriver({"can0"});

    // 创建IAP观察者
    auto iap_observer = std::make_shared<IAPObserver>();

    // 创建机器人硬件实例（带IAP观察者）
    auto robot = std::make_shared<RobotHardware>(motor_driver, motor_config, iap_observer);

    // 执行固件更新
    std::cout << "Starting IAP firmware update..." << std::endl;
    robot->start_update("can0", 1, "../../update_files/9NM.bin");
    std::cout << "IAP update completed!" << std::endl;

    return 0;
}
```

### 2. 使用详细的观察者处理IAP反馈

```cpp
class DetailedIAPObserver : public IAPStatusObserver {
public:
    void on_iap_status_feedback(const std::string& interface,
                                uint32_t motor_id,
                                const iap_protocol::IAPStatusMessage& msg) override {
        std::string status_name = iap_protocol::iap_status_to_string(msg);

        // 根据不同的状态采取相应的处理
        switch (msg) {
            case IAPStatusMessage::AJ01:
                std::cout << "[AJ01] APP收到IAP指令，准备进入IAP模式" << std::endl;
                break;
            case IAPStatusMessage::BS00:
                std::cout << "[BS00] Bootloader已启动，等待接收密钥" << std::endl;
                break;
            case IAPStatusMessage::BK01:
                std::cout << "[BK01] 已进入IAP模式" << std::endl;
                break;
            case IAPStatusMessage::BK02:
                std::cout << "[BK02] 正在擦除APP程序..." << std::endl;
                break;
            case IAPStatusMessage::BK03:
                std::cout << "[BK03] 准备接收固件数据" << std::endl;
                break;
            case IAPStatusMessage::BD04:
                std::cout << "[BD04] 正在接收数据..." << std::endl;
                break;
            case IAPStatusMessage::BJ06:
                std::cout << "[BJ06] 固件验证完成，准备跳转到APP" << std::endl;
                break;
            case IAPStatusMessage::AS00:
                std::cout << "[AS00] ✅ APP启动成功！固件更新完成" << std::endl;
                break;
            default:
                std::cout << "[UNKNOWN] 收到未知状态: " << status_name << std::endl;
                break;
        }
    }
};
```

### 3. 在实时控制环境中使用IAP

```cpp
// 在更新前暂停电机控制
robot->pause_status_monitoring();

// 执行固件更新
std::cout << "开始固件更新..." << std::endl;
robot->start_update("can0", 1, "./firmware/9NM.bin");
std::cout << "固件更新完成！" << std::endl;

// 恢复电机控制和反馈监控
robot->resume_status_monitoring();
```

## IAP更新流程

根据bootloader.pdf文档，IAP更新的流程如下：

### Bootloader阶段
1. **BS00**: Bootloader启动，等待"key"命令
2. **等待key**: 3秒内等待"key"命令（触发IAP模式）
3. **BK01**: 收到"key"命令，进入IAP模式
4. **BK02**: 擦除APP程序
5. **BK03**: 准备接收固件数据

### 数据传输阶段
1. **BD04**: 开始接收数据，解析数据包
2. **BD05**: 500ms内未收到新数据，结束接收，验证APP地址
3. **BJ06/BJ07**: 验证APP地址是否正确
4. 如果正确，跳转到APP

### APP阶段
1. **AS00**: APP启动成功
2. 监听IAP更新指令（重复进入IAP模式）
3. **AJ01**: 收到IAP更新指令，重新进入IAP模式

## 协议格式

### IAP触发命令
发送功能操作命令来触发设备进入IAP模式：
```
CAN ID:  0x1400 + motor_id (如：0x1401 用于电机ID=1)
Byte 0:  0x01 - 功能操作命令类型
Byte 1:  0x12 - MOTOR_IAP_UPDATE (MotorFunc枚举值)
```

### IAP固件数据包
设备进入IAP模式后（BD04状态），直接接收固件二进制数据：
```
CAN ID:  0x1400 + motor_id (如：0x1401 用于电机ID=1)
Data:    固件二进制数据 (最多64字节/CAN-FD)
```

设备会在500ms内无数据时自动结束接收数据（BD05状态），然后验证APP地址。

### IAP状态反馈
设备通过CAN-FD发送ASCII编码的4字节状态消息：
```
CAN ID:  0x0000FF + motor_id (如：0x0000FF01 用于电机ID=1)
Data:    4字节ASCII状态消息（大端序）
```

**支持的状态消息：**
- `BS00` - Bootloader启动
- `BK01` - 收到IAP命令，进入IAP模式
- `BK02` - 擦除APP程序
- `BK03` - 准备接收固件数据
- `BD04` - 开始接收数据
- `BD05` - 接收完成（500ms内无数据）
- `BJ06` - APP地址正确，准备跳转
- `BJ07` - 跳转错误
- `AS00` - APP启动成功
- `AJ01` - APP收到IAP更新指令

这些消息由MotorDriverImpl自动解析和处理，通过IAPManager的状态回调通知应用程序。

## 错误处理

IAP更新过程中可能出现以下错误：

| 错误 | 描述 | 解决方案 |
|------|------|---------|
| 无法打开固件文件 | 文件路径错误或文件不存在 | 检查文件路径和文件权限 |
| 固件文件为空 | 文件大小为0 | 检查固件文件是否完整 |
| 更新已在进行中 | 该电机已有进行中的更新 | 等待当前更新完成或取消 |
| 超时 | 设备未及时响应 | 检查CAN总线连接和设备状态 |
| Flash写入失败 | 设备无法写入Flash | 检查设备固件和硬件状态 |

## 状态机说明

### IAPUpdateState 枚举

```cpp
enum class IAPUpdateState : uint8_t {
    IDLE                = 0x00,  // 空闲状态
    LOADING_FIRMWARE    = 0x01,  // 加载固件中
    TRIGGERING_IAP      = 0x02,  // 触发IAP模式中
    TRANSMITTING        = 0x03,  // 传输固件数据中
    WAITING_COMPLETION  = 0x04,  // 等待完成
    COMPLETED           = 0x05,  // 完成
    ERROR               = 0x06,  // 错误
    CANCELLED           = 0x07,  // 已取消
};
```

## 多电机并行更新示例

```cpp
// 同时对多个电机进行IAP更新
std::vector<std::shared_ptr<IAPManager>> managers;

// 电机1: 9NM型号
auto mgr1 = motor_driver->start_iap_update(
    "can0", 0x01, "9NM.bin",
    [](bool ok, const std::string& msg) {
        std::cout << "Motor 1 update: " << (ok ? "Success" : msg) << std::endl;
    }
);
managers.push_back(mgr1);

// 电机2: 51NM型号
auto mgr2 = motor_driver->start_iap_update(
    "can0", 0x02, "51NM.bin",
    [](bool ok, const std::string& msg) {
        std::cout << "Motor 2 update: " << (ok ? "Success" : msg) << std::endl;
    }
);
managers.push_back(mgr2);

// 等待所有更新完成（可选）
for (auto& mgr : managers) {
    while (mgr->get_state() != IAPUpdateState::COMPLETED &&
           mgr->get_state() != IAPUpdateState::ERROR) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
```

## 注意事项

1. **固件文件路径**: 支持相对路径和绝对路径。相对路径将相对于程序的工作目录。

2. **数据包传输**: CAN-FD最大负载为64字节。IAP数据以64字节为单位直接放入CAN报文传输。
   - 固件文件按64字节分块发送
   - 设备在500ms内无数据时自动结束接收
   - 无需等待每个数据包的ACK反馈，提高传输效率

3. **超时时间**:
   - 触发超时（trigger_timeout）: 等待设备进入IAP模式的时间
   - 数据超时（data_timeout）: 不再使用（设备不发送反馈）
   - 完成超时（completion_timeout）: 等待设备完成更新和重启的时间，**应该设置足够长**（建议 >= 10000ms）

4. **无反馈传输**:
   - IAP传输过程中设备**不会发送任何反馈信息**
   - 主控程序发送完所有数据后，应该等待足够的时间让设备完成处理
   - 根据bootloader.pdf，设备500ms内无数据则结束接收

5. **线程安全**: IAP管理器内部采用互斥锁保护共享资源，可以安全地在多个线程中使用。

6. **回调函数**:
   - 完成回调在更新完成（无论成功或失败）时被调用
   - 进度回调在每个数据包传输完成后被调用

7. **电机响应**: 在IAP更新过程中，电机会进入特殊的IAP模式，期间无法响应其他命令。更新完成后重启。

## 常见问题

### Q: 如何检查固件更新是否成功？
A: 使用完成回调函数的success参数，或调用`iap_manager->get_state()`检查最终状态。

### Q: 更新过程中可以中断吗？
A: 可以，调用`iap_manager->cancel_update()`来取消更新。

### Q: 多个电机能否同时更新？
A: 可以。每个电机有独立的IAP管理器，可以并行更新。

### Q: 如何自定义固件文件存储位置？
A: 在调用`start_iap_update()`时传入完整的文件路径即可。

## 相关链接

- [bootloader.pdf](../bootloader.pdf) - 关于Bootloader的详细文档
- [motor_protocol.hpp](../src/protocol/motor_protocol.hpp) - 电机通信协议
- [iap_protocol.hpp](../src/protocol/iap_protocol.hpp) - IAP协议定义
