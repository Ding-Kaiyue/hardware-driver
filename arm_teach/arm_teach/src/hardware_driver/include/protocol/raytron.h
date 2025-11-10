#ifndef RAYTRON_H
#define RAYTRON_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <atomic>
#include "protocol/usb_class.h"

/*
 * @brief Raytron 关节电机协议统一头文件 (新广播帧协议)
 *
 * 包含所有协议定义、参数定义、指令定义
 * 支持多电机控制 (最多6个电机)
 *
 * ========== CANFD 配置参数 ==========
 * 帧类型：        扩展帧 (29-bit CAN ID)
 * 仲裁域波特率：   1Mbps @ 80% 采样点
 * 数据域波特率：   5Mbps @ 75% 采样点
 * FD加速段：      已启用
 *
 * ========== 新协议特点 ==========
 * 广播帧ID：      0x0000 (控制所有电机)
 * 反馈帧ID：      0x0300 + 电机ID (例如电机1返回0x0301)
 * 最大电机数：    6 (ID: 1-6)
 * 控制方式：      广播帧一次性控制所有电机
 */

namespace raytron
{

#pragma pack(1)

// ========== CAN ID 常量 ==========

constexpr uint32_t BROADCAST_CAN_ID = 0x0000;      // 广播帧CAN ID
constexpr uint32_t FEEDBACK_BASE_ID = 0x0300;      // 反馈帧基础ID (反馈ID = 0x0300 + 电机ID)
constexpr uint32_t SINGLE_FEEDBACK_BASE_ID = 0x0200; // 单电机反馈请求基础ID (请求ID = 0x0200 + 电机ID)
constexpr uint8_t MAX_MOTOR_COUNT = 6;             // 最大支持电机数量

// 单电机控制: CAN ID = 电机ID (1-6)
// 例如: 控制电机1, CAN ID = 0x01; 控制电机2, CAN ID = 0x02
// 单电机反馈请求: CAN ID = 0x0200 + 电机ID, 数据长度1, 数据内容0x00

// ========== CAN 帧格式常量 ==========

/**
 * @brief Raytron CAN 帧大小常量
 */
namespace FrameSize {
    constexpr size_t SINGLE_MOTOR_CONTROL_FRAME = 17;  // 单电机控制帧: [数据长度, enable, mode, pos, vel, cur, kp, kd]
    constexpr size_t BROADCAST_REQUEST_FRAME = 3;      // 广播请求反馈帧: [数据长度, 功能, 操作]
    constexpr size_t BROADCAST_SET_ID_FRAME = 3;       // 广播设置ID帧: [数据长度, 功能, ID值]
    constexpr size_t BROADCAST_ENABLE_FRAME = 8;       // 广播使能帧: [数据长度, 功能, 6个标志位]
    constexpr size_t BROADCAST_CONTROL_FRAME = 50;     // 广播全节点控制帧: [数据长度, 功能, 6*8字节数据]
    constexpr size_t FEEDBACK_FRAME = 24;              // 反馈数据帧 (保持原格式不变)
}

/**
 * @brief 单电机控制帧偏移量
 * 格式: [数据长度1字节][使能标志1字节][控制模式1字节][位置4字节][速度4字节][电流4字节][Kp 1字节][Kd 1字节]
 */
namespace SingleMotorControlOffset {
    constexpr size_t DATA_LENGTH = 0;              // 数据长度 (1 byte) = 0x10 (16)
    constexpr size_t ENABLE_FLAG = 1;              // 控制使能标志 (1 byte): 0x00失能, 0x01使能
    constexpr size_t CONTROL_MODE = 2;             // 控制模式 (1 byte): 0x02电流, 0x03力位, 0x04速度, 0x05绝对位置, 0x06增量位置
    constexpr size_t POSITION_START = 3;           // 位置 float32 (4 bytes)
    constexpr size_t VELOCITY_START = 7;           // 速度 float32 (4 bytes)
    constexpr size_t CURRENT_START = 11;           // 电流 float32 (4 bytes)
    constexpr size_t KP = 15;                      // Kp*1000 uint8 (1 byte)
    constexpr size_t KD = 16;                      // Kd*1000 uint8 (1 byte)
}

/**
 * @brief 广播帧功能码
 */
namespace BroadcastFunction {
    constexpr uint8_t REQUEST_FEEDBACK = 0x00;     // 请求反馈
    constexpr uint8_t SET_ID = 0x01;               // 设置ID
    constexpr uint8_t SET_ENABLE = 0x02;           // 设置使能
    constexpr uint8_t CONTROL_ALL = 0x03;          // 全节点控制
}

/**
 * @brief 广播请求反馈帧偏移量
 */
namespace BroadcastRequestOffset {
    constexpr size_t DATA_LENGTH = 0;              // 数据长度 (1 byte) = 0x02
    constexpr size_t FUNCTION = 1;                 // 功能码 (1 byte) = 0x00
    constexpr size_t OPERATION = 2;                // 操作 (1 byte) = 0x00
}

/**
 * @brief 广播使能帧偏移量
 */
namespace BroadcastEnableOffset {
    constexpr size_t DATA_LENGTH = 0;              // 数据长度 (1 byte) = 0x07
    constexpr size_t FUNCTION = 1;                 // 功能码 (1 byte) = 0x02
    constexpr size_t FLAG_START = 2;               // 标志位起始 (6 bytes)
}

/**
 * @brief 广播全节点控制帧偏移量
 * 每个电机8字节: [位置2字节][速度2字节][力矩2字节][Kp 1字节][Kd 1字节]
 */
namespace BroadcastControlOffset {
    constexpr size_t DATA_LENGTH = 0;              // 数据长度 (1 byte) = 0x31 (49)
    constexpr size_t FUNCTION = 1;                 // 功能码 (1 byte) = 0x03
    constexpr size_t MOTOR_DATA_START = 2;         // 电机数据起始 (48 bytes = 6*8)
    constexpr size_t MOTOR_DATA_SIZE = 8;          // 每个电机数据大小

    // 单个电机数据内部偏移
    constexpr size_t POSITION = 0;                 // 位置 int16 (2 bytes)
    constexpr size_t VELOCITY = 2;                 // 速度 int16 (2 bytes)
    constexpr size_t TORQUE = 4;                   // 力矩 int16 (2 bytes)
    constexpr size_t KP = 6;                       // Kp uint8 (1 byte)
    constexpr size_t KD = 7;                       // Kd uint8 (1 byte)
}

/**
 * @brief 反馈帧字节偏移量定义（新协议）
 * 反馈帧ID = 0x0300 + 电机ID
 * 数据长度: 0x17 (23字节数据 + 1字节长度 = 24字节总长)
 */
namespace FeedbackFrameOffset {
    constexpr size_t DATA_LENGTH = 0;              // 数据长度 (1 byte) = 0x17
    constexpr size_t ENABLE_FLAG = 1;              // 当前使能标志 (1 byte) uint8
    constexpr size_t CONTROL_MODE = 2;             // 当前控制模式 (1 byte) uint8
    constexpr size_t POSITION_START = 3;           // 当前位置 (4 bytes) float32
    constexpr size_t VELOCITY_START = 7;           // 当前速度 (4 bytes) float32
    constexpr size_t CURRENT_START = 11;           // 当前电流 (4 bytes) float32
    constexpr size_t ERROR_CODE_START = 15;        // 当前错误码 (4 bytes) uint32
    constexpr size_t VOLTAGE_START = 19;           // 当前电压×10 (2 bytes) uint16
    constexpr size_t TEMPERATURE_START = 21;       // 当前温度×10 (2 bytes) uint16
    constexpr size_t LIMIT_FLAG = 23;              // 限位标志 (1 byte) uint8
}

// ========== 控制命令ID (已废弃，使用广播帧) ==========
// 新协议使用广播帧ID 0x0000，通过功能码区分不同操作

// ========== 反馈响应ID ==========
// 反馈帧ID = 0x0300 + 电机ID
// 例如: 电机1 -> 0x0301, 电机2 -> 0x0302, ..., 电机6 -> 0x0306

// ========== 控制模式 ==========
enum ControlMode : uint8_t
{
    MODE_CURRENT            = 0x02,  // 电流模式
    MODE_FORCE_POSITION     = 0x03,  // 力位混合模式
    MODE_SPEED              = 0x04,  // 速度模式
    MODE_ABSOLUTE_POS       = 0x05,  // 绝对位置模式
    MODE_INCREMENTAL_POS    = 0x06,  // 增量位置模式
};

// ========== 使能标志 ==========
enum EnableFlag : uint8_t
{
    FLAG_DISABLE = 0x00,
    FLAG_ENABLE  = 0x01,
};

// ========== 标志位编码 ==========
// 标志位 = (使能标志 << 4) | 控制模式
inline uint8_t encodeFlag(EnableFlag enable, ControlMode mode) {
    return (static_cast<uint8_t>(enable) << 4) | static_cast<uint8_t>(mode);
}

inline void decodeFlag(uint8_t flag, EnableFlag& enable, ControlMode& mode) {
    enable = static_cast<EnableFlag>((flag >> 4) & 0x0F);
    mode = static_cast<ControlMode>(flag & 0x0F);
}

// ========== 数据缩放因子 ==========
namespace ScaleFactor {
    constexpr float POSITION_SCALE = 100.0f;       // 位置指令 * 100
    constexpr float VELOCITY_SCALE = 100.0f;       // 速度指令 * 100
    constexpr float TORQUE_SCALE = 10.0f;          // 力矩指令 * 10
    constexpr float KP_SCALE = 1000.0f;            // Kp * 1000
    constexpr float KD_SCALE = 1000.0f;            // Kd * 1000
}

// ========== 单个电机控制数据 ==========
struct MotorControlData
{
    float position;     // 位置指令 (实际值，自动缩放100倍)
    float velocity;     // 速度指令 (实际值，自动缩放100倍)
    float torque;       // 力矩指令 (实际值，自动缩放10倍)
    float kp;           // Kp增益 (实际值，自动缩放1000倍)
    float kd;           // Kd增益 (实际值，自动缩放1000倍)

    MotorControlData() : position(0), velocity(0), torque(0), kp(0), kd(0) {}

    MotorControlData(float pos, float vel, float tor, float k_p, float k_d)
        : position(pos), velocity(vel), torque(tor), kp(k_p), kd(k_d) {}
};

// ========== 单个电机状态标志 ==========
struct MotorEnableFlag
{
    EnableFlag enable;
    ControlMode mode;

    MotorEnableFlag() : enable(FLAG_DISABLE), mode(MODE_CURRENT) {}

    MotorEnableFlag(EnableFlag en, ControlMode m) : enable(en), mode(m) {}

    uint8_t encode() const {
        return encodeFlag(enable, mode);
    }

    void decode(uint8_t flag) {
        decodeFlag(flag, enable, mode);
    }
};

// ========== 反馈数据结构 (格式保持不变) ==========
struct FeedbackData
{
    uint8_t dlc;                    // 数据长度
    uint8_t enable_flag;            // 使能标志
    uint8_t control_mode;           // 控制模式
    uint8_t reserved1;              // 保留字节
    float position_feedback;        // 位置反馈 (float32)
    float velocity_feedback;        // 速度反馈 (float32)
    float current_feedback;         // 电流反馈 (float32)
    uint32_t error_code;            // 错误代码
    uint16_t system_voltage;        // 系统电压
    uint16_t temperature;           // 温度
    uint8_t limit_flag;             // 限制标志
};

#pragma pack()

}; // namespace raytron

#endif // RAYTRON_H
