/*********************************************************************
 * @file        motor_protocol.hpp
 * @brief       电机通信协议说明头文件，仅包含协议相关的枚举、结构体定义。
 * 
 * 本文件定义了电机通信协议中的数据结构和枚举类型，供 motor_driver 模块使用。
 * 此文件不包含任何协议数据的构造或解析逻辑。
 * 
 * @author      Kaiyue Ding
 * @version     0.0.1.3
 * @date        2025-07-22
 * 
 * @copyright   Copyright (c) 2025 Raysense Technology. All rights reserved.
 * 
 * @history     2025-07-22 Kaiyue Ding 创建文件，定义基本协议结构。
 *              2025-11-03 Kaiyue Ding 修改协议结构，适配关节模组v0.0.1.5协议
 *********************************************************************/

#ifndef __HARDWARE_DRIVER_MOTOR_PROTOCOL_HPP__
#define __HARDWARE_DRIVER_MOTOR_PROTOCOL_HPP__

#include <vector>
#include <cstdint>
#include <optional>
#include <mutex>
#include <map>
#include "hardware_driver/bus/bus_interface.hpp"
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include <variant>
#include <cstring>
#include <any>

namespace hardware_driver {
namespace motor_protocol {

enum class MotorFeedbackType : uint8_t {
    DATA1 = 0x01,   // Include: enable_flag, motor_mode, position
    DATA2 = 0x02,   // Include: limit_flag, temeprature, velocity
    DATA3 = 0x03,   // Include: voltage, current
    DATA4 = 0x04,   // Include: error_code
};

enum class MotorSendKind : uint32_t {
    MOTOR_CTRL = 0x000,
    FDB_REQ = 0x200,
    FUNC_CTRL = 0x400,
    PARAM_RW = 0x600,
};

// 反馈类型
enum class MotorFeedbackKind : uint32_t{
    MOTOR_CTRL = 0x100, 
    MOTOR_STATUS = 0x300,
    FUNC_RESULT = 0x500,
    PARAM_RESULT = 0x700,
};

/**
 * @brief 电机状态反馈接口数据结构体
 */
struct MotorStatusFeedback {
    motor_driver::Motor_Status status;
    std::string interface;
    uint32_t motor_id;
};

/**
 * @brief 电机函数操作接口数据结构体
 */
struct FuncResultFeedback {
    uint8_t op_code;
    bool success;
    std::string interface;
    uint32_t motor_id;
};

/**
 * @brief 电机参数读写接口数据结构体
 */
struct ParamResultFeedback {
    uint8_t rw_method;
    uint16_t addr;
    uint8_t data_type;
    std::any data;
    std::string interface;
    uint32_t motor_id;
};

using MotorFeedback = std::variant<MotorStatusFeedback, FuncResultFeedback, ParamResultFeedback>;

enum class OperationMethod : uint8_t {
    READ = 0x01,
    WRITE = 0x02,
};

enum class MotorControlMode : uint8_t {
    EFFORT_MODE         = 0x02,
    MIT_MODE             = 0x03,
    SPEED_MODE           = 0x04,
    POSITION_ABS_MODE    = 0x05,
    POSITION_INC_MODE    = 0x06
};

enum class MotorFunc : uint8_t {
    PARAM_RESET = 0x01,                 // 参数恢复默认
    PARAM_SAVE_TO_FLASH = 0x02,         // 参数保存到Flash
    CLEAR_ERROR_CODE = 0x03,            // 清除错误码
    MOTOR_ZERO_POS_SET = 0x04,          // 设置当前位置为电机零位
    MOTOR_FIND_ZERO_POS = 0x11,         // 电机自动寻零
    MOTOR_IAP_UPDATE = 0x12,            // 电机IAP更新
    MOTOR_HALL_CALIBRATION = 0x13,      // 电机霍尔校准
    MOTOR_CURRENT_CALIBRATION = 0x14,   // 电机电流校准
    MOTOR_ENCODER_CALIBRATION = 0x15,   // 电机编码器校准
    MOTOR_SOFTWARE_RESET = 0x16         // 电机软件复位
};

enum class ParameterEnum : uint16_t {
    SYS_CAN_ID_BASE = 0x0000,
    SERVO_CONTROL_MODE = 0x0001,
    SERVO_ENABLE_FLAG = 0x0002,
    LIMIT_POS_FLAG = 0x0003,
    SYS_CAN_TYPE = 0x0004,
    SYS_INT_RESERVER_3 = 0x0005,
    SYS_INT_RESERVER_2 = 0x0006,
    SYS_INT_RESERVER_1 = 0x0007,
    LIMIT_POS_MAX = 0x0008,
    LIMIT_POS_MIN = 0x0009,
    LIMIT_POS_ZERO = 0x000A,
    LIMIT_VELOCITY_MAX = 0x000B,
    LIMIT_VELOCITY_ACC_T1 = 0x000C,
    LIMIT_CURRENT_MAX = 0x000D,
    LIMIT_VOLTAGE_MAX = 0x000E,
    LIMIT_VELOCITY_ACC_T3 = 0x000F,
};


// 解包函数：将通用总线包解析为电机状态
// 根据通信方式（总线类型）来选择不同的解析策略
std::optional<MotorFeedback> parse_feedback(const bus::GenericBusPacket& packet);

// 为不同总线类型提供重载或不同的函数
std::optional<MotorFeedback> parse_can_feedback(const bus::GenericBusPacket& /*packet*/);
std::optional<MotorFeedback> parse_canfd_feedback(const bus::GenericBusPacket& packet);
std::optional<MotorFeedback> parse_ethercat_feedback(const bus::GenericBusPacket& /*packet*/);

/**
 * @brief 打包禁用电机命令，电机工作在失能模式
 * @return bool 返回是否成功打包
 */
bool pack_disable_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    uint8_t mode);

/**
 * @brief 打包批量禁用电机命令，电机工作在失能模式, 指定所有电机是同一个工作模式
 * @return bool 返回是否成功打包
 */
bool pack_disable_all_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    std::vector<uint8_t> disable_flags, uint8_t mode);

/**
 * @brief 打包启用电机命令，电机工作在指定模式
 * @return bool 返回是否成功打包
 */
bool pack_enable_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    uint8_t mode);

/**
 * @brief 打包批量启用电机命令，电机工作在指定模式, 指定所有电机是同一个工作模式
 * @return bool 返回是否成功打包
 */
bool pack_enable_all_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    std::vector<uint8_t> enable_flags, uint8_t mode);

/**
 * @brief 打包控制命令，电机工作的控制打包函数只负责填充data和len
 * @return bool 返回是否成功打包
 */
bool pack_control_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    float position, float velocity, float effort, float kp, float kd);

/**
 * @brief 打包批量控制命令，支持最多 6 个电机
 * @param data CAN 数据缓冲区
 * @param len 数据长度（输出参数）
 * @param positions 位置数组[0..5]
 * @param velocities 速度数组[0..5]
 * @param efforts 力矩数组[0..5]
 * @param kps kp 参数数组[0..5]
 * @param kds kd 参数数组[0..5]
 * @return bool 返回是否成功打包
 */
bool pack_control_all_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    const std::array<float, 6>& positions, const std::array<float, 6>& velocities,
    const std::array<float, 6>& efforts, const std::array<float, 6>& kps, const std::array<float, 6>& kds);

/**
 * @brief 打包参数读取命令，读取电机参数
 * @return bool 返回是否成功打包
 */
bool pack_param_read(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
                            uint16_t param_addr);

/**
 * @brief 打包参数写入命令，写入电机参数
 * @return bool 返回是否成功打包
 */
bool pack_param_write(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
                        uint16_t param_addr, int32_t param_value);

bool pack_param_write(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
                        uint16_t param_addr, float param_value);
/**
 * @brief 打包功能操作命令，执行电机功能操作
 * @return bool 返回是否成功打包
 */
bool pack_function_operation(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
                             uint8_t op_code);
/**
 * @brief 打包反馈请求命令，请求某个电机反馈
 * @return bool 返回是否成功打包
 */
bool pack_motor_feedback_request(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len);
/**
 * @brief 打包反馈请求命令，请求所有电机反馈
 * @return bool 返回是否成功打包
 */
bool pack_motor_feedback_request_all(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len);

}   // namespace motor_protocol
}   // namespace hardware_driver

#endif    // __HARDWARE_DRIVER_MOTOR_PROTOCOL_HPP__
