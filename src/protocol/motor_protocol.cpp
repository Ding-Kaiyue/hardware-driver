/****************************************************************************************
 * @file        motor_protocol.cpp
 * @brief       电机协议实现文件，负责解析从总线收到的反馈数据和构造发送到总线的指令数据
 * 
 * 1. 实现了针对 CAN、CAN FD、EtherCAT 等总线反馈的协议数据解析函数
 * 2. 打包各个模式控制电机、参数读写、函数操作、请求反馈等指令的函数。
 * @attention   parse_can_feedback() 函数需要根据具体的电机型号和协议进行实现
 *              parse_ethercat_feedback() 函数需要根据具体的电机型号和协议进行实现
 * @author      Kaiyue Ding
 * @version     0.0.1.3
 * @date        2025-07-22
 * 
 * @copyright   Copyright (c) 2025 Raysense Technology. All rights reserved.
 * 
 * @history     2025-07-22 Kaiyue Ding 创建文件，实现 CAN FD 协议解析
 ***************************************************************************************/

#include "motor_protocol.hpp"


// Helper functions for byte manipulation, can be moved to a separate utility file
namespace {
    // 将float转为高位在前的字节序并写入数组
    static void float_to_big_endian_bytes(float value, uint8_t* out) {
        union {
            float f;
            uint8_t b[4];
        } u;
        u.f = value;
        out[0] = u.b[3];
        out[1] = u.b[2];
        out[2] = u.b[1];
        out[3] = u.b[0];
    }

    // 从高位在前的字节流中读取float
    static float big_endian_bytes_to_float(const uint8_t* in) {
        union {
            uint8_t b[4];
            float f;
        } u;
        u.b[3] = in[0];
        u.b[2] = in[1];
        u.b[1] = in[2];
        u.b[0] = in[3];
        return u.f;
    }

    // 从高位在前的字节流中读取uint32
    static uint32_t big_endian_bytes_to_uint32(const uint8_t* in) {
        return (static_cast<uint32_t>(in[0]) << 24) |
               (static_cast<uint32_t>(in[1]) << 16) |
               (static_cast<uint32_t>(in[2]) << 8)  |
               (static_cast<uint32_t>(in[3]));
    }
    
    // 从高位在前的字节流中读取int32
    static int32_t big_endian_bytes_to_int32(const uint8_t* in) {
        return (static_cast<int32_t>(in[0]) << 24) |
               (static_cast<int32_t>(in[1]) << 16) |
               (static_cast<int32_t>(in[2]) << 8)  |
               (static_cast<int32_t>(in[3]));
    }

    // 从高位在前的字节流中读取uint16
    static uint16_t big_endian_bytes_to_uint16(const uint8_t* in) {
        return (static_cast<uint16_t>(in[0]) << 8) | (static_cast<uint16_t>(in[1]));
    }

    // 将int32转为大端序字节
    static void int32_to_big_endian_bytes(int32_t value, uint8_t* out) {
        out[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
        out[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
        out[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
        out[3] = static_cast<uint8_t>(value & 0xFF);
    }
}

namespace hardware_driver {
namespace motor_protocol {

// 总协议解析入口
std::optional<MotorFeedback> parse_feedback(const bus::GenericBusPacket& packet) {
    switch (packet.protocol_type) {
        case bus::BusProtocolType::CAN: 
            return parse_can_feedback(packet);
        case bus::BusProtocolType::CAN_FD:
            return parse_canfd_feedback(packet);
        case bus::BusProtocolType::ETHERCAT:
            return parse_ethercat_feedback(packet);
        default:
            return std::nullopt;
    }
    return std::nullopt;
}

// TODO: CAN协议解析
std::optional<MotorFeedback> parse_can_feedback(const bus::GenericBusPacket& /*packet*/) {
    return std::nullopt;
}

/**
 * @brief 解析CAN FD协议的反馈数据
 * @param packet 通用总线数据包结构体
 */
std::optional<MotorFeedback> parse_canfd_feedback(const bus::GenericBusPacket& packet) {
    uint32_t id = packet.id;
    uint32_t base = id & 0xFFFFFF00;
    uint8_t motor_id = id & 0xFF;
    const uint8_t* p_data = packet.data.data();
    if (base == 0x100) { // 电机状态反馈 TODO: 修改为0x300
        MotorStatusFeedback feedback;
        feedback.interface = packet.interface;
        feedback.motor_id = motor_id;
        feedback.status.enable_flag = p_data[1];
        feedback.status.motor_mode = p_data[2];
        feedback.status.position = big_endian_bytes_to_float(p_data + 3);
        feedback.status.velocity = big_endian_bytes_to_float(p_data + 7);
        feedback.status.effort = big_endian_bytes_to_float(p_data + 11);
        feedback.status.error_code = big_endian_bytes_to_uint32(p_data + 15);
        feedback.status.voltage = big_endian_bytes_to_uint16(p_data + 19);
        feedback.status.temperature = big_endian_bytes_to_uint16(p_data + 21);
        feedback.status.limit_flag = p_data[23];
        return feedback;
    } else if (base == 0x500) { // 函数操作反馈
        FuncResultFeedback feedback;
        feedback.interface = packet.interface;
        feedback.motor_id = motor_id;
        feedback.op_code = p_data[1];
        feedback.success = (p_data[2] == 0x01);
        return feedback;
    } else if (base == 0x700) { // 参数读写反馈
        ParamResultFeedback feedback;
        feedback.interface = packet.interface;
        feedback.motor_id = motor_id;
        feedback.rw_method = p_data[1];
        feedback.addr = big_endian_bytes_to_uint16(p_data + 2);
        feedback.data_type = p_data[4];
        if (feedback.data_type == 0x01) {    // int
            feedback.data = big_endian_bytes_to_int32(p_data + 5);
        } else if (feedback.data_type == 0x02) { // float
            feedback.data = big_endian_bytes_to_float(p_data + 5);
        }
        return feedback;
    }
    return std::nullopt;
}

// TODO: Ethercat协议解析
std::optional<MotorFeedback> parse_ethercat_feedback(const bus::GenericBusPacket& /*packet*/) {
    return std::nullopt;
}

bool pack_disable_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len)
{
    data[0] = 0x02;
    data[1] = 0x00;
    data[2] = 0x04;     // delete this field in the future
    len = 3;
    return true;
}

bool pack_enable_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    uint8_t mode)
{
    data[0] = 0x02;
    data[1] = 0x01;
    data[2] = mode;   // maybe delete this field in the future
    len = 3;
    return true;
}

bool pack_mit_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    float position, float velocity, float effort) 
{
    data[0] = 0x0E;
    data[1] = 0x01; // enable
    data[2] = static_cast<uint8_t>(MotorControlMode::MIT_MODE);
    float_to_big_endian_bytes(position, data.data() + 3);
    float_to_big_endian_bytes(velocity, data.data() + 7);
    float_to_big_endian_bytes(effort, data.data() + 11);
    len = 15;
    return true;
}

bool pack_position_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    float position)
{
    data[0] = 0x0E;
    data[1] = 0x01; // enable
    data[2] = static_cast<uint8_t>(MotorControlMode::POSITION_ABS_MODE);
    float_to_big_endian_bytes(position, data.data() + 3);
    std::memset(data.data() + 7, 0, sizeof(float));
    std::memset(data.data() + 11, 0, sizeof(float));
    len = 15;
    return true;
}

bool pack_velocity_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    float velocity)
{
    data[0] = 0x0E;
    data[1] = 0x01; // enable
    data[2] = static_cast<uint8_t>(MotorControlMode::SPEED_MODE);
    std::memset(data.data() + 3, 0, sizeof(float));
    float_to_big_endian_bytes(velocity, data.data() + 7);
    std::memset(data.data() + 11, 0, sizeof(float));
    len = 15;
    return true;
}

bool pack_effort_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    float effort)
{
    data[0] = 0x0E;
    data[1] = 0x01; // enable
    data[2] = static_cast<uint8_t>(MotorControlMode::EFFORT_MODE);
    std::memset(data.data() + 3, 0, sizeof(float));
    std::memset(data.data() + 7, 0, sizeof(float));
    float_to_big_endian_bytes(effort, data.data() + 11);
    len = 15;
    return true;
}

bool pack_param_read(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    uint16_t param_addr) 
{
    data[0] = 0x03;
    data[1] = 0x01;
    data[2] = static_cast<uint8_t>(param_addr >> 8);
    data[3] = static_cast<uint8_t>(param_addr & 0xFF);
    len = 4;
    return true;
}

bool pack_param_write(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    uint16_t param_addr, int32_t param_value)
{
    data[0] = 0x08;
    data[1] = 0x02;
    data[2] = static_cast<uint8_t>(param_addr >> 8);
    data[3] = static_cast<uint8_t>(param_addr & 0xFF);
    data[4] = 0x01;
    int32_to_big_endian_bytes(param_value, data.data() + 5);
    len = 9;
    return true;
}

bool pack_param_write(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    uint16_t param_addr, float param_value)
{
    data[0] = 0x08;
    data[1] = 0x02;
    data[2] = static_cast<uint8_t>(param_addr >> 8);
    data[3] = static_cast<uint8_t>(param_addr & 0xFF);
    data[4] = 0x02;
    float_to_big_endian_bytes(param_value, data.data() + 5);
    len = 9;
    return true;
}

bool pack_function_operation(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
    uint8_t op_code)
{
    data[0] = 0x01;
    data[1] = op_code;
    len = 2;
    return true;
}

bool pack_motor_feedback_request(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len) 
{
    data[0] = 0x00;
    len = 1;
    return true;
}

bool pack_motor_feedback_request_all(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& /*data*/, size_t& len)
{
    len = 0;
    return true;
}

}      // namespace motor_protocol
}      // namespace hardware_driver