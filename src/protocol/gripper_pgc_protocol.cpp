/****************************************************************************************
 * @file        gripper_pgc_protocol.cpp
 * @brief       夹爪协议实现文件，负责解析从总线收到的反馈数据和构造发送到总线的指令数据
 * 
 * 1. 实现了针对 CAN FD总线反馈的协议数据解析函数
 * 2. 打包各个模式控制夹爪、参数读写、函数操作、请求反馈等指令的函数。
 * @attention   parse_canfd_feedback() 函数需要根据具体的夹爪型号和协议进行实现
 * @author      Kaiyue Ding
 * @version     0.0.0.1
 * @date        2026-03-09
 * 
 * @copyright   Copyright (c) 2025 Raysense Technology. All rights reserved.
 * 
 * @history     2026-03-09 Kaiyue Ding 创建文件，实现 CAN FD 协议解析
 ***************************************************************************************/

#include "gripper_pgc_protocol.hpp"

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
namespace gripper_pgc_protocol {

std::optional<GripperFeedback> parse_canfd_feedback(const bus::GenericBusPacket& packet) {
    uint32_t id = packet.id;
    const uint8_t* p_data = packet.data.data();
    if (id == 0x6F) {
        GripperFeedback feedback;
        feedback.interface = packet.interface;
        feedback.gripper_id = id;
        feedback.status.position = 0;   // no feedback on position in PGC protocol
        feedback.status.velocity = 0;   // no feedback on velocity in PGC protocol
        feedback.status.force = 0;  // no feedback on force in PGC protocol
        feedback.status.status = 0;     // no specific status code defined in PGC protocol
        feedback.status.action_status = p_data[1];
        switch (feedback.status.action_status) {
            case 0x00: {
                feedback.status.is_moving = true;
                break;
            }
            case 0x01: {
                feedback.status.is_moving = false;
                feedback.status.has_object = false;
                break;
            }
            case 0x02: {
                feedback.status.is_moving = false;
                feedback.status.has_object = true;
                break;
            }
            case 0x03: {
                feedback.status.has_object = false;
                break;
            }
            default: break;
        }
        return std::make_optional(feedback);
    }
    return std::nullopt;
}

bool pack_pgc_control_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
                             uint8_t velocity, uint8_t effort, uint8_t position) {
    uint16_t Pos = position * 10;
    assert(0x01 <= velocity && velocity <= 0x64);
    assert(0x14 <= effort && effort <= 0x64);
    assert(0x00 <= Pos && Pos <= 0x03E8);

    data[0] = 0x02;
    data[1] = 0x01;
    data[2] = velocity >> 8;
    data[3] = velocity & 0xFF;
    data[4] = effort >> 8;
    data[5] = effort & 0xFF;
    data[6] = Pos >> 8;
    data[7] = Pos & 0xFF;
    len = 8;
    return true;
}

bool pack_pgc_status_req_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len) {
    data[0] = 0x02;
    data[1] = 0x02;
    len = 2;
    return true;
}

bool pack_pgc_status_initialize(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len) {
    data[0] = 0x02;
    data[1] = 0x03;
    len = 2;
    return true;
}

}       // namespace gripper_pgc_protocol
}       // namespace hardware_driver