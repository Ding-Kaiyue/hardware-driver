#include <gtest/gtest.h>
#include "protocol/motor_protocol.hpp"
#include <cstring>
#include <cmath>

using namespace hardware_driver;
using namespace hardware_driver::motor_protocol;
using hardware_driver::bus::GenericBusPacket;

// 浮点数比较辅助
constexpr float EPSILON = 1e-5f;

// 大端序转换函数（与协议实现保持一致）
namespace {
    // 将float转为大端序字节并写入数组
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

    // 从大端序字节流中读取float
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

    // 从大端序字节流中读取uint32
    static uint32_t __attribute__((unused)) big_endian_bytes_to_uint32(const uint8_t* in) {
        return (static_cast<uint32_t>(in[0]) << 24) |
               (static_cast<uint32_t>(in[1]) << 16) |
               (static_cast<uint32_t>(in[2]) << 8)  |
               (static_cast<uint32_t>(in[3]));
    }
    
    // 从大端序字节流中读取int32
    static int32_t big_endian_bytes_to_int32(const uint8_t* in) {
        return (static_cast<int32_t>(in[0]) << 24) |
               (static_cast<int32_t>(in[1]) << 16) |
               (static_cast<int32_t>(in[2]) << 8)  |
               (static_cast<int32_t>(in[3]));
    }

    // 从大端序字节流中读取uint16
    static uint16_t __attribute__((unused)) big_endian_bytes_to_uint16(const uint8_t* in) {
        return (static_cast<uint16_t>(in[0]) << 8) | (static_cast<uint16_t>(in[1]));
    }

    // 将uint32转为大端序字节
    static void uint32_to_big_endian_bytes(uint32_t value, uint8_t* out) {
        out[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
        out[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
        out[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
        out[3] = static_cast<uint8_t>(value & 0xFF);
    }

    // 将int32转为大端序字节
    static void int32_to_big_endian_bytes(int32_t value, uint8_t* out) {
        out[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
        out[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
        out[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
        out[3] = static_cast<uint8_t>(value & 0xFF);
    }

    // 将uint16转为大端序字节
    static void uint16_to_big_endian_bytes(uint16_t value, uint8_t* out) {
        out[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
        out[1] = static_cast<uint8_t>(value & 0xFF);
    }
}

TEST(MotorProtocolTest, PackDisableCommand) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    ASSERT_TRUE(pack_disable_command(data, len));
    EXPECT_EQ(len, 3u);
    EXPECT_EQ(data[0], 0x02);
    EXPECT_EQ(data[1], 0x00);
    EXPECT_EQ(data[2], 0x04);
}

TEST(MotorProtocolTest, PackMITCommand) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    float pos = 1.23f, vel = 2.34f, eff = 3.45f;
    ASSERT_TRUE(pack_mit_command(data, len, pos, vel, eff));
    EXPECT_EQ(len, 15u);
    EXPECT_EQ(data[0], 0x0E);
    EXPECT_EQ(data[1], 0x01);
    EXPECT_EQ(data[2], static_cast<uint8_t>(MotorControlMode::MIT_MODE));
    float pos2, vel2, eff2;
    pos2 = big_endian_bytes_to_float(data.data() + 3);
    vel2 = big_endian_bytes_to_float(data.data() + 7);
    eff2 = big_endian_bytes_to_float(data.data() + 11);
    EXPECT_NEAR(pos2, pos, EPSILON);
    EXPECT_NEAR(vel2, vel, EPSILON);
    EXPECT_NEAR(eff2, eff, EPSILON);
}

TEST(MotorProtocolTest, PackPositionCommand) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    float pos = 5.67f;
    ASSERT_TRUE(pack_position_command(data, len, pos));
    EXPECT_EQ(len, 15u);
    EXPECT_EQ(data[2], static_cast<uint8_t>(MotorControlMode::POSITION_ABS_MODE));
    float pos2;
    pos2 = big_endian_bytes_to_float(data.data() + 3);
    EXPECT_NEAR(pos2, pos, EPSILON);
}

TEST(MotorProtocolTest, PackVelocityCommand) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    float vel = 7.89f;
    ASSERT_TRUE(pack_velocity_command(data, len, vel));
    EXPECT_EQ(len, 15u);
    EXPECT_EQ(data[2], static_cast<uint8_t>(MotorControlMode::SPEED_MODE));
    float vel2;
    vel2 = big_endian_bytes_to_float(data.data() + 7);
    EXPECT_NEAR(vel2, vel, EPSILON);
}

TEST(MotorProtocolTest, PackEffortCommand) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    float eff = 9.87f;
    ASSERT_TRUE(pack_effort_command(data, len, eff));
    EXPECT_EQ(len, 15u);
    EXPECT_EQ(data[2], static_cast<uint8_t>(MotorControlMode::EFFORT_MODE));
    float eff2;
    eff2 = big_endian_bytes_to_float(data.data() + 11);
    EXPECT_NEAR(eff2, eff, EPSILON);
}

TEST(MotorProtocolTest, PackParamRead) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    uint16_t addr = 0x1234;
    ASSERT_TRUE(pack_param_read(data, len, addr));
    EXPECT_EQ(len, 4u);
    EXPECT_EQ(data[0], 0x03);
    EXPECT_EQ(data[1], 0x01);
    EXPECT_EQ(data[2], static_cast<uint8_t>(addr >> 8));
    EXPECT_EQ(data[3], static_cast<uint8_t>(addr & 0xFF));
}

TEST(MotorProtocolTest, PackParamWriteInt) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    uint16_t addr = 0x2345;
    int32_t value = -123456;
    ASSERT_TRUE(pack_param_write(data, len, addr, value));
    EXPECT_EQ(len, 9u);
    EXPECT_EQ(data[0], 0x08);
    EXPECT_EQ(data[1], 0x02);
    EXPECT_EQ(data[2], static_cast<uint8_t>(addr >> 8));
    EXPECT_EQ(data[3], static_cast<uint8_t>(addr & 0xFF));
    EXPECT_EQ(data[4], 0x01);
    int32_t value2;
    value2 = big_endian_bytes_to_int32(data.data() + 5);
    EXPECT_EQ(value2, value);
}

TEST(MotorProtocolTest, PackParamWriteFloat) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    uint16_t addr = 0x3456;
    float value = 12.34f;
    pack_param_write(data, len, addr, value);
    EXPECT_EQ(data[0], 0x08);
    EXPECT_EQ(data[1], 0x02);
    EXPECT_EQ(data[2], static_cast<uint8_t>(addr >> 8));
    EXPECT_EQ(data[3], static_cast<uint8_t>(addr & 0xFF));
    EXPECT_EQ(data[4], 0x02);
    float value2;
    value2 = big_endian_bytes_to_float(data.data() + 5);
    EXPECT_NEAR(value2, value, EPSILON);
}

TEST(MotorProtocolTest, PackFunctionOperation) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    uint8_t op = 0x11;
    ASSERT_TRUE(pack_function_operation(data, len, op));
    EXPECT_EQ(len, 2u);
    EXPECT_EQ(data[0], 0x01);
    EXPECT_EQ(data[1], op);
}

TEST(MotorProtocolTest, PackMotorFeedbackRequest) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    ASSERT_TRUE(pack_motor_feedback_request(data, len));
    EXPECT_EQ(len, 1u);
    EXPECT_EQ(data[0], 0x00);
}

TEST(MotorProtocolTest, PackMotorFeedbackRequestAll) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 123;
    ASSERT_TRUE(pack_motor_feedback_request_all(data, len));
    EXPECT_EQ(len, 0u);
}

// ================== 反馈解析测试 ===================

TEST(MotorProtocolTest, ParseCanfdMotorStatusFeedback) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN_FD;
    pkt.interface = "can0";
    pkt.id = 0x101; // 0x100 + motor_id
    pkt.len = 24;
    // 构造数据 - 使用大端序
    pkt.data[1] = 1; // enable_flag
    pkt.data[2] = 3; // motor_mode
    float pos = 1.23f, vel = 2.34f, eff = 3.45f;
    float_to_big_endian_bytes(pos, pkt.data.data() + 3);
    float_to_big_endian_bytes(vel, pkt.data.data() + 7);
    float_to_big_endian_bytes(eff, pkt.data.data() + 11);
    uint32_t err = 0x12345678;
    uint32_to_big_endian_bytes(err, pkt.data.data() + 15);
    uint16_t volt = 330;
    uint16_to_big_endian_bytes(volt, pkt.data.data() + 19);
    uint16_t temp = 55;
    uint16_to_big_endian_bytes(temp, pkt.data.data() + 21);
    pkt.data[23] = 1; // limit_flag
    auto ret = parse_canfd_feedback(pkt);
    ASSERT_TRUE(ret.has_value());
    auto* p = std::get_if<MotorStatusFeedback>(&ret.value());
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->interface, "can0");
    EXPECT_EQ(p->motor_id, 1u);
    EXPECT_EQ(p->status.enable_flag, 1u);
    EXPECT_EQ(p->status.motor_mode, 3u);
    EXPECT_NEAR(p->status.position, pos, EPSILON);
    EXPECT_NEAR(p->status.velocity, vel, EPSILON);
    EXPECT_NEAR(p->status.effort, eff, EPSILON);
    EXPECT_EQ(p->status.error_code, err);
    EXPECT_EQ(p->status.voltage, volt);
    EXPECT_EQ(p->status.temperature, temp);
    EXPECT_EQ(p->status.limit_flag, 1u);
}

TEST(MotorProtocolTest, ParseCanfdFuncResultFeedback) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN_FD;
    pkt.interface = "can0";
    pkt.id = 0x501;
    pkt.len = 3;
    pkt.data[1] = 0x11; // op_code
    pkt.data[2] = 0x01; // success
    auto ret = parse_canfd_feedback(pkt);
    ASSERT_TRUE(ret.has_value());
    auto* p = std::get_if<FuncResultFeedback>(&ret.value());
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->interface, "can0");
    EXPECT_EQ(p->motor_id, 1u);
    EXPECT_EQ(p->op_code, 0x11);
    EXPECT_TRUE(p->success);
}

TEST(MotorProtocolTest, ParseCanfdParamResultFeedbackInt) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN_FD;
    pkt.interface = "can0";
    pkt.id = 0x701;
    pkt.len = 9;
    pkt.data[1] = 0x01; // rw_method
    uint16_t addr = 0x1234;
    uint16_to_big_endian_bytes(addr, pkt.data.data() + 2);
    pkt.data[4] = 0x01; // data_type int
    int32_t val = -123456;
    int32_to_big_endian_bytes(val, pkt.data.data() + 5);
    auto ret = parse_canfd_feedback(pkt);
    ASSERT_TRUE(ret.has_value());
    auto* p = std::get_if<ParamResultFeedback>(&ret.value());
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->interface, "can0");
    EXPECT_EQ(p->motor_id, 1u);
    EXPECT_EQ(p->rw_method, 0x01);
    EXPECT_EQ(p->addr, 0x1234);
    EXPECT_EQ(p->data_type, 0x01);
    ASSERT_TRUE(p->data.has_value());
    EXPECT_EQ(std::any_cast<int32_t>(p->data), val);
}

TEST(MotorProtocolTest, ParseCanfdParamResultFeedbackFloat) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN_FD;
    pkt.interface = "can0";
    pkt.id = 0x701;
    pkt.len = 9;
    pkt.data[1] = 0x01; // rw_method
    uint16_t addr = 0x1234;
    uint16_to_big_endian_bytes(addr, pkt.data.data() + 2);
    pkt.data[4] = 0x02; // data_type float
    float val = 12.34f;
    float_to_big_endian_bytes(val, pkt.data.data() + 5);
    auto ret = parse_canfd_feedback(pkt);
    ASSERT_TRUE(ret.has_value());
    auto* p = std::get_if<ParamResultFeedback>(&ret.value());
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->interface, "can0");
    EXPECT_EQ(p->motor_id, 1u);
    EXPECT_EQ(p->rw_method, 0x01);
    EXPECT_EQ(p->addr, 0x1234);
    EXPECT_EQ(p->data_type, 0x02);
    ASSERT_TRUE(p->data.has_value());
    EXPECT_NEAR(std::any_cast<float>(p->data), val, EPSILON);
}

// ========== 异常/边界用例 ==========

TEST(MotorProtocolTest, ParseCanfdFeedbackUnknownId) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN_FD;
    pkt.interface = "can0";
    pkt.id = 0x999;
    pkt.len = 8;
    auto ret = parse_canfd_feedback(pkt);
    EXPECT_FALSE(ret.has_value());
}

TEST(MotorProtocolTest, ParseFeedbackWrongProtocol) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::UNKNOWN;
    pkt.interface = "can0";
    pkt.id = 0x301;
    pkt.len = 24;
    auto ret = parse_feedback(pkt);
    EXPECT_FALSE(ret.has_value());
} 