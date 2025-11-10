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

// ==================== 指令打包测试 ====================

TEST(MotorProtocolTest, PackDisableCommand) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    ASSERT_TRUE(pack_disable_command(data, len, 0x04));
    EXPECT_EQ(len, 3u);
    EXPECT_EQ(data[0], 0x02);
    EXPECT_EQ(data[1], 0x00);
    EXPECT_EQ(data[2], 0x04);
}

TEST(MotorProtocolTest, PackDisableCommandVariousModes) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;

    // 测试多个模式
    for (uint8_t mode = 0x00; mode <= 0x06; ++mode) {
        SCOPED_TRACE("Mode: " + std::to_string(mode));
        ASSERT_TRUE(pack_disable_command(data, len, mode));
        EXPECT_EQ(len, 3u);
        EXPECT_EQ(data[0], 0x02);
        EXPECT_EQ(data[1], 0x00);
        EXPECT_EQ(data[2], mode);
    }
}

TEST(MotorProtocolTest, PackEnableCommand) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    ASSERT_TRUE(pack_enable_command(data, len, 0x03));
    EXPECT_EQ(len, 3u);
    EXPECT_EQ(data[0], 0x02);
    EXPECT_EQ(data[1], 0x01);
    EXPECT_EQ(data[2], 0x03);
}

TEST(MotorProtocolTest, PackEnableCommandVariousModes) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;

    // 测试多个电机模式
    for (uint8_t mode = 0x02; mode <= 0x06; ++mode) {
        SCOPED_TRACE("Mode: " + std::to_string(mode));
        ASSERT_TRUE(pack_enable_command(data, len, mode));
        EXPECT_EQ(len, 3u);
        EXPECT_EQ(data[0], 0x02);
        EXPECT_EQ(data[1], 0x01);
        EXPECT_EQ(data[2], mode);
    }
}

TEST(MotorProtocolTest, PackDisableAllCommand) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    std::array<uint8_t, 6> disable_flags = {1, 1, 1, 1, 1, 1};
    std::array<uint8_t, 6> modes = {0x02, 0x03, 0x04, 0x05, 0x06, 0x02};

    ASSERT_TRUE(pack_disable_all_command(data, len, disable_flags, modes));
    EXPECT_EQ(len, 8u);  // 2 + 6
    EXPECT_EQ(data[0], 7u);  // 6 + 1
    EXPECT_EQ(data[1], 0x02);

    // 验证每个电机的打包数据
    for (size_t i = 0; i < 6; ++i) {
        uint8_t expected = (disable_flags[i] << 4) | modes[i];
        EXPECT_EQ(data[2 + i], expected);
    }
}

TEST(MotorProtocolTest, PackEnableAllCommand) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    std::array<uint8_t, 6> enable_flags = {1, 1, 1, 0, 0, 1};
    std::array<uint8_t, 6> modes = {0x03, 0x03, 0x04, 0x02, 0x05, 0x03};

    ASSERT_TRUE(pack_enable_all_command(data, len, enable_flags, modes));
    EXPECT_EQ(len, 8u);  // 2 + 6
    EXPECT_EQ(data[0], 7u);  // 6 + 1
    EXPECT_EQ(data[1], 0x02);

    // 验证每个电机的打包数据
    for (size_t i = 0; i < 6; ++i) {
        uint8_t expected = (enable_flags[i] << 4) | modes[i];
        EXPECT_EQ(data[2 + i], expected);
    }
}

TEST(MotorProtocolTest, PackControlCommand) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    float pos = 1.23f, vel = 2.34f, eff = 3.45f;
    float kp = 0.05f, kd = 0.01f;

    ASSERT_TRUE(pack_control_command(data, len, pos, vel, eff, kp, kd));
    EXPECT_EQ(len, 15u);
    EXPECT_EQ(data[0], 0x0E);

    float pos2 = big_endian_bytes_to_float(data.data() + 1);
    float vel2 = big_endian_bytes_to_float(data.data() + 5);
    float eff2 = big_endian_bytes_to_float(data.data() + 9);

    EXPECT_NEAR(pos2, pos, EPSILON);
    EXPECT_NEAR(vel2, vel, EPSILON);
    EXPECT_NEAR(eff2, eff, EPSILON);
    EXPECT_EQ(data[13], static_cast<uint8_t>(kp * 1000));
    EXPECT_EQ(data[14], static_cast<uint8_t>(kd * 1000));
}

TEST(MotorProtocolTest, PackControlCommandNegativeValues) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    float pos = -5.67f, vel = -3.21f, eff = -2.34f;
    float kp = 0.1f, kd = 0.05f;

    ASSERT_TRUE(pack_control_command(data, len, pos, vel, eff, kp, kd));
    EXPECT_EQ(len, 15u);
    EXPECT_EQ(data[0], 0x0E);

    float pos2 = big_endian_bytes_to_float(data.data() + 1);
    float vel2 = big_endian_bytes_to_float(data.data() + 5);
    float eff2 = big_endian_bytes_to_float(data.data() + 9);

    EXPECT_NEAR(pos2, pos, EPSILON);
    EXPECT_NEAR(vel2, vel, EPSILON);
    EXPECT_NEAR(eff2, eff, EPSILON);
}

TEST(MotorProtocolTest, PackControlAllCommand) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    std::array<float, 6> positions = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    std::array<float, 6> velocities = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
    std::array<float, 6> efforts = {0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f};
    std::array<float, 6> kps = {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f};
    std::array<float, 6> kds = {0.001f, 0.002f, 0.003f, 0.004f, 0.005f, 0.006f};

    ASSERT_TRUE(pack_control_all_command(data, len, positions, velocities, efforts, kps, kds));
    EXPECT_EQ(len, 50u);  // 1 + 1 + 6*8 = 50
    EXPECT_EQ(data[0], 49u);  // 6*8 + 1
    EXPECT_EQ(data[1], 0x03);

    // 验证每个电机的打包数据
    for (size_t i = 0; i < 6; ++i) {
        size_t offset = 2 + i * 8;

        // 验证位置（int16_t，单位0.01）
        int16_t pos_int = static_cast<int16_t>(positions[i] * 100);
        int16_t pos_packed = (static_cast<int16_t>(data[offset]) << 8) | data[offset + 1];
        EXPECT_EQ(pos_packed, pos_int);

        // 验证速度（int16_t，单位0.01）
        int16_t vel_int = static_cast<int16_t>(velocities[i] * 100);
        int16_t vel_packed = (static_cast<int16_t>(data[offset + 2]) << 8) | data[offset + 3];
        EXPECT_EQ(vel_packed, vel_int);

        // 验证力矩（int16_t，单位0.01）
        int16_t eff_int = static_cast<int16_t>(efforts[i] * 100);
        int16_t eff_packed = (static_cast<int16_t>(data[offset + 4]) << 8) | data[offset + 5];
        EXPECT_EQ(eff_packed, eff_int);

        // 验证 kp 和 kd
        EXPECT_EQ(data[offset + 6], static_cast<uint8_t>(kps[i] * 1000));
        EXPECT_EQ(data[offset + 7], static_cast<uint8_t>(kds[i] * 1000));
    }
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

TEST(MotorProtocolTest, PackParamReadVariousAddresses) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;

    std::array<uint16_t, 4> addresses = {0x0000, 0x0001, 0x000A, 0x000F};
    for (uint16_t addr : addresses) {
        SCOPED_TRACE("Address: 0x" + std::to_string(addr));
        ASSERT_TRUE(pack_param_read(data, len, addr));
        EXPECT_EQ(len, 4u);
        EXPECT_EQ(data[0], 0x03);
        EXPECT_EQ(data[1], 0x01);
        EXPECT_EQ(data[2], static_cast<uint8_t>(addr >> 8));
        EXPECT_EQ(data[3], static_cast<uint8_t>(addr & 0xFF));
    }
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
    EXPECT_EQ(data[4], 0x01);  // int32 data type

    int32_t value2 = big_endian_bytes_to_int32(data.data() + 5);
    EXPECT_EQ(value2, value);
}

TEST(MotorProtocolTest, PackParamWriteIntVariousValues) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    uint16_t addr = 0x0008;

    std::array<int32_t, 5> values = {0, 100000, -100000, INT32_MAX, INT32_MIN};
    for (int32_t val : values) {
        SCOPED_TRACE("Value: " + std::to_string(val));
        ASSERT_TRUE(pack_param_write(data, len, addr, val));
        EXPECT_EQ(len, 9u);
        EXPECT_EQ(data[0], 0x08);
        EXPECT_EQ(data[4], 0x01);

        int32_t value2 = big_endian_bytes_to_int32(data.data() + 5);
        EXPECT_EQ(value2, val);
    }
}

TEST(MotorProtocolTest, PackParamWriteFloat) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    uint16_t addr = 0x3456;
    float value = 12.34f;

    ASSERT_TRUE(pack_param_write(data, len, addr, value));
    EXPECT_EQ(len, 9u);
    EXPECT_EQ(data[0], 0x08);
    EXPECT_EQ(data[1], 0x02);
    EXPECT_EQ(data[2], static_cast<uint8_t>(addr >> 8));
    EXPECT_EQ(data[3], static_cast<uint8_t>(addr & 0xFF));
    EXPECT_EQ(data[4], 0x02);  // float data type

    float value2 = big_endian_bytes_to_float(data.data() + 5);
    EXPECT_NEAR(value2, value, EPSILON);
}

TEST(MotorProtocolTest, PackParamWriteFloatVariousValues) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;
    uint16_t addr = 0x0009;

    std::array<float, 6> values = {0.0f, -3.14f, 123.456f, -999.999f, 0.001f, 1000.0f};
    for (float val : values) {
        SCOPED_TRACE("Value: " + std::to_string(val));
        ASSERT_TRUE(pack_param_write(data, len, addr, val));
        EXPECT_EQ(len, 9u);
        EXPECT_EQ(data[4], 0x02);

        float value2 = big_endian_bytes_to_float(data.data() + 5);
        EXPECT_NEAR(value2, val, EPSILON);
    }
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

TEST(MotorProtocolTest, PackFunctionOperationVariousOps) {
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE> data = {0};
    size_t len = 0;

    // 测试各个函数操作码
    std::array<uint8_t, 9> ops = {
        0x01,  // PARAM_RESET
        0x02,  // PARAM_SAVE_TO_FLASH
        0x03,  // CLEAR_ERROR_CODE
        0x04,  // MOTOR_ZERO_POS_SET
        0x11,  // MOTOR_FIND_ZERO_POS
        0x12,  // MOTOR_IAP_UPDATE
        0x13,  // MOTOR_HALL_CALIBRATION
        0x14,  // MOTOR_CURRENT_CALIBRATION
        0x15   // MOTOR_ENCODER_CALIBRATION
    };

    for (uint8_t op : ops) {
        SCOPED_TRACE("Op code: 0x" + std::to_string(op));
        ASSERT_TRUE(pack_function_operation(data, len, op));
        EXPECT_EQ(len, 2u);
        EXPECT_EQ(data[0], 0x01);
        EXPECT_EQ(data[1], op);
    }
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
    size_t len = 0;

    ASSERT_TRUE(pack_motor_feedback_request_all(data, len));
    EXPECT_EQ(len, 3u);
    EXPECT_EQ(data[0], 0x02);
    EXPECT_EQ(data[1], 0x00);
    EXPECT_EQ(data[2], 0x00);
}

// ================== 反馈解析测试 ===================

TEST(MotorProtocolTest, ParseCanfdMotorStatusFeedback) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN_FD;
    pkt.interface = "can0";
    pkt.id = 0x301;  // 0x300 + motor_id=1
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

TEST(MotorProtocolTest, ParseCanfdMotorStatusFeedbackVariousMotorIds) {
    std::array<uint32_t, 5> motor_ids = {0, 1, 5, 127, 255};

    for (uint32_t motor_id : motor_ids) {
        SCOPED_TRACE("Motor ID: " + std::to_string(motor_id));
        GenericBusPacket pkt;
        pkt.protocol_type = bus::BusProtocolType::CAN_FD;
        pkt.interface = "can0";
        pkt.id = 0x300 | motor_id;
        pkt.len = 24;

        pkt.data[1] = 1;
        pkt.data[2] = 3;
        float pos = 1.23f;
        float_to_big_endian_bytes(pos, pkt.data.data() + 3);
        float_to_big_endian_bytes(2.34f, pkt.data.data() + 7);
        float_to_big_endian_bytes(3.45f, pkt.data.data() + 11);
        uint32_to_big_endian_bytes(0x12345678, pkt.data.data() + 15);
        uint16_to_big_endian_bytes(330, pkt.data.data() + 19);
        uint16_to_big_endian_bytes(55, pkt.data.data() + 21);
        pkt.data[23] = 1;

        auto ret = parse_canfd_feedback(pkt);
        ASSERT_TRUE(ret.has_value());
        auto* p = std::get_if<MotorStatusFeedback>(&ret.value());
        ASSERT_NE(p, nullptr);
        EXPECT_EQ(p->motor_id, motor_id);
    }
}

TEST(MotorProtocolTest, ParseCanfdFuncResultFeedback) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN_FD;
    pkt.interface = "can0";
    pkt.id = 0x501;  // 0x500 + motor_id=1
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

TEST(MotorProtocolTest, ParseCanfdFuncResultFeedbackFailure) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN_FD;
    pkt.interface = "can0";
    pkt.id = 0x502;  // 0x500 + motor_id=2
    pkt.len = 3;
    pkt.data[1] = 0x04; // op_code
    pkt.data[2] = 0x00; // failure

    auto ret = parse_canfd_feedback(pkt);
    ASSERT_TRUE(ret.has_value());
    auto* p = std::get_if<FuncResultFeedback>(&ret.value());
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->motor_id, 2u);
    EXPECT_EQ(p->op_code, 0x04);
    EXPECT_FALSE(p->success);
}

TEST(MotorProtocolTest, ParseCanfdFuncResultFeedbackVariousOps) {
    std::array<uint8_t, 9> ops = {
        0x01,  // PARAM_RESET
        0x02,  // PARAM_SAVE_TO_FLASH
        0x03,  // CLEAR_ERROR_CODE
        0x04,  // MOTOR_ZERO_POS_SET
        0x11,  // MOTOR_FIND_ZERO_POS
        0x12,  // MOTOR_IAP_UPDATE
        0x13,  // MOTOR_HALL_CALIBRATION
        0x14,  // MOTOR_CURRENT_CALIBRATION
        0x15   // MOTOR_ENCODER_CALIBRATION
    };

    for (uint8_t op : ops) {
        SCOPED_TRACE("Op code: 0x" + std::to_string(op));
        GenericBusPacket pkt;
        pkt.protocol_type = bus::BusProtocolType::CAN_FD;
        pkt.interface = "can0";
        pkt.id = 0x503;  // motor_id=3
        pkt.len = 3;
        pkt.data[1] = op;
        pkt.data[2] = 0x01;

        auto ret = parse_canfd_feedback(pkt);
        ASSERT_TRUE(ret.has_value());
        auto* p = std::get_if<FuncResultFeedback>(&ret.value());
        ASSERT_NE(p, nullptr);
        EXPECT_EQ(p->op_code, op);
    }
}

TEST(MotorProtocolTest, ParseCanfdParamResultFeedbackInt) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN_FD;
    pkt.interface = "can0";
    pkt.id = 0x701;  // 0x700 + motor_id=1
    pkt.len = 9;
    pkt.data[1] = 0x01; // rw_method (READ)
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

TEST(MotorProtocolTest, ParseCanfdParamResultFeedbackIntVariousValues) {
    std::array<int32_t, 5> values = {0, 100000, -100000, INT32_MAX, INT32_MIN};

    for (int32_t val : values) {
        SCOPED_TRACE("Value: " + std::to_string(val));
        GenericBusPacket pkt;
        pkt.protocol_type = bus::BusProtocolType::CAN_FD;
        pkt.interface = "can0";
        pkt.id = 0x704;  // motor_id=4
        pkt.len = 9;
        pkt.data[1] = 0x01;
        uint16_to_big_endian_bytes(0x0008, pkt.data.data() + 2);
        pkt.data[4] = 0x01;
        int32_to_big_endian_bytes(val, pkt.data.data() + 5);

        auto ret = parse_canfd_feedback(pkt);
        ASSERT_TRUE(ret.has_value());
        auto* p = std::get_if<ParamResultFeedback>(&ret.value());
        ASSERT_NE(p, nullptr);
        EXPECT_EQ(p->data_type, 0x01);
        ASSERT_TRUE(p->data.has_value());
        EXPECT_EQ(std::any_cast<int32_t>(p->data), val);
    }
}

TEST(MotorProtocolTest, ParseCanfdParamResultFeedbackFloat) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN_FD;
    pkt.interface = "can0";
    pkt.id = 0x702;  // 0x700 + motor_id=2
    pkt.len = 9;
    pkt.data[1] = 0x01; // rw_method (READ)
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
    EXPECT_EQ(p->motor_id, 2u);
    EXPECT_EQ(p->rw_method, 0x01);
    EXPECT_EQ(p->addr, 0x1234);
    EXPECT_EQ(p->data_type, 0x02);
    ASSERT_TRUE(p->data.has_value());
    EXPECT_NEAR(std::any_cast<float>(p->data), val, EPSILON);
}

TEST(MotorProtocolTest, ParseCanfdParamResultFeedbackFloatVariousValues) {
    std::array<float, 6> values = {0.0f, -3.14f, 123.456f, -999.999f, 0.001f, 1000.0f};

    for (float val : values) {
        SCOPED_TRACE("Value: " + std::to_string(val));
        GenericBusPacket pkt;
        pkt.protocol_type = bus::BusProtocolType::CAN_FD;
        pkt.interface = "can0";
        pkt.id = 0x705;  // motor_id=5
        pkt.len = 9;
        pkt.data[1] = 0x01;
        uint16_to_big_endian_bytes(0x0009, pkt.data.data() + 2);
        pkt.data[4] = 0x02;
        float_to_big_endian_bytes(val, pkt.data.data() + 5);

        auto ret = parse_canfd_feedback(pkt);
        ASSERT_TRUE(ret.has_value());
        auto* p = std::get_if<ParamResultFeedback>(&ret.value());
        ASSERT_NE(p, nullptr);
        EXPECT_EQ(p->data_type, 0x02);
        ASSERT_TRUE(p->data.has_value());
        EXPECT_NEAR(std::any_cast<float>(p->data), val, EPSILON);
    }
}

// ========== 异常/边界用例 ==========

TEST(MotorProtocolTest, ParseCanfdFeedbackUnknownId) {
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN_FD;
    pkt.interface = "can0";
    pkt.id = 0x999;  // 未知的ID基地址
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

TEST(MotorProtocolTest, ParseFeedbackCANProtocol) {
    // CAN协议当前未实现，应该返回null
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::CAN;
    pkt.interface = "can0";
    pkt.id = 0x301;
    pkt.len = 8;
    auto ret = parse_feedback(pkt);
    EXPECT_FALSE(ret.has_value());
}

TEST(MotorProtocolTest, ParseFeedbackETHERCATProtocol) {
    // EtherCAT协议当前未实现，应该返回null
    GenericBusPacket pkt;
    pkt.protocol_type = bus::BusProtocolType::ETHERCAT;
    pkt.interface = "eth0";
    pkt.id = 0x301;
    pkt.len = 8;
    auto ret = parse_feedback(pkt);
    EXPECT_FALSE(ret.has_value());
} 