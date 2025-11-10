#include "protocol/raytron_motor_ctrl.h"
#include <cstring>
#include <algorithm>
#include <iomanip>
#include <iostream>

namespace raytron
{

// ========== 协议编码接口 (新协议) ==========

canfd_frame RaytronProtocolConverter::encodeControl(uint8_t motor_id, raytron::EnableFlag enable,
                                                     raytron::ControlMode mode, float pos, float vel, float cur,
                                                     float kp, float kd)
{
    // 新协议单电机控制: CAN ID = 电机ID (1-6)
    // 格式: [数据长度][使能标志][控制模式][位置float32][速度float32][电流float32][Kp*1000 uint8][Kd*1000 uint8]
    using namespace raytron::SingleMotorControlOffset;

    std::vector<uint8_t> frame_data(raytron::FrameSize::SINGLE_MOTOR_CONTROL_FRAME, 0x00);
    frame_data[DATA_LENGTH] = 0x10;  // 16字节数据

    // 使能标志和控制模式
    frame_data[ENABLE_FLAG] = static_cast<uint8_t>(enable);
    frame_data[CONTROL_MODE] = static_cast<uint8_t>(mode);

    // 编码位置 (float32, 大端)
    DataConverter::encodeFloatToFrame(frame_data, POSITION_START, pos);

    // 编码速度 (float32, 大端)
    DataConverter::encodeFloatToFrame(frame_data, VELOCITY_START, vel);

    // 编码电流 (float32, 大端)
    DataConverter::encodeFloatToFrame(frame_data, CURRENT_START, cur);

    // 编码Kp和Kd (Kp和Kd需要×1000后转为uint8)
    frame_data[KP] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, kp * 1000.0f)));
    frame_data[KD] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, kd * 1000.0f)));
    
    // CAN ID = 电机ID
    return dataToFrame(frame_data, motor_id);
}

canfd_frame RaytronProtocolConverter::encodeRequestFeedback(uint8_t motor_id)
{
    // 单电机反馈请求: CAN ID = 0x0200 + 电机ID, 数据长度1, 数据内容0x00
    std::vector<uint8_t> frame_data(1, 0x00);
    uint32_t can_id = raytron::SINGLE_FEEDBACK_BASE_ID + motor_id;
    return dataToFrame(frame_data, can_id);
}

canfd_frame RaytronProtocolConverter::encodeBroadcastRequestFeedback()
{
    using namespace raytron::BroadcastRequestOffset;
    using namespace raytron::BroadcastFunction;

    std::vector<uint8_t> frame_data(raytron::FrameSize::BROADCAST_REQUEST_FRAME, 0x00);
    frame_data[DATA_LENGTH] = 0x02;
    frame_data[FUNCTION] = REQUEST_FEEDBACK;
    frame_data[OPERATION] = 0x00;

    return dataToFrame(frame_data, raytron::BROADCAST_CAN_ID);
}

canfd_frame RaytronProtocolConverter::encodeBroadcastSetID(uint8_t motor_id)
{
    using namespace raytron::BroadcastFunction;

    std::vector<uint8_t> frame_data(raytron::FrameSize::BROADCAST_SET_ID_FRAME, 0x00);
    frame_data[0] = 0x02;  // 数据长度
    frame_data[1] = SET_ID;  // 功能码
    frame_data[2] = motor_id;  // ID值 (1-254)

    return dataToFrame(frame_data, raytron::BROADCAST_CAN_ID);
}

canfd_frame RaytronProtocolConverter::encodeBroadcastEnable(const MotorEnableFlag flags[MAX_MOTOR_COUNT], uint8_t count)
{
    using namespace raytron::BroadcastEnableOffset;
    using namespace raytron::BroadcastFunction;

    count = std::min(count, raytron::MAX_MOTOR_COUNT);

    std::vector<uint8_t> frame_data(raytron::FrameSize::BROADCAST_ENABLE_FRAME, 0x00);
    frame_data[DATA_LENGTH] = 0x07;
    frame_data[FUNCTION] = SET_ENABLE;

    // 编码标志位: (使能 << 4) | 控制模式
    for (uint8_t i = 0; i < count; ++i) {
        frame_data[FLAG_START + i] = flags[i].encode();
    }

    return dataToFrame(frame_data, raytron::BROADCAST_CAN_ID);
}

canfd_frame RaytronProtocolConverter::encodeBroadcastControlAll(const MotorControlData motor_data[MAX_MOTOR_COUNT], uint8_t count)
{
    using namespace raytron::BroadcastControlOffset;
    using namespace raytron::BroadcastFunction;
    using namespace raytron::ScaleFactor;

    count = std::min(count, raytron::MAX_MOTOR_COUNT);

    std::vector<uint8_t> frame_data(raytron::FrameSize::BROADCAST_CONTROL_FRAME, 0x00);
    frame_data[DATA_LENGTH] = 0x31;  // 49字节
    frame_data[FUNCTION] = CONTROL_ALL;

    // 编码每个电机的控制数据
    for (uint8_t i = 0; i < count; ++i) {
        size_t offset = MOTOR_DATA_START + i * MOTOR_DATA_SIZE;

        // 位置指令 * 100 -> int16
        int16_t pos_scaled = static_cast<int16_t>(motor_data[i].position * POSITION_SCALE);
        DataConverter::encodeInt16ToFrame(frame_data, offset + POSITION, pos_scaled);

        // 速度指令 * 100 -> int16
        int16_t vel_scaled = static_cast<int16_t>(motor_data[i].velocity * VELOCITY_SCALE);
        DataConverter::encodeInt16ToFrame(frame_data, offset + VELOCITY, vel_scaled);

        // 力矩指令 * 10 -> int16
        int16_t torque_scaled = static_cast<int16_t>(motor_data[i].torque * TORQUE_SCALE);
        DataConverter::encodeInt16ToFrame(frame_data, offset + TORQUE, torque_scaled);

        // Kp * 1000 -> uint8
        frame_data[offset + KP] = static_cast<uint8_t>(motor_data[i].kp * KP_SCALE);

        // Kd * 1000 -> uint8
        frame_data[offset + KD] = static_cast<uint8_t>(motor_data[i].kd * KD_SCALE);
    }

    return dataToFrame(frame_data, raytron::BROADCAST_CAN_ID);
}

// ========== 协议解码接口 ==========

void RaytronProtocolConverter::decodeFeedback(uint32_t can_id, const uint8_t* data, uint8_t dlc, MotorData& out_motor)
{
    uint8_t motor_id = can_id & 0xFF;
    out_motor.motor_id = motor_id;

    if (dlc < raytron::FrameSize::FEEDBACK_FRAME) {
        return;  // 数据长度不足
    }

    // 使能标志和控制模式
    out_motor.enable_flag = static_cast<raytron::EnableFlag>(
        data[raytron::FeedbackFrameOffset::ENABLE_FLAG]);
    out_motor.control_mode = static_cast<raytron::ControlMode>(
        data[raytron::FeedbackFrameOffset::CONTROL_MODE]);

    // 位置反馈 (float32, 大端)
    out_motor.position = DataConverter::assemble_float_be(
        data[raytron::FeedbackFrameOffset::POSITION_START + 0],
        data[raytron::FeedbackFrameOffset::POSITION_START + 1],
        data[raytron::FeedbackFrameOffset::POSITION_START + 2],
        data[raytron::FeedbackFrameOffset::POSITION_START + 3]);

    // 速度反馈 (float32, 大端)
    out_motor.velocity = DataConverter::assemble_float_be(
        data[raytron::FeedbackFrameOffset::VELOCITY_START + 0],
        data[raytron::FeedbackFrameOffset::VELOCITY_START + 1],
        data[raytron::FeedbackFrameOffset::VELOCITY_START + 2],
        data[raytron::FeedbackFrameOffset::VELOCITY_START + 3]);

    // 电流反馈 (float32, 大端)
    out_motor.current = DataConverter::assemble_float_be(
        data[raytron::FeedbackFrameOffset::CURRENT_START + 0],
        data[raytron::FeedbackFrameOffset::CURRENT_START + 1],
        data[raytron::FeedbackFrameOffset::CURRENT_START + 2],
        data[raytron::FeedbackFrameOffset::CURRENT_START + 3]);

    // 错误代码 (uint32, 大端)
    out_motor.error_code = DataConverter::assemble_uint32_be(
        data[raytron::FeedbackFrameOffset::ERROR_CODE_START + 0],
        data[raytron::FeedbackFrameOffset::ERROR_CODE_START + 1],
        data[raytron::FeedbackFrameOffset::ERROR_CODE_START + 2],
        data[raytron::FeedbackFrameOffset::ERROR_CODE_START + 3]);

    // 电压×10 (uint16, 大端) - 需要除以10
    uint16_t voltage_x10 = DataConverter::assemble_uint16_be(
        data[raytron::FeedbackFrameOffset::VOLTAGE_START + 0],
        data[raytron::FeedbackFrameOffset::VOLTAGE_START + 1]);
    out_motor.voltage = voltage_x10 / 10;

    // 温度×10 (uint16, 大端) - 需要除以10
    uint16_t temp_x10 = DataConverter::assemble_uint16_be(
        data[raytron::FeedbackFrameOffset::TEMPERATURE_START + 0],
        data[raytron::FeedbackFrameOffset::TEMPERATURE_START + 1]);
    out_motor.temperature = temp_x10 / 10;

    out_motor.timestamp = std::chrono::system_clock::now();
}

// ========== 内部辅助方法 ==========

canfd_frame RaytronProtocolConverter::dataToFrame(const std::vector<uint8_t>& data, uint32_t can_id) const
{
    canfd_frame frame{};
    std::memset(&frame, 0, sizeof(frame));  // 清零整个帧

    frame.can_id = can_id;  // 使用标准11位CAN ID
    frame.flags = CANFD_FDF;  // CANFD frame

    // 复制数据（最多64字节）
    size_t data_len = std::min(data.size(), size_t(64));
    std::copy(data.begin(), data.begin() + data_len, frame.data);

    // frame.len 保存实际数据长度，DLC转换在 Usb2CanfdManager::sendFrame 中进行
    frame.len = static_cast<uint8_t>(data_len);

    return frame;
}

};  // namespace raytron
