#include "protocol/teaching_device.h"

namespace teaching_device
{

bool TeachingDeviceProtocol::decodeCommand(uint32_t can_id, const uint8_t* data, uint8_t dlc, CommandCode& out_command)
{
    // 验证CAN ID
    if (can_id != CMD_RECEIVE_ID) {
        return false;
    }

    // 验证数据长度
    if (dlc < 1 || data == nullptr) {
        return false;
    }

    // 解码命令码
    uint8_t cmd = data[0];

    // 验证命令码范围
    if (cmd < CMD_START_TEACHING || cmd > CMD_RECORD_MIN_ANGLE) {
        return false;
    }

    out_command = static_cast<CommandCode>(cmd);
    return true;
}

canfd_frame TeachingDeviceProtocol::encodeResponse(CommandCode command)
{
    canfd_frame frame{};
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id = CMD_RESPONSE_ID;
    frame.flags = CANFD_FDF;
    frame.len = 1;
    frame.data[0] = static_cast<uint8_t>(command);

    return frame;
}

const char* TeachingDeviceProtocol::commandToString(CommandCode command)
{
    switch (command)
    {
        case CMD_START_TEACHING:
            return "START_TEACHING";
        case CMD_END_TEACHING:
            return "END_TEACHING";
        case CMD_ENTER_CALIBRATION:
            return "ENTER_CALIBRATION";
        case CMD_RECORD_MAX_ANGLE:
            return "RECORD_MAX_ANGLE";
        case CMD_RECORD_MIN_ANGLE:
            return "RECORD_MIN_ANGLE";
        default:
            return "UNKNOWN";
    }
}

} // namespace teaching_device
