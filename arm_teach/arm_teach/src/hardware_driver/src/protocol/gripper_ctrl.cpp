#include "protocol/gripper_ctrl.h"
#include <algorithm>
#include <cstring>

namespace gripper
{

// ========== 协议编码接口 ==========

canfd_frame GripperProtocolConverter::encodeControl(uint8_t gripper_id,
                                                     uint8_t pos_cmd,
                                                     uint8_t force_cmd,
                                                     uint8_t vel_cmd,
                                                     uint8_t acc_cmd,
                                                     uint8_t dec_cmd)
{
    canfd_frame frame{};
    frame.can_id = gripper_id;  // 使用gripper_id作为CAN ID
    frame.len = 8;  // 固定8字节
    frame.flags = CANFD_FDF;  // CANFD frame

    // D0: Reserved
    frame.data[0] = 0x00;
    // D1: Position Command
    frame.data[1] = pos_cmd;
    // D2: Force Command
    frame.data[2] = force_cmd;
    // D3: Velocity Command
    frame.data[3] = vel_cmd;
    // D4: Acceleration Command
    frame.data[4] = acc_cmd;
    // D5: Deceleration Command
    frame.data[5] = dec_cmd;
    // D6-D7: Reserved
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    return frame;
}

canfd_frame GripperProtocolConverter::encodeSimpleControl(uint8_t gripper_id,
                                                           uint8_t pos_cmd,
                                                           uint8_t force_cmd)
{
    // 使用默认速度、加速度、减速度
    return encodeControl(gripper_id, pos_cmd, force_cmd, 255, 255, 255);
}

canfd_frame GripperProtocolConverter::encodeCloseGripper(uint8_t gripper_id, uint8_t force_cmd)
{
    // 位置0表示夹紧
    return encodeSimpleControl(gripper_id, 0x00, force_cmd);
}

canfd_frame GripperProtocolConverter::encodeOpenGripper(uint8_t gripper_id)
{
    // 位置255表示完全张开，力矩设为0
    return encodeSimpleControl(gripper_id, 0xFF, 0x00);
}

// ========== 协议解码接口 ==========

void GripperProtocolConverter::decodeStatus(uint32_t can_id, const uint8_t* data, uint8_t dlc, GripperData& out_gripper)
{
    // 确保数据长度足够
    if (dlc < 8 || !data) {
        return;
    }

    // D0: Fault Code
    out_gripper.fault_code = data[0];

    // D1: State
    out_gripper.state = data[1];

    // D2: Position
    out_gripper.position = data[2];

    // D3: Velocity
    out_gripper.velocity = data[3];

    // D4: Force
    out_gripper.force = data[4];

    // D5-D7: Reserved
    out_gripper.reserved[0] = data[5];
    out_gripper.reserved[1] = data[6];
    out_gripper.reserved[2] = data[7];
}

const char* GripperProtocolConverter::getFaultCodeDescription(uint8_t fault_code) const
{
    switch (fault_code) {
        case FAULT_NO_ERROR:
            return "无故障";
        case FAULT_OVER_TEMP:
            return "过温警报";
        case FAULT_OVER_SPEED:
            return "超速警报";
        case FAULT_INIT_FAILED:
            return "初始化故障警报";
        case FAULT_LIMIT_CHECK:
            return "超限检测警报";
        default:
            return "未知故障";
    }
}

const char* GripperProtocolConverter::getStateDescription(uint8_t state) const
{
    switch (state) {
        case STATE_TARGET_REACHED:
            return "已达到目标位置";
        case STATE_MOVING:
            return "夹爪移动中";
        case STATE_JAM:
            return "夹爪堵转";
        case STATE_OBJECT_DROPPED:
            return "物体掉落";
        default:
            return "未知状态";
    }
}

}; // namespace gripper
