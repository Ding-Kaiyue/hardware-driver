#ifndef TEACHING_DEVICE_H
#define TEACHING_DEVICE_H

#include <cstdint>
#include <linux/can.h>
#include <cstring>

/**
 * @brief 示教设备协议
 *
 * 本机作为外部设备：
 * - 接收 CAN ID 0xA01 (来自STM32的命令)
 * - 发送 CAN ID 0xA02 (响应给STM32)
 *
 * 命令码定义：
 * 1 - CMD_START_TEACHING    : 开始示教
 * 2 - CMD_END_TEACHING      : 结束示教
 * 3 - CMD_ENTER_CALIBRATION : 进入校准模式
 * 4 - CMD_RECORD_MAX_ANGLE  : 记录最大角度
 * 5 - CMD_RECORD_MIN_ANGLE  : 记录最小角度
 */

namespace teaching_device
{

// ========== CAN ID 常量 ==========
constexpr uint32_t CMD_RECEIVE_ID = 0xA01;  // 接收命令的CAN ID
constexpr uint32_t CMD_RESPONSE_ID = 0xA02; // 发送响应的CAN ID

// ========== 命令码定义 ==========
enum CommandCode : uint8_t
{
    CMD_START_TEACHING    = 1,  // 开始示教
    CMD_END_TEACHING      = 2,  // 结束示教
    CMD_ENTER_CALIBRATION = 3,  // 进入校准模式
    CMD_RECORD_MAX_ANGLE  = 4,  // 记录最大角度
    CMD_RECORD_MIN_ANGLE  = 5,  // 记录最小角度
};

/**
 * @brief 示教设备协议转换器
 */
class TeachingDeviceProtocol
{
public:
    TeachingDeviceProtocol() = default;
    ~TeachingDeviceProtocol() = default;

    /**
     * @brief 解码接收到的命令帧
     * @param can_id CAN ID (应该是 0xA01)
     * @param data 数据指针
     * @param dlc 数据长度 (应该是1)
     * @param out_command 输出的命令码
     * @return 解码是否成功
     */
    bool decodeCommand(uint32_t can_id, const uint8_t* data, uint8_t dlc, CommandCode& out_command);

    /**
     * @brief 编码响应帧
     * @param command 要响应的命令码
     * @return 编码后的CAN帧
     */
    canfd_frame encodeResponse(CommandCode command);

    /**
     * @brief 将命令码转换为字符串描述
     * @param command 命令码
     * @return 命令描述字符串
     */
    static const char* commandToString(CommandCode command);
};

} // namespace teaching_device

#endif // TEACHING_DEVICE_H
