/*********************************************************************
 * @file        iap_protocol.hpp
 * @brief       IAP固件更新协议接口（主机侧）
 *
 * 定义了电机Bootloader与APP程序的IAP（In-Application Programming）交互协议，
 * 包含固件文件加载、指令打包、反馈解析等接口。
 *
 * 支持的状态机流程：
 *  APP模式下 -> 发送[0x01,0x12]请求IAP -> 收到AJ01
 *  Boot模式下 -> 打印BS00 -> 发送"key" -> 收到BK01~BK03
 *  Boot模式接收固件 -> BD04/BD05 -> 校验 -> BJ06 -> AS00
 *
 * @version     1.0.0
 * @date        2025-10-31
 * @copyright   Raysense Technology
 *********************************************************************/

#ifndef __HARDWARE_DRIVER_IAP_PROTOCOL_HPP__
#define __HARDWARE_DRIVER_IAP_PROTOCOL_HPP__

#include <vector>
#include <string>
#include <array>
#include <optional>
#include <cstdint>
#include "hardware_driver/bus/bus_interface.hpp"
#include "hardware_driver/driver/motor_driver_interface.hpp"

namespace hardware_driver {
namespace iap_protocol {

/**
 * @brief IAP反馈信息结构
 * 包含设备通过CAN发送的状态消息
 */
struct IAPFeedback {
    motor_driver::IAPStatus status_msg;   // 状态消息
    uint32_t motor_id;             // 电机ID（从CAN ID提取）
};

/**
 * @brief 从文件加载固件数据
 * @return 是否成功
 */
bool load_firmware_from_file(const std::string& filename, std::vector<uint8_t>& firmware_data);

/**
 * @brief 打包 APP 模式下进入 IAP 的更新请求帧
 * Data = [0x01, 0x12]
 */
bool pack_enter_iap_request(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len);

/**
 * @brief 打包 Boot 模式下发送 "key" 命令帧
 * Data = ['k','e','y']
 */
bool pack_send_key(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len);

/**
 * @brief 打包 Boot 模式下发送固件数据帧
 * 每帧 ≤ 64字节
 */
bool pack_firmware_frame(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
                         const std::vector<uint8_t>& firmware_data,
                         size_t offset, size_t chunk_size = 64);

/**
 * @brief 解析IAP反馈帧（4字节ASCII编码）
 */
std::optional<IAPFeedback> parse_iap_feedback(const bus::GenericBusPacket& packet);

/**
 * @brief 将IAP状态消息转换为字符串
 */
std::string iap_status_to_string(motor_driver::IAPStatus msg);

}   // namespace iap_protocol
}   // namespace hardware_driver

#endif    // __HARDWARE_DRIVER_IAP_PROTOCOL_HPP__
