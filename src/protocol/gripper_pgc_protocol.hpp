#ifndef __HARDWARE_DRIVER_PGC_GRIPPER_PROTOCOL_HPP__
#define __HARDWARE_DRIVER_PGC_GRIPPER_PROTOCOL_HPP__

#include <vector>
#include <cstdint>
#include <optional>
#include <mutex>
#include <map>
#include "hardware_driver/bus/bus_interface.hpp"
#include "hardware_driver/driver/gripper_driver_interface.hpp"
#include <variant>
#include <cstring>
#include <cassert>
#include <any>

namespace hardware_driver {
namespace gripper_pgc_protocol {

/**
 * @brief PGC夹爪状态反馈接口数据结构体
 */
struct GripperFeedback {
    gripper_driver::GripperStatus status;
    std::string interface;
    uint32_t gripper_id;
};

// 解包函数：将通用总线包解析为夹爪状态
std::optional<GripperFeedback> parse_canfd_feedback(const bus::GenericBusPacket& packet);

/**
 * @brief 打包控制命令，夹爪工作的控制打包函数只负责填充data和len
 * @return bool 返回是否成功打包
 */ 
bool pack_pgc_control_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
                             uint8_t velocity, uint8_t effort, uint8_t position);

/**
 * @brief 打包状态查询命令，查询夹爪状态的指令
 * @return bool 返回是否成功打包
 */
bool pack_pgc_status_req_command(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len);

/**
 * @brief 打包状态初始化命令，夹爪上电或重置后需要发送此命令进行状态初始化
 * @return bool 返回是否成功打包
 */
bool pack_pgc_status_initialize(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len);

}  // namespace gripper_pgc_protocol
}  // namespace hardware_driver

#endif    // __HARDWARE_DRIVER_PGC_GRIPPER_PROTOCOL_HPP__