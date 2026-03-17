#ifndef __HARDWARE_DRIVER_OMNIPICKER_GRIPPER_PROTOCOL_HPP__
#define __HARDWARE_DRIVER_OMNIPICKER_GRIPPER_PROTOCOL_HPP__

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "hardware_driver/bus/bus_interface.hpp"
#include "hardware_driver/driver/gripper_driver_interface.hpp"

namespace hardware_driver {
namespace gripper_omnipicker_protocol {

constexpr uint32_t CANFD_2_RS485_ID = 0x3F;
constexpr uint32_t SEND_GRIPPER_ID = 0x5F;
constexpr uint32_t RECV_GRIPPER_ID = 0x6F;

struct GripperFeedback {
    gripper_driver::GripperStatus status;
    std::string interface;
    uint32_t gripper_id;
};

// 解包：解析 OmniPicker 反馈
std::optional<GripperFeedback> parse_canfd_feedback(const bus::GenericBusPacket& packet);

// 打包：发送到 SEND_GRIPPER_ID，控制命令 [0x01,0x01,pos,vel,force,acc,dec,0x00]
bool pack_omnipicker_control_command(
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data,
    size_t& len,
    uint8_t position,
    uint8_t velocity,
    uint8_t effort,
    uint8_t acc = 0xFF,
    uint8_t dec = 0xFF);

}  // namespace gripper_omnipicker_protocol
}  // namespace hardware_driver

#endif  // __HARDWARE_DRIVER_OMNIPICKER_GRIPPER_PROTOCOL_HPP__
