#include "gripper_omnipicker_protocol.hpp"

#include <algorithm>

namespace hardware_driver {
namespace gripper_omnipicker_protocol {

namespace {
enum OmniPickerActionStatus : uint8_t {
    REACHED_DES = 0x00,
    GIRPPER_MOVING = 0x01,
    GRIPPER_JAM = 0x02,
    OBJ_FALL = 0x03
};

}  // namespace

std::optional<GripperFeedback> parse_canfd_feedback(const bus::GenericBusPacket& packet) {
    if (packet.id != RECV_GRIPPER_ID || packet.len < 6) {
        return std::nullopt;
    }

    const uint8_t* p = packet.data.data();
    if (p[0] != 0x01) {
        return std::nullopt;
    }

    GripperFeedback feedback;
    feedback.interface = packet.interface;
    feedback.gripper_id = packet.id;

    feedback.status.status = p[1];
    feedback.status.action_status = p[2];
    feedback.status.position = p[3];
    feedback.status.velocity = p[4];
    feedback.status.force = p[5];

    const uint8_t action = p[2];
    feedback.status.is_moving = (action == GIRPPER_MOVING);
    feedback.status.has_object = (action == REACHED_DES);

    return feedback;
}


bool pack_omnipicker_control_command(
    std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data,
    size_t& len,
    uint8_t position,
    uint8_t velocity,
    uint8_t effort,
    uint8_t acc,
    uint8_t dec) {
    data.fill(0);

    data[0] = 0x01;
    data[1] = 0x01;
    // OmniPicker raw protocol bytes: 0x00~0xFF
    data[2] = position;
    data[3] = velocity;
    data[4] = effort;
    data[5] = acc;
    data[6] = dec;
    data[7] = 0x00;
    len = 8;
    return true;
}

}  // namespace gripper_omnipicker_protocol
}  // namespace hardware_driver
