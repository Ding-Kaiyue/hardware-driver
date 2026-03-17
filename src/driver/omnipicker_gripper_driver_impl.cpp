/*********************************************************************
 * @file        omnipicker_gripper_driver_impl.cpp
 * @brief       OmniPicker 夹爪驱动实现
 *********************************************************************/

#include "omnipicker_gripper_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"
#include "../protocol/gripper_omnipicker_protocol.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace hardware_driver {
namespace gripper_driver {

namespace {
const char* fault_code_to_string(uint8_t code) {
    switch (code) {
        case 0x00: return "No fault";
        case 0x01: return "Over temperature";
        case 0x02: return "Overspeed";
        case 0x03: return "Initialization fault";
        case 0x04: return "Overlimit";
        default:   return "Unknown fault";
    }
}
}  // namespace

OmniPickerGripperDriverImpl::OmniPickerGripperDriverImpl(std::shared_ptr<bus::BusInterface> bus)
    : bus_(std::move(bus)) {
    if (!bus_) {
        throw std::runtime_error("[OmniPickerGripperDriver] Bus interface is nullptr");
    }

    auto canfd_bus = std::dynamic_pointer_cast<bus::CanFdBus>(bus_);
    if (canfd_bus) {
        for (const auto& interface : canfd_bus->get_interface_names()) {
            canfd_bus->set_extended_frame(interface, true);
            canfd_bus->set_fd_mode(interface, true);
        }
    }

    bus_->async_receive([this](const bus::GenericBusPacket& packet) {
        this->receive_callback(packet);
    });

    std::cout << "[OmniPickerGripperDriver] Driver initialized successfully" << std::endl;
}

OmniPickerGripperDriverImpl::~OmniPickerGripperDriverImpl() = default;

void OmniPickerGripperDriverImpl::control_gripper(const std::string& interface,
                                                 GripperType gripper_type,
                                                 uint8_t position,
                                                 uint8_t velocity,
                                                 uint8_t effort) {
    if (gripper_type != GripperType::OmniPicker) {
        std::cerr << "[OmniPickerGripperDriver] This driver only supports OmniPicker gripper type"
                  << std::endl;
        return;
    }

    bus::GenericBusPacket ctrl_packet;
    ctrl_packet.interface = interface;
    ctrl_packet.id = gripper_omnipicker_protocol::SEND_GRIPPER_ID;
    ctrl_packet.protocol_type = bus::BusProtocolType::CAN_FD;
    // 智元 OmniPicker 协议:
    // ID=0x5F, [0x01, 0x01, Pos, Speed, Force, Acc, Dec, 0x00]
    // 其中 Pos/Speed/Force 均是 0x00~0xFF，Acc/Dec 默认固定 0xFF
    gripper_omnipicker_protocol::pack_omnipicker_control_command(
        ctrl_packet.data, ctrl_packet.len, position, velocity, effort, 0xFF, 0xFF);

    if (!send_packet(ctrl_packet)) {
        std::cerr << "[OmniPickerGripperDriver] Failed to send control packet" << std::endl;
    }
}

void OmniPickerGripperDriverImpl::send_raw_data(const std::string& interface,
                                                const uint8_t* raw_data,
                                                size_t raw_data_len) {
    if (!raw_data || raw_data_len == 0) {
        std::cerr << "[OmniPickerGripperDriver] Invalid raw data" << std::endl;
        return;
    }

    bus::GenericBusPacket raw_packet;
    raw_packet.interface = interface;
    raw_packet.id = gripper_omnipicker_protocol::SEND_GRIPPER_ID;
    raw_packet.protocol_type = bus::BusProtocolType::CAN_FD;
    raw_packet.len = std::min(raw_data_len, static_cast<size_t>(bus::MAX_BUS_DATA_SIZE));
    std::copy(raw_data, raw_data + raw_packet.len, raw_packet.data.begin());

    if (!send_packet(raw_packet)) {
        std::cerr << "[OmniPickerGripperDriver] Failed to send raw packet" << std::endl;
    }
}

void OmniPickerGripperDriverImpl::open_gripper(const std::string& interface,
                                               GripperType gripper_type,
                                               uint8_t velocity,
                                               uint8_t effort) {
    // OmniPicker protocol: 0xFF = fully open
    control_gripper(interface, gripper_type, 0xFF, velocity, effort);
}

void OmniPickerGripperDriverImpl::close_gripper(const std::string& interface,
                                                GripperType gripper_type,
                                                uint8_t velocity,
                                                uint8_t effort) {
    // OmniPicker protocol: 0x00 = fully close
    control_gripper(interface, gripper_type, 0x00, velocity, effort);
}

void OmniPickerGripperDriverImpl::add_observer(std::shared_ptr<GripperStatusObserver> observer) {
    if (!observer) {
        std::cerr << "[OmniPickerGripperDriver] Cannot add nullptr observer" << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.push_back(observer);
}

void OmniPickerGripperDriverImpl::remove_observer(std::shared_ptr<GripperStatusObserver> observer) {
    if (!observer) {
        return;
    }

    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.erase(
        std::remove(observers_.begin(), observers_.end(), observer),
        observers_.end());
}

void OmniPickerGripperDriverImpl::receive_callback(const bus::GenericBusPacket& packet) {
    auto feedback = gripper_omnipicker_protocol::parse_canfd_feedback(packet);
    if (!feedback.has_value()) {
        return;
    }

    const GripperStatus status = feedback->status;

    if (status.status != 0x00) {
        std::cerr << "[OmniPickerGripperDriver] Fault code=0x"
                  << std::hex << static_cast<int>(status.status) << std::dec
                  << " (" << fault_code_to_string(status.status) << ")"
                  << std::endl;
    }

    notify_observers(packet.interface, 1, status);

    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        status_cache_[packet.interface][1] = status;
    }
}

void OmniPickerGripperDriverImpl::notify_observers(const std::string& interface,
                                                   uint32_t gripper_id,
                                                   const GripperStatus& status) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    for (auto& observer : observers_) {
        if (observer) {
            observer->on_gripper_status_update(interface, gripper_id, status);
        }
    }
}

bool OmniPickerGripperDriverImpl::send_packet(const bus::GenericBusPacket& packet) {
    return bus_->send(packet);
}

}  // namespace gripper_driver
}  // namespace hardware_driver
