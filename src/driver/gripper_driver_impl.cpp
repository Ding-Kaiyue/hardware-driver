/*********************************************************************
 * @file        gripper_driver_impl.cpp
 * @brief       夹爪驱动实现
 *********************************************************************/

#include "gripper_driver_impl.hpp"

#include <algorithm>
#include <iostream>

#include "../protocol/gripper_omnipicker_protocol.hpp"
#include "../protocol/gripper_pgc_protocol.hpp"

namespace hardware_driver {
namespace gripper_driver {

namespace {
constexpr uint32_t kSendGripperId = 0x5F;
}  // namespace

GripperDriverImpl::GripperDriverImpl(std::shared_ptr<bus::BusInterface> bus)
    : bus_(std::move(bus)) {
    if (!bus_) {
        throw std::runtime_error("[GripperDriver] Bus interface is nullptr");
    }

    // Register rx callback once; feedback parser will dispatch by CAN id/payload.
    bus_->async_receive([this](const bus::GenericBusPacket& packet) {
        this->receive_callback(packet);
    });
}

GripperDriverImpl::~GripperDriverImpl() {
    stop_receive();
}

void GripperDriverImpl::control_gripper(const std::string& interface,
                                        GripperType gripper_type,
                                        uint8_t position,
                                        uint8_t velocity,
                                        uint8_t effort) {
    if (gripper_type == GripperType::Raw_Frame) {
        std::cerr << "[GripperDriver] Raw_Frame is not supported by control_gripper(), "
                     "use send_raw_data()"
                  << std::endl;
        return;
    }

    if (gripper_type == GripperType::OmniPicker) {
        bus::GenericBusPacket ctrl_packet;
        ctrl_packet.interface = interface;
        ctrl_packet.id = kSendGripperId;
        ctrl_packet.protocol_type = bus::BusProtocolType::CAN_FD;
        // OmniPicker protocol uses raw bytes (0x00~0xFF); acc/dec are fixed to 0xFF.
        gripper_omnipicker_protocol::pack_omnipicker_control_command(
            ctrl_packet.data, ctrl_packet.len, position, velocity, effort, 0xFF, 0xFF);

        if (!send_packet(ctrl_packet)) {
            std::cerr << "[GripperDriver] Failed to send OmniPicker command" << std::endl;
        }
        return;
    }

    // PGC_Gripper path
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = kSendGripperId;
    packet.protocol_type = bus::BusProtocolType::CAN_FD;

    uint8_t pgc_position = std::min<uint8_t>(position, 100);
    uint8_t pgc_velocity = std::max<uint8_t>(1, std::min<uint8_t>(velocity, 100));
    uint8_t pgc_effort = std::max<uint8_t>(20, std::min<uint8_t>(effort, 100));

    gripper_pgc_protocol::pack_pgc_control_command(
        packet.data, packet.len, pgc_velocity, pgc_effort, pgc_position);

    if (!send_packet(packet)) {
        std::cerr << "[GripperDriver] Failed to send PGC command" << std::endl;
    }
}

void GripperDriverImpl::send_raw_data(const std::string& interface,
                                      const uint8_t* raw_data,
                                      size_t raw_data_len) {
    if (!raw_data || raw_data_len == 0) {
        std::cerr << "[GripperDriver] Invalid raw data" << std::endl;
        return;
    }

    bus::GenericBusPacket raw_packet;
    raw_packet.interface = interface;
    raw_packet.id = kSendGripperId;
    raw_packet.protocol_type = bus::BusProtocolType::CAN_FD;
    raw_packet.len = std::min(raw_data_len, static_cast<size_t>(bus::MAX_BUS_DATA_SIZE));
    std::copy(raw_data, raw_data + raw_packet.len, raw_packet.data.begin());

    if (!send_packet(raw_packet)) {
        std::cerr << "[GripperDriver] Failed to send raw packet" << std::endl;
    }
}

void GripperDriverImpl::open_gripper(const std::string& interface,
                                     GripperType gripper_type,
                                     uint8_t velocity,
                                     uint8_t effort) {
    if (gripper_type == GripperType::OmniPicker) {
        // OmniPicker: 0xFF = fully open
        control_gripper(interface, gripper_type, 0xFF, velocity, effort);
    } else {
        // PGC: 0 = fully open
        control_gripper(interface, gripper_type, 0, velocity, effort);
    }
}

void GripperDriverImpl::close_gripper(const std::string& interface,
                                      GripperType gripper_type,
                                      uint8_t velocity,
                                      uint8_t effort) {
    if (gripper_type == GripperType::OmniPicker) {
        // OmniPicker: 0x00 = fully close
        control_gripper(interface, gripper_type, 0x00, velocity, effort);
    } else {
        // PGC: 100 = fully close
        control_gripper(interface, gripper_type, 100, velocity, effort);
    }
}

void GripperDriverImpl::add_observer(std::shared_ptr<GripperStatusObserver> observer) {
    if (!observer) {
        std::cerr << "[GripperDriver] Cannot add nullptr observer" << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.push_back(observer);
}

void GripperDriverImpl::remove_observer(std::shared_ptr<GripperStatusObserver> observer) {
    if (!observer) {
        return;
    }

    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.erase(std::remove(observers_.begin(), observers_.end(), observer), observers_.end());
}

void GripperDriverImpl::start_receive() {}

void GripperDriverImpl::stop_receive() {}

void GripperDriverImpl::receive_callback(const bus::GenericBusPacket& packet) {
    if (auto omni_feedback = gripper_omnipicker_protocol::parse_canfd_feedback(packet);
        omni_feedback.has_value()) {
        notify_observers(packet.interface, omni_feedback->gripper_id, omni_feedback->status);
        std::lock_guard<std::mutex> lock(cache_mutex_);
        status_cache_[packet.interface][omni_feedback->gripper_id] = omni_feedback->status;
        return;
    }

    if (auto pgc_feedback = gripper_pgc_protocol::parse_canfd_feedback(packet);
        pgc_feedback.has_value()) {
        notify_observers(packet.interface, pgc_feedback->gripper_id, pgc_feedback->status);
        std::lock_guard<std::mutex> lock(cache_mutex_);
        status_cache_[packet.interface][pgc_feedback->gripper_id] = pgc_feedback->status;
    }
}

void GripperDriverImpl::notify_observers(const std::string& interface,
                                         uint32_t gripper_id,
                                         const GripperStatus& status) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    for (auto& observer : observers_) {
        if (observer) {
            observer->on_gripper_status_update(interface, gripper_id, status);
        }
    }
}

bool GripperDriverImpl::send_packet(const bus::GenericBusPacket& packet) {
    return bus_->send(packet);
}

}  // namespace gripper_driver
}  // namespace hardware_driver
