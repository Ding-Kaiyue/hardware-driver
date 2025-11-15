/*********************************************************************
 * @file        gripper_driver_impl.cpp
 * @brief       夹爪驱动实现
 *********************************************************************/

#include "gripper_driver_impl.hpp"
#include <iostream>
#include <algorithm>

namespace hardware_driver {
namespace gripper_driver {

GripperDriverImpl::GripperDriverImpl(std::shared_ptr<bus::BusInterface> bus)
    : bus_(bus) {
    if (!bus_) {
        throw std::runtime_error("[GripperDriver] Bus interface is nullptr");
    }
}

GripperDriverImpl::~GripperDriverImpl() {
    stop_receive();
}

void GripperDriverImpl::control_gripper(const std::string& interface,
                                       GripperType gripper_type,
                                       uint8_t position,
                                       uint8_t velocity,
                                       uint8_t effort) {
    // 转换为协议枚举类型
    gripper_protocol::GripperType proto_type =
        static_cast<gripper_protocol::GripperType>(gripper_type);

    // 构造控制数据包
    auto packets = gripper_protocol::GripperProtocolBuilder::build_gripper_control(
        interface, proto_type, position, velocity, effort, nullptr, 0);

    // 发送所有数据包
    for (const auto& packet : packets) {
        if (!send_packet(packet)) {
            std::cerr << "[GripperDriver] Failed to send control packet" << std::endl;
        }
    }
}

void GripperDriverImpl::send_raw_data(const std::string& interface,
                                     const uint8_t* raw_data,
                                     size_t raw_data_len) {
    if (!raw_data || raw_data_len == 0) {
        std::cerr << "[GripperDriver] Invalid raw data" << std::endl;
        return;
    }

    // 使用Raw_Frame模式
    auto packets = gripper_protocol::GripperProtocolBuilder::build_gripper_control(
        interface,
        gripper_protocol::GripperType::Raw_Frame,
        0, 0, 0,
        raw_data, raw_data_len);

    // 发送所有数据包
    for (const auto& packet : packets) {
        if (!send_packet(packet)) {
            std::cerr << "[GripperDriver] Failed to send raw data packet" << std::endl;
        }
    }
}

void GripperDriverImpl::open_gripper(const std::string& interface,
                                    GripperType gripper_type,
                                    uint8_t velocity,
                                    uint8_t effort) {
    // 打开夹爪 = 位置设置为0（最大张开）
    control_gripper(interface, gripper_type, 0, velocity, effort);
}

void GripperDriverImpl::close_gripper(const std::string& interface,
                                     GripperType gripper_type,
                                     uint8_t velocity,
                                     uint8_t effort) {
    // 关闭夹爪 = 位置设置为100（完全闭合）
    control_gripper(interface, gripper_type, 100, velocity, effort);
}

void GripperDriverImpl::add_observer(std::shared_ptr<GripperStatusObserver> observer) {
    if (!observer) {
        std::cerr << "[GripperDriver] Cannot add nullptr observer" << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.push_back(observer);
    std::cout << "[GripperDriver] Observer added, total: " << observers_.size() << std::endl;
}

void GripperDriverImpl::remove_observer(std::shared_ptr<GripperStatusObserver> observer) {
    if (!observer) {
        return;
    }

    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.erase(
        std::remove(observers_.begin(), observers_.end(), observer),
        observers_.end());
    std::cout << "[GripperDriver] Observer removed, total: " << observers_.size() << std::endl;
}

void GripperDriverImpl::start_receive() {
    std::cout << "[GripperDriver] Starting async receive..." << std::endl;

    // 注册异步接收回调
    bus_->async_receive([this](const bus::GenericBusPacket& packet) {
        this->receive_callback(packet);
    });
}

void GripperDriverImpl::stop_receive() {
    std::cout << "[GripperDriver] Stopping receive..." << std::endl;
    // 总线层负责停止接收
}

void GripperDriverImpl::receive_callback(const bus::GenericBusPacket& packet) {
    // 只处理夹爪相关的CAN ID
    if (packet.id == gripper_protocol::RECV_GRIPPER_ID) {
        // 解析夹爪反馈
        gripper_protocol::GripperProtocolBuilder::parse_gripper_feedback(packet);

        // 构造状态数据（简化版，实际需要根据协议解析）
        GripperStatus status;
        if (packet.len >= 6) {
            // 根据第一个字节判断夹爪类型
            if (packet.data[0] == 0x01) {
                // OmniPicker反馈
                gripper_protocol::OmniPickerFrame frame(packet.data.data(), packet.len);
                status.position = frame.get_position();
                status.velocity = frame.get_speed();
                status.force = frame.get_force();
                status.status = static_cast<uint8_t>(frame.get_status());
                status.action_status = static_cast<uint8_t>(frame.get_action_status());
                status.is_moving = (frame.get_action_status() ==
                                   gripper_protocol::OmniPickerActionStatus::GRIPPER_MOVING);
                status.has_object = (frame.get_action_status() ==
                                    gripper_protocol::OmniPickerActionStatus::REACHED_DES);
            } else if (packet.data[0] == 0x02) {
                // PGC反馈
                gripper_protocol::PGCGripperFrame frame(packet.data.data(), packet.len);
                status.status = static_cast<uint8_t>(frame.get_status());
                status.is_moving = (frame.get_status() ==
                                   gripper_protocol::PGCGripperStatus::GRIPPER_MOVING);
                status.has_object = (frame.get_status() ==
                                    gripper_protocol::PGCGripperStatus::GRIPPER_STOP_SOMETHING);
            }

            // 通知观察者
            notify_observers(packet.interface, 1, status);  // 假设gripper_id=1

            // 更新缓存
            {
                std::lock_guard<std::mutex> lock(cache_mutex_);
                status_cache_[packet.interface][1] = status;
            }
        }
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
    std::lock_guard<std::mutex> lock(send_mutex_);
    return bus_->send(packet);
}

}  // namespace gripper_driver
}  // namespace hardware_driver
