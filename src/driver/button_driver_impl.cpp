/*********************************************************************
 * @file        button_driver_impl.cpp
 * @brief       按键驱动CAN总线实现
 *********************************************************************/

#include "button_driver_impl.hpp"
#include <iostream>
#include <algorithm>

namespace hardware_driver {
namespace button_driver {

ButtonDriverImpl::ButtonDriverImpl(std::shared_ptr<bus::BusInterface> bus)
    : bus_(std::move(bus)) {
    std::cout << "[ButtonDriver] 按键驱动初始化完成" << std::endl;
}

void ButtonDriverImpl::send_replay_complete(const std::string& interface) {
    if (!bus_) {
        std::cerr << "[ButtonDriver] 错误: 总线未初始化" << std::endl;
        return;
    }

    // 构建发送数据包
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = BUTTON_TX_CAN_ID;  // 0x7F
    packet.len = 4;
    std::fill(packet.data.begin(), packet.data.end(), 0);
    // 填充 "FXJS" (复现结束)
    std::copy(REPLAY_COMPLETE_CODE, REPLAY_COMPLETE_CODE + 4, packet.data.begin());

    bus_->send(packet);
    std::cout << "[ButtonDriver] 发送复现完成信号 (FXJS) -> " << interface << std::endl;
}

void ButtonDriverImpl::add_observer(std::shared_ptr<ButtonEventObserver> observer) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.push_back(observer);
}

void ButtonDriverImpl::remove_observer(std::shared_ptr<ButtonEventObserver> observer) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.erase(
        std::remove(observers_.begin(), observers_.end(), observer),
        observers_.end()
    );
}

void ButtonDriverImpl::set_receive_callback(ReceiveCallback callback) {
    // 此实现不使用回调方式，而是通过 handle_can_packet 接收数据
    (void)callback;
}

void ButtonDriverImpl::handle_can_packet(const std::string& interface, uint32_t can_id,
                                         const uint8_t* data, size_t len) {
    // 只处理按键数据 (CAN ID 0x8F)
    if (can_id != BUTTON_RX_CAN_ID) {
        return;
    }

    // 解析协议码
    ButtonStatus status = parse_button_code(data, len);
    if (status == ButtonStatus::NONE) {
        return;
    }

    std::lock_guard<std::mutex> lock(status_mutex_);

    // 防抖处理: 只有状态变化时才触发
    // 例外: TEACH_REPEAT 可以重复触发
    if (status != last_status_ || status == ButtonStatus::TEACH_REPEAT) {
        const char* status_names[] = {"NONE", "ENTRY_TEACH", "EXIT_TEACH", "TEACH_REPEAT"};
        std::cout << "[ButtonDriver] 按键事件: " << status_names[static_cast<int>(status)]
                  << " (" << interface << ")" << std::endl;

        notify_observers(interface, status);

        if (status == ButtonStatus::TEACH_REPEAT) {
            // 复现模式触发后清零，允许下次重复触发
            last_status_ = ButtonStatus::NONE;
        } else {
            last_status_ = status;
        }
    }
}

void ButtonDriverImpl::notify_observers(const std::string& interface, ButtonStatus status) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    for (auto& observer : observers_) {
        if (observer) {
            observer->on_button_event(interface, status);
        }
    }
}

ButtonStatus ButtonDriverImpl::parse_button_code(const uint8_t* data, size_t len) {
    if (len < 4) {
        return ButtonStatus::NONE;
    }

    // 提取4字节ASCII码
    char code[5] = {
        static_cast<char>(data[0]),
        static_cast<char>(data[1]),
        static_cast<char>(data[2]),
        static_cast<char>(data[3]),
        '\0'
    };

    // 查表匹配
    for (size_t i = 0; i < PROTOCOL_COUNT; ++i) {
        if (std::memcmp(PROTOCOL_TABLE[i].code, code, 4) == 0) {
            return PROTOCOL_TABLE[i].status;
        }
    }

    return ButtonStatus::NONE;
}

}  // namespace button_driver
}  // namespace hardware_driver
