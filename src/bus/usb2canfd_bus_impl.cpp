#include "bus/usb2canfd_bus_impl.hpp"
#include "protocol/usb_class.h"

namespace hardware_driver {
namespace bus {

Usb2CanfdBus::Usb2CanfdBus(
    const std::vector<std::string>& devices,
    uint32_t arbitration_bitrate,
    uint32_t data_bitrate)
    : arbitration_bitrate_(arbitration_bitrate),
      data_bitrate_(data_bitrate)
{
    for (size_t i = 0; i < devices.size(); ++i) {
        std::string interface_name = "usb_canfd_" + std::to_string(i);
        interface_names_.push_back(interface_name);
        interface_to_sn_[interface_name] = devices[i];
    }
    init();
}

Usb2CanfdBus::Usb2CanfdBus(const std::vector<std::string>& devices)
    : arbitration_bitrate_(DEFAULT_ARBITRATION_BITRATE),
      data_bitrate_(DEFAULT_DATA_BITRATE)
{
    for (size_t i = 0; i < devices.size(); ++i) {
        std::string interface_name = "usb_canfd_" + std::to_string(i);
        interface_names_.push_back(interface_name);
        interface_to_sn_[interface_name] = devices[i];
    }
    init();
}

Usb2CanfdBus::~Usb2CanfdBus() {
    running_ = false;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    for (auto& thread : receive_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    receive_threads_.clear();

    usb_devices_.clear();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void Usb2CanfdBus::init() {
    for (const auto& interface : interface_names_) {
        auto it = interface_to_sn_.find(interface);
        if (it == interface_to_sn_.end()) {
            std::cerr << "[Usb2CanfdBus] Device SN not found for interface: " << interface << std::endl;
            continue;
        }

        try {
            auto usb_dev = std::make_shared<usb_class>(
                arbitration_bitrate_,
                data_bitrate_,
                it->second
            );

            if (!usb_dev) {
                std::cerr << "[Usb2CanfdBus] Failed to initialize usb_class for interface: " << interface << std::endl;
                continue;
            }

            usb_devices_[interface] = usb_dev;
            std::cout << "[Usb2CanfdBus] Initialized interface: " << interface << " with device: " << it->second << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[Usb2CanfdBus] Exception initializing interface " << interface << ": " << e.what() << std::endl;
        }
    }
}

bool Usb2CanfdBus::send(const GenericBusPacket& packet) {
    std::string interface = packet.interface;

    if (interface.empty() && !interface_names_.empty()) {
        interface = interface_names_[0];
    }

    auto it = usb_devices_.find(interface);
    if (it == usb_devices_.end()) {
        std::cerr << "[Usb2CanfdBus] Interface not found: " << interface << std::endl;
        return false;
    }

    can_tx_type frame_tx{};
    frame_tx.can_id = packet.id;
    frame_tx.can_type = 1;
    frame_tx.fram_type = 1;
    frame_tx.fd_acc = 1;
    frame_tx.id_type = 1;
    frame_tx.id_increase = 0;
    frame_tx.cmd = 0;
    frame_tx.send_time = 1;
    frame_tx.interval = 0;
    frame_tx.dlc = packet.len;
    std::memcpy(frame_tx.data, packet.data.data(), packet.len);

    it->second->set_tx_frame(&frame_tx);
    it->second->send_data();

    return true;
}

bool Usb2CanfdBus::receive(GenericBusPacket& packet) {
    // USB2CANFD设备的接收主要通过异步回调机制实现
    // 此方法作为备选接口，当前实现基于异步架构
    (void)packet;  // 显式标记参数已使用，避免编译警告
    return false;
}

void Usb2CanfdBus::async_receive(const std::function<void(const GenericBusPacket&)>& callback) {
    receive_callback_ = callback;
    running_ = true;

    for (const auto& interface : interface_names_) {
        auto it = usb_devices_.find(interface);
        if (it != usb_devices_.end()) {
            it->second->setFrameCallback([this, interface](can_value_type& value) {
                canfd_frame frame{};
                frame.can_id = value.head.id;
                frame.len = 8;
                std::memcpy(frame.data, value.data, 8);
                this->on_frame_received(interface, frame);
            });
            receive_threads_.emplace_back(&Usb2CanfdBus::receive_loop, this, interface);
        }
    }
}

std::vector<std::string> Usb2CanfdBus::get_interface_names() const {
    return interface_names_;
}

std::vector<std::string> Usb2CanfdBus::getDeviceSerialNumbers() const {
    std::vector<std::string> sns;
    for (const auto& pair : interface_to_sn_) {
        sns.push_back(pair.second);
    }
    return sns;
}

bool Usb2CanfdBus::isDeviceReady(const std::string& interface) const {
    auto it = usb_devices_.find(interface);
    if (it == usb_devices_.end()) {
        return false;
    }
    return it->second != nullptr;
}

std::string Usb2CanfdBus::getDeviceSerialNumber(const std::string& interface) const {
    auto it = interface_to_sn_.find(interface);
    if (it == interface_to_sn_.end()) {
        return "";
    }
    return it->second;
}

void Usb2CanfdBus::receive_loop(const std::string& interface) {
    // USB设备内部线程处理接收，此线程主要用于保持运行状态
    // 实际接收通过usb_class内部的接收线程和回调机制完成
    (void)interface;  // 显式标记参数已使用，避免编译警告
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Usb2CanfdBus::on_frame_received(const std::string& interface, const canfd_frame& frame) {
    if (!receive_callback_) {
        return;
    }

    GenericBusPacket packet;
    packet.interface = interface;
    packet.id = frame.can_id;
    packet.len = frame.len;
    packet.protocol_type = BusProtocolType::CAN_FD;
    std::memcpy(packet.data.data(), frame.data, frame.len);

    receive_callback_(packet);
}

}   // namespace bus
}   // namespace hardware_driver
