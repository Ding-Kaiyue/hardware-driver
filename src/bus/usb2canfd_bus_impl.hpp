#ifndef __USB2CANFD_BUS_IMPL_HPP__
#define __USB2CANFD_BUS_IMPL_HPP__

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <thread>
#include <atomic>
#include <functional>
#include <linux/can.h>
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <chrono>
#include "hardware_driver/bus/bus_interface.hpp"

class usb_class;

namespace hardware_driver {
namespace bus {

class Usb2CanfdBus : public BusInterface {
public:
    static constexpr uint32_t DEFAULT_ARBITRATION_BITRATE = 1000000;
    static constexpr uint32_t DEFAULT_DATA_BITRATE = 5000000;

    Usb2CanfdBus(const std::vector<std::string>& devices, uint32_t arbitration_bitrate, uint32_t data_bitrate);
    Usb2CanfdBus(const std::vector<std::string>& devices);
    ~Usb2CanfdBus();

    void init() override;
    bool send(const GenericBusPacket& packet) override;
    bool receive(GenericBusPacket& packet) override;
    void async_receive(const std::function<void(const GenericBusPacket&)>& callback) override;

    std::vector<std::string> get_interface_names() const override;

    std::vector<std::string> getDeviceSerialNumbers() const;
    std::string getDeviceSerialNumber(const std::string& interface) const;
    bool isDeviceReady(const std::string& interface) const;

private:
    void receive_loop(const std::string& interface);
    void on_frame_received(const std::string& interface, const canfd_frame& frame);

private:
    std::unordered_map<std::string, std::shared_ptr<usb_class>> usb_devices_;
    std::unordered_map<std::string, std::string> interface_to_sn_;
    std::vector<std::string> interface_names_;
    uint32_t arbitration_bitrate_;
    uint32_t data_bitrate_;

    std::function<void(const GenericBusPacket&)> receive_callback_;
    std::vector<std::thread> receive_threads_;
    std::atomic<bool> running_{false};
};

}   // namespace bus
}   // namespace hardware_driver

#endif   // __USB2CANFD_BUS_IMPL_HPP__
