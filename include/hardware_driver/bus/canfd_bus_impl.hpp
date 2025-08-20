#ifndef __CANFD_BUS_IMPL_HPP__
#define __CANFD_BUS_IMPL_HPP__

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <thread>
#include <atomic>
#include <functional>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <linux/can/netlink.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <net/if.h>
#include <unistd.h>
#include <stdexcept>
#include <cstring>
#include <iostream>
#include "hardware_driver/bus/bus_interface.hpp"

namespace hardware_driver {
namespace bus {

struct SocketDeleter {
    void operator()(int* sock) const {
        if (sock && *sock >= 0) {
            ::close(*sock);
        }
        delete sock;
    }
};

class CanFdBus : public BusInterface {
public:
    // 默认波特率常量
    static constexpr uint32_t DEFAULT_ARBITRATION_BITRATE = 1000000;  // 1 Mbps
    static constexpr uint32_t DEFAULT_DATA_BITRATE = 5000000;         // 5 Mbps
    
    // 构造函数重载
    CanFdBus(const std::vector<std::string>& interfaces, uint32_t arbitration_bitrate, uint32_t data_bitrate);
    CanFdBus(const std::vector<std::string>& interfaces); // 使用默认波特率
    ~CanFdBus();

    void init() override;
    bool send(const GenericBusPacket& packet) override;
    bool receive(GenericBusPacket& packet) override;
    void async_receive(const std::function<void(const GenericBusPacket&)>& callback) override;
    
    std::vector<std::string> get_interface_names() const override;

    void set_extended_frame(const std::string& interface, bool use_extended);
    void set_fd_mode(const std::string& interface, bool use_fd);

private:
    using SocketPtr = std::unique_ptr<int, SocketDeleter>;
    SocketPtr bind_can_socket(const std::string& interface, bool enable_loopback = false);
    
    void receive_loop(const std::string& interface);
private:
    std::unordered_map<std::string, SocketPtr> interface_sockets_;
    std::unordered_map<std::string, bool> extended_frame_flags_;    // extended frame flag for each interface
    std::unordered_map<std::string, bool> canfd_flags_;            // canfd flag for each interface
    std::vector<std::string> interface_names_;
    uint32_t arbitration_bitrate_;
    uint32_t data_bitrate_;

    std::function<void(const GenericBusPacket&)> receive_callback_;
    std::vector<std::thread> receive_threads_;
    std::atomic<bool> running_{false};

};

}   // namespace bus
}   // namespace hardware_driver


#endif   // __CANFD_BUS_IMPL_HPP__
 