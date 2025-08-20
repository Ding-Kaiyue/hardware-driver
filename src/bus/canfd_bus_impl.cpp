#include "bus/canfd_bus_impl.hpp"

namespace hardware_driver {
namespace bus {

CanFdBus::CanFdBus(const std::vector<std::string>& interfaces, uint32_t arbitration_bitrate, uint32_t data_bitrate)
    : interface_names_(interfaces), 
      arbitration_bitrate_(arbitration_bitrate), 
      data_bitrate_(data_bitrate) 
{
    init();
}

CanFdBus::CanFdBus(const std::vector<std::string>& interfaces)
    : interface_names_(interfaces), 
      arbitration_bitrate_(DEFAULT_ARBITRATION_BITRATE), 
      data_bitrate_(DEFAULT_DATA_BITRATE) 
{
    init();
}

CanFdBus::~CanFdBus() {
    running_ = false;
    
    // 等待一小段时间让接收循环自然结束
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    for (auto& thread : receive_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    receive_threads_.clear();
    
    // 关闭所有socket连接
    for (auto& pair : interface_sockets_) {
        if (pair.second && *pair.second >= 0) {
            close(*pair.second);
        }
    }
    interface_sockets_.clear();
    
    // 额外的清理延时，避免内核队列溢出警告
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/**
 * @brief Init the CANFD bus parameters
 */
void CanFdBus::init() {
    // 加载Jetson Orin CAN内核模块
    // std::string modprobe_cmd = "modprobe can && modprobe can_raw && modprobe mttcan";
    // system(modprobe_cmd.c_str());
    
    for (const auto& interface_name : interface_names_) {
        // // 禁用接口
        // std::string disable_cmd = "ip link set " + interface_name + " down";
        // system(disable_cmd.c_str());
        
        // // 设置qlen
        // std::string qlen_cmd = "ip link set " + interface_name + " txqueuelen 1000";
        // system(qlen_cmd.c_str());

        // // 设置CAN FD参数
        // // std::string enable_cmd = "ip link set " + interface_name + " up type can bitrate " + 
        // //                         std::to_string(arbitration_bitrate_) + " dbitrate " + std::to_string(data_bitrate_) + 
        // //                         " fd on berr-reporting on sample-point 0.8 dsample-point 0.75 restart-ms 1000";
        std::string enable_cmd = "ip link set " + interface_name + " up type can bitrate " + 
                                std::to_string(arbitration_bitrate_) + " sample-point 0.8 dbitrate " + std::to_string(data_bitrate_)
                                + " dsample-point 0.75 fd on loopback off restart-ms 100";
        // system(enable_cmd.c_str());
        
        // 绑定socket
        try {
            interface_sockets_[interface_name] = bind_can_socket(interface_name);
        } catch (const std::exception& e) {
            std::cerr << "[CanFdBus] Warning: Failed to bind socket for interface " << interface_name 
                      << ": " << e.what() << std::endl;
            // 不抛出异常，继续处理其他接口
        }
    }
}

/**
 * @brief Bind the CANFD socket to the specified interface
 * 
 * @param interface_name The name of the CAN interface to bind to
 * @param enable_fd Enable CANFD support, default is true
 * @param enable_loopback Enable loopback mode, default is false
 */
CanFdBus::SocketPtr CanFdBus::bind_can_socket(const std::string& interface, bool enable_loopback) {
    // Create CAN socket
    int raw_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (raw_socket < 0) {
        throw std::runtime_error("Failed to create CAN socket for " + interface);
    }
   SocketPtr temp_sock(new int(raw_socket));

    // Set CANFD mode to non-blocking
    if (fcntl(*temp_sock, F_SETFL, O_NONBLOCK) < 0) {
        throw std::runtime_error("Failed to set CAN socket to non-blocking for " + interface);
    }

    // Get interface index
    struct ifreq ifr {};
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(*temp_sock, SIOCGIFINDEX, &ifr) < 0) {
        throw std::runtime_error("Failed to get CAN interface index for " + interface);
    }

    // Set loopback mode(can choose)
    int loopback = enable_loopback ? 1 : 0;
    if (setsockopt(*temp_sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0) {
        throw std::runtime_error("Failed to set CAN loopback mode for " + interface);
    }

    // Set CANFD mode(can choose)
    bool enable_fd = canfd_flags_.count(interface) ? canfd_flags_[interface] : true;
    int canfd_enable = enable_fd ? 1 : 0;
    if (setsockopt(*temp_sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_enable, sizeof(canfd_enable)) < 0) {
        throw std::runtime_error("Failed to set CAN FD mode for " + interface);
    }

    // bind to the CAN interface selected
    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(*temp_sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        throw std::runtime_error("Failed to bind to CAN interface " + interface);
    }
    return temp_sock;
}

bool CanFdBus::send(const bus::GenericBusPacket& packet) {
    auto it = interface_sockets_.find(packet.interface);
    if (it == interface_sockets_.end()) {
        throw std::runtime_error("CAN interface " + packet.interface + " not found");
    }

    int sock = *(it->second);

    const bool use_canfd = canfd_flags_.count(packet.interface) ? canfd_flags_.at(packet.interface) : true;
    const bool use_extended = extended_frame_flags_.count(packet.interface) ? extended_frame_flags_.at(packet.interface) : true;
    const uint32_t id = packet.id;
    
    if (use_canfd) {
        if (packet.len > CANFD_MAX_DLEN) {
            throw std::runtime_error("CAN FD message too large (" + std::to_string(packet.len) + " > 64 bytes)");
        }
        struct canfd_frame frame {};
        frame.len = static_cast<__u8>(packet.len);
        frame.can_id = use_extended ? (id | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
        frame.flags = CANFD_FDF;
        std::memcpy(frame.data, packet.data.data(), packet.len);
        return ::send(sock, &frame, sizeof(frame), 0) == sizeof(frame);
    } else {
        if (packet.len > CAN_MAX_DLEN) {
            throw std::runtime_error("Standard CAN message too large (" + std::to_string(packet.len) + " > 8 bytes)");
        }
        struct can_frame frame {};
        frame.can_id = use_extended ? (id | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
        frame.can_dlc = static_cast<__u8>(packet.len);
        std::memcpy(frame.data, packet.data.data(), packet.len);
        return ::send(sock, &frame, sizeof(frame), 0) == sizeof(frame);
    }
}

// bool CanFdBus::receive(const std::string& interface, uint32_t& id, std::vector<uint8_t>& data) {
//     auto it = interface_sockets_.find(interface);
//     if (it == interface_sockets_.end()) {
//         throw std::runtime_error("CAN interface " + interface + " not found");
//     }

//     int sock = *(it->second);

//     const bool use_canfd = canfd_flags_.count(interface) ? canfd_flags_.at(interface) : true;
//     if (use_canfd) {
//         // Receive CAN FD message
//         struct canfd_frame frame {};
//         ssize_t recv_size = ::recv(sock, &frame, sizeof(frame), 0);
//         if (recv_size < 0) {
//             throw std::runtime_error("Failed to receive CAN FD message on " + interface);
//         }
//         if (recv_size != CANFD_MTU) {
//             throw std::runtime_error("Invalid CAN FD frame size (" + std::to_string(recv_size) + " bytes) on interface '" + interface + "'");
//         }
//         id = frame.can_id & (frame.can_id & CAN_EFF_FLAG ? CAN_EFF_MASK : CAN_SFF_MASK);
//         data.assign(frame.data, frame.data + frame.len);
//     } else {
//         // Receive CAN message
//         struct can_frame frame {};
//         ssize_t recv_size = ::recv(sock, &frame, sizeof(frame), 0);
//         if (recv_size < 0) {
//             throw std::runtime_error("Failed to receive CAN message on " + interface);
//         }
//         if (recv_size != CAN_MTU) {
//             throw std::runtime_error("Invalid CAN frame size (" + std::to_string(recv_size) + " bytes) on interface '" + interface + "'");
//         }
//         id = frame.can_id & (frame.can_id & CAN_EFF_FLAG ? CAN_EFF_MASK : CAN_SFF_MASK);
//         data.assign(frame.data, frame.data + frame.can_dlc);
//     }
//     return true;
// }

bool CanFdBus::receive(bus::GenericBusPacket& packet) {
    auto it = interface_sockets_.find(packet.interface);
    if (it == interface_sockets_.end()) {
        throw std::runtime_error("CAN interface '" + packet.interface + "' not found");
    }
    int sock = *(it->second);
    const bool use_canfd = canfd_flags_.count(packet.interface) ? canfd_flags_.at(packet.interface) : true;

    if (use_canfd) {
        struct canfd_frame frame {};
        ssize_t recv_size = ::recv(sock, &frame, sizeof(frame), 0);
        if (recv_size < 0) {
            // 在非阻塞模式下，EAGAIN 或 EWOULDBLOCK 表示没有数据，是正常情况
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return false;
            }
            std::cerr << "[CanFdBus] Warning: No CAN FD frame received on interface '" << packet.interface << "'" << std::endl;
        }
        if (recv_size != sizeof(struct canfd_frame)) {
            std::cerr << "[CanFdBus] Warning: Invalid CAN FD frame size on interface '" << packet.interface << "'" << std::endl;
        }

        packet.id = frame.can_id & (frame.can_id & CAN_EFF_FLAG ? CAN_EFF_MASK : CAN_SFF_MASK);
        packet.len = frame.len;
        packet.protocol_type = bus::BusProtocolType::CAN_FD; // **设置协议类型**
        std::memcpy(packet.data.data(), frame.data, frame.len);
    } else {
        struct can_frame frame {};
        ssize_t recv_size = ::recv(sock, &frame, sizeof(frame), 0);
        if (recv_size < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return false;
            }
            std::cerr << "[CanFdBus] Warning: No CAN frame received on interface '" << packet.interface << "'" << std::endl;
        }
        if (recv_size != sizeof(struct can_frame)) {
            std::cerr << "[CanFdBus] Warning: Invalid CAN frame size on interface '" << packet.interface << "'" << std::endl;
        }

        packet.id = frame.can_id & (frame.can_id & CAN_EFF_FLAG ? CAN_EFF_MASK : CAN_SFF_MASK);
        packet.len = frame.can_dlc;
        packet.protocol_type = bus::BusProtocolType::CAN; // **设置协议类型**
        std::memcpy(packet.data.data(), frame.data, frame.can_dlc);
    }

    return true;
}

std::vector<std::string> CanFdBus::get_interface_names() const{
    return interface_names_;
}

void CanFdBus::set_extended_frame(const std::string& interface, bool use_extended) {
    // Check if the interface exists
    auto it = interface_sockets_.find(interface);
    if (it == interface_sockets_.end()) {
        throw std::runtime_error("CAN interface " + interface + " not found");
    }

    // Set CAN frame format
    extended_frame_flags_[interface] = use_extended;
}

void CanFdBus::set_fd_mode(const std::string& interface, bool use_fd) {
    // Check if the interface exists
    auto it = interface_sockets_.find(interface);
    if (it == interface_sockets_.end()) {
        throw std::runtime_error("CAN interface " + interface + " not found");
    }

    // Set CAN FD mode
    canfd_flags_[interface] = use_fd;
}

void CanFdBus::async_receive(const std::function<void(const bus::GenericBusPacket&)>& callback) {
    receive_callback_ = callback;
    running_ = true;
    // 为每个接口启动一个线程，用于接收数据
    for (const auto& interface : interface_names_) {
        receive_threads_.emplace_back(&CanFdBus::receive_loop, this, interface);
    }
}

void CanFdBus::receive_loop(const std::string& interface) {
    while (running_) {
        bus::GenericBusPacket packet;
        packet.interface = interface;
        if (receive(packet)) {
            if (receive_callback_) {
                receive_callback_(packet);
            }
        } else {
            // 没有数据，避免CPU空转
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

}   // namespace canfd_bus
}    // namespace hardware_driver
