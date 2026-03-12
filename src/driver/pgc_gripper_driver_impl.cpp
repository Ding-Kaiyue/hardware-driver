/*********************************************************************
 * @file        pgc_gripper_driver_impl.cpp
 * @brief       PGC夹爪驱动实现
 *********************************************************************/

#include "pgc_gripper_driver_impl.hpp"
#include <iostream>
#include <algorithm>
#include <cstring>

namespace hardware_driver {
namespace gripper_driver {

PGCGripperDriverImpl::PGCGripperDriverImpl(std::shared_ptr<bus::BusInterface> bus)
    : bus_(std::move(bus)) {
    if (!bus_) {
        throw std::runtime_error("[PGCGripperDriver] Bus interface is nullptr");
    }

    running_ = true;

    // 注册异步接收回调 - 直接在总线层处理，不启动单独的接收线程
    bus_->async_receive([this](const bus::GenericBusPacket& packet) {
        // 只处理夹爪相关的CAN ID (0x6F = RECV_GRIPPER_ID)
        if (packet.id == 0x6F) {
            this->receive_callback(packet);
        }
    });

    std::cout << "[PGCGripperDriver] Driver initialized successfully" << std::endl;
}

PGCGripperDriverImpl::~PGCGripperDriverImpl() {
    stop_receive();
    std::cout << "[PGCGripperDriver] Driver destroyed" << std::endl;
}

void PGCGripperDriverImpl::control_gripper(const std::string& interface,
                                          GripperType gripper_type,
                                          uint8_t position,
                                          uint8_t velocity,
                                          uint8_t effort) {
    if (gripper_type != GripperType::PGC_Gripper) {
        std::cerr << "[PGCGripperDriver] This driver only supports PGC gripper type" << std::endl;
        return;
    }

    // 构建PGC控制帧
    auto frame_data = build_pgc_control_frame(velocity, effort, position);

    // 构建CAN-FD数据包
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x5F;  // SEND_GRIPPER_ID
    packet.len = frame_data.size();
    std::copy(frame_data.begin(), frame_data.end(), packet.data.begin());

    if (!send_packet(packet)) {
        std::cerr << "[PGCGripperDriver] Failed to send control command" << std::endl;
    } else {
        std::cout << "[PGCGripperDriver] Control command sent: pos=" << (int)position
                  << "% vel=" << (int)velocity << "% effort=" << (int)effort << "%" << std::endl;
    }
}

void PGCGripperDriverImpl::send_raw_data(const std::string& interface,
                                        const uint8_t* raw_data,
                                        size_t raw_data_len) {
    if (!raw_data || raw_data_len == 0) {
        std::cerr << "[PGCGripperDriver] Invalid raw data" << std::endl;
        return;
    }

    // 直接使用CAN-FD格式封装
    auto frame_data = wrap_with_canfd_frame(
        std::vector<uint8_t>(raw_data, raw_data + raw_data_len)
    );

    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x5F;  // SEND_GRIPPER_ID
    packet.len = frame_data.size();
    std::copy(frame_data.begin(), frame_data.end(), packet.data.begin());

    if (!send_packet(packet)) {
        std::cerr << "[PGCGripperDriver] Failed to send raw data" << std::endl;
    }
}

void PGCGripperDriverImpl::open_gripper(const std::string& interface,
                                       GripperType gripper_type,
                                       uint8_t velocity,
                                       uint8_t effort) {
    // 打开夹爪 = 位置设置为0（最大张开）
    control_gripper(interface, gripper_type, 0, velocity, effort);
}

void PGCGripperDriverImpl::close_gripper(const std::string& interface,
                                        GripperType gripper_type,
                                        uint8_t velocity,
                                        uint8_t effort) {
    // 关闭夹爪 = 位置设置为100（完全闭合）
    control_gripper(interface, gripper_type, 100, velocity, effort);
}

void PGCGripperDriverImpl::add_observer(std::shared_ptr<GripperStatusObserver> observer) {
    if (!observer) {
        std::cerr << "[PGCGripperDriver] Cannot add nullptr observer" << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.push_back(observer);
    std::cout << "[PGCGripperDriver] Observer added, total: " << observers_.size() << std::endl;
}

void PGCGripperDriverImpl::remove_observer(std::shared_ptr<GripperStatusObserver> observer) {
    if (!observer) {
        return;
    }

    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.erase(
        std::remove(observers_.begin(), observers_.end(), observer),
        observers_.end()
    );
    std::cout << "[PGCGripperDriver] Observer removed, total: " << observers_.size() << std::endl;
}

void PGCGripperDriverImpl::start_receive() {
    std::cout << "[PGCGripperDriver] Receive already started in constructor" << std::endl;
}

void PGCGripperDriverImpl::stop_receive() {
    std::cout << "[PGCGripperDriver] Stopping receive..." << std::endl;
    running_ = false;
}

void PGCGripperDriverImpl::query_gripper_status(const std::string& interface) {
    // 构建状态查询帧
    auto frame_data = build_pgc_status_query_frame();

    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x5F;  // SEND_GRIPPER_ID
    packet.len = frame_data.size();
    std::copy(frame_data.begin(), frame_data.end(), packet.data.begin());

    if (!send_packet(packet)) {
        std::cerr << "[PGCGripperDriver] Failed to send status query" << std::endl;
    } else {
        std::cout << "[PGCGripperDriver] Status query sent" << std::endl;
    }
}

void PGCGripperDriverImpl::receive_callback(const bus::GenericBusPacket& packet) {
    // 只处理夹爪相关的CAN ID
    if (packet.id == 0x6F) {  // RECV_GRIPPER_ID
        // 解析PGC反馈
        GripperStatus status = parse_pgc_status(packet.data.data(), packet.len);

        // 通知观察者
        notify_observers(packet.interface, 1, status);  // 假设gripper_id=1

        // 更新缓存
        {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            status_cache_[packet.interface][1] = status;
        }
    }
}

void PGCGripperDriverImpl::notify_observers(const std::string& interface,
                                           uint32_t gripper_id,
                                           const GripperStatus& status) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    for (auto& observer : observers_) {
        if (observer) {
            observer->on_gripper_status_update(interface, gripper_id, status);
        }
    }
}

bool PGCGripperDriverImpl::send_packet(const bus::GenericBusPacket& packet) {
    return bus_->send(packet);
}

std::vector<uint8_t> PGCGripperDriverImpl::build_pgc_control_frame(uint8_t velocity,
                                                                   uint8_t effort,
                                                                   uint8_t position) {
    std::vector<uint8_t> frame;

    // 参数验证和范围转换
    if (velocity < 1) velocity = 1;
    if (velocity > 100) velocity = 100;
    if (effort < 20) effort = 20;
    if (effort > 100) effort = 100;
    if (position > 100) position = 100;

    // 位置转换：百分比 -> 10倍（0-1000）
    uint16_t pos_value = position * 10;

    // 构建PGC数据包：[0x02, 0x01, velocity_h, velocity_l, effort_h, effort_l, pos_h, pos_l]
    std::vector<uint8_t> pgc_data = {
        PGC_CONTROL_CMD,                              // 0x02
        0x01,                                          // 控制命令标志
        static_cast<uint8_t>(velocity >> 8),         // 速度高字节
        static_cast<uint8_t>(velocity & 0xFF),       // 速度低字节
        static_cast<uint8_t>(effort >> 8),           // 力量高字节
        static_cast<uint8_t>(effort & 0xFF),         // 力量低字节
        static_cast<uint8_t>(pos_value >> 8),        // 位置高字节
        static_cast<uint8_t>(pos_value & 0xFF)       // 位置低字节
    };

    // 使用CAN-FD帧格式封装
    frame = wrap_with_canfd_frame(pgc_data);

    return frame;
}

std::vector<uint8_t> PGCGripperDriverImpl::build_pgc_status_query_frame() {
    std::vector<uint8_t> pgc_data = {
        PGC_STATUS_QUERY,  // 0x02
        0x02               // 状态查询标志
    };

    return wrap_with_canfd_frame(pgc_data);
}

std::vector<uint8_t> PGCGripperDriverImpl::wrap_with_canfd_frame(
    const std::vector<uint8_t>& data) {
    std::vector<uint8_t> frame;

    frame.push_back(CANFD_HEADER_0);      // 0x55
    frame.push_back(CANFD_HEADER_1);      // 0xaa
    frame.push_back(data.size());         // 数据长度

    // 计算校验和（包括长度字节）
    uint8_t checksum = CANFD_HEADER_0 + CANFD_HEADER_1 + data.size();

    // 复制数据并计算校验和
    for (uint8_t byte : data) {
        frame.push_back(byte);
        checksum += byte;
    }

    frame.push_back(checksum);             // 校验和
    frame.push_back(CANFD_TAIL_0);         // 0xeb
    frame.push_back(CANFD_TAIL_1);         // 0xaa

    return frame;
}

GripperStatus PGCGripperDriverImpl::parse_pgc_status(const uint8_t* data, size_t data_len) {
    GripperStatus status = {};

    if (!data || data_len < 2) {
        std::cerr << "[PGCGripperDriver] Invalid status data length" << std::endl;
        return status;
    }

    // PGC状态字节索引为1（第一个字节是0x02）
    uint8_t status_byte = data[1];

    enum PGCStatus : uint8_t {
        GRIPPER_MOVING = 0x00,           // 夹爪移动中
        GRIPPER_STOP_NOTHING = 0x01,     // 夹爪停止，无物体
        GRIPPER_STOP_SOMETHING = 0x02,   // 夹爪停止，有物体
        GRIPPER_STOP_OBJ_FALL = 0x03     // 物体掉落
    };

    status.status = status_byte;

    switch (status_byte) {
        case GRIPPER_MOVING:
            status.is_moving = true;
            status.has_object = false;
            std::cout << "[PGCGripperDriver] Status: Gripper is moving" << std::endl;
            break;

        case GRIPPER_STOP_NOTHING:
            status.is_moving = false;
            status.has_object = false;
            std::cout << "[PGCGripperDriver] Status: Gripper stopped, no object" << std::endl;
            break;

        case GRIPPER_STOP_SOMETHING:
            status.is_moving = false;
            status.has_object = true;
            std::cout << "[PGCGripperDriver] Status: Gripper stopped, object detected" << std::endl;
            break;

        case GRIPPER_STOP_OBJ_FALL:
            status.is_moving = false;
            status.has_object = false;
            std::cerr << "[PGCGripperDriver] Warning: Object has fallen!" << std::endl;
            break;

        default:
            std::cerr << "[PGCGripperDriver] Unknown status: 0x" << std::hex
                     << (int)status_byte << std::dec << std::endl;
            break;
    }

    return status;
}

}  // namespace gripper_driver
}  // namespace hardware_driver
