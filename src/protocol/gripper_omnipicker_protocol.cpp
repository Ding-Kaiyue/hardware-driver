#include "gripper_omnipicker_protocol.hpp"
#include <algorithm>

namespace hardware_driver {
namespace gripper_protocol {

//  Canfd485RawFrame 实现 
Canfd485RawFrame::Canfd485RawFrame(const uint8_t* data, size_t length, bool is_tx)
    : data_len_(0) {
    data_buffer_.fill(0);

    if (is_tx) {
        pack_tx_frame(data, length);
    } else {
        parse_rx_frame(data, length);
    }
}

void Canfd485RawFrame::parse_rx_frame(const uint8_t* data, size_t length) {
    if (length < MAX_EXTRA_FRAME_LEN || length > MAX_CANFD_LEN) {
        std::cerr << "[GripperProtocol] Invalid RX frame length: " << length << std::endl;
        return;
    }

    // 验证帧头
    if (data[0] != 0x55 || data[1] != 0xaa) {
        std::cerr << "[GripperProtocol] Invalid frame header" << std::endl;
        return;
    }

    uint8_t n = data[2];  // 数据长度
    size_t expected_len = n + MAX_EXTRA_FRAME_LEN;

    if (length < expected_len) {
        std::cerr << "[GripperProtocol] Incomplete frame, expected: " << expected_len
                  << ", got: " << length << std::endl;
        return;
    }

    // 计算校验和
    uint8_t checksum = 0;
    for (size_t i = 0; i <= static_cast<size_t>(2 + n); ++i) {
        checksum += data[i];
    }

    if (checksum != data[3 + n]) {
        std::cerr << "[GripperProtocol] Checksum mismatch" << std::endl;
        return;
    }

    // 验证帧尾
    if (data[4 + n] != 0xeb || data[5 + n] != 0xaa) {
        std::cerr << "[GripperProtocol] Invalid frame tail" << std::endl;
        return;
    }

    // 复制数据
    std::copy(data + 3, data + 3 + n, data_buffer_.begin());
    data_len_ = n;
}

void Canfd485RawFrame::pack_tx_frame(const uint8_t* data, size_t length) {
    if (length > MAX_CANFD_LEN) {
        std::cerr << "[GripperProtocol] TX frame length exceeds maximum: " << length << std::endl;
        return;
    }

    // 帧头
    data_buffer_[0] = 0x55;
    data_buffer_[1] = 0xaa;
    data_buffer_[2] = static_cast<uint8_t>(length);  // 数据长度

    // 数据
    std::copy(data, data + length, data_buffer_.begin() + 3);

    // 计算校验和
    uint8_t checksum = 0;
    for (size_t i = 0; i < 3 + length; ++i) {
        checksum += data_buffer_[i];
    }
    data_buffer_[3 + length] = checksum;

    // 帧尾
    data_buffer_[4 + length] = 0xeb;
    data_buffer_[5 + length] = 0xaa;

    data_len_ = length + MAX_EXTRA_FRAME_LEN;
}

//  OmniPickerFrame 实现 
OmniPickerFrame::OmniPickerFrame(uint8_t position, uint8_t speed, uint8_t force,
                                 uint8_t acc, uint8_t dec) {
    data_buffer_.fill(0);

    // 百分比转换为0-255
    uint8_t pos_255 = static_cast<uint8_t>(position * 255 / 100);
    uint8_t spd_255 = static_cast<uint8_t>(speed * 255 / 100);
    uint8_t frc_255 = static_cast<uint8_t>(force * 255 / 100);

    data_buffer_[0] = 0x01;      // 命令字
    data_buffer_[1] = 0x01;      // 子命令
    data_buffer_[2] = pos_255;   // 位置
    data_buffer_[3] = spd_255;   // 速度
    data_buffer_[4] = frc_255;   // 力
    data_buffer_[5] = acc;       // 加速度
    data_buffer_[6] = dec;       // 减速度
    data_buffer_[7] = 0x00;      // 保留
}

OmniPickerFrame::OmniPickerFrame(const uint8_t* data, uint8_t data_len) {
    data_buffer_.fill(0);

    if (data_len < 6) {
        std::cerr << "[GripperProtocol] OmniPicker frame length must be at least 6 bytes" << std::endl;
        return;
    }

    std::copy(data, data + std::min<size_t>(data_len, 8), data_buffer_.begin());

    // 解析状态
    OmniPickerStatus status = static_cast<OmniPickerStatus>(data[1]);
    switch (status) {
        case OmniPickerStatus::NORMAL:
            break;
        case OmniPickerStatus::OVERHEAT:
            std::cerr << "[GripperProtocol] Warning: OmniPicker is overheat!" << std::endl;
            break;
        case OmniPickerStatus::OVERSPEED:
            std::cerr << "[GripperProtocol] Warning: OmniPicker is overspeed!" << std::endl;
            break;
        case OmniPickerStatus::INIT_FAIL:
            std::cerr << "[GripperProtocol] Error: OmniPicker initialization failed!" << std::endl;
            break;
        case OmniPickerStatus::OVERLIMIT:
            std::cerr << "[GripperProtocol] Error: OmniPicker is overlimit!" << std::endl;
            break;
        default:
            std::cerr << "[GripperProtocol] Unknown OmniPicker status" << std::endl;
    }

    // 解析动作状态
    OmniPickerActionStatus action_status = static_cast<OmniPickerActionStatus>(data[2]);
    switch (action_status) {
        case OmniPickerActionStatus::REACHED_DES:
            std::cout << "[GripperProtocol] OmniPicker reached desired position" << std::endl;
            break;
        case OmniPickerActionStatus::GRIPPER_MOVING:
            std::cout << "[GripperProtocol] OmniPicker is moving" << std::endl;
            break;
        case OmniPickerActionStatus::GRIPPER_JAM:
            std::cerr << "[GripperProtocol] Warning: OmniPicker is jammed!" << std::endl;
            break;
        case OmniPickerActionStatus::OBJ_FALL:
            std::cerr << "[GripperProtocol] Warning: Object has fallen from OmniPicker!" << std::endl;
            break;
        default:
            std::cerr << "[GripperProtocol] Unknown OmniPicker action status" << std::endl;
    }
}

// PGCGripperFrame 实现 
PGCGripperFrame::PGCGripperFrame(uint8_t velocity, uint8_t effort, uint8_t position)
    : data_len_(8) {
    data_buffer_.fill(0);

    // 参数范围检查
    if (velocity < 1 || velocity > 100) {
        std::cerr << "[GripperProtocol] PGC velocity out of range (1-100): " << (int)velocity << std::endl;
    }
    if (effort < 20 || effort > 100) {
        std::cerr << "[GripperProtocol] PGC effort out of range (20-100): " << (int)effort << std::endl;
    }

    uint16_t pos_mm = position * 10;  // mm转换为0.1mm单位

    data_buffer_[0] = 0x02;                         // 命令字
    data_buffer_[1] = 0x01;                         // 子命令
    data_buffer_[2] = (velocity >> 8) & 0xFF;       // 速度高字节
    data_buffer_[3] = velocity & 0xFF;              // 速度低字节
    data_buffer_[4] = (effort >> 8) & 0xFF;         // 力高字节
    data_buffer_[5] = effort & 0xFF;                // 力低字节
    data_buffer_[6] = (pos_mm >> 8) & 0xFF;         // 位置高字节
    data_buffer_[7] = pos_mm & 0xFF;                // 位置低字节
}

PGCGripperFrame::PGCGripperFrame()
    : data_len_(2) {
    data_buffer_.fill(0);
    data_buffer_[0] = 0x02;  // 命令字
    data_buffer_[1] = 0x02;  // 查询命令
}

PGCGripperFrame::PGCGripperFrame(const uint8_t* data, uint8_t data_len)
    : data_len_(data_len) {
    data_buffer_.fill(0);

    if (data_len < 2) {
        std::cerr << "[GripperProtocol] PGC frame length must be at least 2 bytes" << std::endl;
        return;
    }

    std::copy(data, data + std::min<size_t>(data_len, 8), data_buffer_.begin());

    // 解析状态
    PGCGripperStatus status = static_cast<PGCGripperStatus>(data[1]);
    switch (status) {
        case PGCGripperStatus::GRIPPER_MOVING:
            std::cout << "[GripperProtocol] PGC Gripper is moving" << std::endl;
            break;
        case PGCGripperStatus::GRIPPER_STOP_NOTHING:
            std::cout << "[GripperProtocol] PGC Gripper stopped, nothing in gripper" << std::endl;
            break;
        case PGCGripperStatus::GRIPPER_STOP_SOMETHING:
            std::cout << "[GripperProtocol] PGC Gripper stopped, something in gripper" << std::endl;
            break;
        case PGCGripperStatus::GRIPPER_STOP_OBJ_FALL:
            std::cerr << "[GripperProtocol] Warning: Object has fallen from PGC Gripper!" << std::endl;
            break;
        default:
            std::cerr << "[GripperProtocol] Unknown PGC Gripper status" << std::endl;
    }
}

// GripperProtocolBuilder 实现 
std::vector<bus::GenericBusPacket> GripperProtocolBuilder::build_gripper_control(
    const std::string& interface,
    GripperType gripper_type,
    uint8_t position,
    uint8_t velocity,
    uint8_t effort,
    const uint8_t* raw_data,
    size_t raw_data_len) {

    std::vector<bus::GenericBusPacket> packets;

    switch (gripper_type) {
        case GripperType::OmniPicker: {
            // 1. 发送唤醒帧
            packets.push_back(build_wakeup_frame(interface, 0x01));

            // 2. 构造OmniPicker控制帧
            OmniPickerFrame frame(position, velocity, effort);
            bus::GenericBusPacket control_packet;
            control_packet.interface = interface;
            control_packet.id = SEND_GRIPPER_ID;
            control_packet.protocol_type = bus::BusProtocolType::CAN_FD;
            control_packet.len = 8;
            std::copy(frame.get_data_ptr(), frame.get_data_ptr() + 8,
                     control_packet.data.begin());
            packets.push_back(control_packet);
            break;
        }

        case GripperType::PGC_Gripper: {
            // 1. 发送唤醒帧
            packets.push_back(build_wakeup_frame(interface, 0x01));

            // 2. 构造PGC控制帧
            PGCGripperFrame frame(velocity, effort, position);
            bus::GenericBusPacket control_packet;
            control_packet.interface = interface;
            control_packet.id = SEND_GRIPPER_ID;
            control_packet.protocol_type = bus::BusProtocolType::CAN_FD;
            control_packet.len = frame.get_data_length();
            std::copy(frame.get_data_ptr(), frame.get_data_ptr() + frame.get_data_length(),
                     control_packet.data.begin());
            packets.push_back(control_packet);
            break;
        }

        case GripperType::Raw_Frame: {
            // 1. 发送模式切换帧
            packets.push_back(build_wakeup_frame(interface, 0x02));

            // 2. 封装原始数据
            if (raw_data && raw_data_len > 0) {
                Canfd485RawFrame raw_frame(raw_data, raw_data_len, true);
                bus::GenericBusPacket raw_packet;
                raw_packet.interface = interface;
                raw_packet.id = CANFD_2_RS485_ID;
                raw_packet.protocol_type = bus::BusProtocolType::CAN_FD;
                raw_packet.len = raw_frame.get_data_length();
                std::copy(raw_frame.get_data_ptr(),
                         raw_frame.get_data_ptr() + raw_frame.get_data_length(),
                         raw_packet.data.begin());
                packets.push_back(raw_packet);
            }
            break;
        }

        default:
            std::cerr << "[GripperProtocol] Unknown gripper type: "
                     << static_cast<int>(gripper_type) << std::endl;
            break;
    }

    return packets;
}

void GripperProtocolBuilder::parse_gripper_feedback(const bus::GenericBusPacket& packet) {
    // 根据CAN ID判断是哪种夹爪的反馈
    if (packet.id == RECV_GRIPPER_ID) {
        // 解析夹爪反馈数据
        if (packet.len >= 6) {
            // 根据第一个字节判断夹爪类型
            if (packet.data[0] == 0x01) {
                // OmniPicker反馈
                OmniPickerFrame feedback(packet.data.data(), packet.len);
            } else if (packet.data[0] == 0x02) {
                // PGC反馈
                PGCGripperFrame feedback(packet.data.data(), packet.len);
            }
        }
    }
}

bus::GenericBusPacket GripperProtocolBuilder::build_wakeup_frame(
    const std::string& interface, uint8_t mode) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = CANFD_2_RS485_ID;
    packet.protocol_type = bus::BusProtocolType::CAN_FD;
    packet.len = 8;
    packet.data.fill(0);
    packet.data[0] = mode;  // 0x01=普通模式, 0x02=原始帧模式
    return packet;
}

}  // namespace gripper_protocol
}  // namespace hardware_driver
