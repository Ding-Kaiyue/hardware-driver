/*********************************************************************
 * @file        iap_protocol.cpp
 * @brief       IAP固件更新协议实现文件
 *********************************************************************/

#include "iap_protocol.hpp"
#include <fstream>
#include <cstring>
#include <iostream>

namespace hardware_driver {
namespace iap_protocol {

// ========== 文件操作 ==========

bool load_firmware_from_file(const std::string& filename, std::vector<uint8_t>& firmware_data) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);

    if (!file.is_open()) {
        std::cerr << "[IAP] ❌ Cannot open firmware: " << filename << std::endl;
        return false;
    }

    std::streamsize file_size = file.tellg();
    if (file_size <= 0) {
        std::cerr << "[IAP] ❌ Firmware file empty\n";
        return false;
    }

    file.seekg(0, std::ios::beg);
    firmware_data.resize(file_size);
    file.read(reinterpret_cast<char*>(firmware_data.data()), file_size);
    file.close();

    std::cout << "[IAP] ✅ Firmware loaded: " << filename << " (" << file_size << " bytes)\n";
    return true;
}

// ========== 打包函数 ==========

bool pack_enter_iap_request(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len) {
    data[0] = 0x01;
    data[1] = 0x12;
    len = 2;
    return true;
}

bool pack_send_key(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len) {
    data[0] = 'k';
    data[1] = 'e';
    data[2] = 'y';
    len = 3;
    return true;
}


bool pack_firmware_frame(std::array<uint8_t, bus::MAX_BUS_DATA_SIZE>& data, size_t& len,
                         const std::vector<uint8_t>& firmware_data,
                         size_t offset, size_t chunk_size) {

    if (offset >= firmware_data.size()) {
        std::cerr << "[IAP] ⚠ Invalid firmware offset\n";
        return false;
    }

    size_t send_size = std::min(chunk_size, firmware_data.size() - offset);
    std::memcpy(data.data(), firmware_data.data() + offset, send_size);
    len = send_size;
    return true;
}

// ========== 反馈解析函数 ==========

std::optional<IAPFeedback> parse_iap_feedback(const bus::GenericBusPacket& packet) {
    // 检查CAN ID是否为IAP反馈格式（0xFF00 + motor_id）
    // IAP反馈的ID范围应该是 0xFF00-0xFFFF
    if ((packet.id & 0xFF00) != 0xFF00) {
        return std::nullopt;
    }

    // 检查数据包长度（至少4字节）
    if (packet.data.size() < 4) {
        return std::nullopt;
    }

    uint32_t ascii_code = (packet.data[0] << 24) |
                          (packet.data[1] << 16) |
                          (packet.data[2] << 8)  |
                           packet.data[3];

    IAPFeedback fb{};
    fb.status_msg = static_cast<IAPStatusMessage>(ascii_code);
    fb.motor_id   = packet.id & 0xFF;
    return fb;
}

std::string iap_status_to_string(IAPStatusMessage msg) {
    switch (msg) {
        case IAPStatusMessage::BS00: return "BS00";
        case IAPStatusMessage::BK01: return "BK01";
        case IAPStatusMessage::BK02: return "BK02";
        case IAPStatusMessage::BK03: return "BK03";
        case IAPStatusMessage::BD04: return "BD04";
        case IAPStatusMessage::BD05: return "BD05";
        case IAPStatusMessage::BJ06: return "BJ06";
        case IAPStatusMessage::BJ07: return "BJ07";
        case IAPStatusMessage::AS00: return "AS00";
        case IAPStatusMessage::AJ01: return "AJ01";
        default: return "UNKNOWN";
    }
}

}   // namespace iap_protocol
}   // namespace hardware_driver
