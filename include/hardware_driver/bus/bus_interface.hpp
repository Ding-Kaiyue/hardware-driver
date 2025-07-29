#ifndef __BUS_INTERFACE_HPP__
#define __BUS_INTERFACE_HPP__

#include <string>
#include <array>
#include <vector>
#include <cstdint>
#include <functional>

namespace hardware_driver {
namespace bus {

// 新增：定义总线协议类型
enum class BusProtocolType {
    UNKNOWN,
    CAN,
    CAN_FD,
    ETHERCAT
};

constexpr size_t MAX_BUS_DATA_SIZE = 64;     ///< 最大总线数据大小

/**
 * @brief 通用总线数据包结构体，用于发送和接收数据
 */
struct GenericBusPacket {
    std::string interface;         ///< 总线接口名称，如 "can0"、"ethercat0" 等
    uint32_t id;                   ///< 帧 ID 或主地址(取决于总线类型)
    std::array<uint8_t, MAX_BUS_DATA_SIZE> data;      ///< 数据
    size_t len;                    ///< 数据长度
    BusProtocolType protocol_type; ///< 协议类型

    // 使用默认构造函数并初始化成员
    GenericBusPacket() : id(0), len(0), protocol_type(BusProtocolType::UNKNOWN) {}
};

class BusInterface {
public:
    virtual ~BusInterface() = default;

    virtual void init() = 0;

    virtual bool send(const GenericBusPacket& packet) = 0;

    virtual bool receive(GenericBusPacket& packet) = 0;
    
    virtual void async_receive(const std::function<void(const GenericBusPacket&)>& callback) = 0;

    virtual std::vector<std::string> get_interface_names() const = 0;

};

}   // namespace bus
}   // namespace hardware_driver

#endif   // __BUS_INTERFACE_HPP__
