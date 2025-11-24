#ifndef __HARDWARE_DRIVER_GRIPPER_OMNIPICKER_PROTOCOL_HPP__
#define __HARDWARE_DRIVER_GRIPPER_OMNIPICKER_PROTOCOL_HPP__

#include <cstdint>
#include <array>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <iostream>
#include "hardware_driver/bus/bus_interface.hpp"

namespace hardware_driver {
namespace gripper_protocol {

// CAN ID 定义 
constexpr uint32_t CANFD_2_RS485_ID = 0x3F;  ///< CANFD到RS485转换器ID
constexpr uint32_t RS485_2_CANFD_ID = 0x4F;  ///< RS485到CANFD转换器ID
constexpr uint32_t SEND_GRIPPER_ID  = 0x5F;  ///< 发送夹爪控制指令ID
constexpr uint32_t RECV_GRIPPER_ID  = 0x6F;  ///< 接收夹爪反馈ID

// 夹爪类型枚举 
enum class GripperType : uint8_t {
    OmniPicker  = 0,  ///< 智元
    PGC_Gripper = 1,  ///< 大還
    Raw_Frame   = 2,  ///< 透传数据
};

// OmniPicker 夹爪状态枚举 
enum class OmniPickerStatus : uint8_t {
    NORMAL      = 0x00,  ///< 正常
    OVERHEAT    = 0x01,  ///< 过热
    OVERSPEED   = 0x02,  ///< 超速
    INIT_FAIL   = 0x03,  ///< 初始化失败
    OVERLIMIT   = 0x04   ///< 超限
};

//OmniPicker夹爪动作状态
enum class OmniPickerActionStatus : uint8_t {
    REACHED_DES     = 0x00,  ///< 到达目标位置
    GRIPPER_MOVING  = 0x01,  ///< 夹爪移动中
    GRIPPER_JAM     = 0x02,  ///< 夹爪堵转
    OBJ_FALL        = 0x03   ///< 物体掉落
};

// PGC 夹爪状态枚举 
enum class PGCGripperStatus : uint8_t {
    GRIPPER_MOVING          = 0x00,  ///< 夹爪移动中
    GRIPPER_STOP_NOTHING    = 0x01,  ///< 夹爪停止且无物体
    GRIPPER_STOP_SOMETHING  = 0x02,  ///< 夹爪停止且有物体
    GRIPPER_STOP_OBJ_FALL   = 0x03   ///< 物体掉落
};

// CANFD-RS485 原始帧封装 
class Canfd485RawFrame {
public:
    static constexpr size_t MAX_CANFD_LEN = 64;        ///< 最大CANFD数据长度
    static constexpr size_t MAX_EXTRA_FRAME_LEN = 6;   ///< 帧头尾额外长度

    /**
     * @brief 构造函数
     * @param data 数据指针
     * @param length 数据长度
     * @param is_tx true=发送帧(打包), false=接收帧(解包)
     */
    Canfd485RawFrame(const uint8_t* data = nullptr, size_t length = 0, bool is_tx = false);

    /**
     * @brief 获取帧数据指针
     */
    const uint8_t* get_data_ptr() const { return data_buffer_.data(); }

    /**
     * @brief 获取帧数据长度
     */
    size_t get_data_length() const { return data_len_; }

private:
    std::array<uint8_t, MAX_CANFD_LEN + MAX_EXTRA_FRAME_LEN> data_buffer_;
    size_t data_len_;

    void parse_rx_frame(const uint8_t* data, size_t length);
    void pack_tx_frame(const uint8_t* data, size_t length);
};

// OmniPicker 夹爪协议帧 
class OmniPickerFrame {
public:
    /**
     * @brief 构造发送帧
     * @param position 位置 (0-100%)
     * @param speed 速度 (0-100%)
     * @param force 力 (0-100%)
     * @param acc 加速度 (默认0xFF=最大)
     * @param dec 减速度 (默认0xFF=最大)
     */
    OmniPickerFrame(uint8_t position, uint8_t speed, uint8_t force,
                    uint8_t acc = 0xFF, uint8_t dec = 0xFF);

    /**
     * @brief 构造接收帧（解析反馈）
     * @param data 接收数据指针
     * @param data_len 数据长度
     */
    OmniPickerFrame(const uint8_t* data, uint8_t data_len);

    /**
     * @brief 获取帧数据指针
     */
    const uint8_t* get_data_ptr() const { return data_buffer_.data(); }

    /**
     * @brief 获取位置反馈 (0-255)
     */
    uint8_t get_position() const { return data_buffer_[3]; }

    /**
     * @brief 获取速度反馈 (0-255)
     */
    uint8_t get_speed() const { return data_buffer_[4]; }

    /**
     * @brief 获取力反馈 (0-255)
     */
    uint8_t get_force() const { return data_buffer_[5]; }

    /**
     * @brief 获取夹爪状态
     */
    OmniPickerStatus get_status() const { return static_cast<OmniPickerStatus>(data_buffer_[1]); }

    /**
     * @brief 获取动作状态
     */
    OmniPickerActionStatus get_action_status() const {
        return static_cast<OmniPickerActionStatus>(data_buffer_[2]);
    }

private:
    std::array<uint8_t, 8> data_buffer_;
};

// PGC 夹爪协议帧 
class PGCGripperFrame {
public:
    /**
     * @brief 构造控制帧
     * @param velocity 速度 (1-100)
     * @param effort 力 (20-100)
     * @param position 位置 (0-100mm)
     */
    PGCGripperFrame(uint8_t velocity, uint8_t effort, uint8_t position);

    /**
     * @brief 构造查询帧
     */
    PGCGripperFrame();

    /**
     * @brief 构造接收帧（解析反馈）
     * @param data 接收数据指针
     * @param data_len 数据长度
     */
    PGCGripperFrame(const uint8_t* data, uint8_t data_len);

    /**
     * @brief 获取帧数据指针
     */
    const uint8_t* get_data_ptr() const { return data_buffer_.data(); }

    /**
     * @brief 获取数据长度
     */
    size_t get_data_length() const { return data_len_; }

    /**
     * @brief 获取夹爪状态
     */
    PGCGripperStatus get_status() const {
        return static_cast<PGCGripperStatus>(data_buffer_[1]);
    }

private:
    std::array<uint8_t, 8> data_buffer_;
    size_t data_len_;
};

// 协议构造器类 负责根据夹爪类型构造对应的CAN总线数据包
class GripperProtocolBuilder {
public:
    /**
     * @brief 构造夹爪控制数据包
     * @param interface 总线接口名称 (如"can0")
     * @param gripper_type 夹爪类型
     * @param position 位置 (0-100)
     * @param velocity 速度 (0-100)
     * @param effort 力 (0-100)
     * @param raw_data 原始数据指针（仅Raw_Frame模式使用）
     * @param raw_data_len 原始数据长度（仅Raw_Frame模式使用）
     * @return CAN总线数据包向量（可能包含唤醒帧+控制帧）
     */
    static std::vector<bus::GenericBusPacket> build_gripper_control(
        const std::string& interface,
        GripperType gripper_type,
        uint8_t position,
        uint8_t velocity,
        uint8_t effort,
        const uint8_t* raw_data = nullptr,
        size_t raw_data_len = 0);

    /**
     * @brief 解析夹爪反馈数据包
     * @param packet 接收到的CAN数据包
     * @return 解析结果（具体类型取决于夹爪类型）
     */
    static void parse_gripper_feedback(const bus::GenericBusPacket& packet);

private:
    /**
     * @brief 构造唤醒帧
     */
    static bus::GenericBusPacket build_wakeup_frame(const std::string& interface, uint8_t mode);
};

}  // namespace gripper_protocol
}  // namespace hardware_driver

#endif  // __HARDWARE_DRIVER_GRIPPER_OMNIPICKER_PROTOCOL_HPP__
