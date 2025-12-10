/*********************************************************************
 * @file        button_driver_impl.hpp
 * @brief       按键驱动CAN总线实现
 *********************************************************************/

#ifndef __BUTTON_DRIVER_IMPL_HPP__
#define __BUTTON_DRIVER_IMPL_HPP__

#include "hardware_driver/driver/button_driver_interface.hpp"
#include "hardware_driver/bus/bus_interface.hpp"
#include <vector>
#include <mutex>
#include <atomic>
#include <cstring>

namespace hardware_driver {
namespace button_driver {

// CAN协议常量
constexpr uint32_t BUTTON_RX_CAN_ID = 0x8F;  ///< 接收按键事件的CAN ID
constexpr uint32_t BUTTON_TX_CAN_ID = 0x7F;  ///< 发送LED控制的CAN ID

/// 协议码映射结构
struct ProtocolEntry {
    char code[5];        ///< 4字节ASCII码 + '\0'
    ButtonStatus status; ///< 对应的按键状态
};

/// 协议码映射表
inline const ProtocolEntry PROTOCOL_TABLE[] = {
    {"JRSJ", ButtonStatus::ENTRY_TEACH},   ///< 进入示教 (Jin Ru Shi Jiao)
    {"TCSJ", ButtonStatus::EXIT_TEACH},    ///< 退出示教 (Tui Chu Shi Jiao)
    {"GJFX", ButtonStatus::TEACH_REPEAT},  ///< 轨迹复现 (Gui Ji Fu Xian)
};
constexpr size_t PROTOCOL_COUNT = sizeof(PROTOCOL_TABLE) / sizeof(PROTOCOL_TABLE[0]);

/// 复现完成信号: "FXJS" (Fu Xian Jie Shu)
constexpr uint8_t REPLAY_COMPLETE_CODE[4] = {0x46, 0x58, 0x4A, 0x53};

/**
 * @brief 按键驱动CAN总线实现
 *
 * 通过motor_driver转发的CAN数据包来接收按键事件，
 * 因为motor_driver已经在监听CAN总线。
 */
class ButtonDriverImpl : public ButtonDriverInterface {
public:
    /**
     * @brief 构造函数
     * @param bus CAN总线接口 (用于发送)
     */
    explicit ButtonDriverImpl(std::shared_ptr<bus::BusInterface> bus);
    ~ButtonDriverImpl() override = default;

    /// 发送复现完成信号
    void send_replay_complete(const std::string& interface) override;

    /// 添加观察者
    void add_observer(std::shared_ptr<ButtonEventObserver> observer) override;

    /// 移除观察者
    void remove_observer(std::shared_ptr<ButtonEventObserver> observer) override;

    /// 设置接收回调 (此实现不使用)
    void set_receive_callback(ReceiveCallback callback) override;

    /**
     * @brief 处理CAN数据包 (由motor_driver调用)
     * @param interface CAN接口名称
     * @param can_id CAN ID
     * @param data 数据指针
     * @param len 数据长度
     */
    void handle_can_packet(const std::string& interface, uint32_t can_id,
                          const uint8_t* data, size_t len);

private:
    /// 通知所有观察者
    void notify_observers(const std::string& interface, ButtonStatus status);

    /// 解析按键协议码
    ButtonStatus parse_button_code(const uint8_t* data, size_t len);

    std::shared_ptr<bus::BusInterface> bus_;                    ///< CAN总线接口
    std::vector<std::shared_ptr<ButtonEventObserver>> observers_; ///< 观察者列表
    std::mutex observers_mutex_;                                 ///< 观察者列表锁

    ButtonStatus last_status_ = ButtonStatus::NONE;  ///< 上次状态 (用于防抖)
    std::mutex status_mutex_;                        ///< 状态锁
};

/**
 * @brief 创建CAN FD按键驱动
 * @param bus CAN总线接口
 * @return 按键驱动接口指针
 */
inline std::shared_ptr<ButtonDriverInterface> createCanFdButtonDriver(
    std::shared_ptr<bus::BusInterface> bus) {
    return std::make_shared<ButtonDriverImpl>(bus);
}

}  // namespace button_driver
}  // namespace hardware_driver

#endif  // __BUTTON_DRIVER_IMPL_HPP__
