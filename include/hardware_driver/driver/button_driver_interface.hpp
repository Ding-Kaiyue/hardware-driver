/*********************************************************************
 * @file        button_driver_interface.hpp
 * @brief       按键驱动接口定义
 *
 * 硬件按键通过CAN总线与上位机通信：
 * - 接收: CAN ID 0x8F, 协议码 JRSJ/TCSJ/GJFX
 * - 发送: CAN ID 0x7F, 协议码 FXJS (复现结束)
 *
 * 按键功能：
 * - 短按+长按2秒: 进入示教模式 (LED闪烁)
 * - 长按2秒: 退出示教模式 (LED灭)
 * - 双击: 轨迹复现 (LED常亮)
 *********************************************************************/

#ifndef __BUTTON_DRIVER_INTERFACE_HPP__
#define __BUTTON_DRIVER_INTERFACE_HPP__

#include <string>
#include <cstdint>
#include <memory>
#include <functional>

namespace hardware_driver {
namespace button_driver {

/// 按键状态枚举
enum class ButtonStatus : uint8_t {
    NONE         = 0,  ///< 无状态
    ENTRY_TEACH  = 1,  ///< 进入示教 (短按+长按2秒)
    EXIT_TEACH   = 2,  ///< 退出示教 (长按2秒)
    TEACH_REPEAT = 3,  ///< 轨迹复现 (双击)
};

/// 按键事件观察者接口
class ButtonEventObserver {
public:
    virtual ~ButtonEventObserver() = default;

    /**
     * @brief 按键事件回调
     * @param interface CAN接口名称 (如"can0")
     * @param status 按键状态
     */
    virtual void on_button_event(const std::string& interface, ButtonStatus status) = 0;
};

/// 按键驱动抽象接口
class ButtonDriverInterface {
public:
    virtual ~ButtonDriverInterface() = default;

    /**
     * @brief 发送复现完成信号
     * @param interface CAN接口名称
     *
     * 发送 "FXJS" (0x46, 0x58, 0x4A, 0x53) 到 CAN ID 0x7F
     * 硬件收到后LED会熄灭
     */
    virtual void send_replay_complete(const std::string& interface) = 0;

    /**
     * @brief 添加按键事件观察者
     * @param observer 观察者对象
     */
    virtual void add_observer(std::shared_ptr<ButtonEventObserver> observer) = 0;

    /**
     * @brief 移除按键事件观察者
     * @param observer 观察者对象
     */
    virtual void remove_observer(std::shared_ptr<ButtonEventObserver> observer) = 0;

    /// CAN数据包接收回调类型
    using ReceiveCallback = std::function<void(uint32_t can_id, const uint8_t* data, size_t len)>;

    /**
     * @brief 设置数据接收回调 (可选)
     * @param callback 回调函数
     */
    virtual void set_receive_callback(ReceiveCallback callback) = 0;
};

}  // namespace button_driver
}  // namespace hardware_driver

#endif  // __BUTTON_DRIVER_INTERFACE_HPP__
