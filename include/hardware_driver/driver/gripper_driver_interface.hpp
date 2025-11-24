/*********************************************************************
 * @file        gripper_driver_interface.hpp
 * @brief       夹爪驱动接口定义
 *
 * 本文件定义了夹爪驱动的抽象接口，支持多种夹爪类型控制。
 * 接口设计遵循依赖倒置原则，上层应用依赖于抽象接口而非具体实现。
 *********************************************************************/

#ifndef __GRIPPER_DRIVER_INTERFACE_HPP__
#define __GRIPPER_DRIVER_INTERFACE_HPP__

#include <string>
#include <cstdint>
#include <vector>
#include <memory>

namespace hardware_driver {
namespace gripper_driver {

//夹爪类型枚举
enum class GripperType : uint8_t {
    OmniPicker  = 0,  ///< OmniPicker全向夹爪
    PGC_Gripper = 1,  ///< PGC夹爪
    Raw_Frame   = 2,  ///< 原始帧模式（透传数据）
};

//夹爪状态数据结构
struct GripperStatus {
    uint8_t position;       ///< 位置 (0-255)
    uint8_t velocity;       ///< 速度 (0-255)
    uint8_t force;          ///< 力 (0-255)
    uint8_t status;         ///< 状态码
    uint8_t action_status;  ///< 动作状态码
    bool has_object;        ///< 是否夹持物体
    bool is_moving;         ///< 是否正在移动
};

//实现此接口以接收夹爪状态更新回调
class GripperStatusObserver {
public:
    virtual ~GripperStatusObserver() = default;

    /**
     * @brief 夹爪状态更新回调
     * @param interface 总线接口名称 (如"can0")
     * @param gripper_id 夹爪ID
     * @param status 夹爪状态
     */
    virtual void on_gripper_status_update(const std::string& interface,
                                         uint32_t gripper_id,
                                         const GripperStatus& status) = 0;
};

/**
 * @brief 夹爪驱动抽象接口
 *
 * 定义夹爪控制的标准接口，具体实现由不同总线类型的驱动完成
 */
class GripperDriverInterface {
public:
    virtual ~GripperDriverInterface() = default;

    /**
     * @brief 控制夹爪
     * @param interface 总线接口名称 (如"can0")
     * @param gripper_type 夹爪类型
     * @param position 位置 (0-100%)
     * @param velocity 速度 (0-100%)
     * @param effort 力 (0-100%)
     */
    virtual void control_gripper(const std::string& interface,
                                GripperType gripper_type,
                                uint8_t position,
                                uint8_t velocity,
                                uint8_t effort) = 0;

    /**
     * @brief 发送原始数据（Raw_Frame模式）
     * @param interface 总线接口名称
     * @param raw_data 原始数据指针
     * @param raw_data_len 原始数据长度
     */
    virtual void send_raw_data(const std::string& interface,
                              const uint8_t* raw_data,
                              size_t raw_data_len) = 0;

    /**
     * @brief 打开夹爪（快捷方法）
     * @param interface 总线接口名称
     * @param gripper_type 夹爪类型
     * @param velocity 速度 (0-100%)
     * @param effort 力 (0-100%)
     */
    virtual void open_gripper(const std::string& interface,
                            GripperType gripper_type,
                            uint8_t velocity = 50,
                            uint8_t effort = 50) = 0;

    /**
     * @brief 关闭夹爪（快捷方法）
     * @param interface 总线接口名称
     * @param gripper_type 夹爪类型
     * @param velocity 速度 (0-100%)
     * @param effort 力 (0-100%)
     */
    virtual void close_gripper(const std::string& interface,
                             GripperType gripper_type,
                             uint8_t velocity = 50,
                             uint8_t effort = 50) = 0;

    /**
     * @brief 添加状态观察者
     * @param observer 观察者对象
     */
    virtual void add_observer(std::shared_ptr<GripperStatusObserver> observer) = 0;

    /**
     * @brief 移除状态观察者
     * @param observer 观察者对象
     */
    virtual void remove_observer(std::shared_ptr<GripperStatusObserver> observer) = 0;
};

}  // namespace gripper_driver
}  // namespace hardware_driver

#endif  // __GRIPPER_DRIVER_INTERFACE_HPP__
