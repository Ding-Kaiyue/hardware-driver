#ifndef HARDWARE_DRIVER_HPP
#define HARDWARE_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <unordered_map>

// 包含必要的头文件
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "hardware_driver/interface/robot_hardware.hpp"

namespace hardware_driver {

/**
 * @brief 硬件驱动接口类
 * 
 * 提供简单的API来控制电机和获取状态
 * 用户只需要包含这个头文件就能使用所有功能
 */
class HardwareDriver {
public:
    /**
     * @brief 构造函数
     * @param interfaces CAN接口列表，如 {"can0", "can1"}
     * @param motor_config 电机配置，格式: {{"can0", {1,2,3,4}}, {"can1", {1,2,3,4,5,6,7,8}}}
     */
    HardwareDriver(const std::vector<std::string>& interfaces,
                   const std::map<std::string, std::vector<uint32_t>>& motor_config);
    
    /**
     * @brief 析构函数
     */
    ~HardwareDriver();

    // 禁用拷贝构造和赋值
    HardwareDriver(const HardwareDriver&) = delete;
    HardwareDriver& operator=(const HardwareDriver&) = delete;

    /**
     * @brief 速度模式控制
     * @param interface CAN接口名称
     * @param motor_id 电机ID
     * @param velocity 目标速度 (degrees/s)
     */
    void control_motor_in_velocity_mode(const std::string& interface, uint32_t motor_id, float velocity);

    /**
     * @brief 位置模式控制
     * @param interface CAN接口名称
     * @param motor_id 电机ID
     * @param position 目标位置 (degrees)
     */
    void control_motor_in_position_mode(const std::string& interface, uint32_t motor_id, float position);

    /**
     * @brief 力矩模式控制
     * @param interface CAN接口名称
     * @param motor_id 电机ID
     * @param effort 目标力矩 (Nm)
     */
    void control_motor_in_effort_mode(const std::string& interface, uint32_t motor_id, float effort);

    /**
     * @brief MIT模式控制
     * @param interface CAN接口名称
     * @param motor_id 电机ID
     * @param position 目标位置 (degrees)
     * @param velocity 目标速度 (degrees/s)
     * @param effort 目标力矩 (Nm)
     */
    void control_motor_in_mit_mode(const std::string& interface, uint32_t motor_id, 
                                  float position, float velocity, float effort);

    /**
     * @brief 使能电机
     * @param interface CAN接口名称
     * @param motor_id 电机ID
     * @param mode 电机模式
     */
    void enable_motor(const std::string& interface, uint32_t motor_id, uint8_t mode = 4);

    /**
     * @brief 失能电机
     * @param interface CAN接口名称
     * @param motor_id 电机ID
     */
    void disable_motor(const std::string& interface, uint32_t motor_id);

    /**
     * @brief 获取电机状态
     * @param interface CAN接口名称
     * @param motor_id 电机ID
     * @return 电机状态
     */
    motor_driver::Motor_Status get_motor_status(const std::string& interface, uint32_t motor_id);

    /**
     * @brief 获取接口所有电机状态
     * @param interface CAN接口名称
     * @return 所有电机状态映射
     */
    std::map<std::pair<std::string, uint32_t>, motor_driver::Motor_Status> 
    get_all_motor_status(const std::string& interface);

private:
    std::unique_ptr<RobotHardware> robot_hardware_;
};

} // namespace hardware_driver

// 简化别名
using HardwareDriver = hardware_driver::HardwareDriver;

#endif // HARDWARE_DRIVER_HPP 