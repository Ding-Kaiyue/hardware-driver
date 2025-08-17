#ifndef HARDWARE_UTILITY_HPP
#define HARDWARE_UTILITY_HPP

#include <string>
#include <memory>
#include <vector>
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "hardware_driver/interface/robot_hardware.hpp"

namespace hardware_driver {
/**
 * @brief 设备维护与非实时操作接口类
 * 
 * 提供调试、诊断、校准等非实时控制功能
 * 不直接参与高速控制环，与HardwareDriver并列
 */
class HardwareUtility {
public:
    /**
     * @brief 构造函数
     * @param interfaces CAN接口列表，如 {"can0", "can1"}
     * @param motor_config 电机配置，格式: {{"can0", {1,2,3,4}}, {"can1", {1,2,3,4,5,6,7,8}}}
     */
    HardwareUtility(const std::vector<std::string>& interfaces, 
                    const std::map<std::string, std::vector<uint32_t>>& motor_config);
    
    /**
     * @brief 析构函数
     */
    ~HardwareUtility();

    // 禁用拷贝构造和赋值
    HardwareUtility(const HardwareUtility&) = delete;
    HardwareUtility& operator=(const HardwareUtility&) = delete;

    /**
     * @brief 参数读取
     * @param interface CAN接口名称
     * @param motor_id 电机ID
     * @param address 参数地址
     */
    void param_read(const std::string& interface, uint32_t motor_id, uint16_t address);

    /**
     * @brief 参数写入(int32_t)
     * @param interface CAN接口名称
     * @param motor_id 电机ID
     * @param address 参数地址
     * @param value 参数值
     */
    void param_write(const std::string& interface, uint32_t motor_id, uint16_t address, int32_t value);

    /**
     * @brief 参数写入(float)
     * @param interface CAN接口名称
     * @param motor_id 电机ID
     * @param address 参数地址
     * @param value 参数值
     */
    void param_write(const std::string& interface, uint32_t motor_id, uint16_t address, float value);

    /**
     * @brief 函数操作
     * @param interface CAN接口名称
     * @param motor_id 电机ID
     * @param opcode 操作码
     */
    void function_operation(const std::string& interface, uint32_t motor_id, uint8_t opcode);

    /**
     * @brief 零位设置
     * @param interface CAN接口名称
     * @param motor_ids 电机ID
     */
    void zero_position_set(const std::string& interface, std::vector<uint32_t> motor_ids);

private:
    std::unique_ptr<RobotHardware> robot_hardware_;
};

}   // namespace hardware_driver

// 简化别名
using HardwareUtility = hardware_driver::HardwareUtility;

#endif   // HARDWARE_UTILITY_HPP
