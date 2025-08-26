#ifndef __MOTOR_DRIVER_INTERFACE_HPP__
#define __MOTOR_DRIVER_INTERFACE_HPP__

#include <string>
#include <vector>
#include <map>
#include <any>

namespace hardware_driver {
namespace motor_driver {

// 电机数据结构体定义
typedef struct {
    uint8_t enable_flag : 1;
    uint8_t motor_mode : 4;
    float position;

    uint8_t limit_flag : 1;
    uint16_t temperature : 10;
    float velocity;

    uint16_t voltage : 10;
    float effort;

    uint32_t error_code;
} Motor_Status;

// 电机状态观察者接口
class MotorStatusObserver {
public:
    virtual ~MotorStatusObserver() = default;
    
    // 单个电机状态更新事件
    virtual void on_motor_status_update(const std::string& /*interface*/, 
                                       uint32_t /*motor_id*/, 
                                       const Motor_Status& /*status*/) = 0;
    
    // 批量电机状态更新事件 - 一个接口的所有电机状态
    virtual void on_motor_status_update(const std::string& /*interface*/,
                                       const std::map<uint32_t, Motor_Status>& /*status_all*/) {}
    
    // 函数操作结果事件
    virtual void on_motor_function_result(const std::string& /*interface*/,
                                         uint32_t /*motor_id*/,
                                         uint8_t /*op_code*/,
                                         bool /*success*/) {}
    
    // 参数读写结果事件
    virtual void on_motor_parameter_result(const std::string& /*interface*/,
                                          uint32_t /*motor_id*/,
                                          uint16_t /*address*/,
                                          uint8_t /*data_type*/,
                                          const std::any& /*data*/) {}
};

// 事件总线模式的事件处理器接口 - 与观察者模式保持相同的虚函数签名
class MotorEventHandler {
public:
    virtual ~MotorEventHandler() = default;
    
    // 单个电机状态更新事件
    virtual void on_motor_status_update(const std::string& /*interface*/, 
                                       uint32_t /*motor_id*/, 
                                       const Motor_Status& /*status*/) = 0;
    
    // 批量电机状态更新事件 - 一个接口的所有电机状态
    virtual void on_motor_status_update(const std::string& /*interface*/,
                                       const std::map<uint32_t, Motor_Status>& /*status_all*/) {}
    
    // 函数操作结果事件
    virtual void on_motor_function_result(const std::string& /*interface*/,
                                         uint32_t /*motor_id*/,
                                         uint8_t /*op_code*/,
                                         bool /*success*/) {}
    
    // 参数读写结果事件
    virtual void on_motor_parameter_result(const std::string& /*interface*/,
                                          uint32_t /*motor_id*/,
                                          uint16_t /*address*/,
                                          uint8_t /*data_type*/,
                                          const std::any& /*data*/) {}
};


class MotorDriverInterface {
public:
    virtual ~MotorDriverInterface() = default;

    virtual void disable_motor(const std::string interface, const uint32_t motor_id) = 0;
    virtual void enable_motor(const std::string interface, const uint32_t motor_id, uint8_t mode) = 0;
    virtual void send_position_cmd(const std::string interface, const uint32_t motor_id, float position) = 0;
    virtual void send_velocity_cmd(const std::string interface, const uint32_t motor_id, float velocity) = 0;
    virtual void send_effort_cmd(const std::string interface, const uint32_t motor_id, float effort) = 0;
    virtual void send_mit_cmd(const std::string interface, const uint32_t motor_id, float position, float velocity, float effort) = 0;

    virtual void motor_function_operation(const std::string interface, const uint32_t motor_id, uint8_t operation) = 0;
    virtual void motor_parameter_write(const std::string interface, const uint32_t motor_id, uint16_t address, int32_t value) = 0;
    virtual void motor_parameter_write(const std::string interface, const uint32_t motor_id, uint16_t address, float value) = 0;
    virtual void motor_parameter_read(const std::string interface, const uint32_t motor_id, uint16_t address) = 0;
};


}    // namespace motor_driver
}    // namespace hardware_driver


#endif   // __MOTOR_DRIVER_INTERFACE_HPP__
