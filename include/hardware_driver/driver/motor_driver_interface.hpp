#ifndef __MOTOR_DRIVER_INTERFACE_HPP__
#define __MOTOR_DRIVER_INTERFACE_HPP__

#include <string>
#include <vector>

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
