#ifndef __MOTOR_EVENTS_HPP__
#define __MOTOR_EVENTS_HPP__

#include "event_bus.hpp"
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include <chrono>
#include <map>

namespace hardware_driver {
namespace event {

// 电机状态更新事件
class MotorStatusEvent : public Event {
public:
    MotorStatusEvent(const std::string& interface, uint32_t motor_id, 
                     const motor_driver::Motor_Status& status)
        : interface_(interface), motor_id_(motor_id), status_(status),
          timestamp_(std::chrono::high_resolution_clock::now()) {}
    
    std::string get_type_name() const override {
        return "MotorStatusEvent";
    }
    
    std::string get_topic() const override {
        return "motor." + interface_ + "." + std::to_string(motor_id_) + ".status";
    }
    
    // 访问器
    const std::string& get_interface() const { return interface_; }
    uint32_t get_motor_id() const { return motor_id_; }
    const motor_driver::Motor_Status& get_status() const { return status_; }
    std::chrono::high_resolution_clock::time_point get_timestamp() const { return timestamp_; }
    
private:
    std::string interface_;
    uint32_t motor_id_;
    motor_driver::Motor_Status status_;
    std::chrono::high_resolution_clock::time_point timestamp_;
};

// 批量电机状态更新事件 - 一个接口的所有电机
class MotorBatchStatusEvent : public Event {
public:
    MotorBatchStatusEvent(const std::string& interface, 
                         const std::map<uint32_t, motor_driver::Motor_Status>& status_all)
        : interface_(interface), status_all_(status_all),
          timestamp_(std::chrono::high_resolution_clock::now()) {}
    
    std::string get_type_name() const override {
        return "MotorBatchStatusEvent";
    }
    
    std::string get_topic() const override {
        return "motor." + interface_ + ".batch.status";
    }
    
    // 访问器
    const std::string& get_interface() const { return interface_; }
    const std::map<uint32_t, motor_driver::Motor_Status>& get_status_all() const { return status_all_; }
    std::chrono::high_resolution_clock::time_point get_timestamp() const { return timestamp_; }
    
private:
    std::string interface_;
    std::map<uint32_t, motor_driver::Motor_Status> status_all_;
    std::chrono::high_resolution_clock::time_point timestamp_;
};

// 电机函数操作结果事件
class MotorFunctionResultEvent : public Event {
public:
    MotorFunctionResultEvent(const std::string& interface, uint32_t motor_id,
                            uint8_t op_code, bool success)
        : interface_(interface), motor_id_(motor_id), op_code_(op_code), 
          success_(success), timestamp_(std::chrono::high_resolution_clock::now()) {}
    
    std::string get_type_name() const override {
        return "MotorFunctionResultEvent";
    }
    
    std::string get_topic() const override {
        return "motor." + interface_ + "." + std::to_string(motor_id_) + ".function";
    }
    
    // 访问器
    const std::string& get_interface() const { return interface_; }
    uint32_t get_motor_id() const { return motor_id_; }
    uint8_t get_operation_code() const { return op_code_; }
    bool is_success() const { return success_; }
    std::chrono::high_resolution_clock::time_point get_timestamp() const { return timestamp_; }
    
private:
    std::string interface_;
    uint32_t motor_id_;
    uint8_t op_code_;
    bool success_;
    std::chrono::high_resolution_clock::time_point timestamp_;
};

// 电机参数操作结果事件
class MotorParameterResultEvent : public Event {
public:
    MotorParameterResultEvent(const std::string& interface, uint32_t motor_id,
                             uint16_t address, uint8_t data_type, const std::any& data)
        : interface_(interface), motor_id_(motor_id), address_(address), 
          data_type_(data_type), data_(data), 
          timestamp_(std::chrono::high_resolution_clock::now()) {}
    
    std::string get_type_name() const override {
        return "MotorParameterResultEvent";
    }
    
    std::string get_topic() const override {
        return "motor." + interface_ + "." + std::to_string(motor_id_) + ".parameter";
    }
    
    // 访问器
    const std::string& get_interface() const { return interface_; }
    uint32_t get_motor_id() const { return motor_id_; }
    uint16_t get_address() const { return address_; }
    uint8_t get_data_type() const { return data_type_; }
    const std::any& get_data() const { return data_; }
    std::chrono::high_resolution_clock::time_point get_timestamp() const { return timestamp_; }
    
private:
    std::string interface_;
    uint32_t motor_id_;
    uint16_t address_;
    uint8_t data_type_;
    std::any data_;
    std::chrono::high_resolution_clock::time_point timestamp_;
};

}  // namespace event
}  // namespace hardware_driver

#endif  // __MOTOR_EVENTS_HPP__