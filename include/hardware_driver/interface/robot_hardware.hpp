#ifndef ROBOT_HARDWARE_HPP
#define ROBOT_HARDWARE_HPP

#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <map>

#include <chrono>
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "protocol/motor_protocol.hpp"

class RobotHardware {
public:
    // 使用map配置每个接口对应的电机ID列表
    RobotHardware(std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
                  const std::map<std::string, std::vector<uint32_t>>& interface_motor_config);
    ~RobotHardware();

    // ========== 电机状态获取接口 ==========
    hardware_driver::motor_driver::Motor_Status get_motor_status(const std::string& interface, const uint32_t motor_id);
    // 返回所有或指定接口的电机状态
    std::map<std::pair<std::string, uint32_t>, hardware_driver::motor_driver::Motor_Status>
    get_all_motor_status(const std::string& interface = "");
    
    // ========== 电机控制接口 ==========
    void control_motor_in_mit_mode(const std::string& interface, const uint32_t motor_id, float position, float velocity, float effort);
    void control_motor_in_position_mode(const std::string& interface, const uint32_t motor_id, float position);
    void control_motor_in_velocity_mode(const std::string& interface, const uint32_t motor_id, float velocity);
    void control_motor_in_effort_mode(const std::string& interface, const uint32_t motor_id, float effort);
    void disable_motor(const std::string& interface, const uint32_t motor_id);
    void enable_motor(const std::string& interface, const uint32_t motor_id, uint8_t mode);
    
    // ========== 参数读写接口 ==========
    void motor_parameter_read(const std::string& interface, const uint32_t motor_id, uint16_t address);
    void motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, int32_t value);
    void motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, float value);
    
    // ========== 函数操作接口 ==========
    void motor_function_operation(const std::string& interface, const uint32_t motor_id, uint8_t operation);
    void arm_zero_position_set(const std::string& interface, const uint8_t motor_num);
    
    // ========== 反馈请求接口 ==========
    void motor_feedback_request(const std::string& interface, const uint32_t motor_id);
    void motor_feedback_request_all(const std::string& interface);
    
private:
    std::atomic<bool> high_freq_mode_{false};
    std::chrono::steady_clock::time_point last_control_time_;
    void update_control_time();
    
    // 时序控制相关
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> interface_last_send_time_;
    std::mutex timing_mutex_;
    void ensure_send_interval(const std::string& interface, std::chrono::microseconds min_interval = std::chrono::microseconds(100));
    
    std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver_;
    std::map<std::string, std::vector<uint32_t>> interface_motor_config_;  // 每个接口对应的电机ID列表
    std::atomic<bool> running_;

    // 线程
    std::thread feedback_request_thread_;
    std::thread feedback_process_thread_;
    void request_feedback_thread();
    void process_feedback_thread();

    std::mutex status_mutex_;
    // std::map<std::pair<std::string, uint32_t>, hardware_driver::motor_driver::Motor_Status> status_map_;
    std::unordered_map<std::string_view, std::unordered_map<uint32_t, hardware_driver::motor_driver::Motor_Status>> status_map_;
};


#endif // ROBOT_HARDWARE_HPP