#ifndef __HARDWARE_DRIVER_ROBOT_HARDWARE_HPP__
#define __HARDWARE_DRIVER_ROBOT_HARDWARE_HPP__

#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <map>
#include <condition_variable>
#include <functional>

#include <chrono>
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"
#include "hardware_driver/event/event_bus.hpp"

// ========== 轨迹数据结构定义 ==========
// 简化的轨迹点结构（不依赖ROS2消息）
struct TrajectoryPoint {
    double time_from_start;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
};

// 简化的轨迹结构（不依赖ROS2消息）
struct Trajectory {
    std::vector<std::string> joint_names;
    std::vector<TrajectoryPoint> points;
};

// 电机状态回调函数类型定义
using MotorStatusCallback = std::function<void(const std::string& interface, uint32_t motor_id, const hardware_driver::motor_driver::Motor_Status& status)>;
using MotorBatchStatusCallback = std::function<void(const std::string& interface, const std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>& status_all)>;

class RobotHardware {
public:
    // 单个状态回调构造函数（默认构造函数，用于观察者模式）
    RobotHardware(std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
                  const std::map<std::string, std::vector<uint32_t>>& interface_motor_config,
                  MotorStatusCallback callback = nullptr);
    
    // 批量状态回调构造函数（明确需要传入回调函数）
    RobotHardware(std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
                  const std::map<std::string, std::vector<uint32_t>>& interface_motor_config,
                  MotorBatchStatusCallback batch_callback);
    
    // 事件总线构造函数（推荐用于新项目）
    RobotHardware(std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
                  const std::map<std::string, std::vector<uint32_t>>& interface_motor_config,
                  std::shared_ptr<hardware_driver::event::EventBus> event_bus);
    ~RobotHardware();

    // 状态获取通过回调机制实现，不需要主动查询接口
    
    // ========== 电机控制接口 ==========
    void control_motor_in_mit_mode(const std::string& interface, const uint32_t motor_id, float position, float velocity, float effort);
    void control_motor_in_position_mode(const std::string& interface, const uint32_t motor_id, float position);
    void control_motor_in_velocity_mode(const std::string& interface, const uint32_t motor_id, float velocity);
    void control_motor_in_effort_mode(const std::string& interface, const uint32_t motor_id, float effort);
    void disable_motor(const std::string& interface, const uint32_t motor_id);
    void enable_motor(const std::string& interface, const uint32_t motor_id, uint8_t mode);
    
    // ========== 实时批量控制接口 ==========
    bool send_realtime_velocity_command(const std::string& interface, const std::vector<double>& joint_velocities);
    bool send_realtime_position_command(const std::string& interface, const std::vector<double>& joint_positions);
    bool send_realtime_effort_command(const std::string& interface, const std::vector<double>& joint_efforts);
    bool send_realtime_mit_command(const std::string& interface, const std::vector<double>& joint_positions, const std::vector<double>& joint_velocities, const std::vector<double>& joint_efforts);
    
    // ========== 参数读写接口 ==========
    void motor_parameter_read(const std::string& interface, const uint32_t motor_id, uint16_t address);
    void motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, int32_t value);
    void motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, float value);
    
    // ========== 函数操作接口 ==========
    void motor_function_operation(const std::string& interface, const uint32_t motor_id, uint8_t operation);
    void arm_zero_position_set(const std::string& interface, const std::vector<uint32_t> motor_ids);
        
    // ========== 轨迹执行接口 ==========
    bool execute_trajectory(const std::string& interface, const Trajectory& trajectory);
    
private:
    std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver_;
    std::map<std::string, std::vector<uint32_t>> interface_motor_config_;  // 每个接口对应的电机ID列表

    // 状态回调函数（传递给motor_driver_impl）
    MotorStatusCallback status_callback_;
    MotorBatchStatusCallback batch_status_callback_;
    
    // 事件总线支持
    std::shared_ptr<hardware_driver::event::EventBus> event_bus_;
    
    // 状态聚合器相关
    std::map<std::string, std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>> status_cache_;
    std::mutex cache_mutex_;
    
    // 内部状态聚合方法
    void handle_motor_status_with_aggregation(const std::string& interface, uint32_t motor_id, 
                                            const hardware_driver::motor_driver::Motor_Status& status);
    
};


#endif // __HARDWARE_DRIVER_ROBOT_HARDWARE_HPP__