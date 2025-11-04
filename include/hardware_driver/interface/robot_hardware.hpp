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
#include <array>
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "hardware_driver/event/event_bus.hpp"

// 前向声明，避免在头文件中包含实现类
namespace hardware_driver {
    namespace event {
        class MotorStatusEvent;
    }
    namespace motor_driver {
        class MotorDriverImpl;
    }
    namespace bus {
        class CanFdBus;
    }
}

// 工厂函数声明 - 用于创建具体的驱动实例
namespace hardware_driver {
    // 创建CANFD电机驱动实例
    std::shared_ptr<motor_driver::MotorDriverInterface> createCanFdMotorDriver(
        const std::vector<std::string>& interfaces);
}

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
    
    // 观察者构造函数（用于观察者模式）
    RobotHardware(std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
                  const std::map<std::string, std::vector<uint32_t>>& interface_motor_config,
                  std::shared_ptr<hardware_driver::motor_driver::MotorStatusObserver> observer);

    // IAP观察者构造函数（用于IAP固件更新回调）
    RobotHardware(std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
                  const std::map<std::string, std::vector<uint32_t>>& interface_motor_config,
                  std::shared_ptr<hardware_driver::motor_driver::IAPStatusObserver> iap_observer);

    // 事件总线构造函数（事件总线模式）
    RobotHardware(std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
                  const std::map<std::string, std::vector<uint32_t>>& interface_motor_config,
                  std::shared_ptr<hardware_driver::event::EventBus> event_bus,
                  std::shared_ptr<hardware_driver::motor_driver::MotorEventHandler> event_handler);
    
    ~RobotHardware();

    // 状态获取通过回调机制实现，不需要主动查询接口
    
    // ========== 电机控制接口 ==========
    void control_motor_in_mit_mode(const std::string& interface, const uint32_t motor_id, float position, float velocity, float effort, float kp = 0.0, float kd = 0.0);
    void control_motor_in_position_mode(const std::string& interface, const uint32_t motor_id, float position, float kp = 0.0, float kd = 0.0);
    void control_motor_in_velocity_mode(const std::string& interface, const uint32_t motor_id, float velocity, float kp = 0.0, float kd = 0.0);
    void control_motor_in_effort_mode(const std::string& interface, const uint32_t motor_id, float effort, float kp = 0.0, float kd = 0.0);
    void disable_motor(const std::string& interface, const uint32_t motor_id, uint8_t mode);
    void enable_motor(const std::string& interface, const uint32_t motor_id, uint8_t mode);
    
    // ========== 实时批量控制接口（支持最多 6 个电机） ==========
    void disable_motors(const std::string& interface, const std::vector<uint32_t>& motor_ids, uint8_t mode);
    void enable_motors(const std::string& interface, const std::vector<uint32_t>& motor_ids, uint8_t mode);
    // 使用 std::array<double, 6> 避免动态分配开销
    bool send_realtime_velocity_command(const std::string& interface,
                                       const std::array<double, 6>& joint_velocities,
                                       const std::array<double, 6>& kps = {},
                                       const std::array<double, 6>& kds = {});

    bool send_realtime_position_command(const std::string& interface,
                                       const std::array<double, 6>& joint_positions,
                                       const std::array<double, 6>& kps = {},
                                       const std::array<double, 6>& kds = {});

    bool send_realtime_effort_command(const std::string& interface,
                                     const std::array<double, 6>& joint_efforts,
                                     const std::array<double, 6>& kps = {},
                                     const std::array<double, 6>& kds = {});

    bool send_realtime_mit_command(const std::string& interface,
                                  const std::array<double, 6>& joint_positions,
                                  const std::array<double, 6>& joint_velocities,
                                  const std::array<double, 6>& joint_efforts,
                                  const std::array<double, 6>& kps = {},
                                  const std::array<double, 6>& kds = {});
    
    // ========== 参数读写接口 ==========
    void motor_parameter_read(const std::string& interface, const uint32_t motor_id, uint16_t address);
    void motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, int32_t value);
    void motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, float value);
    
    // ========== 函数操作接口 ==========
    void motor_function_operation(const std::string& interface, const uint32_t motor_id, uint8_t operation);
    void arm_zero_position_set(const std::string& interface, const std::vector<uint32_t> motor_ids);
    
    // ========== IAP固件更新接口 ==========
    void start_update(const std::string& interface, const uint8_t motor_id, const std::string& firmware_file);

    // ========== 轨迹执行接口 ==========
    bool execute_trajectory(const std::string& interface, const Trajectory& trajectory);
    
    // ========== 状态监控控制方法 ==========
    void pause_status_monitoring();
    void resume_status_monitoring();
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
    
    // 状态监控控制相关
    std::shared_ptr<hardware_driver::motor_driver::MotorStatusObserver> current_observer_;
    std::shared_ptr<hardware_driver::motor_driver::IAPStatusObserver> current_iap_observer_;
    std::shared_ptr<hardware_driver::motor_driver::MotorEventHandler> current_event_handler_;
    bool monitoring_paused_ = false;
    
    // 事件订阅管理 - 保持订阅者的生命周期
    std::vector<std::shared_ptr<hardware_driver::event::EventHandler>> event_subscriptions_;
    
    // 内部状态聚合方法
    void handle_motor_status_with_aggregation(const std::string& interface, uint32_t motor_id, 
                                            const hardware_driver::motor_driver::Motor_Status& status);
    
};


#endif // __HARDWARE_DRIVER_ROBOT_HARDWARE_HPP__