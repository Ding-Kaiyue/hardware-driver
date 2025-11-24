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
#include <shared_mutex>
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "hardware_driver/driver/gripper_driver_interface.hpp"
#include "hardware_driver/event/event_bus.hpp"

// 前向声明，避免在头文件中包含实现类
namespace hardware_driver {
    namespace event {
        class MotorStatusEvent;
    }
    namespace motor_driver {
        class MotorDriverImpl;
    }
    namespace gripper_driver {
        class GripperDriverImpl;
    }
    namespace bus {
        class CanFdBus;
        // class Usb2CanFdBus;
    }
}

// 工厂函数声明 - 用于创建具体的驱动实例
namespace hardware_driver {
    // 创建CANFD电机驱动实例
    std::shared_ptr<motor_driver::MotorDriverInterface> createCanFdMotorDriver(
        const std::vector<std::string>& interfaces);

    // 创建CANFD夹爪驱动实例
    std::shared_ptr<gripper_driver::GripperDriverInterface> createCanFdGripperDriver(
        const std::vector<std::string>& interfaces);

    // 创建USB2CANFD电机驱动实例
    // std::shared_ptr<motor_driver::MotorDriverInterface> createUsb2CanfdMotorDriver(
    //     const std::vector<std::string>& device_sns);
}

// 轨迹数据结构定义
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

// ========== 轨迹执行状态枚举 ==========
enum class TrajectoryExecutionState {
    IDLE,           // 闲置状态，未执行
    RUNNING,        // 正在执行
    PAUSED,         // 已暂停
    CANCELLED,      // 已取消
    COMPLETED,      // 已完成
    ERROR           // 执行出错
};

// ========== 轨迹执行进度信息 ==========
struct TrajectoryExecutionProgress {
    TrajectoryExecutionState state;                      // 当前状态
    size_t current_point_index;                          // 当前点索引
    size_t total_points;                                 // 总点数
    double progress_percentage;                          // 进度百分比 (0-100)
    std::chrono::milliseconds elapsed_time;              // 已用时间
    std::chrono::milliseconds estimated_remaining_time;  // 估计剩余时间
    std::string error_message;                           // 错误信息（如果有）
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
    void control_motor_in_mit_mode(const std::string& interface, const uint32_t motor_id, float position, float velocity, float effort, float kp = 0.05, float kd = 0.005);
    void control_motor_in_position_mode(const std::string& interface, const uint32_t motor_id, float position, float kp = 0.05, float kd = 0.005);
    void control_motor_in_velocity_mode(const std::string& interface, const uint32_t motor_id, float velocity, float kp = 0.0, float kd = 0.005);
    void control_motor_in_effort_mode(const std::string& interface, const uint32_t motor_id, float effort, float kp = 0.05, float kd = 0.005);
    void disable_motor(const std::string& interface, const uint32_t motor_id, uint8_t mode);
    void enable_motor(const std::string& interface, const uint32_t motor_id, uint8_t mode);
    
    // 实时批量控制接口（支持最多 6 个电机）
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
    
    // 参数读写接口 
    void motor_parameter_read(const std::string& interface, const uint32_t motor_id, uint16_t address);
    void motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, int32_t value);
    void motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, float value);
    
    //  函数操作接口 
    void motor_function_operation(const std::string& interface, const uint32_t motor_id, uint8_t operation);
    void arm_zero_position_set(const std::string& interface, const std::vector<uint32_t> motor_ids);
    
    // IAP固件更新接口 
    void start_update(const std::string& interface, const uint8_t motor_id, const std::string& firmware_file);

    // ========== 轨迹执行接口 ==========

    // 同步轨迹执行（阻塞式）
    bool execute_trajectory(const std::string& interface, const Trajectory& trajectory);

    // 异步轨迹执行（非阻塞式）
    /**
     * 异步执行轨迹
     * @param interface 接口名称
     * @param trajectory 轨迹数据
     * @param show_progress 是否显示进度条（默认true）
     * @return 执行ID，用于后续控制此执行任务；空字符串表示执行失败
     */
    std::string execute_trajectory_async(
        const std::string& interface,
        const Trajectory& trajectory,
        bool show_progress = true
    );

    /**
     * 暂停指定的轨迹执行
     * @param execution_id 执行ID
     * @return 是否成功暂停
     */
    bool pause_trajectory(const std::string& execution_id);

    /**
     * 恢复指定的轨迹执行
     * @param execution_id 执行ID
     * @return 是否成功恢复
     */
    bool resume_trajectory(const std::string& execution_id);

    /**
     * 取消指定的轨迹执行
     * @param execution_id 执行ID
     * @return 是否成功取消
     */
    bool cancel_trajectory(const std::string& execution_id);

    /**
     * 获取指定执行任务的当前进度
     * @param execution_id 执行ID
     * @param progress 输出参数，返回进度信息
     * @return 是否找到该执行任务
     */
    bool get_execution_progress(const std::string& execution_id, TrajectoryExecutionProgress& progress);

    /**
     * 获取所有正在执行或已暂停的任务ID列表
     * @return 执行ID列表
     */
    std::vector<std::string> get_active_execution_ids() const;

    /**
     * 检查执行任务是否存在
     * @param execution_id 执行ID
     * @return 是否存在
     */
    bool has_execution(const std::string& execution_id) const;

    /**
     * 等待指定的执行任务完成
     * @param execution_id 执行ID
     * @param timeout_ms 超时时间（毫秒，0表示无限等待）
     * @return 是否在超时前完成
     */
    bool wait_for_completion(const std::string& execution_id, int timeout_ms = 0);

    /**
     * 暂停所有正在执行的轨迹
     */
    void pause_all_trajectories();

    /**
     * 恢复所有已暂停的轨迹
     */
    void resume_all_trajectories();

    /**
     * 取消所有正在执行或已暂停的轨迹
     */
    void cancel_all_trajectories();

    /**
     * 获取当前活跃的执行任务数
     * @return 任务数（等于正在执行的接口数）
     */
    size_t get_active_trajectory_count() const;

    // ========== 状态监控控制方法 ==========

    //  轨迹执行接口 
    bool execute_trajectory(const std::string& interface, const Trajectory& trajectory);

    //  夹爪控制接口 
    /**
     * @brief 控制夹爪
     * @param interface 总线接口名称 (如"can0")
     * @param gripper_type 夹爪类型 (0=OmniPicker, 1=PGC_Gripper, 2=Raw_Frame)
     * @param position 位置 (0-100%)
     * @param velocity 速度 (0-100%)
     * @param effort 力 (0-100%)
     */
    void control_gripper(const std::string& interface,
                        uint8_t gripper_type,
                        uint8_t position,
                        uint8_t velocity,
                        uint8_t effort);

    
    void open_gripper(const std::string& interface,
                     uint8_t gripper_type,
                     uint8_t velocity = 50,
                     uint8_t effort = 50);

    
    void close_gripper(const std::string& interface,
                      uint8_t gripper_type,
                      uint8_t velocity = 50,
                      uint8_t effort = 50);

    /**
     * @brief 发送原始数据到夹爪（Raw_Frame模式）
     * @param interface 总线接口名称
     * @param raw_data 原始数据指针
     * @param raw_data_len 原始数据长度
     */
    void send_gripper_raw_data(const std::string& interface,
                              const uint8_t* raw_data,
                              size_t raw_data_len);

    // 状态监控控制方法

    void pause_status_monitoring();
    void resume_status_monitoring();

    // 夹爪驱动设置方法
    void set_gripper_driver(std::shared_ptr<hardware_driver::gripper_driver::GripperDriverInterface> gripper_driver);
private:
    std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver_;
    std::shared_ptr<hardware_driver::gripper_driver::GripperDriverInterface> gripper_driver_;
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

    // ========== 异步轨迹执行相关私有成员 ==========
    struct TrajectoryExecutionTask {
        std::string execution_id;
        std::string interface;
        Trajectory trajectory;
        std::thread executor_thread;

        // 状态和控制
        std::atomic<TrajectoryExecutionState> state{TrajectoryExecutionState::IDLE};
        std::atomic<size_t> current_point_index{0};
        std::atomic<bool> should_stop{false};
        std::atomic<bool> should_pause{false};

        // 计时
        std::chrono::steady_clock::time_point start_time;
        std::chrono::milliseconds pause_elapsed{0};
        std::chrono::steady_clock::time_point pause_start_time;

        // 进度显示控制
        bool show_progress{true};  // 是否显示进度条

        // 同步
        std::mutex state_mutex;
        std::condition_variable state_cv;

        // 结果
        std::string error_message;
    };

    // 执行任务管理
    std::map<std::string, std::shared_ptr<TrajectoryExecutionTask>> trajectory_execution_tasks_;
    mutable std::shared_mutex trajectory_tasks_mutex_;

    // 进度条配置
    std::string progress_display_tty_{"/dev/tty"};

    // 内部执行方法
    void trajectory_execution_worker(std::shared_ptr<TrajectoryExecutionTask> task);
    std::string generate_execution_id();
    void display_trajectory_progress(const std::shared_ptr<TrajectoryExecutionTask>& task);
    std::chrono::milliseconds get_trajectory_total_time(const Trajectory& trajectory) const;
    void cleanup_completed_trajectory_tasks();

    // 内部状态聚合方法
    void handle_motor_status_with_aggregation(const std::string& interface, uint32_t motor_id,
                                            const hardware_driver::motor_driver::Motor_Status& status);

};


#endif // __HARDWARE_DRIVER_ROBOT_HARDWARE_HPP__
