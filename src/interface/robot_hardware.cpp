#include "hardware_driver/interface/robot_hardware.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include "driver/motor_driver_impl.hpp"
#include "driver/gripper_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"
// #include "bus/usb2canfd_bus_impl.hpp"
#include <chrono>
#include <thread>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <cstdio>
#include <atomic>

# define REQUEST_ALL
// # define REQUEST_BY_MOTOR_ID     // 只适用于单条总线

RobotHardware::RobotHardware(
    std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
    const std::map<std::string, std::vector<uint32_t>>& interface_motor_config,
    MotorStatusCallback callback)
    : motor_driver_(std::move(motor_driver)),
      interface_motor_config_(interface_motor_config),
      status_callback_(callback),
      batch_status_callback_(nullptr),
      event_bus_(nullptr),
      current_observer_(nullptr),
      current_event_handler_(nullptr),
      monitoring_paused_(false),
      event_subscriptions_()
{
    // 转换为MotorDriverImpl以访问新接口
    auto motor_driver_impl = std::dynamic_pointer_cast<hardware_driver::motor_driver::MotorDriverImpl>(motor_driver_);

    if (motor_driver_impl) {
        // 注册回调函数
        if (status_callback_) {
            motor_driver_impl->register_feedback_callback(status_callback_);
        }
        
        // 设置电机配置，启动反馈请求
        motor_driver_impl->set_motor_config(interface_motor_config_);
    }
}

// 批量状态回调构造函数
RobotHardware::RobotHardware(
    std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
    const std::map<std::string, std::vector<uint32_t>>& interface_motor_config,
    MotorBatchStatusCallback batch_callback)
    : motor_driver_(std::move(motor_driver)),
      interface_motor_config_(interface_motor_config),
      status_callback_(nullptr),
      batch_status_callback_(batch_callback),
      event_bus_(nullptr),
      current_observer_(nullptr),
      current_event_handler_(nullptr),
      monitoring_paused_(false),
      event_subscriptions_()
{
    // 转换为MotorDriverImpl以访问新接口
    auto motor_driver_impl = std::dynamic_pointer_cast<hardware_driver::motor_driver::MotorDriverImpl>(motor_driver_);

    if (motor_driver_impl) {
        // 注册内部聚合回调函数
        motor_driver_impl->register_feedback_callback(
            [this](const std::string& interface, uint32_t motor_id, 
                   const hardware_driver::motor_driver::Motor_Status& status) {
                this->handle_motor_status_with_aggregation(interface, motor_id, status);
            }
        );
        
        // 设置电机配置，启动反馈请求
        motor_driver_impl->set_motor_config(interface_motor_config_);
    }
}

// 观察者构造函数
RobotHardware::RobotHardware(
    std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
    const std::map<std::string, std::vector<uint32_t>>& interface_motor_config,
    std::shared_ptr<hardware_driver::motor_driver::MotorStatusObserver> observer)
    : motor_driver_(std::move(motor_driver)),
      interface_motor_config_(interface_motor_config),
      status_callback_(nullptr),
      batch_status_callback_(nullptr),
      event_bus_(nullptr),
      current_observer_(observer),
      current_event_handler_(nullptr),
      monitoring_paused_(false),
      event_subscriptions_()
{
    // 转换为MotorDriverImpl以访问观察者接口
    auto motor_driver_impl = std::dynamic_pointer_cast<hardware_driver::motor_driver::MotorDriverImpl>(motor_driver_);

    if (motor_driver_impl && current_observer_) {
        // 添加观察者
        motor_driver_impl->add_observer(current_observer_);

        // 设置电机配置，启动反馈请求
        motor_driver_impl->set_motor_config(interface_motor_config_);

        std::cout << "RobotHardware initialized with Observer - status updates will be handled by observer" << std::endl;
    }
}

// IAP观察者构造函数
RobotHardware::RobotHardware(
    std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
    const std::map<std::string, std::vector<uint32_t>>& interface_motor_config,
    std::shared_ptr<hardware_driver::motor_driver::IAPStatusObserver> iap_observer)
    : motor_driver_(std::move(motor_driver)),
      interface_motor_config_(interface_motor_config),
      status_callback_(nullptr),
      batch_status_callback_(nullptr),
      event_bus_(nullptr),
      current_observer_(nullptr),
      current_iap_observer_(iap_observer),
      current_event_handler_(nullptr),
      monitoring_paused_(false),
      event_subscriptions_()
{
    // 转换为MotorDriverImpl以访问IAP观察者接口
    auto motor_driver_impl = std::dynamic_pointer_cast<hardware_driver::motor_driver::MotorDriverImpl>(motor_driver_);

    if (motor_driver_impl && current_iap_observer_) {
        // 添加IAP观察者
        motor_driver_impl->add_iap_observer(current_iap_observer_);

        // 设置电机配置，启动反馈请求
        motor_driver_impl->set_motor_config(interface_motor_config_);

        std::cout << "RobotHardware initialized with IAPStatusObserver - IAP updates will be handled by observer" << std::endl;
    }
}

// 事件总线构造函数（带事件处理器）
RobotHardware::RobotHardware(
    std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
    const std::map<std::string, std::vector<uint32_t>>& interface_motor_config,
    std::shared_ptr<hardware_driver::event::EventBus> event_bus,
    std::shared_ptr<hardware_driver::motor_driver::MotorEventHandler> event_handler)
    : motor_driver_(std::move(motor_driver)),
      interface_motor_config_(interface_motor_config),
      status_callback_(nullptr),
      batch_status_callback_(nullptr),
      event_bus_(std::move(event_bus)),
      current_observer_(nullptr),
      current_event_handler_(event_handler),
      monitoring_paused_(false),
      event_subscriptions_()
{
    // 转换为MotorDriverImpl以访问新接口
    auto motor_driver_impl = std::dynamic_pointer_cast<hardware_driver::motor_driver::MotorDriverImpl>(motor_driver_);

    if (motor_driver_impl && event_bus_ && current_event_handler_) {
        // 设置事件总线
        motor_driver_impl->set_event_bus(event_bus_);
        
        // 订阅电机状态事件，转发给事件处理器
        event_subscriptions_.push_back(
            event_bus_->subscribe<hardware_driver::event::MotorStatusEvent>(
            [this](const std::shared_ptr<hardware_driver::event::MotorStatusEvent>& event) {
                if (current_event_handler_ && !monitoring_paused_) {
                    current_event_handler_->on_motor_status_update(
                        event->get_interface(), 
                        event->get_motor_id(), 
                        event->get_status()
                    );
                }
                // else {
                //     std::cout << "DEBUG: 事件处理器为空或已暂停 - handler:" << (current_event_handler_ ? "有效" : "空") 
                //               << ", paused:" << monitoring_paused_ << std::endl;
                // }
            }
            )
        );
        
        // 订阅批量电机状态事件
        event_subscriptions_.push_back(
            event_bus_->subscribe<hardware_driver::event::MotorBatchStatusEvent>(
            [this](const std::shared_ptr<hardware_driver::event::MotorBatchStatusEvent>& event) {
                if (current_event_handler_ && !monitoring_paused_) {
                    current_event_handler_->on_motor_status_update(
                        event->get_interface(), 
                        event->get_status_all()
                    );
                }
            }
            )
        );
        
        // 订阅函数操作结果事件
        event_subscriptions_.push_back(
            event_bus_->subscribe<hardware_driver::event::MotorFunctionResultEvent>(
            [this](const std::shared_ptr<hardware_driver::event::MotorFunctionResultEvent>& event) {
                if (current_event_handler_ && !monitoring_paused_) {
                    current_event_handler_->on_motor_function_result(
                        event->get_interface(), 
                        event->get_motor_id(), 
                        event->get_operation_code(), 
                        event->is_success()
                    );
                }
            }
            )
        );
        
        // 订阅参数操作结果事件
        event_subscriptions_.push_back(
            event_bus_->subscribe<hardware_driver::event::MotorParameterResultEvent>(
            [this](const std::shared_ptr<hardware_driver::event::MotorParameterResultEvent>& event) {
                if (current_event_handler_ && !monitoring_paused_) {
                    current_event_handler_->on_motor_parameter_result(
                        event->get_interface(), 
                        event->get_motor_id(), 
                        event->get_address(), 
                        event->get_data_type(), 
                        event->get_data()
                    );
                }
            }
            )
        );
        
        // 设置电机配置，启动反馈请求
        motor_driver_impl->set_motor_config(interface_motor_config_);
        
        std::cout << "RobotHardware initialized with EventBus and EventHandler" << std::endl;
    }
}

RobotHardware::~RobotHardware() = default;

// 内部状态聚合方法实现
void RobotHardware::handle_motor_status_with_aggregation(const std::string& interface, uint32_t motor_id, 
                                                       const hardware_driver::motor_driver::Motor_Status& status) {
    // 调用单个状态回调
    if (status_callback_) {
        status_callback_(interface, motor_id, status);
    }
    
    // 如果有批量回调，进行聚合处理
    if (batch_status_callback_) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        
        // 更新状态缓存
        status_cache_[interface][motor_id] = status;
        
        // 检查是否收集齐该总线上的所有电机数据
        auto config_it = interface_motor_config_.find(interface);
        if (config_it != interface_motor_config_.end()) {
            const auto& expected_motors = config_it->second;
            auto& interface_cache = status_cache_[interface];
            
            // 检查是否所有期望的电机都有数据
            bool all_received = true;
            for (uint32_t expected_motor : expected_motors) {
                if (interface_cache.find(expected_motor) == interface_cache.end()) {
                    all_received = false;
                    break;
                }
            }
            
            // 如果收集齐所有电机数据，调用批量回调
            if (all_received) {
                batch_status_callback_(interface, interface_cache);
                
                // 清空缓存，为下一轮收集做准备
                interface_cache.clear();
            }
        }
    }
}

// 电机控制接口 
// 简化控制接口 - 时间控制由motor_driver_impl内部处理

void RobotHardware::control_motor_in_position_mode(const std::string& interface, const uint32_t motor_id, float position,
                                                float kp, float kd) {
    motor_driver_->send_position_cmd(interface, motor_id, position, kp, kd);
}

void RobotHardware::control_motor_in_velocity_mode(const std::string& interface, const uint32_t motor_id, float velocity,
                                                float kp, float kd) {
    motor_driver_->send_velocity_cmd(interface, motor_id, velocity, kp, kd);
}

void RobotHardware::control_motor_in_effort_mode(const std::string& interface, const uint32_t motor_id, float effort,
                                                float kp, float kd) {
    motor_driver_->send_effort_cmd(interface, motor_id, effort, kp, kd);
}

void RobotHardware::control_motor_in_mit_mode(const std::string& interface, const uint32_t motor_id, float position, float velocity, float effort,
                                                float kp, float kd) {
    motor_driver_->send_mit_cmd(interface, motor_id, position, velocity, effort, kp, kd);
}

void RobotHardware::disable_motor(const std::string& interface, const uint32_t motor_id, uint8_t mode) {
    motor_driver_->disable_motor(interface, motor_id, mode);
}

void RobotHardware::enable_motor(const std::string& interface, const uint32_t motor_id, uint8_t mode) {
    motor_driver_->enable_motor(interface, motor_id, mode);
}

// 实时批量控制接口 
void RobotHardware::disable_motors(const std::string& interface, const std::vector<uint32_t>& motor_ids, uint8_t mode) {
    motor_driver_->disable_all_motors(interface, motor_ids, mode);
}

void RobotHardware::enable_motors(const std::string& interface, const std::vector<uint32_t>& motor_ids, uint8_t mode) {
    motor_driver_->enable_all_motors(interface, motor_ids, mode);
}

bool RobotHardware::send_realtime_velocity_command(const std::string& interface,
                                                  const std::array<double, 6>& joint_velocities,
                                                  const std::array<double, 6>& kps,
                                                  const std::array<double, 6>& kds) {

    auto config_it = interface_motor_config_.find(interface);
    if (config_it == interface_motor_config_.end()) {
        return false;
    }

    try {
        const auto& motor_ids = config_it->second;

        // 验证电机数量不超过 6
        if (motor_ids.size() > 6) {
            return false;
        }

        // 转换为 float array
        std::array<float, 6> velocities = {};
        std::array<float, 6> kps_float = {};
        std::array<float, 6> kds_float = {};

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            velocities[i] = static_cast<float>(joint_velocities[i]);
            kps_float[i] = static_cast<float>(kps[i]);
            kds_float[i] = static_cast<float>(kds[i]);
        }

        // 使用批量控制接口
        motor_driver_->send_velocity_cmd_all(interface, velocities, kps_float, kds_float);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool RobotHardware::send_realtime_position_command(const std::string& interface,
                                                  const std::array<double, 6>& joint_positions,
                                                  const std::array<double, 6>& kps,
                                                  const std::array<double, 6>& kds) {

    auto config_it = interface_motor_config_.find(interface);
    if (config_it == interface_motor_config_.end()) {
        return false;
    }

    try {
        const auto& motor_ids = config_it->second;

        // 验证电机数量不超过 6
        if (motor_ids.size() > 6) {
            return false;
        }

        // 转换为 float array
        std::array<float, 6> positions = {};
        std::array<float, 6> kps_float = {};
        std::array<float, 6> kds_float = {};

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            positions[i] = static_cast<float>(joint_positions[i]);
            kps_float[i] = static_cast<float>(kps[i]);
            kds_float[i] = static_cast<float>(kds[i]);
        }

        // 使用批量控制接口
        motor_driver_->send_position_cmd_all(interface, positions, kps_float, kds_float);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool RobotHardware::send_realtime_effort_command(const std::string& interface,
                                                const std::array<double, 6>& joint_efforts,
                                                const std::array<double, 6>& kps,
                                                const std::array<double, 6>& kds) {

    auto config_it = interface_motor_config_.find(interface);
    if (config_it == interface_motor_config_.end()) {
        return false;
    }

    try {
        const auto& motor_ids = config_it->second;

        // 验证电机数量不超过 6
        if (motor_ids.size() > 6) {
            return false;
        }

        // 转换为 float array
        std::array<float, 6> efforts = {};
        std::array<float, 6> kps_float = {};
        std::array<float, 6> kds_float = {};

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            efforts[i] = static_cast<float>(joint_efforts[i]);
            kps_float[i] = static_cast<float>(kps[i]);
            kds_float[i] = static_cast<float>(kds[i]);
        }

        // 使用批量控制接口
        motor_driver_->send_effort_cmd_all(interface, efforts, kps_float, kds_float);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool RobotHardware::send_realtime_mit_command(const std::string& interface,
                                              const std::array<double, 6>& joint_positions,
                                              const std::array<double, 6>& joint_velocities,
                                              const std::array<double, 6>& joint_efforts,
                                              const std::array<double, 6>& kps,
                                              const std::array<double, 6>& kds) {

    auto config_it = interface_motor_config_.find(interface);
    if (config_it == interface_motor_config_.end()) {
        return false;
    }

    try {
        const auto& motor_ids = config_it->second;

        // 验证电机数量不超过 6
        if (motor_ids.size() > 6) {
            return false;
        }

        // 转换为 float array
        std::array<float, 6> positions = {};
        std::array<float, 6> velocities = {};
        std::array<float, 6> efforts = {};
        std::array<float, 6> kps_float = {};
        std::array<float, 6> kds_float = {};

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            positions[i] = static_cast<float>(joint_positions[i]);
            velocities[i] = static_cast<float>(joint_velocities[i]);
            efforts[i] = static_cast<float>(joint_efforts[i]);
            kps_float[i] = static_cast<float>(kps[i]);
            kds_float[i] = static_cast<float>(kds[i]);
        }

        // 使用批量控制接口
        motor_driver_->send_mit_cmd_all(interface, positions, velocities, efforts, kps_float, kds_float);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

//  轨迹执行接口 
bool RobotHardware::execute_trajectory(const std::string& interface, const Trajectory& trajectory) {
    auto config_it = interface_motor_config_.find(interface);
    if (config_it == interface_motor_config_.end()) {
        return false;
    }
    
    if (trajectory.points.empty()) {
        return false;
    }
    
    try {
        const auto& motor_ids = config_it->second;
        auto start_time = std::chrono::steady_clock::now();
        size_t total_points = trajectory.points.size();
        
        // 使用原子变量跟踪执行进度，供进度条线程使用
        std::atomic<size_t> current_point{0};
        std::atomic<bool> execution_done{false};
        
        // 启动进度条显示线程，直接写入终端设备
        std::thread progress_thread([&current_point, &execution_done, total_points, &interface]() {
            // 尝试直接打开终端设备
            FILE* tty = fopen("/dev/tty", "w");
            if (!tty) {
                tty = stderr; // 如果打开失败，回退到stderr
            }
            
            while (!execution_done) {
                size_t point_idx = current_point.load();
                
                // 只有当有实际进度时才显示进度条（跳过0%的显示）
                if (point_idx > 0) {
                    double progress = static_cast<double>(point_idx) / total_points;
                    int bar_width = 30;
                    int filled = static_cast<int>(progress * bar_width);
                    
                    fprintf(tty, "\r[%s] 轨迹执行进度: [", interface.c_str());
                    for (int i = 0; i < bar_width; ++i) {
                        if (i < filled) fprintf(tty, "█");
                        else fprintf(tty, "░");
                    }
                    fprintf(tty, "] %.1f%% (点 %zu/%zu)", progress * 100.0, point_idx, total_points);
                    fflush(tty);
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            // 显示最终100%进度
            fprintf(tty, "\r[%s] 轨迹执行进度: [", interface.c_str());
            for (int i = 0; i < 30; ++i) {
                fprintf(tty, "█");
            }
            fprintf(tty, "] 100.0%% (点 %zu/%zu)\n", total_points, total_points);
            fflush(tty);
            
            if (tty != stderr) {
                fclose(tty);
            }
        });
        
        // 执行轨迹，不添加任何延时
        for (size_t point_idx = 0; point_idx < trajectory.points.size(); ++point_idx) {
            const auto& point = trajectory.points[point_idx];

            // 计算目标时间并等待
            auto target_time = start_time + std::chrono::duration<double>(point.time_from_start);
            auto current_time = std::chrono::steady_clock::now();

            // 如果还没到时间，就等待
            if (current_time < target_time) {
                std::this_thread::sleep_until(target_time);
            }

            // 使用批量控制发送所有电机的位置命令
            std::array<float, 6> positions = {};
            std::array<float, 6> velocities = {};
            std::array<float, 6> efforts = {};

            for (size_t i = 0; i < motor_ids.size(); ++i) {
                float position = (i < point.positions.size()) ? static_cast<float>(point.positions[i]) : 0.0f;
                positions[i] = position;
                
            }

            // 调用批量MIT控制接口（kps 和 kds 使用默认值 0.05 和 0.005）
            motor_driver_->send_mit_cmd_all(interface, positions, velocities, efforts);

            // 更新进度（供进度条线程使用）
            current_point.store(point_idx + 1);
        }
        
        // 标记执行完成并等待进度条线程结束
        execution_done = true;
        progress_thread.join();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// ========== 参数读写接口 ==========
void RobotHardware::motor_parameter_read(const std::string& interface, const uint32_t motor_id, uint16_t address) {
    motor_driver_->motor_parameter_read(interface, motor_id, address);
}

void RobotHardware::motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, int32_t value) {
    motor_driver_->motor_parameter_write(interface, motor_id, address, value);
}

void RobotHardware::motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, float value) {
    motor_driver_->motor_parameter_write(interface, motor_id, address, value);
}

// ========== 函数操作接口 ==========
void RobotHardware::motor_function_operation(const std::string& interface, const uint32_t motor_id, uint8_t operation) {
    motor_driver_->motor_function_operation(interface, motor_id, operation);
}

void RobotHardware::arm_zero_position_set(const std::string& interface, const std::vector<uint32_t> motor_ids) {
    for (size_t i = 0; i < motor_ids.size(); i++) {
       // 直接发送，在send_packet里加锁保护
       motor_driver_->motor_function_operation(interface, motor_ids.at(i), 4);
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
       motor_driver_->motor_function_operation(interface, motor_ids.at(i), 2);
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void RobotHardware::start_update(const std::string& interface, const uint8_t motor_id, const std::string& firmware_file) {
    motor_driver_->start_update(interface, motor_id, firmware_file);
}

// ========== 状态监控控制方法 ==========
void RobotHardware::pause_status_monitoring() {
    // 转换为MotorDriverImpl以访问内部方法
    auto motor_driver_impl = std::dynamic_pointer_cast<hardware_driver::motor_driver::MotorDriverImpl>(motor_driver_);
    
    if (motor_driver_impl) {
        if (current_observer_) {
            // 观察者模式：移除观察者
            motor_driver_impl->remove_observer(current_observer_);
            monitoring_paused_ = true;
        } else if (status_callback_ || batch_status_callback_) {
            // 回调函数模式：注册空回调函数
            motor_driver_impl->register_feedback_callback(nullptr);
            monitoring_paused_ = true;
        } else if (current_event_handler_) {
            // 事件总线模式：暂停事件处理（通过标记实现）
            monitoring_paused_ = true;
        }
    }
}

void RobotHardware::resume_status_monitoring() {
    // 转换为MotorDriverImpl以访问内部方法
    auto motor_driver_impl = std::dynamic_pointer_cast<hardware_driver::motor_driver::MotorDriverImpl>(motor_driver_);
    
    if (motor_driver_impl && monitoring_paused_) {
        if (current_observer_) {
            // 观察者模式：重新添加观察者
            motor_driver_impl->add_observer(current_observer_);
            monitoring_paused_ = false;
        } else if (status_callback_) {
            // 单个状态回调模式：重新注册原回调函数
            motor_driver_impl->register_feedback_callback(status_callback_);
            monitoring_paused_ = false;
        } else if (batch_status_callback_) {
            // 批量状态回调模式：重新注册内部聚合回调
            motor_driver_impl->register_feedback_callback(
                [this](const std::string& interface, uint32_t motor_id, 
                       const hardware_driver::motor_driver::Motor_Status& status) {
                    this->handle_motor_status_with_aggregation(interface, motor_id, status);
                }
            );
            monitoring_paused_ = false;
        } else if (current_event_handler_) {
            // 事件总线模式：恢复事件处理
            monitoring_paused_ = false;
        }
    }
}

// ========== 异步轨迹执行实现（新的简化版本） ==========

std::string RobotHardware::execute_trajectory_async(
    const std::string& interface,
    const Trajectory& trajectory,
    bool show_progress) {

    // 检查接口是否存在
    auto config_it = interface_motor_config_.find(interface);
    if (config_it == interface_motor_config_.end()) {
        return "";
    }

    if (trajectory.points.empty()) {
        return "";
    }

    // 检查该接口是否已有执行中的轨迹（包括IDLE、RUNNING、PAUSED状态）
    // {
    //     std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);
    //     for (const auto& [exec_id, task] : trajectory_execution_tasks_) {
    //         if (task->interface == interface) {
    //             auto state = task->state.load();
    //             // 任何非终止状态都表示轨迹占用了接口
    //             if (state != TrajectoryExecutionState::COMPLETED &&
    //                 state != TrajectoryExecutionState::CANCELLED &&
    //                 state != TrajectoryExecutionState::ERROR) {
    //                 return "";
    //             }
    //         }
    //     }
    // }

    // 生成执行ID并创建执行任务
    std::string execution_id = generate_execution_id();
    auto task = std::make_shared<TrajectoryExecutionTask>();
    task->execution_id = execution_id;
    task->interface = interface;
    task->trajectory = trajectory;
    task->state = TrajectoryExecutionState::RUNNING;  // 立即设为RUNNING，防止竞态
    task->show_progress = show_progress;

    // 添加到任务队列
    {
        std::unique_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);
        trajectory_execution_tasks_[execution_id] = task;
    }

    // 启动执行线程
    task->executor_thread = std::thread(&RobotHardware::trajectory_execution_worker, this, task);

    return execution_id;
}

bool RobotHardware::pause_trajectory(const std::string& execution_id) {
    std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);

    auto it = trajectory_execution_tasks_.find(execution_id);
    if (it == trajectory_execution_tasks_.end()) {
        return false;
    }

    auto task = it->second;
    auto state = task->state.load();

    if (state == TrajectoryExecutionState::RUNNING) {
        {
            std::unique_lock<std::mutex> state_lock(task->state_mutex);
            task->should_pause = true;
            task->state = TrajectoryExecutionState::PAUSED;
        }
        task->state_cv.notify_one();
        return true;
    }

    return false;
}

bool RobotHardware::resume_trajectory(const std::string& execution_id) {
    std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);

    auto it = trajectory_execution_tasks_.find(execution_id);
    if (it == trajectory_execution_tasks_.end()) {
        return false;
    }

    auto task = it->second;
    auto state = task->state.load();

    if (state == TrajectoryExecutionState::PAUSED) {
        {
            std::unique_lock<std::mutex> state_lock(task->state_mutex);
            task->should_pause = false;
            task->pause_elapsed += std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - task->pause_start_time);
            task->state = TrajectoryExecutionState::RUNNING;
        }
        task->state_cv.notify_one();
        return true;
    }

    return false;
}

bool RobotHardware::cancel_trajectory(const std::string& execution_id) {
    std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);

    auto it = trajectory_execution_tasks_.find(execution_id);
    if (it == trajectory_execution_tasks_.end()) {
        return false;
    }

    auto task = it->second;
    auto state = task->state.load();

    if (state == TrajectoryExecutionState::RUNNING || state == TrajectoryExecutionState::PAUSED) {
        {
            std::unique_lock<std::mutex> state_lock(task->state_mutex);
            task->should_stop = true;
            task->state = TrajectoryExecutionState::CANCELLED;
        }
        task->state_cv.notify_one();
        return true;
    }

    return false;
}

bool RobotHardware::get_execution_progress(const std::string& execution_id,
                                          TrajectoryExecutionProgress& progress) {
    std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);

    auto it = trajectory_execution_tasks_.find(execution_id);
    if (it == trajectory_execution_tasks_.end()) {
        return false;
    }

    auto task = it->second;

    {
        std::unique_lock<std::mutex> state_lock(task->state_mutex);

        progress.state = task->state.load();
        progress.current_point_index = task->current_point_index.load();
        progress.total_points = task->trajectory.points.size();
        progress.error_message = task->error_message;

        // 计算进度百分比
        if (progress.total_points > 0) {
            progress.progress_percentage = (static_cast<double>(progress.current_point_index) / progress.total_points) * 100.0;
        } else {
            progress.progress_percentage = 0.0;
        }

        auto now = std::chrono::steady_clock::now();
        progress.elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - task->start_time) - task->pause_elapsed;

        auto total_time = get_trajectory_total_time(task->trajectory);
        progress.estimated_remaining_time = total_time - progress.elapsed_time;
        if (progress.estimated_remaining_time.count() < 0) {
            progress.estimated_remaining_time = std::chrono::milliseconds(0);
        }
    }

    return true;
}

std::vector<std::string> RobotHardware::get_active_execution_ids() const {
    std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);

    std::vector<std::string> active_ids;
    for (const auto& [exec_id, task] : trajectory_execution_tasks_) {
        auto state = task->state.load();
        if (state == TrajectoryExecutionState::RUNNING ||
            state == TrajectoryExecutionState::PAUSED) {
            active_ids.push_back(exec_id);
        }
    }

    return active_ids;
}

bool RobotHardware::has_execution(const std::string& execution_id) const {
    std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);
    return trajectory_execution_tasks_.find(execution_id) != trajectory_execution_tasks_.end();
}

bool RobotHardware::wait_for_completion(const std::string& execution_id, int timeout_ms) {
    std::shared_ptr<TrajectoryExecutionTask> task;

    {
        std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);
        auto it = trajectory_execution_tasks_.find(execution_id);
        if (it == trajectory_execution_tasks_.end()) {
            return false;
        }
        task = it->second;
    }

    std::unique_lock<std::mutex> state_lock(task->state_mutex);

    auto pred = [task]() {
        auto state = task->state.load();
        return state == TrajectoryExecutionState::COMPLETED ||
               state == TrajectoryExecutionState::CANCELLED ||
               state == TrajectoryExecutionState::ERROR;
    };

    bool result;
    if (timeout_ms == 0) {
        task->state_cv.wait(state_lock, pred);
        result = true;
    } else {
        result = task->state_cv.wait_for(state_lock, std::chrono::milliseconds(timeout_ms), pred);
    }

    // 任务完成后，清理该任务（防止资源泄漏）
    if (result) {
        state_lock.unlock();
        std::unique_lock<std::shared_mutex> tasks_lock(trajectory_tasks_mutex_);
        if (task->executor_thread.joinable()) {
            task->executor_thread.join();
        }
        trajectory_execution_tasks_.erase(execution_id);
    }

    return result;
}

void RobotHardware::pause_all_trajectories() {
    std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);

    for (const auto& [exec_id, task] : trajectory_execution_tasks_) {
        if (task->state.load() == TrajectoryExecutionState::RUNNING) {
            {
                std::unique_lock<std::mutex> state_lock(task->state_mutex);
                task->should_pause = true;
                task->state = TrajectoryExecutionState::PAUSED;
            }
            task->state_cv.notify_one();
        }
    }
}

void RobotHardware::resume_all_trajectories() {
    std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);

    for (const auto& [exec_id, task] : trajectory_execution_tasks_) {
        if (task->state.load() == TrajectoryExecutionState::PAUSED) {
            {
                std::unique_lock<std::mutex> state_lock(task->state_mutex);
                task->should_pause = false;
                task->pause_elapsed += std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - task->pause_start_time);
                task->state = TrajectoryExecutionState::RUNNING;
            }
            task->state_cv.notify_one();
        }
    }
}

void RobotHardware::cancel_all_trajectories() {
    std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);

    for (const auto& [exec_id, task] : trajectory_execution_tasks_) {
        auto state = task->state.load();
        if (state == TrajectoryExecutionState::RUNNING ||
            state == TrajectoryExecutionState::PAUSED) {
            {
                std::unique_lock<std::mutex> state_lock(task->state_mutex);
                task->should_stop = true;
                task->state = TrajectoryExecutionState::CANCELLED;
            }
            task->state_cv.notify_one();
        }
    }
}

size_t RobotHardware::get_active_trajectory_count() const {
    std::shared_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);

    size_t count = 0;
    for (const auto& [exec_id, task] : trajectory_execution_tasks_) {
        auto state = task->state.load();
        if (state == TrajectoryExecutionState::RUNNING ||
            state == TrajectoryExecutionState::PAUSED) {
            count++;
        }
    }

    return count;
}

// 私有执行线程函数
void RobotHardware::trajectory_execution_worker(std::shared_ptr<TrajectoryExecutionTask> task) {
    try {
        auto config_it = interface_motor_config_.find(task->interface);
        if (config_it == interface_motor_config_.end()) {
            std::unique_lock<std::mutex> state_lock(task->state_mutex);
            task->state = TrajectoryExecutionState::ERROR;
            task->error_message = "Interface not found";
            state_lock.unlock();
            task->state_cv.notify_all();
            return;
        }

        const auto& motor_ids = config_it->second;
        size_t total_points = task->trajectory.points.size();

        // 更新状态为RUNNING
        {
            std::unique_lock<std::mutex> state_lock(task->state_mutex);
            task->state = TrajectoryExecutionState::RUNNING;
            task->start_time = std::chrono::steady_clock::now();
            task->current_point_index = 0;
        }

        // 打开进度条输出
        FILE* tty = nullptr;
        if (task->show_progress) {
            tty = fopen(progress_display_tty_.c_str(), "w");
            if (!tty) {
                tty = stderr;
            }
        }

        auto last_progress_update = std::chrono::steady_clock::now();

        // 执行轨迹
        for (size_t point_idx = 0; point_idx < total_points; ++point_idx) {
            // 检查是否应该停止或暂停
            {
                std::unique_lock<std::mutex> state_lock(task->state_mutex);

                if (task->should_stop) {
                    task->state = TrajectoryExecutionState::CANCELLED;
                    state_lock.unlock();

                    if (tty && tty != stderr) {
                        fclose(tty);
                    }
                    return;
                }

                // 处理暂停
                while (task->should_pause) {
                    if (task->pause_start_time == std::chrono::steady_clock::time_point()) {
                        task->pause_start_time = std::chrono::steady_clock::now();
                    }

                    state_lock.unlock();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    state_lock.lock();
                }
            }

            const auto& point = task->trajectory.points[point_idx];

            // 计算目标时间并等待（考虑暂停时间偏移）
            auto adjusted_start = task->start_time + task->pause_elapsed;
            auto target_time = adjusted_start + std::chrono::duration<double>(point.time_from_start);
            auto current_time = std::chrono::steady_clock::now();

            if (current_time < target_time) {
                std::this_thread::sleep_until(target_time);
            }

            // 发送控制命令
            std::array<float, 6> positions = {};
            std::array<float, 6> velocities = {};
            std::array<float, 6> efforts = {};

            for (size_t i = 0; i < motor_ids.size(); ++i) {
                float position = (i < point.positions.size()) ? static_cast<float>(point.positions[i]) : 0.0f;
                positions[i] = position;
            }

            motor_driver_->send_mit_cmd_all(task->interface, positions, velocities, efforts);

            // 更新进度
            {
                std::unique_lock<std::mutex> state_lock(task->state_mutex);
                task->current_point_index = point_idx + 1;
            }

            // 显示进度条（仅当 show_progress 为 true 时）
            if (task->show_progress && tty) {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_progress_update).count() >= 100) {
                    display_trajectory_progress(task);
                    last_progress_update = now;
                }
            }
        }

        // 执行完成
        {
            std::unique_lock<std::mutex> state_lock(task->state_mutex);
            task->state = TrajectoryExecutionState::COMPLETED;
            task->current_point_index = total_points;
        }

        // 显示最终进度
        if (task->show_progress && tty) {
            display_trajectory_progress(task);
            fprintf(tty, "\n");
            fflush(tty);
        }

        if (tty && tty != stderr) {
            fclose(tty);
        }

        task->state_cv.notify_all();

    } catch (const std::exception& e) {
        std::unique_lock<std::mutex> state_lock(task->state_mutex);
        task->state = TrajectoryExecutionState::ERROR;
        task->error_message = std::string(e.what());
        state_lock.unlock();
        task->state_cv.notify_all();
    }
}

std::string RobotHardware::generate_execution_id() {
    static std::atomic<uint64_t> counter{0};
    auto id = counter.fetch_add(1);
    return "exec_" + std::to_string(id) + "_" +
           std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
}

void RobotHardware::display_trajectory_progress(const std::shared_ptr<TrajectoryExecutionTask>& task) {
    FILE* tty = fopen(progress_display_tty_.c_str(), "w");
    if (!tty) {
        tty = stderr;
    }

    TrajectoryExecutionProgress progress;
    if (!get_execution_progress(task->execution_id, progress)) {
        if (tty && tty != stderr) {
            fclose(tty);
        }
        return;
    }

    if (progress.total_points > 0) {
        int bar_width = 30;
        double progress_percentage = (static_cast<double>(progress.current_point_index) / progress.total_points) * 100.0;
        int filled = static_cast<int>((progress_percentage / 100.0) * bar_width);

        fprintf(tty, "\r[%s] [", task->interface.c_str());
        for (int i = 0; i < bar_width; ++i) {
            if (i < filled) fprintf(tty, "█");
            else fprintf(tty, "░");
        }
        fprintf(tty, "] %.0f%% 点 %zu/%zu | 耗时: %ldms   ",
                progress_percentage,
                progress.current_point_index,
                progress.total_points,
                static_cast<long>(progress.elapsed_time.count()));
        fflush(tty);
    }

    if (tty && tty != stderr) {
        fclose(tty);
    }
}

std::chrono::milliseconds RobotHardware::get_trajectory_total_time(const Trajectory& trajectory) const {
    if (trajectory.points.empty()) {
        return std::chrono::milliseconds(0);
    }
    return std::chrono::milliseconds(
        static_cast<long long>(trajectory.points.back().time_from_start * 1000)
    );
}

void RobotHardware::cleanup_completed_trajectory_tasks() {
    std::unique_lock<std::shared_mutex> lock(trajectory_tasks_mutex_);

    std::vector<std::string> to_remove;
    for (auto& [exec_id, task] : trajectory_execution_tasks_) {
        auto state = task->state.load();
        if (state == TrajectoryExecutionState::COMPLETED ||
            state == TrajectoryExecutionState::CANCELLED ||
            state == TrajectoryExecutionState::ERROR) {
            if (task->executor_thread.joinable()) {
                task->executor_thread.join();
            }
            to_remove.push_back(exec_id);
        }
    }

    for (const auto& exec_id : to_remove) {
        trajectory_execution_tasks_.erase(exec_id);
    }
}

// ========== 工厂函数实现 ==========

// 夹爪驱动设置方法实现
void RobotHardware::set_gripper_driver(
    std::shared_ptr<hardware_driver::gripper_driver::GripperDriverInterface> gripper_driver) {
    gripper_driver_ = std::move(gripper_driver);
    if (gripper_driver_) {
        std::cout << "[RobotHardware] Gripper driver initialized successfully" << std::endl;
    }
}

// 夹爪控制方法实现
void RobotHardware::control_gripper(const std::string& interface,
                                   uint8_t gripper_type,
                                   uint8_t position,
                                   uint8_t velocity,
                                   uint8_t effort) {
    if (!gripper_driver_) {
        std::cerr << "[RobotHardware] Gripper driver not initialized" << std::endl;
        return;
    }

    auto type = static_cast<hardware_driver::gripper_driver::GripperType>(gripper_type);
    gripper_driver_->control_gripper(interface, type, position, velocity, effort);
}

void RobotHardware::open_gripper(const std::string& interface,
                                uint8_t gripper_type,
                                uint8_t velocity,
                                uint8_t effort) {
    if (!gripper_driver_) {
        std::cerr << "[RobotHardware] Gripper driver not initialized" << std::endl;
        return;
    }

    auto type = static_cast<hardware_driver::gripper_driver::GripperType>(gripper_type);
    gripper_driver_->open_gripper(interface, type, velocity, effort);
}

void RobotHardware::close_gripper(const std::string& interface,
                                 uint8_t gripper_type,
                                 uint8_t velocity,
                                 uint8_t effort) {
    if (!gripper_driver_) {
        std::cerr << "[RobotHardware] Gripper driver not initialized" << std::endl;
        return;
    }

    auto type = static_cast<hardware_driver::gripper_driver::GripperType>(gripper_type);
    gripper_driver_->close_gripper(interface, type, velocity, effort);
}

void RobotHardware::send_gripper_raw_data(const std::string& interface,
                                         const uint8_t* raw_data,
                                         size_t raw_data_len) {
    if (!gripper_driver_) {
        std::cerr << "[RobotHardware] Gripper driver not initialized" << std::endl;
        return;
    }

    gripper_driver_->send_raw_data(interface, raw_data, raw_data_len);
}

// 工厂函数实现 
namespace hardware_driver {
    std::shared_ptr<motor_driver::MotorDriverInterface> createCanFdMotorDriver(
        const std::vector<std::string>& interfaces) {
        // 创建CANFD总线
        auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);

        // 创建电机驱动实例
        return std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
    }

    std::shared_ptr<gripper_driver::GripperDriverInterface> createCanFdGripperDriver(
        const std::vector<std::string>& interfaces) {
        // 创建CANFD总线
        auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);

        // 创建夹爪驱动实例
        return std::make_shared<hardware_driver::gripper_driver::GripperDriverImpl>(bus);
    }

    // std::shared_ptr<motor_driver::MotorDriverInterface> createUsb2CanfdMotorDriver(
    //     const std::vector<std::string>& device_sns) {
    //     // 创建USB2CANFD总线
    //     auto bus = std::make_shared<hardware_driver::bus::Usb2CanfdBus>(device_sns);
    //
    //     // 创建电机驱动实例
    //     return std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
    // }
}
