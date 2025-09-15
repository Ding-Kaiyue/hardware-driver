#include "hardware_driver/interface/robot_hardware.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"
#include <chrono>
#include <thread>
#include <cstring>

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

// ========== 电机控制接口 ==========
// 简化控制接口 - 时间控制由motor_driver_impl内部处理

void RobotHardware::control_motor_in_position_mode(const std::string& interface, const uint32_t motor_id, float position) {
    motor_driver_->send_position_cmd(interface, motor_id, position);
}

void RobotHardware::control_motor_in_velocity_mode(const std::string& interface, const uint32_t motor_id, float velocity) {
    motor_driver_->send_velocity_cmd(interface, motor_id, velocity);
}

void RobotHardware::control_motor_in_effort_mode(const std::string& interface, const uint32_t motor_id, float effort) {
    motor_driver_->send_effort_cmd(interface, motor_id, effort);
}

void RobotHardware::control_motor_in_mit_mode(const std::string& interface, const uint32_t motor_id, float position, float velocity, float effort) {
    motor_driver_->send_mit_cmd(interface, motor_id, position, velocity, effort);
}

void RobotHardware::disable_motor(const std::string& interface, const uint32_t motor_id) {
    motor_driver_->disable_motor(interface, motor_id);
}

void RobotHardware::enable_motor(const std::string& interface, const uint32_t motor_id, uint8_t mode) {
    motor_driver_->enable_motor(interface, motor_id, mode);
}

// ========== 实时批量控制接口 ==========
bool RobotHardware::send_realtime_velocity_command(const std::string& interface, const std::vector<double>& joint_velocities) {
    auto config_it = interface_motor_config_.find(interface);
    if (config_it == interface_motor_config_.end()) {
        return false;
    }
    
    try {
        const auto& motor_ids = config_it->second;
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            float velocity = (i < joint_velocities.size()) ? static_cast<float>(joint_velocities[i]) : 0.0f;
            control_motor_in_velocity_mode(interface, motor_ids[i], velocity);
        }
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool RobotHardware::send_realtime_position_command(const std::string& interface, const std::vector<double>& joint_positions) {
    auto config_it = interface_motor_config_.find(interface);
    if (config_it == interface_motor_config_.end()) {
        return false;
    }
    
    try {        
        const auto& motor_ids = config_it->second;
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            float position = (i < joint_positions.size()) ? static_cast<float>(joint_positions[i]) : 0.0f;
            control_motor_in_position_mode(interface, motor_ids[i], position);
        }
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool RobotHardware::send_realtime_effort_command(const std::string& interface, const std::vector<double>& joint_efforts) {
    auto config_it = interface_motor_config_.find(interface);
    if (config_it == interface_motor_config_.end()) {
        return false;
    }
    
    try {
        const auto& motor_ids = config_it->second;
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            float effort = (i < joint_efforts.size()) ? static_cast<float>(joint_efforts[i]) : 0.0f;
            control_motor_in_effort_mode(interface, motor_ids[i], effort);
        }
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool RobotHardware::send_realtime_mit_command(const std::string& interface, const std::vector<double>& joint_positions, const std::vector<double>& joint_velocities, const std::vector<double>& joint_efforts) {
    auto config_it = interface_motor_config_.find(interface);
    if (config_it == interface_motor_config_.end()) {
        return false;
    }
    
    try {
        const auto& motor_ids = config_it->second;
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            float position = (i < joint_positions.size()) ? static_cast<float>(joint_positions[i]) : 0.0f;
            float velocity = (i < joint_velocities.size()) ? static_cast<float>(joint_velocities[i]) : 0.0f;
            float effort = (i < joint_efforts.size()) ? static_cast<float>(joint_efforts[i]) : 0.0f;
            control_motor_in_mit_mode(interface, motor_ids[i], position, velocity, effort);
        }
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// ========== 轨迹执行接口 ==========
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
        
        for (const auto& point : trajectory.points) {
            // 计算目标时间
            auto target_time = start_time + std::chrono::duration<double>(point.time_from_start);
            
            // 等待到正确时间
            std::this_thread::sleep_until(target_time);
            
            // 发送控制命令到每个电机
            for (size_t i = 0; i < motor_ids.size() && i < point.positions.size(); ++i) {
                float position = static_cast<float>(point.positions[i]);
                float velocity = (i < point.velocities.size()) ? static_cast<float>(point.velocities[i]) : 0.0f;
                float acceleration = (i < point.accelerations.size()) ? static_cast<float>(point.accelerations[i]) : 0.0f;
                
                // 使用MIT模式控制，复用现有的频率控制和优先级机制
                // control_motor_in_mit_mode(interface, motor_ids[i], position, velocity, acceleration);
                control_motor_in_position_mode(interface, motor_ids[i], position);
            }
        }
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

// ========== 工厂函数实现 ==========
namespace hardware_driver {
    std::shared_ptr<motor_driver::MotorDriverInterface> createCanFdMotorDriver(
        const std::vector<std::string>& interfaces) {
        // 创建CANFD总线
        auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
        
        // 创建电机驱动实例
        return std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
    }
}
