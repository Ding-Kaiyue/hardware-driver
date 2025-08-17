#include "hardware_driver/interface/robot_hardware.hpp"
#include <chrono>
#include <thread>
#include <cstring>

# define REQUEST_ALL
// # define REQUEST_BY_MOTOR_ID     // 只适用于单条总线

RobotHardware::RobotHardware(
    std::shared_ptr<hardware_driver::motor_driver::MotorDriverInterface> motor_driver,
    const std::map<std::string, std::vector<uint32_t>>& interface_motor_config)
    : motor_driver_(std::move(motor_driver)),
      interface_motor_config_(interface_motor_config),
      running_(true)
{
    // 启动两个线程
    feedback_request_thread_ = std::thread(&RobotHardware::request_feedback_thread, this);
    feedback_process_thread_ = std::thread(&RobotHardware::process_feedback_thread, this);
}

RobotHardware::~RobotHardware() {
    running_ = false;
    if (feedback_request_thread_.joinable()) feedback_request_thread_.join();
    if (feedback_process_thread_.joinable()) feedback_process_thread_.join();
}

// ========== 电机状态获取接口 ==========
hardware_driver::motor_driver::Motor_Status RobotHardware::get_motor_status(const std::string& interface, const uint32_t motor_id) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    auto interface_it = status_map_.find(interface);
    if (interface_it != status_map_.end()) {
        auto motor_it = interface_it->second.find(motor_id);
        if (motor_it != interface_it->second.end()) {
            return motor_it->second;
        }
    }
    // 如果没找到，返回默认状态
    return hardware_driver::motor_driver::Motor_Status{};
}

std::map<std::pair<std::string, uint32_t>, hardware_driver::motor_driver::Motor_Status>
RobotHardware::get_all_motor_status(const std::string& interface) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    std::map<std::pair<std::string, uint32_t>, hardware_driver::motor_driver::Motor_Status> result;
    
    if (interface.empty()) {
        // 返回所有接口的所有电机状态
        for (const auto& [interface_name, motor_map] : status_map_) {
            for (const auto& [motor_id, status] : motor_map) {
                result[{std::string(interface_name), motor_id}] = status;
            }
        }
    } else {
        // 返回指定接口的所有电机状态
        auto interface_it = status_map_.find(interface);
        if (interface_it != status_map_.end()) {
            for (const auto& [motor_id, status] : interface_it->second) {
                result[{interface, motor_id}] = status;
            }
        }
    }
    return result;
}

// ========== 电机控制接口 ==========
void RobotHardware::update_control_time() {
    last_control_time_ = std::chrono::steady_clock::now();
    high_freq_mode_ = true;
}

void RobotHardware::ensure_send_interval(const std::string& interface, std::chrono::microseconds min_interval) {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    auto now = std::chrono::steady_clock::now();
    auto it = interface_last_send_time_.find(interface);
    
    if (it != interface_last_send_time_.end()) {
        auto time_since_last = now - it->second;
        if (time_since_last < min_interval) {
            auto sleep_time = min_interval - time_since_last;
            std::this_thread::sleep_for(sleep_time);
        }
    }
    
    interface_last_send_time_[interface] = std::chrono::steady_clock::now();
}

void RobotHardware::control_motor_in_position_mode(const std::string& interface, const uint32_t motor_id, float position) {
    update_control_time();
    ensure_send_interval(interface, std::chrono::microseconds(200)); // 5kHz发送频率
    // 直接发送，在send_packet里加锁保护
    motor_driver_->send_position_cmd(interface, motor_id, position);
}

void RobotHardware::control_motor_in_velocity_mode(const std::string& interface, const uint32_t motor_id, float velocity) {
    update_control_time();
    ensure_send_interval(interface, std::chrono::microseconds(200)); // 5kHz发送频率，平衡性能和可靠性
    // 直接发送，在send_packet里加锁保护
    motor_driver_->send_velocity_cmd(interface, motor_id, velocity);
}

void RobotHardware::control_motor_in_effort_mode(const std::string& interface, const uint32_t motor_id, float effort) {
    update_control_time();
    ensure_send_interval(interface, std::chrono::microseconds(200)); // 5kHz发送频率，平衡性能和可靠性
    // 直接发送，在send_packet里加锁保护
    motor_driver_->send_effort_cmd(interface, motor_id, effort);
}

void RobotHardware::control_motor_in_mit_mode(const std::string& interface, const uint32_t motor_id, float position, float velocity, float effort) {
    update_control_time();
    ensure_send_interval(interface, std::chrono::microseconds(200)); // 5kHz发送频率，平衡性能和可靠性
    // 直接发送，在send_packet里加锁保护
    motor_driver_->send_mit_cmd(interface, motor_id, position, velocity, effort);
}

void RobotHardware::disable_motor(const std::string& interface, const uint32_t motor_id) {
    update_control_time();
    // 失能命令使用更短的间隔，确保快速响应
    ensure_send_interval(interface, std::chrono::microseconds(200)); // 5kHz发送频率，平衡性能和可靠性
    // 直接发送，在send_packet里加锁保护
    motor_driver_->disable_motor(interface, motor_id);
}

void RobotHardware::enable_motor(const std::string& interface, const uint32_t motor_id, uint8_t mode) {
    update_control_time();
    ensure_send_interval(interface, std::chrono::microseconds(200)); // 5kHz发送频率，平衡性能和可靠性
    // 直接发送，在send_packet里加锁保护
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
                control_motor_in_mit_mode(interface, motor_ids[i], position, velocity, acceleration);
            }
        }
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// ========== 参数读写接口 ==========
void RobotHardware::motor_parameter_read(const std::string& interface, const uint32_t motor_id, uint16_t address) {
    // 直接发送，在send_packet里加锁保护
    motor_driver_->motor_parameter_read(interface, motor_id, address);
}

void RobotHardware::motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, int32_t value) {
    // 直接发送，在send_packet里加锁保护
    motor_driver_->motor_parameter_write(interface, motor_id, address, value);
}

void RobotHardware::motor_parameter_write(const std::string& interface, const uint32_t motor_id, uint16_t address, float value) {
    // 直接发送，在send_packet里加锁保护
    motor_driver_->motor_parameter_write(interface, motor_id, address, value);
}

// ========== 函数操作接口 ==========
void RobotHardware::motor_function_operation(const std::string& interface, const uint32_t motor_id, uint8_t operation) {
    // 直接发送，在send_packet里加锁保护
    motor_driver_->motor_function_operation(interface, motor_id, operation);
}

void RobotHardware::arm_zero_position_set(const std::string& interface, const std::vector<uint32_t> motor_ids) {
    for (size_t i = 0; i < motor_ids.size(); i++) {
       // 直接发送，在send_packet里加锁保护
       motor_driver_->motor_function_operation(interface, motor_ids.at(i), 4);
       std::this_thread::sleep_for(std::chrono::milliseconds(10));
       motor_driver_->motor_function_operation(interface, motor_ids.at(i), 2);
       std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



// ========== 反馈请求接口 ==========
void RobotHardware::motor_feedback_request(const std::string& interface, const uint32_t motor_id) {
    motor_driver_->motor_feedback_request(interface, motor_id);
}

void RobotHardware::motor_feedback_request_all(const std::string& interface) {
    motor_driver_->motor_feedback_request_all(interface);
}

void RobotHardware::request_feedback_thread() {
    #ifdef REQUEST_BY_MOTOR_ID
    // 为每个电机初始化下次请求时间，使用array避免动态内存分配
    struct MotorTimer {
        const char* interface;  // 使用const char*避免string拷贝
        uint32_t motor_id;
        std::chrono::steady_clock::time_point next_time;
    };

    // 假设单条总线最多支持8个电机，可以根据实际需求调整
    constexpr size_t MAX_MOTORS = 8;
    std::array<MotorTimer, MAX_MOTORS> motor_timers;
    size_t motor_count = 0;

    // 初始化电机定时器（用于REQUEST_BY_MOTOR_ID）
    for (const auto& [interface, motor_ids] : interface_motor_config_) {
        for (const auto& motor_id : motor_ids) {
            if (motor_count < MAX_MOTORS) {
                motor_timers[motor_count] = {interface.c_str(), motor_id, std::chrono::steady_clock::now()};
                motor_count++;
            }
        }
    }
    #endif
    
    #ifdef REQUEST_ALL
    // 为每个接口创建独立的定时器
    struct InterfaceTimer {
        const char* interface;  // 使用const char*避免string拷贝
        std::chrono::steady_clock::time_point next_time;
    };
    // 假设最多支持4条总线，可以根据实际需求调整
    constexpr size_t MAX_INTERFACES = 4;
    std::array<InterfaceTimer, MAX_INTERFACES> interface_timers;
    size_t interface_count = 0;

    // 初始化接口定时器（用于REQUEST_ALL），错开传输时间
    size_t interface_index = 0;
    for (const auto& [interface, motor_ids] : interface_motor_config_) {
        if (interface_count < MAX_INTERFACES) {
            // 错开每个接口的初始时间，避免同时传输
            auto initial_time = std::chrono::steady_clock::now() + 
                              std::chrono::microseconds(interface_index * 200); // 每个接口错开200μs
            interface_timers[interface_count] = {interface.c_str(), initial_time};
            interface_count++;
            interface_index++;
        }
    }
    #endif
    const int high_freq_us = 400; // 2.5kHz
    const int low_freq_us = 50000; // 20Hz
    const auto high_freq_timeout = std::chrono::milliseconds(100); // 100ms无控制命令则降频

    while (running_) {
        auto now = std::chrono::steady_clock::now();
        
        #ifdef REQUEST_BY_MOTOR_ID
        // 按电机ID分别请求反馈，只有一条总线使用的情况下，限制请求频率在2k以下，避免总线负载过高
        for (size_t i = 0; i < motor_count; ++i) {
            auto& timer = motor_timers[i];
            if (now >= timer.next_time) {
                // 确定频率
                bool is_high_freq = high_freq_mode_ && (now - last_control_time_ <= high_freq_timeout);
                int period_us = is_high_freq ? high_freq_us : low_freq_us;
                
                // 发送请求
                motor_driver_->motor_feedback_request(timer.interface, timer.motor_id);
                
                // 更新下次请求时间
                timer.next_time = now + std::chrono::microseconds(period_us);
            }
        }
        #endif

        #ifdef REQUEST_ALL
        // 方式2：按接口一次性请求所有电机反馈（当前使用）
        for (const auto& [interface, motor_ids] : interface_motor_config_) {
            // 为每个接口找到对应的定时器
            for (size_t i = 0; i < interface_count; ++i) {
                if (strcmp(interface_timers[i].interface, interface.c_str()) == 0) {
                    auto& timer = interface_timers[i];
                    if (now >= timer.next_time) {
                        // 确定频率
                        bool is_high_freq = high_freq_mode_ && (now - last_control_time_ <= high_freq_timeout);
                        int period_us = is_high_freq ? high_freq_us : low_freq_us;
                        
                        // 发送请求所有电机的反馈
                        motor_driver_->motor_feedback_request_all(interface);
                        
                        // 更新下次请求时间
                        timer.next_time = now + std::chrono::microseconds(period_us);
                    }
                    break;
                }
            }
        }
        #endif
        // 短暂休眠，避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::microseconds(25)); // 进一步减少到25us
    }
}

void RobotHardware::process_feedback_thread() {
    auto next_time = std::chrono::high_resolution_clock::now();

    while (running_) {
        // 遍历每个接口及其对应的电机ID列表
        for (const auto& [interface, motor_ids] : interface_motor_config_) {
            for (const auto& motor_id : motor_ids) {
                // 直接获取状态，send_packet里已经加锁保护
                auto status = motor_driver_->get_motor_status(interface, motor_id);
                std::lock_guard<std::mutex> status_lock(status_mutex_);
                status_map_[interface][motor_id] = status;
            }
        }
        next_time += std::chrono::microseconds(200); // 200us = 0.2ms = 5khz
        std::this_thread::sleep_until(next_time);
    }
}

// ========== HardwareDriver实现 ==========
#include "hardware_driver.hpp"
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"

namespace hardware_driver {

HardwareDriver::HardwareDriver(const std::vector<std::string>& interfaces,
                               const std::map<std::string, std::vector<uint32_t>>& motor_config,
                               const std::map<std::string, std::string>& label_to_interface_map)
    : label_to_interface_map_(label_to_interface_map) {
    // 创建CAN总线
    auto bus = std::make_shared<bus::CanFdBus>(interfaces);
    
    // 创建电机驱动
    auto motor_driver = std::make_shared<motor_driver::MotorDriverImpl>(bus);
    
    // 创建硬件接口
    robot_hardware_ = std::make_unique<RobotHardware>(motor_driver, motor_config);
}

HardwareDriver::HardwareDriver(const std::vector<std::string>& interfaces,
                               const std::map<std::string, std::vector<uint32_t>>& motor_config)
    : label_to_interface_map_({}) {
    // 创建CAN总线
    auto bus = std::make_shared<bus::CanFdBus>(interfaces);
    
    // 创建电机驱动
    auto motor_driver = std::make_shared<motor_driver::MotorDriverImpl>(bus);
    
    // 创建硬件接口
    robot_hardware_ = std::make_unique<RobotHardware>(motor_driver, motor_config);
}

HardwareDriver::~HardwareDriver() = default;

void HardwareDriver::control_motor_in_velocity_mode(const std::string& interface, uint32_t motor_id, float velocity) {
    robot_hardware_->control_motor_in_velocity_mode(interface, motor_id, velocity);
}

void HardwareDriver::control_motor_in_position_mode(const std::string& interface, uint32_t motor_id, float position) {
    robot_hardware_->control_motor_in_position_mode(interface, motor_id, position);
}

void HardwareDriver::control_motor_in_effort_mode(const std::string& interface, uint32_t motor_id, float effort) {
    robot_hardware_->control_motor_in_effort_mode(interface, motor_id, effort);
}

void HardwareDriver::control_motor_in_mit_mode(const std::string& interface, uint32_t motor_id, 
                                               float position, float velocity, float effort) {
    robot_hardware_->control_motor_in_mit_mode(interface, motor_id, position, velocity, effort);
}

void HardwareDriver::enable_motor(const std::string& interface, uint32_t motor_id, uint8_t mode) {
    robot_hardware_->enable_motor(interface, motor_id, mode);
}

void HardwareDriver::disable_motor(const std::string& interface, uint32_t motor_id) {
    robot_hardware_->disable_motor(interface, motor_id);
}

motor_driver::Motor_Status HardwareDriver::get_motor_status(const std::string& interface, uint32_t motor_id) {
    return robot_hardware_->get_motor_status(interface, motor_id);
}

std::map<std::pair<std::string, uint32_t>, motor_driver::Motor_Status> 
HardwareDriver::get_all_motor_status(const std::string& interface) {
    return robot_hardware_->get_all_motor_status(interface);
}

bool HardwareDriver::send_realtime_velocity_command(const std::string& interface, const std::vector<double>& joint_velocities) {
    return robot_hardware_->send_realtime_velocity_command(interface, joint_velocities);
}

bool HardwareDriver::send_realtime_position_command(const std::string& interface, const std::vector<double>& joint_positions) {
    return robot_hardware_->send_realtime_position_command(interface, joint_positions);
}

bool HardwareDriver::send_realtime_effort_command(const std::string& interface, const std::vector<double>& joint_efforts) {
    return robot_hardware_->send_realtime_effort_command(interface, joint_efforts);
}

bool HardwareDriver::send_realtime_mit_command(const std::string& interface, const std::vector<double>& joint_positions, const std::vector<double>& joint_velocities, const std::vector<double>& joint_efforts) {
    return robot_hardware_->send_realtime_mit_command(interface, joint_positions, joint_velocities, joint_efforts);
}

bool HardwareDriver::execute_trajectory(const std::string& label, const Trajectory& trajectory) {
    std::string interface = get_interface_from_label(label);
    if (interface.empty()) {
        return false;
    }
    return robot_hardware_->execute_trajectory(interface, trajectory);
}

std::string HardwareDriver::get_interface_from_label(const std::string& label) const {
    auto it = label_to_interface_map_.find(label);
    if (it != label_to_interface_map_.end()) {
        return it->second;
    }
    // 如果没有找到映射，假设label就是interface名称
    return label;
}

} // namespace hardware_driver

// ========== HardwareUtility实现 ==========
#include "hardware_utility.hpp"
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"

namespace hardware_driver {

HardwareUtility::HardwareUtility(const std::vector<std::string>& interfaces,
                                 const std::map<std::string, std::vector<uint32_t>>& motor_config) {
    // 创建CAN总线
    auto bus = std::make_shared<bus::CanFdBus>(interfaces);
    
    // 创建电机驱动
    auto motor_driver = std::make_shared<motor_driver::MotorDriverImpl>(bus);
    
    // 创建硬件接口
    robot_hardware_ = std::make_unique<RobotHardware>(motor_driver, motor_config);  
}

HardwareUtility::~HardwareUtility() = default;

void HardwareUtility::param_read(const std::string& interface, uint32_t motor_id, uint16_t address) {
    robot_hardware_->motor_parameter_read(interface, motor_id, address);
}

void HardwareUtility::param_write(const std::string& interface, uint32_t motor_id, uint16_t address, int32_t value) {
    robot_hardware_->motor_parameter_write(interface, motor_id, address, value);
}

void HardwareUtility::param_write(const std::string& interface, uint32_t motor_id, uint16_t address, float value) {
    robot_hardware_->motor_parameter_write(interface, motor_id, address, value);
}

void HardwareUtility::function_operation(const std::string& interface, uint32_t motor_id, uint8_t opcode) {
    robot_hardware_->motor_function_operation(interface, motor_id, opcode);
}

void HardwareUtility::zero_position_set(const std::string& interface, std::vector<uint32_t> motor_ids) {
    robot_hardware_->arm_zero_position_set(interface, motor_ids);
}

}  // namespace hardware_driver