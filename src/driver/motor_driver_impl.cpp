#include "motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"
#include <thread>
#include <algorithm>

namespace hardware_driver {
namespace motor_driver {

// #define PRINT_DEBUG 

MotorDriverImpl::MotorDriverImpl(std::shared_ptr<bus::BusInterface> bus)
    : bus_(std::move(bus))
{
    // 设置CAN FD和扩展帧
    auto canfd_bus = std::dynamic_pointer_cast<bus::CanFdBus>(bus_);
    if (canfd_bus) {
        bool use_canfd_ = true;
        bool use_extended_ = true;
        for (const auto& interface : canfd_bus->get_interface_names()) {
            canfd_bus->set_extended_frame(interface, use_extended_);
            canfd_bus->set_fd_mode(interface, use_canfd_);
        }
    }
    // 初始化控制时间
    last_control_time_ = std::chrono::steady_clock::now();

    // 注册异步接收回调 - 只负责入队，不阻塞接收线程
    bus_->async_receive([this](const bus::GenericBusPacket& packet) {
        {
            std::lock_guard<std::mutex> lock(receive_mutex_);
            
            // 检查接收队列大小，实现背压控制
            if (receive_queue_.size() >= MAX_QUEUE_SIZE) {
                // 队列满时，丢弃最旧的数据，保留最新的数据
                receive_queue_.pop();
                std::cerr << "Warning: Receive queue overflow, dropping oldest packet" << std::endl;
            }
            
            receive_queue_.push(packet);
        }
        receive_cv_.notify_one();
    });

    // 启动三线程架构
    data_processing_thread_ = std::thread(&MotorDriverImpl::data_processing_worker, this);
    feedback_request_thread_ = std::thread(&MotorDriverImpl::feedback_request_worker, this);
    control_thread_ = std::thread(&MotorDriverImpl::control_worker, this);
}

MotorDriverImpl::~MotorDriverImpl() {
    // 停止三线程
    running_ = false;
    
    // 唤醒所有等待的线程以便它们能够退出
    control_cv_.notify_all();
    receive_cv_.notify_all();
    
    if (data_processing_thread_.joinable()) {
        data_processing_thread_.join();
    }
    if (feedback_request_thread_.joinable()) {
        feedback_request_thread_.join();
    }
    if (control_thread_.joinable()) {
        control_thread_.join();
        cleanup_cpu_binding();
    }

    std::cout << "MotorDriverImpl destroyed, all resources cleaned up." << std::endl;
}

// ========== 新增公共接口实现 ==========
void MotorDriverImpl::register_feedback_callback(FeedbackCallback callback) {
    feedback_callback_ = std::move(callback);
}

void MotorDriverImpl::set_motor_config(const std::map<std::string, std::vector<uint32_t>>& config) {
    interface_motor_config_ = config;
    std::cout << "电机配置已设置，开始监控反馈：" << std::endl;
    for (const auto& [interface, motor_ids] : interface_motor_config_) {
        std::cout << "  " << interface << ": [";
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            std::cout << motor_ids[i];
            if (i < motor_ids.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
}

void MotorDriverImpl::pause_feedback_request() {
    // 保存当前配置并清空，停止反馈请求
    saved_motor_config_ = interface_motor_config_;
    interface_motor_config_.clear();
    feedback_request_paused_.store(true, std::memory_order_release);
    std::cout << "[Feedback] Paused feedback request" << std::endl;
}

void MotorDriverImpl::resume_feedback_request() {
    // 恢复之前保存的配置
    interface_motor_config_ = saved_motor_config_;
    feedback_request_paused_.store(false, std::memory_order_release);
    std::cout << "[Feedback] Resumed feedback request" << std::endl;
}

bool MotorDriverImpl::send_control_command_timeout(const bus::GenericBusPacket& packet, std::chrono::milliseconds timeout) {
    // 有界队列：带超时的安全版本，避免程序永久阻塞
    {
        std::unique_lock<std::mutex> lock(control_mutex_);
        
        // 超时等待直到队列有空间
        if (!control_cv_.wait_for(lock, timeout, [this] { 
            return control_priority_queue_.size() < MAX_QUEUE_SIZE; 
        })) {
            // 超时警告但不阻塞程序
            std::cerr << "Warning: Control queue full (size: " << control_priority_queue_.size() 
                      << "), command may be delayed or dropped" << std::endl;
            return false;   // 返回值表示是否成功加入队列
        }
        
        // 使用默认优先级
        control_priority_queue_.emplace(packet, CommandPriority::LOW);
    }
    
    // 唤醒控制线程
    control_cv_.notify_one();
    return true;
}

void MotorDriverImpl::send_control_command(const bus::GenericBusPacket& packet) {
    // 默认使用低优先级（普通控制命令）
    send_control_command(packet, CommandPriority::LOW);
}

void MotorDriverImpl::send_control_command(const bus::GenericBusPacket& packet, CommandPriority priority) {
    // 重复命令过滤：避免队列被相同命令填满
    if (is_duplicate_command(packet)) {
        // 跳过重复命令，但不报错
        return;
    }
    
    // 优先级队列：高优先级命令会优先处理
    {
        std::unique_lock<std::mutex> lock(control_mutex_);
        
        // 队列满时警告，但依然等待（确保不丢包）
        if (control_priority_queue_.size() >= MAX_QUEUE_SIZE * 0.8) {  // 80%时开始警告
            std::cerr << "Warning: Control queue is " << (control_priority_queue_.size() * 100 / MAX_QUEUE_SIZE) 
                      << "% full (" << control_priority_queue_.size() << "/" << MAX_QUEUE_SIZE << ")" << std::endl;
        }
        
        // 阻塞等待，绝不丢包
        control_cv_.wait(lock, [this] { 
            return control_priority_queue_.size() < MAX_QUEUE_SIZE; 
        });
        
        // 创建优先级命令并加入队列
        control_priority_queue_.emplace(packet, priority);
    }
    
    // 唤醒控制线程
    control_cv_.notify_one();
}

void MotorDriverImpl::send_emergency_stop(const std::string& interface, uint32_t motor_id) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;

    // 创建紧急停止命令（速度设为0，位置和力矩不变，kp=0, kd=0）
    if (motor_protocol::pack_control_command(packet.data, packet.len, 0.0f, 0.0f, 0.0f, 0, 0)) {
        // 使用最高优先级
        send_control_command(packet, CommandPriority::EMERGENCY);

        std::cout << "Emergency stop sent for motor " << interface << ":" << motor_id << std::endl;
    } else {
        std::cerr << "Failed to create emergency stop command for motor " << interface << ":" << motor_id << std::endl;
    }
}

void MotorDriverImpl::disable_motor(const std::string interface, const uint32_t motor_id, uint8_t mode) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;  // 与其他控制命令保持一致

    if (motor_protocol::pack_disable_command(packet.data, packet.len, mode)) {
        // 失能命令使用高优先级
        send_control_command(packet, CommandPriority::HIGH);
    }

    // 保存电机的模式信息
    {
        std::lock_guard<std::mutex> lock(motor_modes_mutex_);
        Motor_Key key{interface, motor_id};
        motor_modes_[key] = mode;
    }
}

void MotorDriverImpl::disable_all_motors(const std::string interface, std::vector<uint32_t> motor_ids, uint8_t mode) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x00;  // 对所有电机发送失能命令

    std::array<uint8_t, 6> disable_flags;
    std::array<uint8_t, 6> mode_array;
    disable_flags.fill(0x00);  // 初始化所有电机为失能状态

    {
        std::lock_guard<std::mutex> lock(motor_modes_mutex_);

        // 为所有电机填充模式数组
        for (size_t i = 0; i < 6; ++i) {
            uint32_t motor_id = i + 1;  // 电机ID从1-6
            Motor_Key key{interface, motor_id};

            // 如果电机在指定列表中，设置失能标志为0，使用传入的mode
            if (std::find(motor_ids.begin(), motor_ids.end(), motor_id) != motor_ids.end()) {
                disable_flags[i] = 0x00;  // 设置为失能
                mode_array[i] = mode;     // 使用指定的mode
            } else {
                // 其他电机保持当前状态，使用其当前模式
                disable_flags[i] = 0x00;  // 不改变状态（默认失能）
                auto it = motor_modes_.find(key);
                mode_array[i] = (it != motor_modes_.end()) ? it->second : 0x04;  // 默认速度模式
            }
        }
    }

    if (motor_protocol::pack_disable_all_command(packet.data, packet.len, disable_flags, mode_array)) {
        // 失能命令使用高优先级
        send_control_command(packet, CommandPriority::HIGH);
    }
}

void MotorDriverImpl::enable_motor(const std::string interface, const uint32_t motor_id, uint8_t mode) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;

    if (motor_protocol::pack_enable_command(packet.data, packet.len, mode)) {
        // 使能命令使用高优先级
        send_control_command(packet, CommandPriority::HIGH);
    }

    // 保存电机的模式信息
    {
        std::lock_guard<std::mutex> lock(motor_modes_mutex_);
        Motor_Key key{interface, motor_id};
        motor_modes_[key] = mode;
    }
}

void MotorDriverImpl::enable_all_motors(const std::string interface, std::vector<uint32_t> motor_ids, uint8_t mode) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x00;  // 对所有电机发送使能命令

    std::array<uint8_t, 6> enable_flags;
    std::array<uint8_t, 6> mode_array;
    enable_flags.fill(0x00);  // 初始化所有电机为失能状态

    {
        std::lock_guard<std::mutex> lock(motor_modes_mutex_);

        // 为所有电机填充模式数组
        for (size_t i = 0; i < 6; ++i) {
            uint32_t motor_id = i + 1;  // 电机ID从1-6
            Motor_Key key{interface, motor_id};

            // 如果电机在指定列表中，使用新的mode并设置使能标志为1
            if (std::find(motor_ids.begin(), motor_ids.end(), motor_id) != motor_ids.end()) {
                enable_flags[i] = 0x01;  // 设置为使能
                mode_array[i] = mode;    // 使用新的mode
                motor_modes_[key] = mode;  // 保存新模式
            } else {
                // 其他电机保持失能，使用其当前模式
                enable_flags[i] = 0x00;  // 保持失能
                auto it = motor_modes_.find(key);
                mode_array[i] = (it != motor_modes_.end()) ? it->second : 0x04;  // 默认速度模式
            }
        }
    }

    if (motor_protocol::pack_enable_all_command(packet.data, packet.len, enable_flags, mode_array)) {
        // 使能命令使用高优先级
        send_control_command(packet, CommandPriority::HIGH);
    }
}
    
void MotorDriverImpl::send_position_cmd(const std::string interface, const uint32_t motor_id, 
    float position, float kp, float kd) {

    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;

    // 位置命令：只控制位置，速度和力矩设为0
    if (motor_protocol::pack_control_command(packet.data, packet.len, position, 0.0f, 0.0f, (uint8_t)(1000 * kp), (uint8_t)(1000 * kd))) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::send_velocity_cmd(const std::string interface, const uint32_t motor_id, 
    float velocity, float kp, float kd) {

    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;

    // 速度命令：只控制速度，位置和力矩设为0
    if (motor_protocol::pack_control_command(packet.data, packet.len, 0.0f, velocity, 0.0f, (uint8_t)(1000 * kp), (uint8_t)(1000 * kd))) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::send_effort_cmd(const std::string interface, const uint32_t motor_id, 
    float effort, float kp, float kd) {

    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;

    // 力矩命令：只控制力矩，位置和速度设为0
    if (motor_protocol::pack_control_command(packet.data, packet.len, 0.0f, 0.0f, effort, kp, kd)) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::send_mit_cmd(const std::string interface, const uint32_t motor_id, 
    float position, float velocity, float effort, 
    float kp, float kd) {

    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;

    // MIT模式命令
    if (motor_protocol::pack_control_command(packet.data, packet.len, position, velocity, effort, kp, kd)) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::send_control_cmd(const std::string interface,
    std::vector<float> positions, std::vector<float> velocities, std::vector<float> efforts,
    std::vector<float> kps, std::vector<float> kds) {

    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x00;  // 广播给所有电机

    // 转换 vector 为 std::array<float, 6>
    std::array<float, 6> pos_arr = {};
    std::array<float, 6> vel_arr = {};
    std::array<float, 6> eff_arr = {};
    std::array<float, 6> kps_arr = {};
    std::array<float, 6> kds_arr = {};

    for (size_t i = 0; i < std::min(size_t(6), positions.size()); ++i) {
        pos_arr[i] = positions[i];
    }
    for (size_t i = 0; i < std::min(size_t(6), velocities.size()); ++i) {
        vel_arr[i] = velocities[i];
    }
    for (size_t i = 0; i < std::min(size_t(6), efforts.size()); ++i) {
        eff_arr[i] = efforts[i];
    }
    for (size_t i = 0; i < std::min(size_t(6), kps.size()); ++i) {
        kps_arr[i] = kps[i];
    }
    for (size_t i = 0; i < std::min(size_t(6), kds.size()); ++i) {
        kds_arr[i] = kds[i];
    }

    // 同时控制多个电机
    if (motor_protocol::pack_control_all_command(packet.data, packet.len, pos_arr, vel_arr, eff_arr, kps_arr, kds_arr)) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::motor_parameter_read(const std::string interface, const uint32_t motor_id, uint16_t address) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id + 0x600;
    
    if (motor_protocol::pack_param_read(packet.data, packet.len, address)) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::motor_parameter_write(const std::string interface, const uint32_t motor_id, uint16_t address, int32_t value) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id + 0x600;
    
    if (motor_protocol::pack_param_write(packet.data, packet.len, address, value)) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::motor_parameter_write(const std::string interface, const uint32_t motor_id, uint16_t address, float value) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id + 0x600;
    
    if (motor_protocol::pack_param_write(packet.data, packet.len, address, value)) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::motor_function_operation(const std::string interface, const uint32_t motor_id, uint8_t operation) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id + 0x400;
    
    if (motor_protocol::pack_function_operation(packet.data, packet.len, operation)) {
        send_control_command(packet);
    }
}

// ========== 三线程工作函数实现 ==========
void MotorDriverImpl::control_worker() {
    // 设置控制线程的CPU亲和性
    set_thread_cpu_affinity(timing_config_.control_cpu_core);
    
    while (running_) {
        std::unique_lock<std::mutex> lock(control_mutex_);
        
        // 等待控制命令，没有命令时阻塞
        control_cv_.wait(lock, [this] { 
            return !running_ || !control_priority_queue_.empty(); 
        });
        
        if (!running_) break;
        
        // 批量处理控制命令，保证200us间隔
        auto next_send_time = std::chrono::steady_clock::now();
        
        while (!control_priority_queue_.empty()) {
            // 每次都取最高优先级的命令（确保抢占式调度）
            auto priority_cmd = control_priority_queue_.top();
            control_priority_queue_.pop();
            auto packet = priority_cmd.packet;
            
            lock.unlock();  // 释放锁进行发送
            
            try {
                // 混合时序控制：粗粒度sleep + 精确忙等待
                auto now = std::chrono::steady_clock::now();
                if (next_send_time > now) {
                    auto remaining_time = next_send_time - now;
                    
                    // 如果剩余时间大于50μs，先用sleep_until节省CPU
                    if (remaining_time > std::chrono::microseconds(50)) {
                        std::this_thread::sleep_until(next_send_time - std::chrono::microseconds(50));
                    }
                    
                    // 最后50μs用忙等待保证精确时序
                    while (std::chrono::steady_clock::now() < next_send_time) {
                        std::this_thread::yield();  // 让出时间片但保持活跃
                    }
                }
                
                // 发送控制命令
                bus_->send(packet);
                
                // 更新控制时间，切换到高频模式
                last_control_time_ = std::chrono::steady_clock::now();
                high_freq_mode_.store(true, std::memory_order_relaxed);
                
                // 计算下次发送时间，使用配置的控制间隔
                next_send_time += timing_config_.control_interval;
                
            } catch (const std::exception& e) {
                std::cerr << "Error sending control command: " << e.what() << std::endl;
                // 出错时也要更新下次发送时间，避免时间错乱
                next_send_time += timing_config_.control_interval;
            }
            
            lock.lock();  // 重新获取锁检查队列
            
            // 处理完当前批次后，通知等待的生产者线程（用于有界队列的背压控制）
            control_cv_.notify_all();
        }
    }
}

void MotorDriverImpl::feedback_request_worker() {
    // 使用配置的时序参数
    const auto& high_freq_interval = timing_config_.high_freq_feedback;
    const auto& low_freq_interval = timing_config_.low_freq_feedback;
    const auto& mode_timeout = timing_config_.mode_timeout;
    
    auto next_request_time = std::chrono::steady_clock::now();
    
    while (running_) {
        auto now = std::chrono::steady_clock::now();
        
        // 检查频率模式切换
        if (high_freq_mode_.load(std::memory_order_relaxed) && 
           (now - last_control_time_) > mode_timeout) {
            high_freq_mode_.store(false, std::memory_order_relaxed);
        }
        
        // 定时发送反馈请求
        if (now >= next_request_time) {
            auto interval = high_freq_mode_.load(std::memory_order_relaxed) ? 
                           high_freq_interval : low_freq_interval;
            
            try {
                for (const auto& [interface, motor_ids] : interface_motor_config_) {
                    (void) motor_ids;  // 避免未使用变量警告
                    auto feedback_packet = create_feedback_request_all(interface);
                    bus_->send(feedback_packet);
                }
            } catch (const std::exception& e) {
                std::cerr << "Error sending feedback request: " << e.what() << std::endl;
            }
            
            next_request_time = now + interval;
        }
        
        // 短暂休眠，避免过度占用CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void MotorDriverImpl::data_processing_worker() {
    while (running_) {
        bus::GenericBusPacket packet;
        {
            std::unique_lock<std::mutex> lock(receive_mutex_);
            receive_cv_.wait(lock, [this] { 
                return !running_ || !receive_queue_.empty(); 
            });
            if (!running_) break;
            packet = std::move(receive_queue_.front());
            receive_queue_.pop();
        }
        handle_bus_packet(packet);  // 处理单个数据包
    }
}

bus::GenericBusPacket MotorDriverImpl::create_feedback_request_all(const std::string& interface) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x00;  // 广播ID，请求所有电机反馈

    motor_protocol::pack_motor_feedback_request_all(packet.data, packet.len);
    return packet;
}

void MotorDriverImpl::handle_bus_packet(const bus::GenericBusPacket& packet) {
    // 首先尝试解析 IAP 状态反馈（ASCII: "BS00", "BK01", ...）
    auto iap_feedback_opt = iap_protocol::parse_iap_feedback(packet);
    if (iap_feedback_opt) {
        std::cout << "[IAP] ⇦ Received "
                << iap_protocol::iap_status_to_string(iap_feedback_opt->status_msg)
                << " from motor " << iap_feedback_opt->motor_id << std::endl;

        // 通知所有注册的 IAP 观察者
        notify_iap_observers(packet.interface, iap_feedback_opt->motor_id,
                            iap_feedback_opt->status_msg);
        return;
    }

    auto feedback_opt = motor_protocol::parse_feedback(packet);
    if (!feedback_opt) return;

    // 使用std::visit处理不同类型的反馈
    std::visit([this](auto&& feedback) {
        using T = std::decay_t<decltype(feedback)>;
        
        if constexpr (std::is_same_v<T, motor_protocol::MotorStatusFeedback>) {
            // 处理电机状态反馈 - 使用线程安全哈希表，只保存最新状态
            Motor_Key key{feedback.interface, feedback.motor_id};
            
            // 使用shared_mutex的写锁进行更新
            {
                std::unique_lock<std::shared_mutex> lock(status_map_mutex_);
                status_map_[key] = feedback.status;  // 线程安全更新状态
            }
            
            // 立即调用回调函数（向后兼容）
            if (feedback_callback_) {
                feedback_callback_(feedback.interface, feedback.motor_id, feedback.status);
            }
            
            // 通知所有观察者
            notify_motor_status_observers(feedback.interface, feedback.motor_id, feedback.status);
            
            // 发布事件总线事件
            emit_motor_status_event(feedback.interface, feedback.motor_id, feedback.status);
            
#ifdef PRINT_DEBUG
            std::cout << "Interface: " << feedback.interface << " Motor ID: " 
            << feedback.motor_id << " enable_flag: " << feedback.status.enable_flag 
            << " motor_mode: " << feedback.status.motor_mode << " position: " << feedback.status.position
            << " limit_flag: " << feedback.status.limit_flag << " temperature: " << feedback.status.temperature
            << " velocity: " << feedback.status.velocity << " voltage: " << feedback.status.voltage
            << " effort: " << feedback.status.effort << " error_code: " << feedback.status.error_code
            << std::endl;
#endif
        } else if constexpr (std::is_same_v<T, motor_protocol::FuncResultFeedback>) {
            // 处理函数操作反馈
            std::cout << "Interface: " << feedback.interface << " Motor ID: " 
                        << feedback.motor_id << " Operation: " << static_cast<int>(feedback.op_code)
                        << " Success: " << feedback.success << std::endl;
            
            // 通知观察者
            notify_function_result_observers(feedback.interface, feedback.motor_id, feedback.op_code, feedback.success);
            
            // 发布事件总线事件
            emit_motor_function_result_event(feedback.interface, feedback.motor_id, feedback.op_code, feedback.success);
        } else if constexpr (std::is_same_v<T, motor_protocol::ParamResultFeedback>) {
            // 处理参数读写反馈
            std::cout << "Interface: " << feedback.interface << " Motor ID: " 
                        << feedback.motor_id << " Address: " << feedback.addr 
                        << " Data Type: " << static_cast<int>(feedback.data_type) 
                        << " Data: ";
            if (feedback.data_type == 0x01) {  // int
                std::cout << std::any_cast<int32_t>(feedback.data);
            } else if (feedback.data_type == 0x02) {  // float   
                std::cout << std::any_cast<float>(feedback.data);
            }
            std::cout << std::endl;
            
            // 通知观察者
            notify_parameter_result_observers(feedback.interface, feedback.motor_id, 
                                            feedback.addr, feedback.data_type, feedback.data);
            
            // 发布事件总线事件
            emit_motor_parameter_result_event(feedback.interface, feedback.motor_id, 
                                            feedback.addr, feedback.data_type, feedback.data);
        }
    }, *feedback_opt);
}

void MotorDriverImpl::set_thread_cpu_affinity(int cpu_core) {
    if (cpu_core < 0) return;  // -1表示不设置CPU绑定
    
    // 检查系统负载，如果过高则取消绑定
    double system_load = get_system_load_average();
    if (system_load > 15.0) {  // 15/20 = 75%负载
        std::cout << "System load too high (" << system_load 
                  << "), skipping CPU binding for stability" << std::endl;
        return;
    }
    
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);
    
    pthread_t current_thread = pthread_self();
    int result = pthread_setaffinity_np(current_thread, sizeof(cpuset), &cpuset);
    
    if (result == 0) {
        std::cout << "Thread bound to CPU core " << cpu_core 
                  << " (system load: " << system_load << ")" << std::endl;
    } else {
        std::cerr << "Failed to bind thread to CPU core " << cpu_core 
                  << ", error: " << result << std::endl;
    }
}

double MotorDriverImpl::get_system_load_average() const {
    double load_avg[3];
    if (getloadavg(load_avg, 1) == -1) {
        std::cerr << "Failed to get system load average" << std::endl;
        return 0.0;  // 获取失败时返回0，允许CPU绑定
    }
    return load_avg[0];  // 返回1分钟平均负载
}

void MotorDriverImpl::cleanup_cpu_binding() {
    // 重置CPU亲和性为所有核心可用
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    
    // 设置所有20个核心都可用
    for (int i = 2; i < 20; ++i) {
        CPU_SET(i, &cpuset);
    }
    
    pthread_t current_thread = pthread_self();
    int result = pthread_setaffinity_np(current_thread, sizeof(cpuset), &cpuset);
    
    if (result == 0) {
        std::cout << "CPU binding cleaned up, thread can use all cores" << std::endl;
    }
}

// ========== 观察者模式实现 ==========
void MotorDriverImpl::add_observer(std::shared_ptr<MotorStatusObserver> observer) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.push_back(std::weak_ptr<MotorStatusObserver>(observer));
}

void MotorDriverImpl::add_iap_observer(std::shared_ptr<IAPStatusObserver> observer) {
    std::lock_guard<std::mutex> lock(iap_observers_mutex_);
    iap_observers_.push_back(std::weak_ptr<IAPStatusObserver>(observer));
}

void MotorDriverImpl::remove_observer(std::shared_ptr<MotorStatusObserver> observer) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.erase(
        std::remove_if(observers_.begin(), observers_.end(),
            [&observer](const std::weak_ptr<MotorStatusObserver>& weak_obs) {
                return weak_obs.lock() == observer;
            }), 
        observers_.end());
}

void MotorDriverImpl::remove_iap_observer(std::shared_ptr<IAPStatusObserver> observer) {
    std::lock_guard<std::mutex> lock(iap_observers_mutex_);
    iap_observers_.erase(
        std::remove_if(iap_observers_.begin(), iap_observers_.end(),
            [&observer](const std::weak_ptr<IAPStatusObserver>& weak_obs) {
                return weak_obs.lock() == observer;
            }), 
        iap_observers_.end());
}

void MotorDriverImpl::notify_motor_status_observers(const std::string& interface, uint32_t motor_id, const Motor_Status& status) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    
    // 使用迭代器遍历，自动清理失效的观察者
    for (auto it = observers_.begin(); it != observers_.end(); ) {
        if (auto observer = it->lock()) {
            try {
                observer->on_motor_status_update(interface, motor_id, status);
                ++it;
            } catch (const std::exception& e) {
                std::cerr << "Observer error in motor status update: " << e.what() << std::endl;
                ++it;
            }
        } else {
            // 移除失效的观察者
            it = observers_.erase(it);
        }
    }
}

void MotorDriverImpl::notify_function_result_observers(const std::string& interface, uint32_t motor_id, uint8_t op_code, bool success) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    
    for (auto it = observers_.begin(); it != observers_.end(); ) {
        if (auto observer = it->lock()) {
            try {
                observer->on_motor_function_result(interface, motor_id, op_code, success);
                ++it;
            } catch (const std::exception& e) {
                std::cerr << "Observer error in function result: " << e.what() << std::endl;
                ++it;
            }
        } else {
            it = observers_.erase(it);
        }
    }
}

void MotorDriverImpl::notify_parameter_result_observers(const std::string& interface, uint32_t motor_id, uint16_t address, uint8_t data_type, const std::any& data) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    
    for (auto it = observers_.begin(); it != observers_.end(); ) {
        if (auto observer = it->lock()) {
            try {
                observer->on_motor_parameter_result(interface, motor_id, address, data_type, data);
                ++it;
            } catch (const std::exception& e) {
                std::cerr << "Observer error in parameter result: " << e.what() << std::endl;
                ++it;
            }
        } else {
            it = observers_.erase(it);
        }
    }
}

void MotorDriverImpl::notify_iap_observers(const std::string& interface, uint32_t motor_id, iap_protocol::IAPStatusMessage msg) {
    // 保存最新反馈到 map，供 wait_for_feedback 使用
    {
        std::lock_guard<std::mutex> lock(iap_feedback_mutex_);
        iap_latest_feedback_[motor_id] = msg;
        iap_feedback_cv_.notify_all();  // 通知所有等待者
    }

    // 通知所有注册的 IAP 观察者
    std::lock_guard<std::mutex> lock(iap_observers_mutex_);

    for (auto it = iap_observers_.begin(); it != iap_observers_.end(); ) {
        if (auto observer = it->lock()) {
            try {
                observer->on_iap_status_feedback(interface, motor_id, msg);
                ++it;
            } catch (const std::exception& e) {
                std::cerr << "Observer error in IAP status feedback: " << e.what() << std::endl;
                ++it;
            }
        } else {
            it = iap_observers_.erase(it);
        }
    }
}

// ========== 事件总线集成实现 ==========
void MotorDriverImpl::set_event_bus(std::shared_ptr<event::EventBus> event_bus) {
    std::lock_guard<std::mutex> lock(event_bus_mutex_);
    event_bus_ = std::move(event_bus);
}

std::shared_ptr<event::EventBus> MotorDriverImpl::get_event_bus() const {
    std::lock_guard<std::mutex> lock(event_bus_mutex_);
    return event_bus_;
}

void MotorDriverImpl::emit_motor_status_event(const std::string& interface, uint32_t motor_id, const Motor_Status& status) {
    std::shared_ptr<event::EventBus> event_bus;
    {
        std::lock_guard<std::mutex> lock(event_bus_mutex_);
        event_bus = event_bus_;
    }
    
    if (event_bus) {
        try {
            auto event = std::make_shared<event::MotorStatusEvent>(interface, motor_id, status);
            event_bus->publish(event);
        } catch (const std::exception& e) {
            std::cerr << "Error emitting motor status event: " << e.what() << std::endl;
        }
    }
    // else {
    //     std::cout << "DEBUG: event_bus为空，无法发布事件" << std::endl;
    // }
}

void MotorDriverImpl::emit_motor_batch_status_event(const std::string& interface, const std::map<uint32_t, Motor_Status>& status_all) {
    std::shared_ptr<event::EventBus> event_bus;
    {
        std::lock_guard<std::mutex> lock(event_bus_mutex_);
        event_bus = event_bus_;
    }
    
    if (event_bus) {
        try {
            auto event = std::make_shared<event::MotorBatchStatusEvent>(interface, status_all);
            event_bus->publish(event);
        } catch (const std::exception& e) {
            std::cerr << "Error emitting motor batch status event: " << e.what() << std::endl;
        }
    }
}

void MotorDriverImpl::emit_motor_function_result_event(const std::string& interface, uint32_t motor_id, uint8_t op_code, bool success) {
    std::shared_ptr<event::EventBus> event_bus;
    {
        std::lock_guard<std::mutex> lock(event_bus_mutex_);
        event_bus = event_bus_;
    }
    
    if (event_bus) {
        try {
            auto event = std::make_shared<event::MotorFunctionResultEvent>(interface, motor_id, op_code, success);
            event_bus->publish(event);
        } catch (const std::exception& e) {
            std::cerr << "Error emitting motor function result event: " << e.what() << std::endl;
        }
    }
}

void MotorDriverImpl::emit_motor_parameter_result_event(const std::string& interface, uint32_t motor_id, uint16_t address, uint8_t data_type, const std::any& data) {
    std::shared_ptr<event::EventBus> event_bus;
    {
        std::lock_guard<std::mutex> lock(event_bus_mutex_);
        event_bus = event_bus_;
    }
    
    if (event_bus) {
        try {
            auto event = std::make_shared<event::MotorParameterResultEvent>(interface, motor_id, address, data_type, data);
            event_bus->publish(event);
        } catch (const std::exception& e) {
            std::cerr << "Error emitting motor parameter result event: " << e.what() << std::endl;
        }
    }
}

bool MotorDriverImpl::is_duplicate_command(const bus::GenericBusPacket& packet) {
    Motor_Key key{packet.interface, packet.id};
    
    std::lock_guard<std::mutex> lock(last_commands_mutex_);
    
    // 查找该电机的最后一条命令
    auto it = last_commands_.find(key);
    if (it == last_commands_.end()) {
        // 首次命令，不是重复
        last_commands_[key] = packet;
        return false;
    }
    
    // 比较命令内容是否相同
    const auto& last_packet = it->second;
    bool is_duplicate = (packet.len == last_packet.len) && 
                       (std::memcmp(packet.data.data(), last_packet.data.data(), packet.len) == 0);
    
    if (!is_duplicate) {
        // 更新最后命令
        last_commands_[key] = packet;
    }
    
    return is_duplicate;
}

// ========== IAP固件更新接口实现 ==========

void MotorDriverImpl::start_update(const std::string& interface,
                                   uint32_t motor_id,
                                   const std::string& firmware_file)
{
    // 暂停反馈请求以减少CAN总线干扰
    pause_feedback_request();

    std::vector<uint8_t> firmware_data;

    if (!iap_protocol::load_firmware_from_file(firmware_file, firmware_data)) {
        std::cerr << "[IAP] Load firmware failed." << std::endl;
        resume_feedback_request();  // 恢复反馈请求
        return;
    }

    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x1400 + motor_id;

    try {
        // 1. APP 模式下请求进入IAP
        if (iap_protocol::pack_enter_iap_request(packet.data, packet.len)) {
            send_control_command(packet);
        }
        wait_for_feedback(interface, motor_id, iap_protocol::IAPStatusMessage::AJ01, 2000);
        wait_for_feedback(interface, motor_id, iap_protocol::IAPStatusMessage::BS00, 3000);

        // 2. Boot模式下发送key
        if (iap_protocol::pack_send_key(packet.data, packet.len)) {
            send_control_command(packet);
        }
        wait_for_feedback(interface, motor_id, iap_protocol::IAPStatusMessage::BK01, 2000);
        wait_for_feedback(interface, motor_id, iap_protocol::IAPStatusMessage::BK02, 2000);
        wait_for_feedback(interface, motor_id, iap_protocol::IAPStatusMessage::BK03, 2000);

        // 3. Boot模式下发送固件数据
        for (size_t offset = 0; offset < firmware_data.size(); offset += 64) {
            if (iap_protocol::pack_firmware_frame(packet.data, packet.len,
                                                firmware_data, offset)) {
                send_control_command(packet);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        // 等待数据接收完成（BJ06 在 500ms 无数据时自动发出）
        wait_for_feedback(interface, motor_id, iap_protocol::IAPStatusMessage::BJ06, 3000);
        wait_for_feedback(interface, motor_id, iap_protocol::IAPStatusMessage::AS00, 3000);
        std::cout << "[IAP] ✅ Firmware update completed for motor "
                  << interface << ":" << motor_id << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[IAP] Error during firmware update: " << e.what() << std::endl;
    }

    // 恢复反馈请求
    resume_feedback_request();
}

std::optional<hardware_driver::iap_protocol::IAPFeedback> MotorDriverImpl::wait_for_feedback(
    const std::string& /*interface*/,
    uint32_t motor_id,
    hardware_driver::iap_protocol::IAPStatusMessage expected_msg,
    uint32_t timeout_ms)
{
    std::unique_lock<std::mutex> lock(iap_feedback_mutex_);

    auto start_time = std::chrono::steady_clock::now();
    while (true) {
        // 检查是否已经收到期望的反馈
        auto it = iap_latest_feedback_.find(motor_id);
        if (it != iap_latest_feedback_.end() && it->second == expected_msg) {
            iap_protocol::IAPFeedback feedback{};
            feedback.motor_id = motor_id;
            feedback.status_msg = it->second;
            return feedback;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        if (elapsed.count() > timeout_ms) {
            std::cerr << "[IAP] ❌ Timeout waiting for "
                      << iap_status_to_string(expected_msg)
                      << " from motor " << motor_id << std::endl;
            return std::nullopt;
        }

        // 等待新反馈到达
        iap_feedback_cv_.wait_for(lock, std::chrono::milliseconds(10));
    }
}

// ========== 批量控制接口实现 ==========
void MotorDriverImpl::send_position_cmd_all(const std::string interface,
                                          const std::array<float, 6>& positions,
                                          const std::array<float, 6>& kps,
                                          const std::array<float, 6>& kds) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x000;  // 批量控制使用广播ID

    // 位置控制时速度和力矩为 0
    std::array<float, 6> velocities = {};
    std::array<float, 6> efforts = {};

    if (motor_protocol::pack_control_all_command(packet.data, packet.len, positions, velocities, efforts, kps, kds)) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::send_velocity_cmd_all(const std::string interface,
                                          const std::array<float, 6>& velocities,
                                          const std::array<float, 6>& kps,
                                          const std::array<float, 6>& kds) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x000;  // 批量控制使用广播ID

    // 速度控制时位置和力矩为 0
    std::array<float, 6> positions = {};
    std::array<float, 6> efforts = {};

    if (motor_protocol::pack_control_all_command(packet.data, packet.len, positions, velocities, efforts, kps, kds)) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::send_effort_cmd_all(const std::string interface,
                                        const std::array<float, 6>& efforts,
                                        const std::array<float, 6>& kps,
                                        const std::array<float, 6>& kds) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x000;  // 批量控制使用广播ID

    // 力矩控制时位置和速度为 0
    std::array<float, 6> positions = {};
    std::array<float, 6> velocities = {};

    if (motor_protocol::pack_control_all_command(packet.data, packet.len, positions, velocities, efforts, kps, kds)) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::send_mit_cmd_all(const std::string interface,
                                     const std::array<float, 6>& positions,
                                     const std::array<float, 6>& velocities,
                                     const std::array<float, 6>& efforts,
                                     const std::array<float, 6>& kps,
                                     const std::array<float, 6>& kds) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0x000;  // 批量控制使用广播ID

    if (motor_protocol::pack_control_all_command(packet.data, packet.len, positions, velocities, efforts, kps, kds)) {
        send_control_command(packet);
    }
}

}   // namespace motor_driver
}   // namespace hardware_driver


