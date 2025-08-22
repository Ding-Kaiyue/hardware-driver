#include "motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"

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

void MotorDriverImpl::send_control_command(const bus::GenericBusPacket& packet) {
    // 无界队列：直接推入，保证每条命令都能发送
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        control_queue_.push(packet);
    }
    
    // 唤醒控制线程
    control_cv_.notify_one();
}

void MotorDriverImpl::disable_motor(const std::string interface, const uint32_t motor_id) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;  // 与其他控制命令保持一致
    
    if (motor_protocol::pack_disable_command(packet.data, packet.len)) {
        // 所有命令统一按时间顺序发送
        send_control_command(packet);
    }
}

void MotorDriverImpl::enable_motor(const std::string interface, const uint32_t motor_id, uint8_t mode) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;
    
    if (motor_protocol::pack_enable_command(packet.data, packet.len, mode)) {
        // 所有命令统一按时间顺序发送  
        send_control_command(packet);
    }
}

void MotorDriverImpl::send_position_cmd(const std::string interface, const uint32_t motor_id, float position) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;
    
    if (motor_protocol::pack_position_command(packet.data, packet.len, position)) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::send_velocity_cmd(const std::string interface, const uint32_t motor_id, float velocity) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;
    
    if (motor_protocol::pack_velocity_command(packet.data, packet.len, velocity)) {
        send_control_command(packet);
    } 
}

void MotorDriverImpl::send_effort_cmd(const std::string interface, const uint32_t motor_id, float effort) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;
    
    if (motor_protocol::pack_effort_command(packet.data, packet.len, effort)) {
        send_control_command(packet);
    }
}

void MotorDriverImpl::send_mit_cmd(const std::string interface, const uint32_t motor_id, float position, float velocity, float effort) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;
    
    if (motor_protocol::pack_mit_command(packet.data, packet.len, position, velocity, effort)) {
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
            return !running_ || !control_queue_.empty(); 
        });
        
        if (!running_) break;
        
        // 批量处理控制命令，保证200us间隔
        auto next_send_time = std::chrono::steady_clock::now();
        
        while (!control_queue_.empty()) {
            auto packet = control_queue_.front();
            control_queue_.pop();
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

void MotorDriverImpl::remove_observer(std::shared_ptr<MotorStatusObserver> observer) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.erase(
        std::remove_if(observers_.begin(), observers_.end(),
            [&observer](const std::weak_ptr<MotorStatusObserver>& weak_obs) {
                return weak_obs.lock() == observer;
            }), 
        observers_.end());
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

}   // namespace motor_driver
}   // namespace hardware_driver


